#include "cluster.h"
#include <iomanip>
#include <osqp.h>


namespace Inference{

void csc2Eigen(csc cscM, Eigen::MatrixXd &M)
{
    M = Eigen::MatrixXd::Zero(cscM.m, cscM.n);
    for(int j=0; j<cscM.n; j++){
        for(int k=cscM.p[j]; k<cscM.p[j+1]; k++){
            M(cscM.i[k], j) = cscM.x[k];
        }
    }
}

void Segment::selectAnchor(int stride)
{
    std::vector<int> nAnchorIndices;
    for(int i=0; i<anchorIndices.size(); i+=stride){
        nAnchorIndices.push_back(anchorIndices[i]);
    }
    nAnchorIndices.back() = anchorIndices.back();
    anchorIndices.swap(nAnchorIndices);
}

bool Segment::QpSpline()
{
    if(anchorIndices.size() < 3){
        return false;
    }
    int interValNum = anchorIndices.size() - 1;    
    c_int n = 4*interValNum;          //number of opt variables
    std::vector<c_float> coeffs_x(n, 0.f); //initial coeff for x variable
    std::vector<c_float> coeffs_y(n, 0.f); //initial coeff for y variable
    std::vector<c_float> coeffs_z(n, 0.f); //initial coeff for z variable
    c_int P_nnz = 10*interValNum;     //only save upper part
    std::vector<c_float> P_x(P_nnz);  //non zero element of P
    std::vector<c_int> P_i(P_nnz);    //row index per element of P
    std::vector<c_int> P_p(n+1);      //accumulated non zero element number of P by column
    std::vector<c_float> qx(n), qy(n), qz(n);   //q vector in optimization     

    P_p[0] = 0;
    //ts = std::vector<double>(cloud->size(), 0.f);
    //fit cubic function at each interval initially
    for(int i=0; i<interValNum; i++){
        int index_0 = anchorIndices[i], index_1 = anchorIndices[i+1];
        Eigen::Vector3d basePos = cloud->at(index_0).getVector3fMap().cast<double>();
        Eigen::Vector3d disp = cloud->at(index_1).getVector3fMap().cast<double>() - basePos;
        double disp_squaredNorm = disp.squaredNorm();
        Eigen::Matrix4d P_interval = Eigen::Matrix4d::Zero();
        Eigen::Vector4d q_interval_x = Eigen::Vector4d::Zero(),
                        q_interval_y = Eigen::Vector4d::Zero(),
                        q_interval_z = Eigen::Vector4d::Zero();
        Eigen::Vector4d tmp;
        for(int k=index_0; k<=index_1; k++){
            Eigen::Vector3d currPos = cloud->at(k).getVector3fMap().cast<double>();
            double dt = (currPos - basePos).dot(disp)/disp_squaredNorm, 
                   dt_2 = dt*dt, dt_3 = dt*dt_2;
            //ts[k] = dt;
            tmp<<1.f, dt, dt_2, dt_3;
            P_interval += tmp*tmp.transpose();
            q_interval_x -= tmp*currPos(0);
            q_interval_y -= tmp*currPos(1);
            q_interval_z -= tmp*currPos(2);
        }
        //P_interval += 1e-8*Eigen::Matrix4d::Identity();
        P_interval.block<2,2>(2,2) += Eigen::Matrix2d::Identity();
        Eigen::Vector4d coeff_x = -P_interval.inverse()*q_interval_x,
                        coeff_y = -P_interval.inverse()*q_interval_y,
                        coeff_z = -P_interval.inverse()*q_interval_z;
        
        int coeffsIndex = 4*i, PIndex = 10*i;
        for(int j=0; j<4; j++){
            coeffs_x[coeffsIndex+j] = coeff_x(j);
            coeffs_y[coeffsIndex+j] = coeff_y(j);
            coeffs_z[coeffsIndex+j] = coeff_z(j);
            qx[coeffsIndex+j] = q_interval_x(j);
            qy[coeffsIndex+j] = q_interval_y(j);
            qz[coeffsIndex+j] = q_interval_z(j);
            P_p[coeffsIndex+j+1] = P_p[coeffsIndex+j] + j + 1;
            PIndex += j;
            for(int k=0; k<=j; k++){
                P_x[PIndex+k] = P_interval(k,j);
                P_i[PIndex+k] = k+coeffsIndex;
            }
        }
    }
    c_int m = 3*(interValNum-1);    //number of constraint
    std::vector<c_float> l(m, -1e-6);
    std::vector<c_float> u(m, 1e-6);
    c_int A_nnz = 12*(interValNum-1); //number of nonzero element in A
    std::vector<c_float> A_x(A_nnz);  //all non zero elements in A
    std::vector<c_int> A_i(A_nnz);    //row index per element in A
    std::vector<c_int> A_p(n+1);      //accumulated non zero element number of A by column
    A_p[0] = 0;
    //assign the first block
    A_p[1] = 1; 
    A_p[2] = 3;
    A_p[3] = 6; 
    A_p[4] = 9;

    A_x[0] = 1.0; 
    A_x[1] = 1.0; 
    A_x[2] = 1.0; 
    A_x[3] = 1.0; 
    A_x[4] = 2.0; 
    A_x[5] = 2.0; 
    A_x[6] = 1.0; 
    A_x[7] = 3.0; 
    A_x[8] = 6.0;

    A_i[0] = 0;    
    A_i[1] = 0; 
    A_i[2] = 1;     
    A_i[3] = 0; 
    A_i[4] = 1; 
    A_i[5] = 2;    
    A_i[6] = 0; 
    A_i[7] = 1; 
    A_i[8] = 2; 
    for(int i=1; i<interValNum-1; i++){
        int colStartIndex = 4*i;
        A_p[colStartIndex+1] = A_p[colStartIndex] + 2;
        A_p[colStartIndex+2] = A_p[colStartIndex+1] + 3;
        A_p[colStartIndex+3] = A_p[colStartIndex+2] + 4;
        A_p[colStartIndex+4] = A_p[colStartIndex+3] + 3;

        int nzStartIndex = 9 + 12*(i-1);
        int rowStartIndex = 3*(i-1);

        A_x[nzStartIndex] = -1.0; 
        A_x[nzStartIndex+1] = 1.0;
        A_x[nzStartIndex+2] = -1.0; 
        A_x[nzStartIndex+3] = 1.0; 
        A_x[nzStartIndex+4] = 1.0;
        A_x[nzStartIndex+5] = -2.0;
        A_x[nzStartIndex+6] = 1.0;
        A_x[nzStartIndex+7] = 2.0;
        A_x[nzStartIndex+8] = 2.0;
        A_x[nzStartIndex+9] = 1.0;
        A_x[nzStartIndex+10] = 3.0;
        A_x[nzStartIndex+11] = 6.0;

        A_i[nzStartIndex] = rowStartIndex; 
        A_i[nzStartIndex+1] = rowStartIndex+3;       
        A_i[nzStartIndex+2] = rowStartIndex+1; 
        A_i[nzStartIndex+3] = rowStartIndex+3;
        A_i[nzStartIndex+4] = rowStartIndex+4;        
        A_i[nzStartIndex+5] = rowStartIndex+2;
        A_i[nzStartIndex+6] = rowStartIndex+3;
        A_i[nzStartIndex+7] = rowStartIndex+4;
        A_i[nzStartIndex+8] = rowStartIndex+5;        
        A_i[nzStartIndex+9] = rowStartIndex+3;
        A_i[nzStartIndex+10] = rowStartIndex+4;
        A_i[nzStartIndex+11] = rowStartIndex+5;
    }
    //assign the last block
    int colStartIndex = 4*(interValNum-1);
    A_p[colStartIndex+1] = A_p[colStartIndex] + 1;
    A_p[colStartIndex+2] = A_p[colStartIndex+1] + 1;
    A_p[colStartIndex+3] = A_p[colStartIndex+2] + 1;
    A_p[colStartIndex+4] = A_p[colStartIndex+3];
    int nzStartIndex = 9 + 12*(interValNum-2);
    A_x[nzStartIndex] = -1.0;
    A_x[nzStartIndex+1] = -1.0;
    A_x[nzStartIndex+2] = -2.0;
    int rowStartIndex = 3*(interValNum-2);
    A_i[nzStartIndex] = rowStartIndex;
    A_i[nzStartIndex+1] = rowStartIndex+1;
    A_i[nzStartIndex+2] = rowStartIndex+2;

    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data    
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x.data(), P_i.data(), P_p.data());
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x.data(), A_i.data(), A_p.data());
    data->l = l.data();
    data->u = u.data();
     
    // Define solver settings as default    
    osqp_set_default_settings(settings);
    settings->verbose = false;
    settings->alpha = 1.0; // Change alpha parameter
    

    // cubic spline in x variable
    data->q = qx.data();
    exitflag = osqp_setup(&work, data, settings);
    osqp_warm_start_x(work, coeffs_x.data());
    osqp_solve(work);    
    mCoeffs_x.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_x[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_x[i].transpose()<<std::endl;
    }

    // cubic spline in y variable
    osqp_update_lin_cost(work, qy.data());
    osqp_warm_start_x(work, coeffs_y.data());
    osqp_solve(work);
    mCoeffs_y.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_y[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }

    // cubic spline in z variable
    osqp_update_lin_cost(work, qz.data());
    osqp_warm_start_x(work, coeffs_z.data());
    osqp_solve(work);
    mCoeffs_z.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_z[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }
    // Cleanup
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);    
    // std::cout<<"successfully run Segment::QpSpline"<<std::endl;    

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Segment::smoothTrjCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<mCoeffs_x.size(); i++){
        double h = 1.0/10;
        for(int j=0; j<10; j++){
            double dt = h*j, dt_2 = dt*dt, dt_3=dt*dt_2;
            double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
            double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
            double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
            trjCloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }
    return trjCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Segment::smoothTrjCloud2()
{
    auto cubicFunCal = [&](const Eigen::Vector4d &fun_params, const double &t){
        double res = fun_params[0] + fun_params[1]*t + fun_params[2]*pow(t, 2) + fun_params[3]*pow(t, 3);
        return res;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<mCoeffs_x.size(); i++){
        double XS = cubicFunCal(mCoeffs_x[i], 0.0);
        double YS = cubicFunCal(mCoeffs_y[i], 0.0);
        double ZS = cubicFunCal(mCoeffs_z[i], 0.0);

        double XE = cubicFunCal(mCoeffs_x[i], 1.0);
        double YE = cubicFunCal(mCoeffs_y[i], 1.0);
        double ZE = cubicFunCal(mCoeffs_z[i], 1.0);

        double len = std::sqrt(pow(XS-XE, 2) + pow(YS - YE, 2) + pow(ZS - ZE, 2));
        int num = int(len);
        double gap = 1.0;
        if(num > 1)
            gap = 1.0 / num;

        if(i == mCoeffs_x.size() - 1)
        {
            for(int j=0; j<= num; j++){
                double dt = gap*j, dt_2 = dt*dt, dt_3=dt*dt_2;
                double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
                double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
                double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
                trjCloudIndexMap[i+j] = i;
            }
        }
        else
        {
            for(int j=0; j< num; j++){
                double dt = gap*j, dt_2 = dt*dt, dt_3=dt*dt_2;
                double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
                double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
                double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
                trjCloudIndexMap[i+j] = i;
            }
        }

    }
    return trjCloud;
}

void Segment::addSegmentCoeff(nlohmann::json &obj, int segmentId)
{
    nlohmann::json singleBoundary;
    singleBoundary["id"] = segmentId;
    for(int i=0; i<mCoeffs_x.size(); i++){
        singleBoundary["coeffs_x"].push_back({mCoeffs_x[i](0), mCoeffs_x[i](1), mCoeffs_x[i](2), mCoeffs_x[i](3)});
        singleBoundary["coeffs_y"].push_back({mCoeffs_y[i](0), mCoeffs_y[i](1), mCoeffs_y[i](2), mCoeffs_y[i](3)});
        singleBoundary["coeffs_z"].push_back({mCoeffs_z[i](0), mCoeffs_z[i](1), mCoeffs_z[i](2), mCoeffs_z[i](3)});
    }
    //color 0-未确认 1-白色 2-黄色 3-橙色 4-蓝色 5-绿色 6-灰色 7-左灰右黄 8-左黄右白 9-左白右黄
    singleBoundary["color"] = 1;
    //marking 0-未确认 1-单实线 2-单虚线 3-短粗虚线 4-双实线 5-双虚线 6-左实右虚 7-左虚右实 8-导流区 9-虚拟车道 10-虚拟路口
    singleBoundary["marking"] = 1;
    //types 0-未调查 1-车道标线 2-防护栏 10-锥桶 11-离散型障碍物
    singleBoundary["types"] = 1;
    //ldm 纵向减速标识 true-是 false-否
    singleBoundary["ldm"] = false;
    obj["laneBoundary"].push_back(singleBoundary);
}

void Segment::addSegmentCoeff_straightLine(nlohmann::json &obj, int segmentId)
{
    nlohmann::json singleBoundary;
    singleBoundary["id"] = segmentId;
    for(int i=1; i<anchorIndices.size(); i++){
        Eigen::Vector3d p0 = cloud->at(anchorIndices[i-1]).getVector3fMap().cast<double>(),
                        p1 = cloud->at(anchorIndices[i]).getVector3fMap().cast<double>();
        singleBoundary["coeffs_x"].push_back(nlohmann::json::array({p0(0), p1(0)-p0(0), 0.f, 0.f}));
        singleBoundary["coeffs_y"].push_back(nlohmann::json::array({p0(1), p1(1)-p0(1), 0.f, 0.f}));
        singleBoundary["coeffs_z"].push_back(nlohmann::json::array({p0(2), p1(2)-p0(2), 0.f, 0.f}));
    }  
    obj["laneBoundary"].push_back(singleBoundary);  
}

void Segment::DecideOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                                const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree)
{
    double totalDisp = 0.f;
    std::vector<int> nearestKeyPoseIndex(anchorIndices.size());
    std::vector<int> indices;
    std::vector<float> squaredDistances;
    for(int i=0; i<anchorIndices.size(); i++){
        indices.clear();
        squaredDistances.clear();
        kdtree->nearestKSearch(cloud->at(anchorIndices[i]), 1, indices, squaredDistances);
        nearestKeyPoseIndex[i] = indices[0];
    }
    int forwardCount = 0, reverseCount = 0;
    for(int i=1; i<nearestKeyPoseIndex.size(); i++){
        if(nearestKeyPoseIndex[i] > nearestKeyPoseIndex[i-1]){
            forwardCount++;
        }else{
            reverseCount++;
        }
    }
    // std::cout<<"forwardCount: "<<forwardCount<<" reverseCount: "<<reverseCount<<std::endl;
    if(reverseCount > forwardCount){
        int nCloud = cloud->size();
        for(int i=0; i<nCloud/2; i++){
            std::swap(cloud->at(i), cloud->at(nCloud-1-i));
        }
        int n = anchorIndices.size();
        std::vector<int> anchorIndices_reverse(anchorIndices.size());
        for(int i=0; i<n; i++) anchorIndices_reverse[i] = nCloud-1-anchorIndices[n-1-i];
        anchorIndices.swap(anchorIndices_reverse);
    }
}

} //namespace Inference

