#pragma once
#include <vector>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "json.h"
#include <iomanip>
#include <osqp.h>

void DBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, int minPts,
            std::vector<std::vector<int>> &clusters)
{
    if (cloud == NULL || cloud->size() == 0)
    {
        std::cout << "input cloud is null at Inference::DBSCAN" << std::endl;
        return;
    }
    // create kdtree for fast neighborhood search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    // neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);

    auto RangeQuery = [&](pcl::PointXYZ searchPoint)
    {
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        kdtree.radiusSearch(searchPoint, radius, neighbors, pointRadiusSquaredDistance);
    };

    std::vector<int> label(cloud->size(), -1); //-1 undefined, 0 noise, 1-n class
    int clusterId = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (label[i] >= 0)
        {
            continue;
        }
        RangeQuery(cloud->at(i)); // contains point i itself
        if (neighbors.size() < minPts)
        {
            label[i] = 0; // label as noise
            continue;
        }
        label[i] = ++clusterId;
        std::vector<bool> inQueue(cloud->size(), false);
        std::queue<int> q;
        for (auto id : neighbors)
        {
            if (id != i && label[id] <= 0)
            {
                q.push(id);
                inQueue[id] = true;
            }
        }
        while (!q.empty())
        {
            int currentId = q.front();
            q.pop();
            // Change Noise to border point
            if (label[currentId] == 0)
                label[currentId] = clusterId;
            // previously processed
            if (label[currentId] >= 0)
                continue;
            label[currentId] = clusterId;
            RangeQuery(cloud->at(currentId));
            if (neighbors.size() >= minPts)
            {
                for (auto id : neighbors)
                {
                    if (label[id] < 0 && !inQueue[id])
                    {
                        q.push(id);
                        inQueue[id] = true;
                    }
                }
            }
        }
    }
    clusters.resize(clusterId);
    int noiseNum = 0;
    for (int i = 0; i < label.size(); i++)
    {
        if (label[i] > 0)
        {
            clusters[label[i] - 1].push_back(i);
        }
        else
        {
            noiseNum += 1;
        }
    }
    std::cout << "finish DBSCAN cluster with noise points number: " << noiseNum << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleByGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution)
{
    std::map<std::pair<int, int>, std::pair<Eigen::Vector3f, int>> grid;
    for (int i = 0; i < cloud->size(); i++)
    {
        Eigen::Vector3f pos = cloud->at(i).getVector3fMap();
        std::pair<int, int> index(pos(0) / resolution, pos(1) / resolution);
        if (grid.count(index))
        {
            grid[index].first += pos;
            grid[index].second += 1;
        }
        else
        {
            grid[index] = std::make_pair(pos, 1);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto iter = grid.begin(); iter != grid.end(); iter++)
    {
        Eigen::Vector3f pos = (iter->second).first / (iter->second).second;
        pcl::PointXYZ pt(pos(0), pos(1), pos(2));
        sampledCloud->push_back(pt);
    }
    return sampledCloud;
}

struct Segment
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // std::vector<double> ts;
    std::vector<int> anchorIndices;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_x; // optimized x coeff
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_y; // optimized y coeff
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_z; // optimized z coeff

    Segment() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
    void DecideOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                           const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree);
    bool QpSpline();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud2();
    void selectAnchor(int stride);
    void addSegmentCoeff(nlohmann::json &obj, int segmentId);
    void addSegmentCoeff_straightLine(nlohmann::json &obj, int segmentId);
};

void ExtractSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Segment> &segments,
                    float radius = 2.f, int minPts = 3)
{
    pcl::PointCloud<pcl::PointXY>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXY>);
    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXY pt_plane(cloud->at(i).x, cloud->at(i).y);
        planeCloud->push_back(pt_plane);
    }

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(planeCloud);

    // neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);

    auto NeighborLineFit = [&](int queryId, Eigen::Vector2f &direction)
    {
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        pcl::PointXY searchPoint = planeCloud->at(queryId);
        kdtree.radiusSearch(searchPoint, radius, neighbors, pointRadiusSquaredDistance);
        // std::cout<<"NeighborLineFit, queryId: "<<queryId<<" neighbors size: "<<neighbors.size()<<std::endl;
        if (neighbors.size() < minPts)
        {
            // std::cout<<"in NeighborLineFit, insufficient neighbor points"<<std::endl;
            return false; // not enough neighborhood points
        }
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        for (auto id : neighbors)
        {
            centroid += Eigen::Vector2f(planeCloud->at(id).x, planeCloud->at(id).y);
        }
        centroid = centroid / neighbors.size();
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (auto id : neighbors)
        {
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            Eigen::Vector2f diff = pos - centroid;
            cov += diff * diff.transpose();
        }
        cov = cov / neighbors.size();
        cov = (cov + cov.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
        es.compute(cov);
        Eigen::Vector2f eigenvalues = es.eigenvalues();
        std::cout << "eigenvalues值：" << eigenvalues(0) << " , " << eigenvalues(1) << std::endl;
        if (eigenvalues(1) < 25 * eigenvalues(0))
        {
            // std::cout<<"in NeighborLineFit, fail PCA check"<<std::endl;
            return false;
        }
        Eigen::Matrix2f eigenvectors = es.eigenvectors();
        direction = eigenvectors.col(1);
        return true;
    };

    std::vector<std::pair<int, float>> disps;
    disps.reserve(100);
    Eigen::Vector2f direction;

    struct Anchor
    {
        int index;                 // index in cloud
        Eigen::Vector2f direction; // direction in xy plane
        Anchor(int index, Eigen::Vector2f direction) : index(index), direction(direction) {}
    };
    std::list<Anchor> anchors; // list of anchors for expansion on both sides
    std::list<int> indices;    // list of cloud point index for expansion on both sides
    std::vector<bool> labeled(cloud->size(), false);

    for (int i = 0; i < cloud->size(); i++)
    {
        if (labeled[i])
            continue;
        anchors.clear();
        indices.clear();
        int anchorIndex = i;
        bool bFitSuccess = NeighborLineFit(anchorIndex, direction);
        for (auto id : neighbors)
        {
            labeled[id] = true; // label as searched
        }
        if (!bFitSuccess)
            continue;
        Eigen::Vector2f anchor(planeCloud->at(anchorIndex).x, planeCloud->at(anchorIndex).y);
        // calculate displacement in neighborhood
        disps.clear();
        for (auto id : neighbors)
        {
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            float disp = (pos - anchor).dot(direction);
            disps.push_back(std::make_pair(id, disp));
        }
        // sort points in direction
        std::sort(disps.begin(), disps.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b)
                  { return a.second < b.second; });
        // set the initial anchor
        // std::cout<<"set the initial anchor: "<<anchorIndex<<std::endl;
        anchors.push_back(Anchor(anchorIndex, direction));
        for (auto ele : disps)
        {
            // std::cout<<"("<<ele.first<<", "<<ele.second<<") ";
            indices.push_back(ele.first);
        }
        // std::cout<<"initial segment indices size: "<<indices.size()<<std::endl;
        bool expandToLeft = (indices.front() != anchorIndex),
             expandToRight = (indices.back() != anchorIndex);
        while (expandToLeft || expandToRight)
        {
            if (expandToLeft)
            {
                Anchor leftMostNode = anchors.front();
                Eigen::Vector2f leftMostDirection = leftMostNode.direction;
                int leftMostNewAnchorIndex = indices.front();
                Eigen::Vector2f leftMostNewAnchor(planeCloud->at(leftMostNewAnchorIndex).x,
                                                  planeCloud->at(leftMostNewAnchorIndex).y);
                if (NeighborLineFit(leftMostNewAnchorIndex, direction))
                {
                    // adjust direction to coincide with leftMostDirection
                    if (direction.dot(leftMostDirection) < 0.f)
                        direction = -direction;
                    disps.clear();
                    for (auto id : neighbors)
                    {
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos - leftMostNewAnchor).dot(direction);
                        // std::cout<<"id: "<<id<<" "<<labeled[id]<<" disp: "<<disp<<std::endl;
                        if (labeled[id] || disp > 0.f)
                            continue;
                        disps.push_back(std::make_pair(id, disp));
                    }
                    // std::cout<<"expand to the left: "<<disps.size()<<std::endl;
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b)
                              { return a.second < b.second; });
                    anchors.push_front(Anchor(leftMostNewAnchorIndex, direction));
                    for (auto iter = disps.rbegin(); iter != disps.rend(); iter++)
                    {
                        indices.push_front(iter->first);
                    }
                    expandToLeft = indices.front() != leftMostNewAnchorIndex;
                }
                else
                {
                    expandToLeft = false;
                }
                for (auto id : neighbors)
                {
                    labeled[id] = true; // label as searched
                }
            }
            if (expandToRight)
            {
                Anchor rightMostNode = anchors.back();
                Eigen::Vector2f rightMostDirection = rightMostNode.direction;
                int rightMostNewAnchorIndex = indices.back();
                Eigen::Vector2f rightMostNewAnchor(planeCloud->at(rightMostNewAnchorIndex).x,
                                                   planeCloud->at(rightMostNewAnchorIndex).y);
                if (NeighborLineFit(rightMostNewAnchorIndex, direction))
                {
                    // adjust direction to coincide with rightMostDirection
                    if (direction.dot(rightMostDirection) < 0.f)
                        direction = -direction;
                    disps.clear();
                    for (auto id : neighbors)
                    {
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos - rightMostNewAnchor).dot(direction);
                        if (labeled[id] || disp < 0.f)
                            continue;
                        disps.push_back(std::make_pair(id, disp));
                    }
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b)
                              { return a.second < b.second; });
                    anchors.push_back(Anchor(rightMostNewAnchorIndex, direction));
                    for (auto ele : disps)
                    {
                        indices.push_back(ele.first);
                    }
                    expandToRight = indices.back() != rightMostNewAnchorIndex;
                }
                else
                {
                    expandToRight = false;
                }
                for (auto id : neighbors)
                {
                    labeled[id] = true; // label as searched
                }
            }
            // std::cout<<"after expand segment nodes size: "<<anchors.size()<<" indices size: "<<indices.size()<<std::endl;
        }
        if (anchors.size() < 5)
            continue; // too short segment
        Segment segment;
        auto iter = anchors.begin();
        int segmentCloudIndex = 0;
        for (auto id : indices)
        {
            segment.cloud->push_back(cloud->at(id));
            if (id == iter->index)
            {
                segment.anchorIndices.push_back(segmentCloudIndex);
                iter++;
            }
            segmentCloudIndex += 1;
        }
        segments.push_back(segment);
    }
}

void csc2Eigen(csc cscM, Eigen::MatrixXd &M)
{
    M = Eigen::MatrixXd::Zero(cscM.m, cscM.n);
    for (int j = 0; j < cscM.n; j++)
    {
        for (int k = cscM.p[j]; k < cscM.p[j + 1]; k++)
        {
            M(cscM.i[k], j) = cscM.x[k];
        }
    }
}

void Segment::selectAnchor(int stride)
{
    std::vector<int> nAnchorIndices;
    for (int i = 0; i < anchorIndices.size(); i += stride)
    {
        nAnchorIndices.push_back(anchorIndices[i]);
    }
    nAnchorIndices.back() = anchorIndices.back();
    anchorIndices.swap(nAnchorIndices);
}

bool Segment::QpSpline()
{
    if (anchorIndices.size() < 3)
    {
        return false;
    }
    int interValNum = anchorIndices.size() - 1;
    c_int n = 4 * interValNum;                // number of opt variables
    std::vector<c_float> coeffs_x(n, 0.f);    // initial coeff for x variable
    std::vector<c_float> coeffs_y(n, 0.f);    // initial coeff for y variable
    std::vector<c_float> coeffs_z(n, 0.f);    // initial coeff for z variable
    c_int P_nnz = 10 * interValNum;           // only save upper part
    std::vector<c_float> P_x(P_nnz);          // non zero element of P
    std::vector<c_int> P_i(P_nnz);            // row index per element of P
    std::vector<c_int> P_p(n + 1);            // accumulated non zero element number of P by column
    std::vector<c_float> qx(n), qy(n), qz(n); // q vector in optimization

    P_p[0] = 0;
    // ts = std::vector<double>(cloud->size(), 0.f);
    // fit cubic function at each interval initially
    for (int i = 0; i < interValNum; i++)
    {
        int index_0 = anchorIndices[i], index_1 = anchorIndices[i + 1];
        Eigen::Vector3d basePos = cloud->at(index_0).getVector3fMap().cast<double>();
        Eigen::Vector3d disp = cloud->at(index_1).getVector3fMap().cast<double>() - basePos;
        double disp_squaredNorm = disp.squaredNorm();
        Eigen::Matrix4d P_interval = Eigen::Matrix4d::Zero();
        Eigen::Vector4d q_interval_x = Eigen::Vector4d::Zero(),
                        q_interval_y = Eigen::Vector4d::Zero(),
                        q_interval_z = Eigen::Vector4d::Zero();
        Eigen::Vector4d tmp;
        for (int k = index_0; k <= index_1; k++)
        {
            Eigen::Vector3d currPos = cloud->at(k).getVector3fMap().cast<double>();
            double dt = (currPos - basePos).dot(disp) / disp_squaredNorm,
                   dt_2 = dt * dt, dt_3 = dt * dt_2;
            // ts[k] = dt;
            tmp << 1.f, dt, dt_2, dt_3;
            P_interval += tmp * tmp.transpose();
            q_interval_x -= tmp * currPos(0);
            q_interval_y -= tmp * currPos(1);
            q_interval_z -= tmp * currPos(2);
        }
        // P_interval += 1e-8*Eigen::Matrix4d::Identity();
        P_interval.block<2, 2>(2, 2) += Eigen::Matrix2d::Identity();
        Eigen::Vector4d coeff_x = -P_interval.inverse() * q_interval_x,
                        coeff_y = -P_interval.inverse() * q_interval_y,
                        coeff_z = -P_interval.inverse() * q_interval_z;

        int coeffsIndex = 4 * i, PIndex = 10 * i;
        for (int j = 0; j < 4; j++)
        {
            coeffs_x[coeffsIndex + j] = coeff_x(j);
            coeffs_y[coeffsIndex + j] = coeff_y(j);
            coeffs_z[coeffsIndex + j] = coeff_z(j);
            qx[coeffsIndex + j] = q_interval_x(j);
            qy[coeffsIndex + j] = q_interval_y(j);
            qz[coeffsIndex + j] = q_interval_z(j);
            P_p[coeffsIndex + j + 1] = P_p[coeffsIndex + j] + j + 1;
            PIndex += j;
            for (int k = 0; k <= j; k++)
            {
                P_x[PIndex + k] = P_interval(k, j);
                P_i[PIndex + k] = k + coeffsIndex;
            }
        }
    }
    c_int m = 3 * (interValNum - 1); // number of constraint
    std::vector<c_float> l(m, -1e-6);
    std::vector<c_float> u(m, 1e-6);
    c_int A_nnz = 12 * (interValNum - 1); // number of nonzero element in A
    std::vector<c_float> A_x(A_nnz);      // all non zero elements in A
    std::vector<c_int> A_i(A_nnz);        // row index per element in A
    std::vector<c_int> A_p(n + 1);        // accumulated non zero element number of A by column
    A_p[0] = 0;
    // assign the first block
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
    for (int i = 1; i < interValNum - 1; i++)
    {
        int colStartIndex = 4 * i;
        A_p[colStartIndex + 1] = A_p[colStartIndex] + 2;
        A_p[colStartIndex + 2] = A_p[colStartIndex + 1] + 3;
        A_p[colStartIndex + 3] = A_p[colStartIndex + 2] + 4;
        A_p[colStartIndex + 4] = A_p[colStartIndex + 3] + 3;

        int nzStartIndex = 9 + 12 * (i - 1);
        int rowStartIndex = 3 * (i - 1);

        A_x[nzStartIndex] = -1.0;
        A_x[nzStartIndex + 1] = 1.0;
        A_x[nzStartIndex + 2] = -1.0;
        A_x[nzStartIndex + 3] = 1.0;
        A_x[nzStartIndex + 4] = 1.0;
        A_x[nzStartIndex + 5] = -2.0;
        A_x[nzStartIndex + 6] = 1.0;
        A_x[nzStartIndex + 7] = 2.0;
        A_x[nzStartIndex + 8] = 2.0;
        A_x[nzStartIndex + 9] = 1.0;
        A_x[nzStartIndex + 10] = 3.0;
        A_x[nzStartIndex + 11] = 6.0;

        A_i[nzStartIndex] = rowStartIndex;
        A_i[nzStartIndex + 1] = rowStartIndex + 3;
        A_i[nzStartIndex + 2] = rowStartIndex + 1;
        A_i[nzStartIndex + 3] = rowStartIndex + 3;
        A_i[nzStartIndex + 4] = rowStartIndex + 4;
        A_i[nzStartIndex + 5] = rowStartIndex + 2;
        A_i[nzStartIndex + 6] = rowStartIndex + 3;
        A_i[nzStartIndex + 7] = rowStartIndex + 4;
        A_i[nzStartIndex + 8] = rowStartIndex + 5;
        A_i[nzStartIndex + 9] = rowStartIndex + 3;
        A_i[nzStartIndex + 10] = rowStartIndex + 4;
        A_i[nzStartIndex + 11] = rowStartIndex + 5;
    }
    // assign the last block
    int colStartIndex = 4 * (interValNum - 1);
    A_p[colStartIndex + 1] = A_p[colStartIndex] + 1;
    A_p[colStartIndex + 2] = A_p[colStartIndex + 1] + 1;
    A_p[colStartIndex + 3] = A_p[colStartIndex + 2] + 1;
    A_p[colStartIndex + 4] = A_p[colStartIndex + 3];
    int nzStartIndex = 9 + 12 * (interValNum - 2);
    A_x[nzStartIndex] = -1.0;
    A_x[nzStartIndex + 1] = -1.0;
    A_x[nzStartIndex + 2] = -2.0;
    int rowStartIndex = 3 * (interValNum - 2);
    A_i[nzStartIndex] = rowStartIndex;
    A_i[nzStartIndex + 1] = rowStartIndex + 1;
    A_i[nzStartIndex + 2] = rowStartIndex + 2;

    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x.data(), P_i.data(), P_p.data());
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x.data(), A_i.data(), A_p.data());
    data->l = l.data();
    data->u = u.data();

    // Define solver settings as default
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter

    // cubic spline in x variable
    data->q = qx.data();
    exitflag = osqp_setup(&work, data, settings);
    osqp_warm_start_x(work, coeffs_x.data());
    osqp_solve(work);
    mCoeffs_x.resize(interValNum);
    for (int i = 0; i < interValNum; i++)
    {
        int index = 4 * i;
        mCoeffs_x[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index + 1], work->solution->x[index + 2], work->solution->x[index + 3]);
        // std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_x[i].transpose()<<std::endl;
    }

    // cubic spline in y variable
    osqp_update_lin_cost(work, qy.data());
    osqp_warm_start_x(work, coeffs_y.data());
    osqp_solve(work);
    mCoeffs_y.resize(interValNum);
    for (int i = 0; i < interValNum; i++)
    {
        int index = 4 * i;
        mCoeffs_y[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index + 1], work->solution->x[index + 2], work->solution->x[index + 3]);
        // std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }

    // cubic spline in z variable
    osqp_update_lin_cost(work, qz.data());
    osqp_warm_start_x(work, coeffs_z.data());
    osqp_solve(work);
    mCoeffs_z.resize(interValNum);
    for (int i = 0; i < interValNum; i++)
    {
        int index = 4 * i;
        mCoeffs_z[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index + 1], work->solution->x[index + 2], work->solution->x[index + 3]);
        // std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }
    // Cleanup
    osqp_cleanup(work);
    if (data)
    {
        if (data->A)
            c_free(data->A);
        if (data->P)
            c_free(data->P);
        c_free(data);
    }
    if (settings)
        c_free(settings);
    std::cout << "successfully run Segment::QpSpline" << std::endl;

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Segment::smoothTrjCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < mCoeffs_x.size(); i++)
    {
        double h = 1.0 / 10;
        for (int j = 0; j < 10; j++)
        {
            double dt = h * j, dt_2 = dt * dt, dt_3 = dt * dt_2;
            double x = mCoeffs_x[i](0) + mCoeffs_x[i](1) * dt + mCoeffs_x[i](2) * dt_2 + mCoeffs_x[i](3) * dt_3;
            double y = mCoeffs_y[i](0) + mCoeffs_y[i](1) * dt + mCoeffs_y[i](2) * dt_2 + mCoeffs_y[i](3) * dt_3;
            double z = mCoeffs_z[i](0) + mCoeffs_z[i](1) * dt + mCoeffs_z[i](2) * dt_2 + mCoeffs_z[i](3) * dt_3;
            trjCloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }
    return trjCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Segment::smoothTrjCloud2()
{
    auto cubicFunCal = [&](const Eigen::Vector4d &fun_params, const double &t)
    {
        double res = fun_params[0] + fun_params[1] * t + fun_params[2] * pow(t, 2) + fun_params[3] * pow(t, 3);
        return res;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < mCoeffs_x.size(); i++)
    {
        double XS = cubicFunCal(mCoeffs_x[i], 0.0);
        double YS = cubicFunCal(mCoeffs_y[i], 0.0);
        double ZS = cubicFunCal(mCoeffs_z[i], 0.0);

        double XE = cubicFunCal(mCoeffs_x[i], 1.0);
        double YE = cubicFunCal(mCoeffs_y[i], 1.0);
        double ZE = cubicFunCal(mCoeffs_z[i], 1.0);

        double len = std::sqrt(pow(XS - XE, 2) + pow(YS - YE, 2) + pow(ZS - ZE, 2));
        int num = int(len);
        double gap = 1.0;
        if (num > 1)
            gap = 1.0 / num;

        if (i == mCoeffs_x.size() - 1)
        {
            for (int j = 0; j <= num; j++)
            {
                double dt = gap * j, dt_2 = dt * dt, dt_3 = dt * dt_2;
                double x = mCoeffs_x[i](0) + mCoeffs_x[i](1) * dt + mCoeffs_x[i](2) * dt_2 + mCoeffs_x[i](3) * dt_3;
                double y = mCoeffs_y[i](0) + mCoeffs_y[i](1) * dt + mCoeffs_y[i](2) * dt_2 + mCoeffs_y[i](3) * dt_3;
                double z = mCoeffs_z[i](0) + mCoeffs_z[i](1) * dt + mCoeffs_z[i](2) * dt_2 + mCoeffs_z[i](3) * dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }
        else
        {
            for (int j = 0; j < num; j++)
            {
                double dt = gap * j, dt_2 = dt * dt, dt_3 = dt * dt_2;
                double x = mCoeffs_x[i](0) + mCoeffs_x[i](1) * dt + mCoeffs_x[i](2) * dt_2 + mCoeffs_x[i](3) * dt_3;
                double y = mCoeffs_y[i](0) + mCoeffs_y[i](1) * dt + mCoeffs_y[i](2) * dt_2 + mCoeffs_y[i](3) * dt_3;
                double z = mCoeffs_z[i](0) + mCoeffs_z[i](1) * dt + mCoeffs_z[i](2) * dt_2 + mCoeffs_z[i](3) * dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }
    }
    return trjCloud;
}

void Segment::addSegmentCoeff(nlohmann::json &obj, int segmentId)
{
    nlohmann::json singleBoundary;
    singleBoundary["id"] = segmentId;
    for (int i = 0; i < mCoeffs_x.size(); i++)
    {
        singleBoundary["coeffs_x"].push_back({mCoeffs_x[i](0), mCoeffs_x[i](1), mCoeffs_x[i](2), mCoeffs_x[i](3)});
        singleBoundary["coeffs_y"].push_back({mCoeffs_y[i](0), mCoeffs_y[i](1), mCoeffs_y[i](2), mCoeffs_y[i](3)});
        singleBoundary["coeffs_z"].push_back({mCoeffs_z[i](0), mCoeffs_z[i](1), mCoeffs_z[i](2), mCoeffs_z[i](3)});
    }
    // color 0-未确认 1-白色 2-黄色 3-橙色 4-蓝色 5-绿色 6-灰色 7-左灰右黄 8-左黄右白 9-左白右黄
    singleBoundary["color"] = 1;
    // marking 0-未确认 1-单实线 2-单虚线 3-短粗虚线 4-双实线 5-双虚线 6-左实右虚 7-左虚右实 8-导流区 9-虚拟车道 10-虚拟路口
    singleBoundary["marking"] = 1;
    // types 0-未调查 1-车道标线 2-防护栏 10-锥桶 11-离散型障碍物
    singleBoundary["types"] = 1;
    // ldm 纵向减速标识 true-是 false-否
    singleBoundary["ldm"] = false;
    obj["laneBoundary"].push_back(singleBoundary);
}

void Segment::addSegmentCoeff_straightLine(nlohmann::json &obj, int segmentId)
{
    nlohmann::json singleBoundary;
    singleBoundary["id"] = segmentId;
    for (int i = 1; i < anchorIndices.size(); i++)
    {
        Eigen::Vector3d p0 = cloud->at(anchorIndices[i - 1]).getVector3fMap().cast<double>(),
                        p1 = cloud->at(anchorIndices[i]).getVector3fMap().cast<double>();
        singleBoundary["coeffs_x"].push_back(nlohmann::json::array({p0(0), p1(0) - p0(0), 0.f, 0.f}));
        singleBoundary["coeffs_y"].push_back(nlohmann::json::array({p0(1), p1(1) - p0(1), 0.f, 0.f}));
        singleBoundary["coeffs_z"].push_back(nlohmann::json::array({p0(2), p1(2) - p0(2), 0.f, 0.f}));
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
    for (int i = 0; i < anchorIndices.size(); i++)
    {
        indices.clear();
        squaredDistances.clear();
        kdtree->nearestKSearch(cloud->at(anchorIndices[i]), 1, indices, squaredDistances);
        nearestKeyPoseIndex[i] = indices[0];
    }
    int forwardCount = 0, reverseCount = 0;
    for (int i = 1; i < nearestKeyPoseIndex.size(); i++)
    {
        if (nearestKeyPoseIndex[i] > nearestKeyPoseIndex[i - 1])
        {
            forwardCount++;
        }
        else
        {
            reverseCount++;
        }
    }
    std::cout << "forwardCount: " << forwardCount << " reverseCount: " << reverseCount << std::endl;
    if (reverseCount > forwardCount)
    {
        int nCloud = cloud->size();
        for (int i = 0; i < nCloud / 2; i++)
        {
            std::swap(cloud->at(i), cloud->at(nCloud - 1 - i));
        }
        int n = anchorIndices.size();
        std::vector<int> anchorIndices_reverse(anchorIndices.size());
        for (int i = 0; i < n; i++)
            anchorIndices_reverse[i] = nCloud - 1 - anchorIndices[n - 1 - i];
        anchorIndices.swap(anchorIndices_reverse);
    }
}

void BFS(const cv::Mat &img, std::vector<std::vector<std::pair<int, int>>> &clusters, int typeId)
{
    std::vector<std::vector<bool>> visited(img.rows, std::vector<bool>(img.cols, false));
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            if (img.at<uchar>(i, j) != typeId || visited[i][j])
            {
                continue;
            }
            std::queue<std::pair<int, int>> q;
            q.push(std::make_pair(i, j));
            visited[i][j] = true;
            std::vector<std::pair<int, int>> cluster;
            while (!q.empty())
            {
                std::pair<int, int> pos = q.front();
                q.pop();
                cluster.push_back(pos);
                int rowIndex = pos.first, colIndex = pos.second;
                if (rowIndex > 0 && img.at<uchar>(rowIndex - 1, colIndex) == typeId &&
                    visited[rowIndex - 1][colIndex] == false)
                {
                    visited[rowIndex - 1][colIndex] = true;
                    q.push(std::make_pair(rowIndex - 1, colIndex));
                }
                if (rowIndex + 1 < img.rows && img.at<uchar>(rowIndex + 1, colIndex) == typeId &&
                    visited[rowIndex + 1][colIndex] == false)
                {
                    visited[rowIndex + 1][colIndex] = true;
                    q.push(std::make_pair(rowIndex + 1, colIndex));
                }
                if (colIndex > 0 && img.at<uchar>(rowIndex, colIndex - 1) == typeId &&
                    visited[rowIndex][colIndex - 1] == false)
                {
                    visited[rowIndex][colIndex - 1] = true;
                    q.push(std::make_pair(rowIndex, colIndex - 1));
                }
                if (colIndex + 1 < img.cols && img.at<uchar>(rowIndex, colIndex + 1) == typeId &&
                    visited[rowIndex][colIndex + 1] == false)
                {
                    visited[rowIndex][colIndex + 1] = true;
                    q.push(std::make_pair(rowIndex, colIndex + 1));
                }
            }
            clusters.push_back(cluster);
        }
    }
}