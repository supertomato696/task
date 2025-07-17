//
//
//

#include "afterProcessCommon.h"
#include "Utils.h"
#include "json.hpp"
#include "proj_api.h"
#include "Geometries/GeometryAlgorithm.h"
#include "./include/CommonUtil.h"
#include "./include/RoadTopoBuild.h"

namespace RoadMapping
{

    void afterProcessCommon::ReadLB(std::string laneBoundaryFromSegDir, Array<LB> &arrLines)
    {
        std::vector<std::string> files;
        Utils::getFiles(laneBoundaryFromSegDir, files, [](std::string str) -> bool
                        { return boost::algorithm::ends_with(str, "_trjCloud.pcd"); });

        for (auto file : files)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::string file_name = laneBoundaryFromSegDir + "/" + file;
            pcl::io::loadPCDFile(file_name, *cloud);
            if (cloud->empty())
                continue;
            LB newLB;
            for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
            {
                pcl::PointXYZ &pcl_p = (*iter);
                Engine::Geometries::Coordinate pntCrood(pcl_p.x, pcl_p.y, pcl_p.z);
                newLB.linePts.Add(pntCrood);
            }
            arrLines.Add(newLB);
        }
    }

    // 读取参考link
    void afterProcessCommon::ReadRfLine(std::string refLinePath, Array<Engine::Geometries::Coordinate> &refLine)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(refLinePath, *cloud);
        if (cloud->empty())
            return;

        for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
        {
            pcl::PointXYZ &pcl_p = (*iter);
            Engine::Geometries::Coordinate pntCrood(pcl_p.x, pcl_p.y, pcl_p.z);
            refLine.Add(pntCrood);
        }

        auto num = refLine.GetCount();
        double dTolerance = 3.0;
        for (int i = 0; i < num - 1;)
        {
            Double distance = refLine[i].Distance(refLine[i + 1]);
            if (distance < dTolerance)
            {
                if (refLine.GetCount() > 2)
                {
                    refLine.Delete(i + 1);
                    num--;
                }
                else
                    break;
            }
            else
            {
                ++i;
            }
        }
    }

    void afterProcessCommon::ReadPolygonPts(std::string dataJson, Engine::Base::Array<Engine::Geometries::Coordinate> &refLine)
    {
        std::ifstream ifs_parseInfo(dataJson);
        if (!ifs_parseInfo.is_open())
        {
            ifs_parseInfo.close();
            return;
        }

        nlohmann::json parseInfo;
        parseInfo << ifs_parseInfo;

        if (!parseInfo.contains("task_type"))
            return;

        if (!parseInfo.contains("task_geom"))
            return;

        // if (parseInfo["task_type"] != "lk2")
        //     return;

        std::string polygon_line = parseInfo["task_geom"];
        int utm_num = parseInfo["utm_num"];

        Utils::GisPolygon new_line;
        boost::geometry::read_wkt(polygon_line, new_line);

        std::vector<double> p_utm_world = parseInfo["t_utm_world"];
        Engine::Geometries::Coordinate t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

        for (const auto &pt : new_line.outer())
        {
            Engine::Geometries::Coordinate newCrood(pt.get<0>(), pt.get<1>(), pt.get<2>());
            Wgs84Toutm(newCrood, utm_num);
            Engine::Geometries::Coordinate xyz_offset = newCrood - t_utm_world;
            refLine.Add(xyz_offset);
        }
    }

    void afterProcessCommon::ReadCenterPts(std::string paraJson, Engine::Base::Array<Engine::Geometries::Coordinate> &centerPts)
    {
        std::ifstream ifs_parseInfo(paraJson);
        if (!ifs_parseInfo.is_open())
        {
            ifs_parseInfo.close();
            return;
        }

        nlohmann::json parseInfo;
        parseInfo << ifs_parseInfo;

        // 只有离散任务才进行相应的处理操作
        if (!parseInfo.contains("task_type"))
            return;

        if (!parseInfo.contains("task_geom"))
            return;

        // if (parseInfo["task_type"] != "lk2")
        //     return;

        if (!parseInfo.contains("sub_crosses_center"))
            return;

        std::string polygon_line = parseInfo["task_geom"];
        int utm_num = parseInfo["utm_num"];

        std::vector<double> p_utm_world = parseInfo["t_utm_world"];
        Engine::Geometries::Coordinate t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

        std::vector<std::vector<double>> crosses_center_pts = parseInfo["sub_crosses_center"];
        for (int i = 0; i < crosses_center_pts.size(); ++i)
        {
            std::vector<double> pt_x_y_z = crosses_center_pts[i];
            Engine::Geometries::Coordinate newCrood(pt_x_y_z[0], pt_x_y_z[1], pt_x_y_z[2]);
            Wgs84Toutm(newCrood, utm_num);
            Engine::Geometries::Coordinate xyz_offset = newCrood - t_utm_world;
            centerPts.Add(xyz_offset);
        }
    }

    void afterProcessCommon::ReadStopLine(std::string stopLineObj, Engine::Base::Array<RM> &arrRMs)
    {
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> resLines;
        ReadObj(stopLineObj, resLines);
        if (!resLines.IsEmpty())
        {
            for (int i = 0; i < resLines.GetCount(); ++i)
            {
                if (resLines[i].GetCount() < 2) // 停止线为2个点
                    continue;
                RM newRm;
                newRm.plygonPnts = resLines[i];
                newRm.ObjectType = 8; // STOP_LINE = 8;//停止线
                arrRMs.Add(newRm);
            }
        }
    }

void afterProcessCommon::ReadCrossWalk(std::string dataDir, Engine::Base::Array<RM> &arrRMs)
    {
        // 处理人行横道
        std::string filepath2 = dataDir + "/" + "crosswalk.obj";
        Array<Array<Coordinate>> resLines2;
        ReadObj(filepath2, resLines2);
        if (!resLines2.IsEmpty())
        {
            for (int i = 0; i < resLines2.GetCount(); ++i)
            {
                int nPts = resLines2[i].GetCount();
                if (nPts < 4)
                    continue;

                // resLines2[i].Delete(nPts - 1);

                RM newRm;
                newRm.plygonPnts = resLines2[i];
                newRm.ObjectType = 10; // CROSS_WALK = 10;//斑马线
                arrRMs.Add(newRm);
            }
        }
    }

    void afterProcessCommon::ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts)
    {
        std::ifstream infile(filepath);
        if (!infile.good())
            return; // 检查文件的存在与否
        std::ifstream ifs(filepath.c_str());
        if (!ifs.is_open())
            return;
        std::vector<Engine::Geometries::Coordinate> vecCoordinates;
        std::string ss;
        while (std::getline(ifs, ss))
        {
            char *sz = const_cast<char *>(ss.data());
            char *str = strtok(sz, " ");
            // vertex
            if (strcmp(str, "v") == 0)
            {
                Engine::Geometries::Coordinate c;
                str = strtok(nullptr, " "); // x
                c.x = atof(str);
                str = strtok(nullptr, " "); // y
                c.y = atof(str);
                str = strtok(nullptr, " "); // z
                c.z = atof(str);
                vecCoordinates.push_back(c);
            }
            else
            {
                str = strtok(nullptr, " "); // first index
                int start_index = atoi(str);
                int end_index = -1;
                while (str != nullptr)
                {
                    end_index = atoi(str);
                    str = strtok(nullptr, " ");
                }
                int size = end_index - start_index + 1;
                Engine::Base::Array<Engine::Geometries::Coordinate> vecLine(size);
                std::copy(vecCoordinates.begin() + start_index - 1, vecCoordinates.begin() + end_index,
                          vecLine.Begin());
                arrArrPts.Add(vecLine);
            }
        }
    }

    void afterProcessCommon::Readtxt(std::string filepath, Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts)
    {
        std::ifstream infile(filepath);
        if (!infile.good())
            return; // 检查文件的存在与否

        std::ifstream ifs(filepath.c_str());
        if (!ifs.is_open())
            return;

        char sz[10240];
        while (ifs.getline(sz, 10240))
        {
            char *str = strtok(sz, " ");
            // vertex
            if (strcmp(str, "v") == 0)
            {
                Engine::Geometries::Coordinate c;
                str = strtok(nullptr, " "); // x
                c.x = atof(str);
                str = strtok(nullptr, " "); // y
                c.y = atof(str);
                str = strtok(nullptr, " "); // z
                c.z = atof(str);
                arrPts.Add(c);
            }
        }
    }

    void afterProcessCommon::ReadRM(std::string dataDir, int label, Engine::Base::Array<RM> &arrRMs)
    {
        std::string labelstr = to_string(label);
        // 提取箭头
        std::string filename = dataDir + "/" + labelstr + "_roadmark.json";

        std::ifstream ifs_RM(filename);
        if (!ifs_RM.is_open())
        {
            ifs_RM.close();
            std::cout << "没有地面标识结果json文件:" << filename << std::endl;
            return;
        }
        nlohmann::json RMJson;
        RMJson << ifs_RM;

        for (const auto &obj : RMJson[labelstr])
        {
            RM newRM;
            auto lf = obj["LeftFront"];
            Engine::Geometries::Coordinate pt_lf = {lf[0], lf[1], lf[2]};
            auto lr = obj["LeftRear"];
            Engine::Geometries::Coordinate pt_lr = {lr[0], lr[1], lr[2]};
            auto rf = obj["RightFront"];
            Engine::Geometries::Coordinate pt_rf = {rf[0], rf[1], rf[2]};
            auto rr = obj["RightRear"];
            Engine::Geometries::Coordinate pt_rr = {rr[0], rr[1], rr[2]};

            newRM.plygonPnts.Add(pt_rf);
            newRM.plygonPnts.Add(pt_lf);
            newRM.plygonPnts.Add(pt_lr);
            newRM.plygonPnts.Add(pt_rr);
            // newRM.plygonPnts.Add(pt_rf);
            newRM.label = label;
            newRM.ObjectType = 7; // 地面标志
            arrRMs.Add(newRM);
        }
    }

    void afterProcessCommon::ReadRM_wq(std::string dataDir, int label, Engine::Base::Array<RM> &arrRMs)
    {
        std::string labelstr = to_string(label);
        // 提取箭头
        std::string filename = dataDir + "/" + labelstr + "_roadmark_dir.json";

        std::ifstream ifs_RM(filename);
        if (!ifs_RM.is_open())
        {
            ifs_RM.close();
            std::cout << "没有地面标识结果json文件:" << filename << std::endl;
            return;
        }
        nlohmann::json RMJson;
        RMJson << ifs_RM;

        for (const auto &obj : RMJson[labelstr])
        {
            RM newRM;
            auto lf = obj["LeftFront"];
            Engine::Geometries::Coordinate pt_lf = {lf[0], lf[1], lf[2]};
            auto lr = obj["LeftRear"];
            Engine::Geometries::Coordinate pt_lr = {lr[0], lr[1], lr[2]};
            auto rf = obj["RightFront"];
            Engine::Geometries::Coordinate pt_rf = {rf[0], rf[1], rf[2]};
            auto rr = obj["RightRear"];
            Engine::Geometries::Coordinate pt_rr = {rr[0], rr[1], rr[2]};
            int arrowtype = obj["ArrowType"];
            newRM.plygonPnts.Add(pt_rf);
            newRM.plygonPnts.Add(pt_lf);
            newRM.plygonPnts.Add(pt_lr);
            newRM.plygonPnts.Add(pt_rr);
            // newRM.plygonPnts.Add(pt_rf);
            newRM.label = label;
            newRM.ArrowType = arrowtype;
            newRM.ObjectType = 7; // 地面标志
            arrRMs.Add(newRM);
        }
    }

    void afterProcessCommon::ReadRMObj(std::string dataDir, Engine::Base::Array<RM> &arrRMs)
    {
        // 处理停止线
        std::string filepath = dataDir + "/" + "stopline.obj";
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> resLines;
        ReadObj(filepath, resLines);
        if (!resLines.IsEmpty())
        {
            for (int i = 0; i < resLines.GetCount(); ++i)
            {
                if (resLines[i].GetCount() < 2) // 停止线为2个点
                    continue;
                RM newRm;
                newRm.plygonPnts = resLines[i];
                newRm.ObjectType = 8; // STOP_LINE = 8;//停止线
                arrRMs.Add(newRm);
            }
        }

        // 处理人行横道
        std::string filepath2 = dataDir + "/" + "crosswalk.obj";
        Array<Array<Coordinate>> resLines2;
        ReadObj(filepath2, resLines2);
        if (!resLines2.IsEmpty())
        {
            for (int i = 0; i < resLines2.GetCount(); ++i)
            {
                int nPts = resLines2[i].GetCount();
                if (nPts < 4) // 多边形，最少为4个点，首尾点相同 只存一个
                    continue;

                resLines2[i].Delete(nPts - 1);

                RM newRm;
                newRm.plygonPnts = resLines2[i];
                newRm.ObjectType = 10; // CROSS_WALK = 10;//斑马线
                arrRMs.Add(newRm);
            }
        }
    }

    void afterProcessCommon::ResampleLBs(Engine::Base::Array<LB> &arrLBs, double tolerance)
    {
        for (int j = 0; j < arrLBs.GetCount(); ++j)
        {
            hdmap_build::CommonUtil::Resample(arrLBs[j].linePts, tolerance);
        }
    }

    void afterProcessCommon::SmoothLBs(Array<LB> &arrLBs, double tolerance)
    {
        for (int j = 0; j < arrLBs.GetCount(); ++j)
        {
            Array<Coordinate> pts;
            hdmap_build::RoadTopoBuild::LineSmooth3(arrLBs[j].linePts, pts);
            arrLBs[j].linePts = pts;
        }
    }

    void afterProcessCommon::ResampleLGs(Array<LG> &arrLGs, double tolerance)
    {
        for (int i = 0; i < arrLGs.GetCount(); ++i)
        {
            ResampleLBs(arrLGs[i].laneGroupLBs, tolerance);
        }
    }

    void afterProcessCommon::ReverseLane(Array<LB> &arrLBs)
    {
        for (int i = 0; i < arrLBs.GetCount(); i++)
            arrLBs[i].linePts.Reverse();
    }

    void afterProcessCommon::ReverseLane2(Array<LB> &arrLBs)
    {
        Array<LB> arrnewLBs;
        for (int i = arrLBs.GetCount() - 1; i >= 0; i--)
        {
            arrLBs[i].linePts.Reverse();
            arrnewLBs.Add(arrLBs[i]);
        }

        arrLBs.Clear();
        arrLBs = arrnewLBs;
    }

    void afterProcessCommon::ClipByPolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, const Engine::Base::Array<Engine::Geometries::Coordinate> &plyCoords)
    {
        if (arrPts.IsEmpty() || plyCoords.IsEmpty())
            return;

        Engine::Geometries::LineString *lineString = hdmap_build::CommonUtil::CoordsToLineString(arrPts);

        Engine::Base::Array<Engine::Geometries::Coordinate *> *newCoords = new Engine::Base::Array<Engine::Geometries::Coordinate *>();
        for (int i = 0; i < plyCoords.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate *newPoint = new Engine::Geometries::Coordinate();
            newPoint->x = plyCoords[i].x;
            newPoint->y = plyCoords[i].y;
            newPoint->z = plyCoords[i].z;
            newCoords->Add(newPoint);
        }

        Engine::Geometries::LinearRing *shellLinearRing = new Engine::Geometries::LinearRing(newCoords);
        Engine::Geometries::Polygon polygon(shellLinearRing);
        Engine::Base::Array<Engine::Geometries::LineString *> vecClipResult;
        Engine::Geometries::GeometryAlgorithm::ClipLineStringByPolygon(*lineString, polygon, vecClipResult);
        if (vecClipResult.IsEmpty())
        {
            arrPts.Clear();
            return;
        }
        arrPts.Clear();
        double maxLength = -1.0;
        for (int i = 0; i < vecClipResult.GetCount(); ++i)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate> linePts;
            hdmap_build::CommonUtil::EngineLinestringToVec3Points(vecClipResult[i], linePts);
            if (linePts.GetCount() > 0)
            {
                double dis = hdmap_build::CommonUtil::GetLength(linePts);
                if (dis > maxLength)
                {
                    dis = maxLength;
                    arrPts = linePts;
                }
            }
        }
    }

    bool afterProcessCommon::IsInPolygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, const Engine::Base::Array<Engine::Geometries::Coordinate> &polyPts)
    {
        Engine::Geometries::Coordinate midpt = hdmap_build::CommonUtil::GetAveragePt(arrPts);
        Engine::Geometries::Polygon geoPolygon;
        line_pts_to_geo_polygon(polyPts, geoPolygon);
        return Geometries::GeometryAlgorithm::PtInPolygon(&geoPolygon, &midpt);
    }

    void afterProcessCommon::line_pts_to_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &plyCoords, Engine::Geometries::Polygon &resPolygon)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate *> *newCoords = new Engine::Base::Array<Engine::Geometries::Coordinate *>();
        for (int i = 0; i < plyCoords.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate *newPoint = new Engine::Geometries::Coordinate();
            newPoint->x = plyCoords[i].x;
            newPoint->y = plyCoords[i].y;
            newPoint->z = plyCoords[i].z;
            newCoords->Add(newPoint);
        }

        Engine::Geometries::LinearRing *shellLinearRing = new Engine::Geometries::LinearRing(newCoords);
        if (!shellLinearRing->IsClosed())
        {
            shellLinearRing->GetCoordinates()->Add(shellLinearRing->GetCoordinateN(0));
        }
        Engine::Geometries::Polygon polygon(shellLinearRing);
        resPolygon = polygon;
    }

    void afterProcessCommon::utmToWgs84(Engine::Geometries::Coordinate &in_pt, int zone)
    {
        Engine::Geometries::Coordinate temp_pt;
        temp_pt.x = in_pt.x;
        temp_pt.y = in_pt.y;
        temp_pt.z = in_pt.z;

        // 转换坐标
        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");

        std::string utm_param = "+proj=utm";
        utm_param = utm_param + " +zone=" + std::to_string(zone) + "N +ellps=WGS84 +no_defs";
        projPJ g_utm = pj_init_plus(utm_param.data());
        pj_transform(g_utm, g_pWGS84, 1, 1,
                     &temp_pt.x, &temp_pt.y, &temp_pt.z);

        in_pt.x = temp_pt.x * RAD2DEG;
        in_pt.y = temp_pt.y * RAD2DEG;
    }

    void afterProcessCommon::Wgs84Toutm(Engine::Geometries::Coordinate &in_pt, int utm_zone)
    {
        Engine::Geometries::Coordinate coord;
        coord.x = in_pt.x * DEG_TO_RAD;
        coord.y = in_pt.y * DEG_TO_RAD;
        coord.z = in_pt.z;

        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
        std::string g_utm_str = "+proj=utm +zone=" + to_string(utm_zone) + "N +ellps=WGS84 +no_defs";
        projPJ g_utm = pj_init_plus(g_utm_str.c_str());
        pj_transform(g_pWGS84, g_utm, 1, 1, &coord.x, &coord.y, &coord.z);
        in_pt = coord;
    }

    void afterProcessCommon::WriteLGToObj(std::string dataDir, std::string keychar, const Engine::Base::Array<LG> &arrLGs)
    {
        for (int i = 0; i < arrLGs.GetCount(); ++i)
        {
            string objFullPath = dataDir + "/" + keychar + "_" + to_string(i) + "_LG.obj";
            ofstream ofs(objFullPath);
            if (!ofs.is_open())
            {
                cout << "obj 保存不成功" << endl;
                continue;
            }

            Array<Array<Engine::Geometries::Coordinate>> lineVec;
            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); j++)
            {
                for (auto k = 0; k < arrLGs[i].laneGroupLBs[j].linePts.GetCount(); k++)
                {
                    Engine::Geometries::Coordinate curCrood = arrLGs[i].laneGroupLBs[j].linePts[k];
                    ofs << std::fixed;
                    ofs << "v " << curCrood.x << " " << curCrood.y << " " << curCrood.z << endl;
                }
                lineVec.Add(arrLGs[i].laneGroupLBs[j].linePts);
            }

            int K = 0;
            for (int j = 0; j < lineVec.GetCount(); j++)
            {
                if (lineVec[j].GetCount() <= 1)
                {
                    continue;
                }
                ofs << "l ";
                for (int k = 0; k < lineVec[j].GetCount(); k++)
                {
                    ofs << ++K << " ";
                }
                ofs << endl;
            }

            ofs.close();
        }
    }

    void afterProcessCommon::WriteLBToObj(std::string dataDir, std::string namechar, const Engine::Base::Array<LB> &arrLBs)
    {

        string objFullPath = dataDir + "/" + "LBs_" + namechar + ".obj";
        ofstream ofs(objFullPath);
        if (!ofs.is_open())
        {
            cout << "obj 保存不成功" << endl;
            return;
        }

        Array<Array<Engine::Geometries::Coordinate>> lineVec;
        for (int j = 0; j < arrLBs.GetCount(); j++)
        {
            for (auto k = 0; k < arrLBs[j].linePts.GetCount(); k++)
            {
                Engine::Geometries::Coordinate curCrood = arrLBs[j].linePts[k];
                ofs << std::fixed;
                ofs << "v " << curCrood.x << " " << curCrood.y << " " << curCrood.z << endl;
            }
            lineVec.Add(arrLBs[j].linePts);
        }

        int K = 0;
        for (int j = 0; j < lineVec.GetCount(); j++)
        {
            if (lineVec[j].GetCount() <= 1)
            {
                continue;
            }
            ofs << "l ";
            for (int k = 0; k < lineVec[j].GetCount(); k++)
            {
                ofs << ++K << " ";
            }
            ofs << endl;
        }

        ofs.close();
    }

    void afterProcessCommon::WriteRMToObj(std::string dataDir, std::string keychar, const Engine::Base::Array<RM> &arrRMs)
    {
        string objFullPath = dataDir + "/" + "RMs_" + keychar + ".obj";
        ofstream ofs(objFullPath);
        if (!ofs.is_open())
        {
            cout << "obj 保存不成功" << endl;
            return;
        }

        Array<Array<Engine::Geometries::Coordinate>> lineVec;
        for (int j = 0; j < arrRMs.GetCount(); j++)
        {
            for (auto k = 0; k < arrRMs[j].plygonPnts.GetCount(); k++)
            {
                Engine::Geometries::Coordinate curCrood = arrRMs[j].plygonPnts[k];
                ofs << std::fixed;
                ofs << "v " << curCrood.x << " " << curCrood.y << " " << curCrood.z << endl;
            }
            lineVec.Add(arrRMs[j].plygonPnts);
        }

        int K = 0;
        for (int j = 0; j < lineVec.GetCount(); j++)
        {
            if (lineVec[j].GetCount() <= 1)
            {
                continue;
            }
            ofs << "l ";
            for (int k = 0; k < lineVec[j].GetCount(); k++)
            {
                ofs << ++K << " ";
            }
            ofs << endl;
        }

        ofs.close();
    }
}
