#include "afterProcess.h"
#include "json.hpp"
#include "Utils.h"
#include "pclPtType.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"
#include "processCrosswalk.h"
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/conditional_removal.h>

#include <boost/filesystem.hpp>
#include "./include/CommonUtil.h"
#include "./include/DataManager.h"
#include "./include/RoadLayer.h"
#include "./include/RoadMark.h"
#include "./include/RoadTopoBuild.h"
#include "./include/LinearObjDuplicateRemoval.h"
#include "../hdmap_server/data-access-engine/data_access_engine.h"

namespace RoadMapping {

    void afterProcess::ClipLinesByLink(Array<LB> &arrLines, Array<Engine::Geometries::Coordinate> &refLine) {
        if (refLine.IsEmpty())
            return;

        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }

        int nLB = arrLines.GetCount() - 1;
        for (int i = nLB; i >= 0; i--) {
            int nPts = arrLines[i].linePts.GetCount() - 1;
            for (int j = nPts; j >= 0; j--) {
                Engine::Geometries::Coordinate pntProject;
                Engine::Base::Int32 nSegIndex = -1;
                bool bFindInLine = true;
                BaseAlgorithm3D::GetDistanceXYPointToLinesegments(arrLines[i].linePts[j], coordinates, pntProject,
                                                                  nSegIndex, bFindInLine);
                if (!bFindInLine) //垂点不在折线上，删除
                    arrLines[i].linePts.Delete(j);
            }
            if (arrLines[i].linePts.GetCount() < 2)
                arrLines.Delete(i);
        }
    }

    void afterProcess::GetLaneGroupByLinkPts(Array<LB> &arrLines, Array<Engine::Geometries::Coordinate> &refLine,
                                             Array<LG> &arrLGs) {
        if (refLine.IsEmpty() || arrLines.IsEmpty())
            return;

        int nLGs = refLine.GetCount();
//        if (nLGs < 3) //参考点小于三个没有切割的必要性
//        {
//            LG newLG;
//            for (int i = 0; i < arrLines.GetCount(); ++i) {
//                newLG.laneGroupLBs.Add(arrLines[i]);
//            }
//            arrLGs.Add(newLG);
//            return;
//        }

        arrLGs.Clear();
        arrLGs.SetSize(nLGs - 1);

        //用参考线切割生成新的laneGroup
        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }

        for (int i = 0; i < arrLines.GetCount(); ++i) { // 车道边界线
            std::map<int, LB> mapLines;
            Engine::Geometries::Coordinate pntProject;
            Engine::Base::Int32 nOriIndex = 0;
            Engine::Base::Int32 nOriPtIndex = 0;
            bool bFindInLine = true;
            int ncut = 0;
            for (int j = 0; j < arrLines[i].linePts.GetCount(); ++j) { // 车道边界线上的点
                Engine::Base::Int32 nSegIndex;
                BaseAlgorithm3D::GetDistanceXYPointToLinesegments(arrLines[i].linePts[j], coordinates, pntProject,
                                                                  nSegIndex, bFindInLine);
                if (nSegIndex != nOriIndex) //投影在link对应区间发生变化
                {
                    if (j - nOriPtIndex > 1) {
                        LB newLB;
                        newLB.isCutByRefE = true;
                        if (ncut < 1)
                            newLB.isCutByRefS = false; //从未被切分
                        else
                            newLB.isCutByRefS = true;//已被切分
                        for (int k = nOriPtIndex; k <= j; ++k) {
                            newLB.linePts.Add(arrLines[i].linePts[k]);
                        }
                        mapLines[nOriIndex] = newLB;
                    }
                    nOriIndex = nSegIndex;
                    nOriPtIndex = j;
                    if (j != 0)
                        ncut++;
                }
            }

            //判断最后一段的情况
            if (arrLines[i].linePts.GetCount() - 1 - nOriPtIndex > 1) {
                LB newLB;
                newLB.isCutByRefE = false;
                if (ncut < 1)
                    newLB.isCutByRefS = false; //从未被切分
                else
                    newLB.isCutByRefS = true;//已被切分
                for (int k = nOriPtIndex; k < arrLines[i].linePts.GetCount(); ++k) {
                    newLB.linePts.Add(arrLines[i].linePts[k]);
                }
                mapLines[nOriIndex] = newLB;
            }

            for (auto &lb: mapLines) {
                arrLGs[lb.first].laneGroupLBs.Add(lb.second);
                arrLGs[lb.first].refIndex = lb.first;
            }
        }

        //删除空的车道组
        int n = arrLGs.GetCount();
        for (int i = n - 1; i >= 0; --i) {
            if (arrLGs[i].laneGroupLBs.GetCount() < 1)
                arrLGs.Delete(i);
        }
    }

    void afterProcess::CalLaneGroupSeq(Array<Engine::Geometries::Coordinate> &refLine, Array<LG> &arrLGs) {
        if (refLine.IsEmpty() || arrLGs.IsEmpty())
            return;

//        for (int i = 0; i < arrLGs.GetCount(); ++i) {
//            //计算投影距离
//            Engine::Geometries::Coordinate pntFrom = refLine[arrLGs[i].refIndex];
//            pntFrom.z = 0.0;
//            Engine::Geometries::Coordinate pntTo = refLine[arrLGs[i].refIndex + 1];
//            pntTo.z = 0.0;
//            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); ++j) {
//                Engine::Geometries::Coordinate pt0 = arrLGs[i].laneGroupLBs[j].linePts[0];
//                pt0.z = 0.0;
//                double dis = BaseAlgorithm3D::DisPtToLine(pntFrom, pntTo, pt0);
//                int leftRight = BaseAlgorithm::PntMatchLine(pntFrom, pntTo, pt0);
//                if (leftRight < 2)
//                    dis = -1.0 * dis;
//                arrLGs[i].laneGroupLBs[j].disToRef = dis;
//            }
//            //将一个车道组内的线按照从左到右进行排序
//            std::sort(arrLGs[i].laneGroupLBs.Begin(), arrLGs[i].laneGroupLBs.End(), [&](const LB &d1, const LB &d2) {
//                return d1.disToRef < d2.disToRef;
//            });
//        }

        //背景：ori采用端点计算到参考线的距离，在道路车道数变化时此场景有失效的情况
        //修改：用最不突出的线的端点向别的线做垂线，然后用垂点判断
        for (int i = 0; i < arrLGs.GetCount(); ++i) {
            //计算比较的垂点
            int hideIndex = 0;
            GetHideLane(arrLGs[i].laneGroupLBs, hideIndex);
            if (hideIndex < 0)
                hideIndex = 0;

            Engine::Geometries::Coordinate oriPrj = arrLGs[i].laneGroupLBs[hideIndex].linePts[0];
            Array<Engine::Geometries::Coordinate> comparePts;
            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); ++j) {
                if (hideIndex == j)
                    comparePts.Add(oriPrj);
                else {
                    Engine::Geometries::Coordinate pntProject;
                    int nSegIndex = 0;
                    BaseAlgorithm3D::GetDistancePointToLinesegments(oriPrj, arrLGs[i].laneGroupLBs[j].linePts,
                                                                    pntProject, nSegIndex);
                    comparePts.Add(pntProject);
                }
            }

            //计算投影距离
            Engine::Geometries::Coordinate pntFrom = refLine[arrLGs[i].refIndex];
            pntFrom.z = 0.0;
            Engine::Geometries::Coordinate pntTo = refLine[arrLGs[i].refIndex + 1];
            pntTo.z = 0.0;

            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); ++j) {
                Engine::Geometries::Coordinate pt0 = comparePts[j];
                pt0.z = 0.0;
                double dis = BaseAlgorithm3D::DisPtToLine(pntFrom, pntTo, pt0);
                int leftRight = BaseAlgorithm::PntMatchLine(pntFrom, pntTo, pt0);
                if (leftRight < 2)
                    dis = -1.0 * dis;
                arrLGs[i].laneGroupLBs[j].disToRef = dis;
            }

            //将一个车道组内的线按照从左到右进行排序
            std::sort(arrLGs[i].laneGroupLBs.Begin(), arrLGs[i].laneGroupLBs.End(), [&](const LB &d1, const LB &d2) {
                return d1.disToRef < d2.disToRef;
            });
        }
    }

    void afterProcess::DeleteShortLine(LG &lg) {
        if (lg.laneGroupLBs.IsEmpty())
            return;
        //计算lg内线的长度
        double maxLen = -1.0;
        for (int i = 0; i < lg.laneGroupLBs.GetCount(); ++i) {
            lg.laneGroupLBs[i].len = hdmap_build::CommonUtil::GetLength(lg.laneGroupLBs[i].linePts);
            if (lg.laneGroupLBs[i].len > maxLen)
                maxLen = lg.laneGroupLBs[i].len;
        }
        //对lg内线按照长度进行排序(由长到短)
//        std::sort(lg.laneGroupLBs.Begin(), lg.laneGroupLBs.End(), [&](const LB &d1, const LB &d2) {
//            return d1.len > d2.len;
//        });

        //删除长度小于0.3倍最长距离的线段
        int nlines = lg.laneGroupLBs.GetCount();
        double compareDis = maxLen * 0.3;
        for (int i = nlines - 1; i >= 0; i--) {
            if (lg.laneGroupLBs[i].len < compareDis)
                lg.laneGroupLBs.Delete(i);
        }
    }

    void afterProcess::CombineLGs(Array<LG> &arrLGs) {
        if (arrLGs.IsEmpty())
            return;
        int nLGs = arrLGs.GetCount();
        for (int i = nLGs - 1; i > 0; i--) {
            int k = i - 1;
            int nLBs = arrLGs[i].laneGroupLBs.GetCount();
            if (nLBs != arrLGs[k].laneGroupLBs.GetCount())
                continue; //车道线不一致不能合并
            //车道线一致的情况下，比较首尾点一致情况
            bool isCombine = true;
            for (int j = 0; j < nLBs; ++j) {
                int prePtsCount = arrLGs[k].laneGroupLBs[j].linePts.GetCount();
                Engine::Geometries::Coordinate endPt = arrLGs[k].laneGroupLBs[j].linePts.GetAt(prePtsCount - 1);
                Engine::Geometries::Coordinate startPt = arrLGs[i].laneGroupLBs[j].linePts.GetAt(0);
                if (endPt.Distance(startPt) > 1.0) //不能合并
                {
                    isCombine = false;
                    break;
                }
            }

            if (isCombine) //合并操作
            {
                for (int j = 0; j < nLBs; ++j) {
                    for (int nn = 1; nn < arrLGs[i].laneGroupLBs[j].linePts.GetCount(); nn++)
                        arrLGs[k].laneGroupLBs[j].linePts.Add(arrLGs[i].laneGroupLBs[j].linePts[nn]);
                }
                arrLGs.Delete(i);
            }
        }
    }

    bool afterProcess::AdjustCutOneLG(LG &lg, Array<int> noCutArr, LG &newlg) {
        //主要处理车道汇入汇出延长导致的错误
        int n = lg.laneGroupLBs.GetCount();
        if (n < 3 || noCutArr.IsEmpty())
            return false;

        int needCut = -1;
        int cutIndex = -1;
        for (int i = 0; i < noCutArr.GetCount(); ++i) {
            int dealindex = noCutArr[i];
            LB deallb = lg.laneGroupLBs[dealindex];
            if (dealindex == 0 || dealindex == n - 1)
                continue; //最左侧和最右侧的数据无需判断

            //判断和右侧线的关系 (1.根据距离判断 2.根据延长后的点是否在正确的位置上)
            LB r_lb = lg.laneGroupLBs[dealindex + 1];
            double dis = r_lb.linePts[0].DistanceXY(deallb.linePts[0]);
            int leftRight = BaseAlgorithm::PntMatchLine(r_lb.linePts[0], r_lb.linePts[1], deallb.linePts[0]);
            if (dis < 1.0 || leftRight == 2) //距离小于正常车道宽度 或者说 没有在右侧线的左边
            {
                for (int j = deallb.linePts.GetCount() - 1; j >= 0; j--) {
                    Engine::Geometries::Coordinate pntProject;
                    int nSegIndex = 0;
                    BaseAlgorithm3D::GetDistancePointToLinesegments(deallb.linePts[j], r_lb.linePts, pntProject,
                                                                    nSegIndex);
                    double disPrj = deallb.linePts[j].DistanceXY(pntProject);
                    if (disPrj <= 1.0) {
                        cutIndex = j;
                        needCut = dealindex;
                        break;
                    }
                }
            }

            if (needCut > 0)
                break;

            //判断和左侧线的关系
            LB l_lb = lg.laneGroupLBs[dealindex - 1];
            dis = l_lb.linePts[0].DistanceXY(deallb.linePts[0]);
            leftRight = BaseAlgorithm::PntMatchLine(l_lb.linePts[0], l_lb.linePts[1], deallb.linePts[0]);
            if (dis < 1.0 || leftRight < 2) //距离小于正常车道宽度 或者说 没有在左侧线的右边
            {
                for (int j = deallb.linePts.GetCount() - 1; j >= 0; j--) {
                    Engine::Geometries::Coordinate pntProject;
                    int nSegIndex = 0;
                    BaseAlgorithm3D::GetDistancePointToLinesegments(deallb.linePts[j], l_lb.linePts, pntProject,
                                                                    nSegIndex);
                    double disPrj = deallb.linePts[j].DistanceXY(pntProject);
                    if (disPrj <= 1.0) {
                        cutIndex = j;
                        needCut = dealindex;
                        break;
                    }
                }
            }

            if (needCut > 0)
                break;
        }

        if (needCut < 0 || cutIndex < 0)
            return false;

        //进行切割操作,分为两段，从起点开始
        Coordinate cutPt = lg.laneGroupLBs[needCut].linePts[cutIndex];
        LG firstlg;
        LG secondlg;
        for (int i = 0; i < n; ++i) {
            if (i != needCut) {
                Engine::Geometries::Coordinate pntProject;
                int nSegIndex = 0;
                BaseAlgorithm3D::GetDistancePointToLinesegments(cutPt, lg.laneGroupLBs[i].linePts, pntProject,
                                                                nSegIndex);
                LB lbS;
                for (int j = 0; j <= nSegIndex; ++j)
                    lbS.linePts.Add(lg.laneGroupLBs[i].linePts[j]);

                if (lbS.linePts.GetCount() > 1)
                    firstlg.laneGroupLBs.Add(lbS);

                LB lbE;
                for (int j = nSegIndex; j < lg.laneGroupLBs[i].linePts.GetCount(); ++j)
                    lbE.linePts.Add(lg.laneGroupLBs[i].linePts[j]);

                if (lbE.linePts.GetCount() > 1) {
                    lbE.isCutByRefE = lg.laneGroupLBs[i].isCutByRefE;
                    secondlg.laneGroupLBs.Add(lbE);
                }

            } else {
                LB lbE;
                for (int j = cutIndex; j < lg.laneGroupLBs[i].linePts.GetCount(); ++j)
                    lbE.linePts.Add(lg.laneGroupLBs[i].linePts[j]);

                if (lbE.linePts.GetCount() > 1) {
                    lbE.isCutByRefE = lg.laneGroupLBs[i].isCutByRefE;
                    secondlg.laneGroupLBs.Add(lbE);
                }
            }
        }

        if (firstlg.laneGroupLBs.GetCount() > 1 && secondlg.laneGroupLBs.GetCount() > 1) {
            lg = secondlg;
            newlg = firstlg;
            return true;
        }

        return false;
    }

    void afterProcess::AdjustLGs(std::string dataDir, Array<LG> &arrLGs) {
        //首先进行连接操作
        //计算连接
        for (int i = 0; i < arrLGs.GetCount(); ++i) {
            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount() - 1; ++j) {
                //判断是否进行连接
                bool isFirst = IsConnect(arrLGs[i].laneGroupLBs[j].linePts, arrLGs[i].laneGroupLBs[j + 1].linePts);
                bool isSecond = IsConnect(arrLGs[i].laneGroupLBs[j + 1].linePts, arrLGs[i].laneGroupLBs[j].linePts);
                if (!isFirst && !isSecond) //j 和 j+1 不能连接
                    continue;
                //进行连接操作
                if (isFirst) //把j+1的点添加到j中
                {
                    for (int k = 0; k < arrLGs[i].laneGroupLBs[j + 1].linePts.GetCount(); k++) {
                        arrLGs[i].laneGroupLBs[j].linePts.Add(arrLGs[i].laneGroupLBs[j + 1].linePts[k]);
                    }
                    arrLGs[i].laneGroupLBs[j].isCutByRefE = arrLGs[i].laneGroupLBs[j + 1].isCutByRefE;
                    arrLGs[i].laneGroupLBs.Delete(j + 1);
                    j--;
                } else {
                    for (int k = 0; k < arrLGs[i].laneGroupLBs[j].linePts.GetCount(); k++) {
                        arrLGs[i].laneGroupLBs[j + 1].linePts.Add(arrLGs[i].laneGroupLBs[j].linePts[k]);
                    }
                    arrLGs[i].laneGroupLBs[j + 1].isCutByRefE = arrLGs[i].laneGroupLBs[j].isCutByRefE;
                    arrLGs[i].laneGroupLBs.Delete(j);
                    j--;
                }
            }
        }

        //删除lg内短距离长度的线,删除不满足两条车到边界的车道线
        int n = arrLGs.GetCount() - 1;
        for (int i = n; i >= 0; --i) {
            //删除lg内短距离长度的线
            if (arrLGs[i].laneGroupLBs.GetCount() < 2)
                arrLGs.Delete(i);
        }

        for (int i = 0; i < arrLGs.GetCount(); ++i) {
            //test输出原始线
            afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_orilg", arrLGs[i].laneGroupLBs);

            //判断起点
            int firstCut = -1;
            Array<int> noCutArr;
            Array<int> FlexibilityArr;
            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); j++) {
                if (arrLGs[i].laneGroupLBs[j].isCutByRefS == true) {
                    if (firstCut < 0)
                        firstCut = j;
                } else
                    noCutArr.Add(j);
            }
            if (noCutArr.IsEmpty())
                std::cout << i << "：起点无需节点对齐" << std::endl;
            else {
                //节点对齐具体操作
                if (firstCut < 0) //没有任何点进行切割，按照最突出对齐
                    GetOutstandLane(arrLGs[i].laneGroupLBs, firstCut);

                std::cout << i << "--起点对齐参考线--" << firstCut << std::endl;
                for (int j = 0; j < noCutArr.GetCount(); ++j) {
                    if (noCutArr[j] == firstCut)
                        continue;
                    FlexibilityArr.Add(noCutArr[j]);
                    Flexibility(arrLGs[i].laneGroupLBs[noCutArr[j]], arrLGs[i].laneGroupLBs[firstCut]);
                }

                //test输出起点对齐后的线
                afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_orilgS", arrLGs[i].laneGroupLBs);

                //对对齐后的线判断是够需要重新切割
                LG newlg;
                if (AdjustCutOneLG(arrLGs[i], FlexibilityArr, newlg)) {
                    arrLGs.InsertAt(i, newlg);
                    i = i + 1;
                    afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_oricutLGS1", newlg.laneGroupLBs);
                    afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_oricutLGS2", arrLGs[i].laneGroupLBs);
                }
            }

            //判断终点
            int firstCut2 = -1;
            Array<int> noCutArr2;
            Array<int> FlexibilityArr2;

            //为了复用代码，将线进行反转
            afterProcessCommon::ReverseLane2(arrLGs[i].laneGroupLBs);

            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); j++) {
                if (arrLGs[i].laneGroupLBs[j].isCutByRefE == true) {
                    if (firstCut2 < 0)
                        firstCut2 = j;
                } else
                    noCutArr2.Add(j);
            }
            if (noCutArr2.IsEmpty()) {
                std::cout << i << "：终点无需节点对齐" << std::endl;
                afterProcessCommon::ReverseLane2(arrLGs[i].laneGroupLBs);
            } else {
                //节点对齐具体操作
                if (firstCut2 < 0) //没有任何点进行切割，按照最突出对齐
                    GetOutstandLane(arrLGs[i].laneGroupLBs, firstCut2);

                std::cout << i << "--终点对齐参考线--" << firstCut2 << std::endl;
                for (int j = 0; j < noCutArr2.GetCount(); ++j) {
                    if (noCutArr2[j] == firstCut2)
                        continue;
                    FlexibilityArr2.Add(noCutArr2[j]);
                    Flexibility(arrLGs[i].laneGroupLBs[noCutArr2[j]], arrLGs[i].laneGroupLBs[firstCut2]);
                }

                //对对齐后的线判断是够需要重新切割
                LG newlg;
                if (AdjustCutOneLG(arrLGs[i], FlexibilityArr2, newlg)) {
                    afterProcessCommon::ReverseLane2(arrLGs[i].laneGroupLBs);
                    afterProcessCommon::ReverseLane2(newlg.laneGroupLBs);

                    if (i == arrLGs.GetCount() - 1)
                        arrLGs.Add(newlg);
                    else
                        arrLGs.InsertAt(i + 1, newlg);

                    afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_oricutLGE1", arrLGs[i].laneGroupLBs);
                    afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_oricutLGE2", arrLGs[i + 1].laneGroupLBs);

                    i = i + 1;
                } else {
                    //反转回新的线
                    afterProcessCommon::ReverseLane2(arrLGs[i].laneGroupLBs);
                }

                //test输出终点对齐后的线
                afterProcessCommon::WriteLBToObj(dataDir, to_string(i) + "_orilgE", arrLGs[i].laneGroupLBs);
            }
        }
    }

    void afterProcess::GetOutstandLane(Array<LB> &arrLBs, int &resIndex) {
        resIndex = -1;
        int nCount = arrLBs.GetCount();
        if (nCount < 2)
            return;

        bool isFind = false;
        for (int i = 0; i < nCount; ++i) {
            if (i + 1 == nCount && !isFind)//最后一根线最突出
            {
                resIndex = i;
                isFind = true;
                break;
            }
            Engine::Geometries::Coordinate e1 = arrLBs[i].linePts[0] - arrLBs[i].linePts[1];
            e1.Normalize();
            for (int j = i + 1; j < nCount; ++j) {
                Engine::Geometries::Coordinate e2 = arrLBs[j].linePts[0] - arrLBs[i].linePts[0];
                e2.Normalize();
                double cross = e1.DotProduct(e2);
                if (cross > 0) //夹角是锐角，说明不是突出的线，垂点在另一条线了
                {
                    isFind = false;
                    break;
                }
                isFind = true;
            }
            if (isFind) {
                resIndex = i;
                break;
            }
        }
        return;
    }

    void afterProcess::GetHideLane(Array<LB> &arrLBs, int &resIndex) {
        resIndex = -1;
        int nCount = arrLBs.GetCount();
        if (nCount < 2)
            return;

        bool isFind = false;
        for (int i = 0; i < nCount; ++i) {
            if (i + 1 == nCount && !isFind)//最后一根线最突出
            {
                resIndex = i;
                isFind = true;
                break;
            }
            Engine::Geometries::Coordinate e1 = arrLBs[i].linePts[0] - arrLBs[i].linePts[1];
            e1.Normalize();
            for (int j = i + 1; j < nCount; ++j) {
                Engine::Geometries::Coordinate e2 = arrLBs[j].linePts[0] - arrLBs[i].linePts[0];
                e2.Normalize();
                double cross = e1.DotProduct(e2);
                if (cross < 0) //夹角是钝角，说明不是隐藏最深的线，比较突出
                {
                    isFind = false;
                    break;
                }
                isFind = true;
            }
            if (isFind) {
                resIndex = i;
                break;
            }
        }
        return;
    }

    void afterProcess::Flexibility(LB &dealCHDLane, const LB &compareCHDLane) {
        Int32 nSegIndex = 0;
        Engine::Geometries::Coordinate pntProject;
        Int32 i = 0;

        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (i = 0; i < compareCHDLane.linePts.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(compareCHDLane.linePts[i]));
        }

        bool bFindInLine = true;
        BaseAlgorithm3D::GetDistanceXYPointToLinesegments(dealCHDLane.linePts[0], coordinates, pntProject, nSegIndex,
                                                          bFindInLine);

        bFindInLine = true;
        int npts_1 = compareCHDLane.linePts.GetCount() - 1;
        if (nSegIndex == 0 && pntProject.DistanceXY(compareCHDLane.linePts[0]) < 0.00001)
        {
            bFindInLine = false;
        }
        else if (nSegIndex == npts_1 - 1 && pntProject.DistanceXY(compareCHDLane.linePts[npts_1]) < 0.00001)
        {
            bFindInLine = false;
        }

        if (bFindInLine == false) {
            //针对两条线并没有投影重合区域的问题
            BaseAlgorithm3D::GetProjectpntToLine(dealCHDLane.linePts[0], compareCHDLane.linePts[0],
                                                 compareCHDLane.linePts[1], pntProject);
        }

        //距离小于5cm不动
        double dis = compareCHDLane.linePts[0].DistanceXY(pntProject);
        if (dis < 0.05)
            return;
        //计算最终拉长的终点
        Engine::Geometries::Coordinate ePnt, pntTempEnd, eRemove;
        Double eRemoveDis = dealCHDLane.linePts[0].DistanceXY(pntProject);

        ePnt = dealCHDLane.linePts[0] - pntProject;
        ePnt.z = 0;
        ePnt.Normalize();
        pntTempEnd = compareCHDLane.linePts[0] + ePnt * eRemoveDis;

        //计算参加橡皮筋计算的点，并将形状移动到起点和终点
        Array<Engine::Geometries::Coordinate> arrClipCoordinates;
        for (i = 0; i <= nSegIndex; i++) {
            arrClipCoordinates.Add(compareCHDLane.linePts[i]);
        }
        arrClipCoordinates.Add(pntProject);

        BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, 0, pntTempEnd);

        Int32 nEndIndex = arrClipCoordinates.GetCount() - 1;
        BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, nEndIndex, dealCHDLane.linePts[0]);

        //将移动后的点添加到原来的线中(终点与原始起点重复，不再添加)
        for (int i = nEndIndex - 1; i >= 0; i--) {
            dealCHDLane.linePts.InsertAt(0, arrClipCoordinates[i]);
        }
        return;
    }



    void afterProcess::ConnectLanes(std::string laneBoundaryFromSegDir, Array<Engine::Geometries::Coordinate> &refLine,
                                    Array<LB> &arrLines) {
        if (arrLines.IsEmpty() || refLine.IsEmpty())
            return;

        //test 输出连接之前的线
        afterProcessCommon::WriteLBToObj(laneBoundaryFromSegDir, "beforeConnect", arrLines);

        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }

        //首先根据参考线进行排序
        for (int i = 0; i < arrLines.GetCount(); ++i) {
            //计算投影距离
            Engine::Geometries::Coordinate pntProject;
            Engine::Base::Int32 nSegIndex;
            Bool bFindInLine = true;
            BaseAlgorithm3D::GetDistanceXYPointToLinesegments(arrLines[i].linePts[0], coordinates, pntProject,
                                                              nSegIndex, bFindInLine);
            Engine::Geometries::Coordinate pntFrom = refLine[nSegIndex];
            pntFrom.z = 0.0;
            Engine::Geometries::Coordinate pntTo = refLine[nSegIndex + 1];
            pntTo.z = 0.0;

            Engine::Geometries::Coordinate testPnt = arrLines[i].linePts[0];
            testPnt.z = 0.0;
            double dis = BaseAlgorithm3D::DisPtToLine(pntFrom, pntTo, testPnt);
            int leftRight = BaseAlgorithm::PntMatchLine(pntFrom, pntTo, testPnt);
            if (leftRight < 2)
                dis = -1.0 * dis;
            arrLines[i].disToRef = dis;
        }

        std::sort(arrLines.Begin(), arrLines.End(), [&](const LB &d1, const LB &d2) {
            return d1.disToRef < d2.disToRef;
        });

        //test 输出连接后结果
        afterProcessCommon::WriteLBToObj(laneBoundaryFromSegDir, "sortConnect", arrLines);

        //计算连接
        for (int i = 0; i < arrLines.GetCount() - 1; ++i) {
            //判断是否进行连接
            bool isFirst = IsConnect(arrLines[i].linePts, arrLines[i + 1].linePts);
            bool isSecond = IsConnect(arrLines[i + 1].linePts, arrLines[i].linePts);
            if (!isFirst && !isSecond) //j 和 j+1 不能连接
                continue;
            //进行连接操作
            if (isFirst) //把j+1的点添加到j中
            {
                for (int k = 0; k < arrLines[i + 1].linePts.GetCount(); k++) {
                    arrLines[i].linePts.Add(arrLines[i + 1].linePts[k]);
                }
                arrLines.Delete(i + 1);
                i--;
            } else {
                for (int k = 0; k < arrLines[i].linePts.GetCount(); k++) {
                    arrLines[i + 1].linePts.Add(arrLines[i].linePts[k]);
                }
                arrLines.Delete(i);
                i--;
            }
        }

        //test 输出连接后结果
        afterProcessCommon::WriteLBToObj(laneBoundaryFromSegDir, "finishConnect", arrLines);
    }

    bool afterProcess::IsConnect(const Array<Engine::Geometries::Coordinate> &firstLine,
                                 const Array<Engine::Geometries::Coordinate> &compareLine) {
        if (firstLine.GetCount() < 2 || compareLine.GetCount() < 2)
            return false;

        //用第一条线的最后2点 与比较线的起点进行计算
        Engine::Geometries::Coordinate e1 = firstLine[firstLine.GetCount() - 1] - firstLine[firstLine.GetCount() - 2];
        e1.Normalize();
        e1.z = 0.0;

        Engine::Geometries::Coordinate e2 = compareLine[0] - firstLine[firstLine.GetCount() - 1];
        e2.Normalize();
        e2.z = 0.0;

        double dot = e1.DotProduct(e2);
        if (dot > 0.99863) //偏航角度设置为5度 cos(3°) = 0.99863 cos(10°) = 0.984808
            return true;

        double dis = compareLine[0].DistanceXY(firstLine[firstLine.GetCount() - 1]);
        if (dot > 0.984808 && dis < 10.0) //偏航角较大 但实际距离较短时也认可连接
            return true;
        return false;
    }

    void afterProcess::RfreshZ(std::string lidarFile, Array<LG> &arrLGs) {
        if (arrLGs.IsEmpty())
            return;

        //读取点云
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

//        //输入道路点和extend点
//        //label： "line": 80, "arrow_id": 100, "road_id": 0
//        pcl::PointCloud<pcl::PointXYZ>::Ptr roadcloud(new pcl::PointCloud<pcl::PointXYZ>);
//        for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
//            MyColorPointType &pcl_p = (*iter);
//            if(pcl_p.label == 0)
//            {
//                roadcloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
//            }
//            if(pcl_p.label == 80)
//            {
//                roadcloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
//            }
//        }
//        if (roadcloud->empty())
//            return;

        pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<MyColorPointType>);
        kdtree_cloud->setInputCloud(pc_ptr); // 设置要搜索的点云，建立KDTree

//        pcl::search::KdTree<MyColorPointType> kdtree_cloud;
//        kdtree_cloud.setInputCloud(pc_ptr);

        int nnn = 0;
        for (int i = 0; i < arrLGs.GetCount(); ++i) {
            for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); ++j) {

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                float radius1 = 0.2f;
                for (int m = 0; m < arrLGs[i].laneGroupLBs[j].linePts.GetCount(); ++m) {
                    Engine::Geometries::Coordinate curPt = arrLGs[i].laneGroupLBs[j].linePts[m];
                    MyColorPointType pt;
                    pt.x = curPt.x;
                    pt.y = curPt.y;
                    pt.z = curPt.z;

                    int K = 1;
                    std::vector<int> Idx;
                    std::vector<float> SquaredDistance;
                    kdtree_cloud->nearestKSearch(pt, K, Idx, SquaredDistance);
                    if (Idx.empty())
                        continue;
                    arrLGs[i].laneGroupLBs[j].linePts[m].z = pc_ptr->at(Idx[0]).z;
//                    std::cout<<nnn<<std::endl;
//                    nnn++;
                }
            }
        }
    }

    void afterProcess::RfreshZ(std::string lidarFile, Array<LB> &arrLBs) {
        if (arrLBs.IsEmpty())
            return;

        //读取点云
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<MyColorPointType>);
        kdtree_cloud->setInputCloud(pc_ptr); // 设置要搜索的点云，建立KDTree

        for (int j = 0; j < arrLBs.GetCount(); ++j) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            float radius1 = 0.2f;
            for (int m = 0; m < arrLBs[j].linePts.GetCount(); ++m) {
                Engine::Geometries::Coordinate curPt = arrLBs[j].linePts[m];
                MyColorPointType pt;
                pt.x = curPt.x;
                pt.y = curPt.y;
                pt.z = curPt.z;

                int K = 1;
                std::vector<int> Idx;
                std::vector<float> SquaredDistance;
                kdtree_cloud->nearestKSearch(pt, K, Idx, SquaredDistance);
                if (Idx.empty())
                    continue;
                arrLBs[j].linePts[m].z = pc_ptr->at(Idx[0]).z;
            }
        }
    }

    void afterProcess::DuplicateRemovald(Array<Array<Engine::Geometries::Coordinate>> arrArrRefLine,
                                         Array<Array<LB>> arrArrLeftLbs, Array<Array<LB>> arrArrRightLbs,
                                         Array<LB> &resArrLBs, double tolerance) {
        if (arrArrRefLine.GetCount() != 2)
            return;//此算法只适用于2个link
        if (arrArrRefLine[0].GetCount() < 2 || arrArrRefLine[1].GetCount() < 2)
            return;
        if (arrArrLeftLbs.GetCount() != 2 || arrArrRightLbs.GetCount() != 2)
            return;

        Coordinate e1 = arrArrRefLine[0][0] - arrArrRefLine[0][arrArrRefLine[0].GetCount() - 1];
        e1.z = 0.0;

        Coordinate e2 = arrArrRefLine[1][0] - arrArrRefLine[1][arrArrRefLine[1].GetCount() - 1];
        e2.z = 0.0;

        if (e1.DotProduct(e2) < 0) //方向相反,相反的两侧。并且反向去重
        {
            Array<hdmap_build::LinearObject *> arrLinearObject1;
            for (int i = 0; i < arrArrLeftLbs[0].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrLeftLbs[0][i].linePts);
                arrLinearObject1.Add(pObj);
            }
            for (int i = 0; i < arrArrRightLbs[1].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrRightLbs[1][i].linePts);
                arrLinearObject1.Add(pObj);
            }

            hdmap_build::LinearObjDuplicateRemoval rm;
            rm.RemoveDuplicate(arrLinearObject1, tolerance, true, 10);

            Array<hdmap_build::LinearObject *> arrLinearObject2;
            for (int i = 0; i < arrArrLeftLbs[1].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrLeftLbs[1][i].linePts);
                arrLinearObject2.Add(pObj);
            }
            for (int i = 0; i < arrArrRightLbs[0].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrRightLbs[0][i].linePts);
                arrLinearObject2.Add(pObj);
            }

            hdmap_build::LinearObjDuplicateRemoval rm2;
            rm2.RemoveDuplicate(arrLinearObject2, tolerance, true, 10);

            Array<LB> arrRBs;
            for (int i = 0; i < arrLinearObject1.GetCount(); ++i) {
                if (arrLinearObject1[i]->m_geometry->GetNumPoints() < 2)
                    continue;
                LB newLB;
                hdmap_build::CommonUtil::EngineLinestringToVec3Points(arrLinearObject1[i]->m_geometry, newLB.linePts);
                arrRBs.Add(newLB);
            }
            for (int i = 0; i < arrLinearObject2.GetCount(); ++i) {
                if (arrLinearObject2[i]->m_geometry->GetNumPoints() < 2)
                    continue;
                LB newLB;
                hdmap_build::CommonUtil::EngineLinestringToVec3Points(arrLinearObject2[i]->m_geometry, newLB.linePts);
                arrRBs.Add(newLB);
            }
            if (!arrRBs.IsEmpty()) {
                resArrLBs.Clear();
                resArrLBs = arrRBs;
            }
        } else {
            Array<hdmap_build::LinearObject *> arrLinearObject1;
            for (int i = 0; i < arrArrLeftLbs[0].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrLeftLbs[0][i].linePts);
                arrLinearObject1.Add(pObj);
            }
            for (int i = 0; i < arrArrLeftLbs[1].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrLeftLbs[1][i].linePts);
                arrLinearObject1.Add(pObj);
            }


            hdmap_build::LinearObjDuplicateRemoval rm;
            rm.RemoveDuplicate(arrLinearObject1, tolerance, true, 10);

            Array<hdmap_build::LinearObject *> arrLinearObject2;

            for (int i = 0; i < arrArrRightLbs[0].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrRightLbs[0][i].linePts);
                arrLinearObject2.Add(pObj);
            }
            for (int i = 0; i < arrArrRightLbs[1].GetCount(); ++i) {
                hdmap_build::LinearObject *pObj = new hdmap_build::LinearObject;
                pObj->m_geometry = hdmap_build::CommonUtil::CoordsToLineString(arrArrRightLbs[1][i].linePts);
                arrLinearObject2.Add(pObj);
            }
            hdmap_build::LinearObjDuplicateRemoval rm2;
            rm2.RemoveDuplicate(arrLinearObject2, tolerance, false, 10);

            Array<LB> arrRBs;
            for (int i = 0; i < arrLinearObject1.GetCount(); ++i) {
                if (arrLinearObject1[i]->m_geometry->GetNumPoints() < 2)
                    continue;
                LB newLB;
                hdmap_build::CommonUtil::EngineLinestringToVec3Points(arrLinearObject1[i]->m_geometry, newLB.linePts);
                arrRBs.Add(newLB);
            }
            for (int i = 0; i < arrLinearObject2.GetCount(); ++i) {
                if (arrLinearObject2[i]->m_geometry->GetNumPoints() < 2)
                    continue;
                LB newLB;
                hdmap_build::CommonUtil::EngineLinestringToVec3Points(arrLinearObject2[i]->m_geometry, newLB.linePts);
                arrRBs.Add(newLB);
            }
            if (!arrRBs.IsEmpty()) {
                resArrLBs.Clear();
                resArrLBs = arrRBs;
            }
        }
    }

    bool afterProcess::IsYellowLine(LB lg, const pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud) {
        bool isYellow = false;
        int nAll = lg.linePts.GetCount();
        int nYellow = 0;

        float radius1 = 0.5f;
        for (int i = 0; i < nAll; ++i) {
            MyColorPointType newPt;
            newPt.x = lg.linePts[i].x;
            newPt.y = lg.linePts[i].y;
            newPt.z = lg.linePts[i].z;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            kdtree_cloud->radiusSearch(newPt, radius1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            if (pointIdxRadiusSearch.size() > 5)
                nYellow++;
        }

        double radio = double(nYellow) / double(nAll);
        std::cout << "双线所占比例：" << radio << std::endl;
        if (radio > 0.50)
            isYellow = true;
        return isYellow;
    }

    void afterProcess::GenerateOffsetLine(Array<Coordinate> &lineCroods, double distance) {
        if (lineCroods.GetCount() < 2)
            return;

        //新的起点
        Coordinate sPt = lineCroods[0];
        Coordinate sNewPt = lineCroods[1];
        BaseAlgorithm3D::RotatePointXY(sPt, 1.5 * PI, sNewPt);
        Coordinate e = sNewPt - sPt;
        e.z = 0.0;
        e.Normalize();

        sNewPt = e * distance + sPt;
        sNewPt.z = sPt.z;
        BaseAlgorithm3D::Flexibility3D(lineCroods, 0, sNewPt);

        //新的终点
        int n = lineCroods.GetCount();
        Coordinate ePt = lineCroods[n - 1];
        Coordinate eNewPt = lineCroods[n - 2];
        BaseAlgorithm3D::RotatePointXY(ePt, 0.5 * PI, eNewPt);
        Coordinate e2 = eNewPt - ePt;
        e2.z = 0.0;
        e2.Normalize();

        eNewPt = e2 * distance + ePt;
        eNewPt.z = ePt.z;
        BaseAlgorithm3D::Flexibility3D(lineCroods, n - 1, eNewPt);
    }

    void afterProcess::LGsDelete(std::string dataDir, Array<LG> &arrLGs) {
        if (arrLGs.IsEmpty())
            return;

        //读入感知点云
        std::string pp_file = dataDir + "/pp_transCloud.pcd";
        pcl::PointCloud<MyColorPointType>::Ptr pp_ptr(new pcl::PointCloud<MyColorPointType>);

        std::ifstream infile(pp_file);
        if (infile.good())
            pcl::io::loadPCDFile(pp_file, *pp_ptr);
        if (pp_ptr->empty())
            return;

        //將感知点云中的黄色点提取出来(目前无颜色信息只能采用双线的代替 intensity=2)
        //条件滤波
        cout << "->正在进行条件滤波..." << endl;
        /*创建条件限定下的滤波器*/
        pcl::PointCloud<MyColorPointType>::Ptr cloud_filtered(new pcl::PointCloud<MyColorPointType>);
        pcl::ConditionAnd<MyColorPointType>::Ptr range_cond(
                new pcl::ConditionAnd<MyColorPointType>());//创建条件定义对象range_cond
        //为条件定义对象添加比较算子
        range_cond->addComparison(pcl::FieldComparison<MyColorPointType>::ConstPtr(
                new pcl::FieldComparison<MyColorPointType>("intensity", pcl::ComparisonOps::EQ, 2)));//双线

        pcl::ConditionalRemoval<MyColorPointType> cr;    //创建滤波器对象
        cr.setCondition(range_cond);                //用条件定义对象初始化
        cr.setInputCloud(pp_ptr);                    //设置待滤波点云
        cr.filter(*cloud_filtered);                    //执行滤波，保存滤波结果于cloud_filtered

        if (cloud_filtered->empty()) {
            std::cout << "无双线点......" << std::endl;
            return;
        }
        pcl::io::savePCDFileBinary(dataDir + "/pp_double.pcd", *cloud_filtered);
        pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<MyColorPointType>);
        kdtree_cloud->setInputCloud(cloud_filtered); // 设置要搜索的点云，建立KDTree

        Array<LG> arrnewLGs;
        int n = arrLGs.GetCount();
        for (int i = n - 1; i >= 0; i--) {
            bool isYellow = false;
            int endIndex = -1;
            int nLBs = arrLGs[i].laneGroupLBs.GetCount();
            for (int j = nLBs - 1; j >= 0; j--) {
                isYellow = IsYellowLine(arrLGs[i].laneGroupLBs[j], kdtree_cloud);
                if (isYellow) {
                    endIndex = j;
                    break;
                }
            }

            std::cout << "首条黄线index :" << endIndex << std::endl;
            if (endIndex < 0 || (endIndex == nLBs - 1)) //没找到或者是最后一条 将数据删除
                continue;

            //针对此场景，需要根据link方向属性判断是删除对象还是将对象进行反向
            bool isDelete = true;
            std::string parse_json = dataDir + "/parse_json.json";
            //读入转换参数
            std::ifstream ifs_parseInfo(parse_json);
            if (!ifs_parseInfo.is_open()) {
                ifs_parseInfo.close();
                std::cout << "没有发现parse_josn：" << parse_json << endl;
            } else {
                nlohmann::json parseInfo;
                parseInfo << ifs_parseInfo;
                std::string center_line = parseInfo["link_geom"];
                int link_direction = parseInfo["link_direction"];
                if (link_direction == 3) {
                    isDelete = false;
                    std::cout << "link 方向为双向，不能直接删除，需进行反向操作" << std::endl;
                } else
                    std::cout << "link 方向为单向，直接删除对向车道" << std::endl;
            }

            //针对黄线场景进行处理
            double moveDis = 0.1;
            if (isDelete) //删除操作
            {
                //双线的场景应该对线进行移动 目前左侧向右移动5cm 右侧向左移动5cm
                GenerateOffsetLine(arrLGs[i].laneGroupLBs[endIndex].linePts, moveDis);

                for (int j = endIndex - 1; j >= 0; j--) {
                    std::cout << "删除线index :" << j << std::endl;
                    arrLGs[i].laneGroupLBs.Delete(j);
                }

                if (arrLGs[i].laneGroupLBs.GetCount() < 2)
                    arrLGs.Delete(i);
            } else  //反向操作
            {
                LG newLG;
                for (int j = endIndex; j >= 0; j--)
                    newLG.laneGroupLBs.Add(arrLGs[i].laneGroupLBs[j]);
                if (newLG.laneGroupLBs.GetCount() > 1) {
                    afterProcessCommon::ReverseLane(newLG.laneGroupLBs);

                    //双线向右移动5cm
                    GenerateOffsetLine(newLG.laneGroupLBs[0].linePts, moveDis);
                    arrnewLGs.Add(newLG);
                }

                GenerateOffsetLine(arrLGs[i].laneGroupLBs[endIndex].linePts, moveDis);

                for (int j = endIndex - 1; j >= 0; j--) {
                    std::cout << "删除线index :" << j << std::endl;
                    arrLGs[i].laneGroupLBs.Delete(j);
                }

                if (arrLGs[i].laneGroupLBs.GetCount() < 2)
                    arrLGs.Delete(i);
            }
        }

        if (!arrnewLGs.IsEmpty())
            arrLGs.Add(arrnewLGs);
    }

    void afterProcess::afterProcessLB(std::string dataDir, const Array<Coordinate>& polygonPts, Array<LG> &arrLGs) {
        Array<LB> arrLines;
        //get lane boundary
        std::string laneBoundaryFromSegDir = dataDir + "/laneBoundaryFromSeg";
        std::string testOutDir = laneBoundaryFromSegDir + "/midout";
        std::string command = "mkdir " + testOutDir + " && chmod 777 " + testOutDir;
        system(command.c_str());
        afterProcessCommon::ReadLB(laneBoundaryFromSegDir, arrLines);
        std::string refLinePath = dataDir + "/laneBoundaryFromSeg/refLink.pcd";
        Array<Engine::Geometries::Coordinate> refLine;
        afterProcessCommon::ReadRfLine(refLinePath, refLine);
        GetLaneGroupByLinkPts(arrLines, refLine, arrLGs);
        CalLaneGroupSeq(refLine, arrLGs);
        afterProcessCommon::WriteLGToObj(testOutDir, "cutbylink1", arrLGs);
        AdjustLGs(testOutDir, arrLGs);
        afterProcessCommon::WriteLGToObj(testOutDir, "adjuestnode2", arrLGs);
        CombineLGs(arrLGs);
        afterProcessCommon::WriteLGToObj(testOutDir, "combinelgs3", arrLGs);
        LGsDelete(dataDir, arrLGs);
        afterProcessCommon::WriteLGToObj(testOutDir, "yellowDelete4", arrLGs);
        afterProcessCommon::ResampleLGs(arrLGs, 0.08);
        CombineLGs(arrLGs);
        afterProcessCommon::WriteLGToObj(testOutDir, "Rcombine5", arrLGs);
        //删除起始和终止处长度小于10米的车道组
        int n = arrLGs.GetCount();
        if (n > 1) {
            if (arrLGs[n - 1].laneGroupLBs.GetCount() > 0) {
                if (hdmap_build::CommonUtil::GetLength(arrLGs[n - 1].laneGroupLBs[0].linePts) < 10.0)
                    arrLGs.Delete(n - 1);
            }

            if (arrLGs[0].laneGroupLBs.GetCount() > 0) {
                if (hdmap_build::CommonUtil::GetLength(arrLGs[0].laneGroupLBs[0].linePts) < 10.0)
                    arrLGs.Delete(0);
            }
        }

        //对线进行截取
        n = arrLGs.GetCount();
        if (n > 1)
        {
            for (int i = n - 1; i >= 0 ; i--)
            {
                int m = arrLGs[i].laneGroupLBs.GetCount();
                for (int j = m - 1; j >= 0; j--) {
                    RoadMapping::afterProcessCommon::ClipByPolygon(arrLGs[i].laneGroupLBs[j].linePts, polygonPts);
                    if (arrLGs[i].laneGroupLBs[j].linePts.GetCount() < 2)
                        arrLGs[i].laneGroupLBs.Delete(j);
                }
                if (arrLGs[i].laneGroupLBs.GetCount() < 2)
                    arrLGs.Delete(i);
            }
        }
        std::string lidarFile = dataDir + "/transCloud.pcd";
        RfreshZ(lidarFile, arrLGs);
        afterProcessCommon::WriteLGToObj(dataDir, "res", arrLGs);
    }

    void afterProcess::afterProcessCrossWalk(std::string dataDir, std::string global_pcd, const Array<Coordinate>& polygonPts, const Engine::Base::Array<RM> &arrStopLineObjs, const Engine::Base::Array<Engine::Geometries::Coordinate> lukou_center_pts, Engine::Base::Array<RM> &arrCrossWalkObjs)
    {
        if(!boost::filesystem::exists(global_pcd))
        {
            std::cout<<"global_pcd 不存在："<<global_pcd<<std::endl;
            return;
        }

        if((dataDir.back() == '/') || (dataDir.back() == '\\'))
            dataDir.erase(dataDir.size()-1);

//        boost::filesystem::path filePath(dataDir);
//        std::string task_path = filePath.parent_path().string();

        std::string lukou_path = dataDir + "/LuKou";
        if(!boost::filesystem::exists(lukou_path))
        {
            std::cout<<"LuKou文件夹 不存在："<<lukou_path<<std::endl;
            return;
        }

        // Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> stop_lines;
        // for(int i = 0; i < arrStopLineObjs.GetCount(); i++)
        // {
        //     if (arrStopLineObjs[i].plygonPnts.GetCount() == 2)
        //         stop_lines.Add(arrStopLineObjs[i].plygonPnts);
        // }
        // hdmap_build::CommonUtil::WriteToOBJ(stop_lines, lukou_path, "stopline_after",true);
        // Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edgePts;
        
        //////////////////////////////////////////车端label生成人行横道/////////////////////////////////
        // processCrosswalk crosswalkprosslabel;
        // crosswalkpross.get_crosswalk_by_stopline(global_pcd, lukou_center_pts, stop_lines, lukou_path, edgePts);

        ///////////////////////云端label生成人行横道(bev结果默认放在 dataDir + "/LuKou/bev_obj)//////////
        // processCrosswalk crosswalkprosspanoseg;
        // std::string bev_obj_dir = dataDir + "/bev_obj";
        // if(!boost::filesystem::exists(bev_obj_dir))
        // {
        //     std::cout<<"bev_obj文件夹 不存在："<<bev_obj_dir<<std::endl;
        //     return;
        // }
        // crosswalkprosspanoseg.run_bev_label_crosswalk(global_pcd, bev_obj_dir, lukou_path, edgePts);
        
        // //将数据根据polygon进行裁剪
        // int n = edgePts.GetCount();
        // for (int i = n - 1; i >= 0; i--)
        // {
        //     Array<Coordinate> clipPts = edgePts[i];
        //     clipPts.Delete(0);
        //     double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
        //     RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
        //     double dis = hdmap_build::CommonUtil::GetLength(clipPts);
        //     if(fabs(disOri - dis) > 0.1)
        //         edgePts.Delete(i);
        // }

        // if (!edgePts.IsEmpty())
        // {
        //     hdmap_build::CommonUtil::WriteToOBJ(edgePts, lukou_path, "crosswalk", true);
        //     for (int i = 0; i < edgePts.GetCount(); ++i) {
        //         int nPts = edgePts[i].GetCount();
        //         if (nPts < 4) //多边形，最少为4个点，首尾点相同 只存一个
        //             continue;

        //         edgePts[i].Delete(nPts - 1);

        //         RM newRm;
        //         newRm.plygonPnts = edgePts[i];
        //         newRm.ObjectType = 10;   //CROSS_WALK = 10;//斑马线
        //         arrCrossWalkObjs.Add(newRm);
        //     }

        //     std::string pcdpath = lukou_path + "/object_crosswalk.pcd";
        //     objsToPointElementPCD(edgePts, pcdpath, 5);
        // }
        Engine::Base::Array<RM> arrRMs;
        afterProcessCommon::ReadCrossWalk(lukou_path, arrRMs);
        if (!arrRMs.IsEmpty())
        {
            int n = arrRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrCrossWalkObjs.Add(arrRMs[i]);
            }
        }
        
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> cross_walks;
        for (int i = 0; i < arrCrossWalkObjs.GetCount(); i++)
        {
                cross_walks.Add(arrCrossWalkObjs[i].plygonPnts);
        } 

        if (!cross_walks.IsEmpty()) {
            std::string crosswalkpcd = lukou_path + "/object_crosswalk.pcd";
            RoadMapping::afterProcess::objsToPointElementPCD(cross_walks, crosswalkpcd, 5);
        }
    }

    void afterProcess::filterRMbyRoi(std::string dataDir, std::vector<std::string> links, Array<RM> &arrRMs) {
        //读取所有link点
        Array<Array<Coordinate *> *> linkLines;
        for (auto link:links) {
            std::string linkPcdPath = dataDir + "/" + link + "/link.pcd";
            Array<Coordinate> linkPnts;
            afterProcessCommon::ReadRfLine(linkPcdPath, linkPnts);

            if (!linkPnts.IsEmpty()) {
                Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
                for (int i = 0; i < linkPnts.GetCount(); ++i) {
                    coordinates->Add(new Engine::Geometries::Coordinate(linkPnts[i]));
                }
                linkLines.Add(coordinates);
            }
        }
        if (linkLines.IsEmpty())
            return;

        int n = arrRMs.GetCount();
        for (int i = n - 1; i >= 0; i--) {
            Coordinate adjustPnt = arrRMs[i].plygonPnts[0];
            adjustPnt.z = 0.0;
            Bool bFindInLine = false;
            for (int j = 0; j < linkLines.GetCount(); ++j) {

                Engine::Geometries::Coordinate pntProject;
                Engine::Base::Int32 nSegIndex;
                BaseAlgorithm3D::GetDistanceXYPointToLinesegments(adjustPnt, linkLines[j], pntProject, nSegIndex,
                                                                  bFindInLine);
                if (bFindInLine)
                    break;
            }
            if (bFindInLine)
                continue;
            arrRMs.Delete(i); //没有覆盖则删除
        }
    }
    void afterProcess::afterProcessStopLine(std::string dataDir, const Array<Coordinate>& polygonPts, Array<RM> &arrResultRMs)
    {
        std::string lukouDir = dataDir + "/" + "LuKou";
        std::string filepath = lukouDir + "/" + "stopline.obj";
        std::cout<< "读入停止线："<< filepath <<std::endl;
        Engine::Base::Array<RM> arrRMs;
        afterProcessCommon::ReadStopLine(filepath, arrRMs);
        if (!arrRMs.IsEmpty())
        {
            int n = arrRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrResultRMs.Add(arrRMs[i]);
            }
        }
    }

    void afterProcess::afterProcessRM(std::string dataDir, std::vector<std::string> links, const Array<Coordinate>& polygonPts, Array<RM> &arrRMs) {
        //读取roadMark的json文件
        std::string RMFromSegDir = dataDir + "/" + "RoadMarkRes";

        //读取箭头
        int label_arrow = 100;
        Array<RM> arrArrowRMs;
        afterProcessCommon::ReadRM_wq(RMFromSegDir, label_arrow, arrArrowRMs);

        //读取sign_line, slow_down_triangle, speed_sign, diamond, bicycle_sign, parking_line
        int label_other = 110;
        Array<RM> arrOtherRMs;
        afterProcessCommon::ReadRM(RMFromSegDir, label_other, arrOtherRMs);

        filterRMbyRoi(dataDir, links, arrArrowRMs);
        filterRMbyRoi(dataDir, links, arrOtherRMs);

        if (!arrArrowRMs.IsEmpty())
        {
            int n = arrArrowRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrArrowRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrRMs.Add(arrArrowRMs[i]);
            }
        }
            

        if (!arrOtherRMs.IsEmpty())
        {
            int n = arrOtherRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrOtherRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrRMs.Add(arrOtherRMs[i]);
            }
        }

        //读取人行横道、停止线
       /* Array<RM> arrLukouRMs;
        std::string lukouDir = dataDir + "/" + "LuKou";
        afterProcessCommon::ReadRMObj(lukouDir, arrLukouRMs);
        if (!arrLukouRMs.IsEmpty())
        {
            int n = arrLukouRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrLukouRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrRMs.Add(arrLukouRMs[i]);
            }
        }*/

        if (!arrRMs.IsEmpty())
        {
            afterProcessCommon::WriteRMToObj(dataDir, "final", arrRMs);
        }
        
    }
    void afterProcess::readJunctionRM(std::string dataDir, std::vector<std::string> links,
            const Array<Coordinate>& polygonPts, Array<RM> &arrRMs){
        //读取人行横道、停止线
        Array<RM> arrLukouRMs;
        std::string lukouDir = dataDir + "/" + "LuKou";
        afterProcessCommon::ReadRMObj(lukouDir, arrLukouRMs);
        if (!arrLukouRMs.IsEmpty())
        {
            int n = arrLukouRMs.GetCount();
            for (int i = n - 1; i >= 0; i--)
            {
                Array<Coordinate> clipPts = arrLukouRMs[i].plygonPnts;
                double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
                RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
                double dis = hdmap_build::CommonUtil::GetLength(clipPts);
                if(fabs(disOri - dis) < 0.1)
                    arrRMs.Add(arrLukouRMs[i]);
            }
        }
    }
    void afterProcess::CalLeftOrRight(Array<Engine::Geometries::Coordinate> &refLine, Array<LB> &arrRBs,
                                      Array<LB> &arrLeftLbs, Array<LB> &arrRightLbs) {
        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }

        //计算参考线上刻度值
        Array<double> measureDis;
        double step = 0.0;
        measureDis.Add(step);
        for (int i = 1; i < refLine.GetCount(); ++i) {
            step += refLine[i].DistanceXY(refLine[i - 1]);
            measureDis.Add(step);
        }

        for (int i = 0; i < arrRBs.GetCount(); ++i) {
            //用起点判断
            //计算投影距离
            Engine::Geometries::Coordinate pntProject;
            Engine::Base::Int32 nSegIndex;
            Bool bFindInLine = true;
            Engine::Geometries::Coordinate pt0 = arrRBs[i].linePts[0];
            pt0.z = 0.0;
            BaseAlgorithm3D::GetDistanceXYPointToLinesegments(pt0, coordinates, pntProject,
                                                              nSegIndex, bFindInLine);
            Engine::Geometries::Coordinate pntFrom = refLine[nSegIndex];
            pntFrom.z = 0.0;
            Engine::Geometries::Coordinate pntTo = refLine[nSegIndex + 1];
            pntTo.z = 0.0;
            double dis = BaseAlgorithm3D::DisPtToLine(pntFrom, pntTo, pt0);
            int leftRight = BaseAlgorithm::PntMatchLine(pntFrom, pntTo, pt0);
            arrRBs[i].disToRefS = measureDis[nSegIndex] + pntProject.DistanceXY(pntFrom);

            //计算到起终点到参考线的距离
            arrRBs[i].SToRef = dis;

            Engine::Geometries::Coordinate ptE = arrRBs[i].linePts[arrRBs[i].linePts.GetCount() - 1];
            ptE.z = 0.0;
            BaseAlgorithm3D::GetDistanceXYPointToLinesegments(ptE, coordinates, pntProject,
                                                              nSegIndex, bFindInLine);
            pntFrom = refLine[nSegIndex];
            pntFrom.z = 0.0;
            pntTo = refLine[nSegIndex + 1];
            pntTo.z = 0.0;
            double dis2 = BaseAlgorithm3D::DisPtToLine(pntFrom, pntTo, ptE);
            arrRBs[i].EToRef = dis2;

            if (leftRight < 2) //左侧的线
            {
                arrRBs[i].leftOrRight = 1;
                arrLeftLbs.Add(arrRBs[i]);
            } else //右侧的线
            {
                arrRBs[i].leftOrRight = 2;
                arrRightLbs.Add(arrRBs[i]);
            }
        }

        //将一个车道组内的线按照刻度值进行排序
        std::sort(arrLeftLbs.Begin(), arrLeftLbs.End(), [&](const LB &d1, const LB &d2) {
            return d1.disToRefS < d2.disToRefS;
        });

        //将一个车道组内的线按照刻度值进行排序
        std::sort(arrRightLbs.Begin(), arrRightLbs.End(), [&](const LB &d1, const LB &d2) {
            return d1.disToRefS < d2.disToRefS;
        });
    }

    void afterProcess::Flexibility(Engine::Geometries::Coordinate sPt, Engine::Geometries::Coordinate ePt,
                                   Array<Engine::Geometries::Coordinate> &refLine,
                                   Array<Engine::Geometries::Coordinate> &arrClipCoordinates) {
        Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinates->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }

        //计算参加橡皮筋计算的点，并将形状移动到起点和终点
        Engine::Geometries::Coordinate pntProject1, pntProject2;
        Engine::Base::Int32 nSegIndex1, nSegIndex2;
        Bool bFindInLine = true;
        BaseAlgorithm3D::GetDistanceXYPointToLinesegments(sPt, coordinates, pntProject1, nSegIndex1, bFindInLine);
        BaseAlgorithm3D::GetDistanceXYPointToLinesegments(ePt, coordinates, pntProject2, nSegIndex2, bFindInLine);

        arrClipCoordinates.Add(pntProject1);
        for (int i = nSegIndex1 + 1; i <= nSegIndex2; i++) {
            arrClipCoordinates.Add(refLine[i]);
        }
        arrClipCoordinates.Add(pntProject2);

        BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, 0, sPt);

        Int32 nEndIndex = arrClipCoordinates.GetCount() - 1;
        BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, nEndIndex, ePt);
    }

    void afterProcess::SingleSideRBDeal(Array<Engine::Geometries::Coordinate> &refLine, Array<LB> &arrRBs) {
        if (arrRBs.IsEmpty())
            return;

        int nlbs = arrRBs.GetCount();
        for (int i = nlbs - 1; i > 0; i--) {
            double diff = fabs(arrRBs[i].SToRef - arrRBs[i - 1].EToRef);
            if (diff > 2.0) //差距太大，不考虑连接
                continue;

            //连接条件 1、不重叠点 2、距离大于5米连接时需要进行橡皮筋拉伸
            Engine::Geometries::Coordinate endPt = arrRBs[i - 1].linePts[arrRBs[i - 1].linePts.GetCount() - 1]; //i-1终点
            Engine::Geometries::Coordinate endPtPrev;
            if (arrRBs[i-1].linePts.GetCount() < 2) {
                endPtPrev = endPt;
            } else {
                endPtPrev = arrRBs[i - 1].linePts[arrRBs[i - 1].linePts.GetCount() - 2]; //i-1终点前一点
            }

            Engine::Geometries::Coordinate e1 = endPt - endPtPrev;
            e1.z = 0;
            e1.Normalize();

            Engine::Geometries::Coordinate e2 = arrRBs[i].linePts[0] - endPt;
            e2.z = 0;
            e2.Normalize();

            if (e1.DotProduct(e2) < 0) {
                Array<int> deleteIndex;
                //计算不重叠区域
                for (int j = 0; j < arrRBs[i].linePts.GetCount(); ++j) {
                    Engine::Geometries::Coordinate e2 = arrRBs[i].linePts[j] - endPt;
                    e2.z = 0;
                    e2.Normalize();
                    if (e1.DotProduct(e2) < 0)
                        deleteIndex.Add(j);
                }

                //删除重叠区域
                int ndelete = deleteIndex.GetCount();
                for (int j = ndelete - 1; j >= 0; j--) {
                    arrRBs[i].linePts.Delete(j);
                }
            }

            if (arrRBs[i].linePts.IsEmpty()) {
                arrRBs.Delete(i);
                continue;
            }

            double dis = endPt.DistanceXY(arrRBs[i].linePts[0]);
            if (dis > 5.0) {
                Array<Engine::Geometries::Coordinate> arrClipCoordinates;
                Flexibility(endPt, arrRBs[i].linePts[0], refLine, arrClipCoordinates);

                for (int j = 1; j < arrClipCoordinates.GetCount() - 1; j++) {
                    arrRBs[i - 1].linePts.Add(arrClipCoordinates[j]);
                }
                for (int j = 0; j < arrRBs[i].linePts.GetCount(); ++j) {
                    arrRBs[i - 1].linePts.Add(arrRBs[i].linePts[j]);
                }
                arrRBs.Delete(i); //连接后删除原有线
            } else {
                //直接连接
                for (int j = 0; j < arrRBs[i].linePts.GetCount(); ++j) {
                    arrRBs[i - 1].linePts.Add(arrRBs[i].linePts[j]);
                }
                arrRBs.Delete(i); //连接后删除原有线
            }
        }
        //计算线到link的平均距离

        Array<Engine::Geometries::Coordinate *> *coordinateslink = new Array<Engine::Geometries::Coordinate *>;
        for (int i = 0; i < refLine.GetCount(); ++i) {
            coordinateslink->Add(new Engine::Geometries::Coordinate(refLine[i]));
        }
        for (int i = 0; i < arrRBs.GetCount(); ++i) {
            Array<Engine::Geometries::Coordinate *> *coordinatesline = new Array<Engine::Geometries::Coordinate *>;
            for (int j = 0; j < arrRBs[i].linePts.GetCount(); ++j) {
                coordinatesline->Add(new Engine::Geometries::Coordinate(arrRBs[i].linePts[j]));
            }
            double aveDis = 0.0;
            BaseAlgorithm3D::GetAverageDistanceLinesegmentsToLinesegments(coordinatesline, coordinateslink, aveDis);
            arrRBs[i].disToRef = aveDis;
        }
    }

    void
    afterProcess::ConnectRBs(std::string dataDir, std::string midDir, Array<Engine::Geometries::Coordinate> &refLine,
                             Array<LB> &arrRBs, Array<LB> &arrLeftLbs, Array<LB> &arrRightLbs) {
        //1.按照link进行左右排序
        std::string refLinePath = dataDir + "/link.pcd";
        afterProcessCommon::ReadRfLine(refLinePath, refLine);
        CalLeftOrRight(refLine, arrRBs, arrLeftLbs, arrRightLbs);

        afterProcessCommon::WriteLBToObj(midDir, "CalLeftOrRight_left", arrLeftLbs);
        afterProcessCommon::WriteLBToObj(midDir, "CalLeftOrRight_right", arrRightLbs);

        //2.单侧进行去重和连接
        SingleSideRBDeal(refLine, arrLeftLbs);
        SingleSideRBDeal(refLine, arrRightLbs);

        afterProcessCommon::WriteLBToObj(midDir, "ConnectRB_left", arrLeftLbs);
        afterProcessCommon::WriteLBToObj(midDir, "ConnectRB_right", arrRightLbs);
    }

    void afterProcess::afterProcessRB(std::string dataDir, std::vector<std::string> links, const Array<Coordinate>& polygonPts, Array<LB> &arrRBs) {
        //get road boundary
        Array<LB> allLinkRBs;
        Array<Array<Engine::Geometries::Coordinate>> arrArrRefLine;
        Array<Array<LB>> arrArrLeftLbs;
        Array<Array<LB>> arrArrRightLbs;
        for (auto link : links) {
            Array<LB> arrLinkRBs;
            Array<LB> arrLeftLbs;
            Array<LB> arrRightLbs;
            std::string linkDataDir = dataDir + "/" + link;
            std::string roadBoundaryFromSegDir = linkDataDir + "/roadBoundaryFromSeg";
            afterProcessCommon::ReadLB(roadBoundaryFromSegDir, arrLinkRBs);
            afterProcessCommon::WriteLBToObj(roadBoundaryFromSegDir, "ori", arrLinkRBs);
            Array<Engine::Geometries::Coordinate> refLine;
            ConnectRBs(linkDataDir, roadBoundaryFromSegDir, refLine, arrLinkRBs, arrLeftLbs, arrRightLbs);
            arrArrRefLine.Add(refLine);
            if (!arrLeftLbs.IsEmpty()) {
                //重刷高程
                std::string lidarFile = linkDataDir + "/transCloud.pcd";
                RfreshZ(lidarFile, arrLeftLbs);
                allLinkRBs.Add(arrLeftLbs);
            }

            if (!arrRightLbs.IsEmpty()) {
                //重刷高程
                std::string lidarFile = linkDataDir + "/transCloud.pcd";
                RfreshZ(lidarFile, arrLeftLbs);
                allLinkRBs.Add(arrRightLbs);
            }

            arrArrLeftLbs.Add(arrLeftLbs);
            arrArrRightLbs.Add(arrRightLbs);
        }

        //去重(多link需要)
        if (arrArrRefLine.GetCount() == 2) {
            DuplicateRemovald(arrArrRefLine, arrArrLeftLbs, arrArrRightLbs, allLinkRBs, 2.0);
            afterProcessCommon::WriteLBToObj(dataDir, "DuplicateRemoval_rb", allLinkRBs);
        }
        arrRBs = allLinkRBs;
        afterProcessCommon::SmoothLBs(arrRBs, 0.10);
        afterProcessCommon::ResampleLBs(arrRBs, 0.10);

        //对线进行截取
        int n = arrRBs.GetCount();
        for (int j = n - 1; j >= 0; j--) {
            RoadMapping::afterProcessCommon::ClipByPolygon(arrRBs[j].linePts, polygonPts);
            if (arrRBs[j].linePts.GetCount() < 2)
                arrRBs.Delete(j);
        }

        afterProcessCommon::WriteLBToObj(dataDir, "final_rb", arrRBs);
    }

    void afterProcess::afterProcessImpassableArea(std::string dataDir, Engine::Base::Array<ImpassableArea>& areas)
    {
        std::string lukou_path = dataDir + "/LuKou";
        if(!boost::filesystem::exists(lukou_path))
        {
            return;
        }

        std::string file_1 = lukou_path + "/impassablearea_8.obj";
        readImpassableArea(file_1, areas, 0);

        std::string file_2 = lukou_path + "/impassablearea_9.obj";
        readImpassableArea(file_2, areas, 1);

        std::string file_3 = lukou_path + "/impassablearea_10.obj";
        readImpassableArea(file_3, areas, 2);

        std::string file_4 = lukou_path + "/impassablearea_11.obj";
        readImpassableArea(file_4, areas, 3);

        std::string file_5 = lukou_path + "/impassablearea_12.obj";
        readImpassableArea(file_5, areas, 3);
    }

    void afterProcess::readImpassableArea(std::string impassablearea_obj, Engine::Base::Array<ImpassableArea>& areas, int subtype)
    {
        if(boost::filesystem::exists(impassablearea_obj))
        {
            Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrlines;
            afterProcessCommon::ReadObj(impassablearea_obj, arrlines); 
            for (int i = 0; i < arrlines.GetCount(); i++)
            {
                ImpassableArea newImpassableArea;
                newImpassableArea.plygonPnts = arrlines[i];
                newImpassableArea.midpt = hdmap_build::CommonUtil::GetAveragePt(arrlines[i]);
                newImpassableArea.isBind = false;
                newImpassableArea.subtype = subtype;
                areas.Add(newImpassableArea);
            }  
        }
    }

    void afterProcess::afterProcessLukou(std::string dataDir, const Engine::Base::Array<Coordinate>& polygonPts, Engine::Base::Array<Junction> &arrJunctions) {
        std::string lukouDir = dataDir + "/" + "LuKou";
        std::string lukouPath = lukouDir + "/lukou.obj";
        if(!boost::filesystem::exists(lukouPath))
        {
            std::cout<<"没有生成路口obj"<<std::endl;
            return;
        }

        Array<Array<Coordinate>> arrArrLines;
        afterProcessCommon::ReadObj(lukouPath, arrArrLines);
        if (arrArrLines.IsEmpty()) {
            return;
        }
        std::cout<<"生成路口个数："<< arrArrLines.GetCount() << std::endl;
        for (int i = 0; i < arrArrLines.GetCount(); ++i) {
            // 待正规task info release后打开
            // if(!afterProcessCommon::IsInPolygon(arrArrLines[i], polygonPts))
            // {
            //     std::cout<< "路口边界："<< i << "不在作业区域内" << std::endl;
            //     continue;
            // }
           
            Junction newJunction;
            newJunction.lukou_polygon_pts = arrArrLines[i];
            arrJunctions.Add(newJunction);
        }
    }

    void
    afterProcess::ConvertUtmToWgsAdd(std::vector<double> t_utm_world, int utm_num, Engine::Base::Array<LG> &arrLGs, Engine::Base::Array<RM> &arrRMs,
                                     Engine::Base::Array<LB> &arrRBs, Engine::Base::Array<Junction> &arrJunctions) {
        if (!arrLGs.IsEmpty()) {
            for (int i = 0; i < arrLGs.GetCount(); ++i) {
                for (int j = 0; j < arrLGs[i].laneGroupLBs.GetCount(); ++j) {
                    UtmToWgsAdd(t_utm_world, utm_num, arrLGs[i].laneGroupLBs[j].linePts);
                }
            }
        }
        if (!arrRMs.IsEmpty()) {
            for (int i = 0; i < arrRMs.GetCount(); ++i) {
                UtmToWgsAdd(t_utm_world, utm_num, arrRMs[i].plygonPnts);
            }
        }
        if (!arrRBs.IsEmpty()) {
            for (int i = 0; i < arrRBs.GetCount(); ++i) {
                UtmToWgsAdd(t_utm_world, utm_num, arrRBs[i].linePts);
            }
        }

        if (!arrJunctions.IsEmpty()) {
            for (int i = 0; i < arrJunctions.GetCount(); ++i) {
                UtmToWgsAdd(t_utm_world, utm_num, arrJunctions[i].lukou_polygon_pts);
                for (int j = 0; j < arrJunctions[i].areas_relation.GetCount(); ++j) {
                    UtmToWgsAdd(t_utm_world, utm_num, arrJunctions[i].areas_relation[j].plygonPnts);
                }
            }
        }
    }

    void afterProcess::UtmToWgsAdd(std::vector<double> t_utm_world, int utm_num,
                                   Array<Engine::Geometries::Coordinate> &pnts) {
        if (t_utm_world.size() != 3) {
            t_utm_world.clear();
            t_utm_world = {0.0, 0.0, 0.0};
        }

        if (pnts.IsEmpty())
            return;

        for (int i = 0; i < pnts.GetCount(); ++i) {
            Engine::Geometries::Coordinate newPt;
            newPt.x = pnts[i].x + t_utm_world[0];
            newPt.y = pnts[i].y + t_utm_world[1];
            newPt.z = pnts[i].z + t_utm_world[2];
            afterProcessCommon::utmToWgs84(newPt, utm_num);
            pnts[i] = newPt;
        }
    }

    void afterProcess::objsToPointElementPCD(const Array<Array<Engine::Geometries::Coordinate>>& arrLines, std::string pcdpath, uint16_t ele_type, uint16_t type1, uint16_t type2, uint16_t type3,float heading, float score)
    {
        pcl::PointCloud<PointElement>::Ptr pointElement(new pcl::PointCloud<PointElement>);
        for(int i = 0; i < arrLines.GetCount(); i++)
        {
            for (int j = 0; j < arrLines[i].GetCount(); j++)
            {
                PointElement newPclPt;
                newPclPt.id = i;
                newPclPt.x = arrLines[i][j].x;
                newPclPt.y = arrLines[i][j].y;
                newPclPt.z = arrLines[i][j].z;
                newPclPt.index = j;
                newPclPt.ele_type = ele_type;
                newPclPt.type1 = type1;
                newPclPt.type2 = type2;
                newPclPt.type3 = type3;
                newPclPt.heading = heading;
                newPclPt.score = score;
                pointElement->points.push_back(newPclPt);
            }
        }

        if(pointElement == NULL || pointElement->points.empty())
        {
            return;
        }

        pcl::io::savePCDFileBinary(pcdpath, *pointElement);
    }
}//namespace RoadMapping
