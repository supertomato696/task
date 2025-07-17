#include "cpl_port.h"
#include "polygonenumerator.h"
#include <cstddef>
#include "cpl_conv.h"
#include "cpl_error.h"

template<class DataType, class EqualityTest>
GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::GDALRasterPolygonEnumeratorT(
    int nConnectednessIn ){
    panPolyIdMap = NULL;
    panPolyValue = NULL;
    nNextPolygonId = 0;
    nPolyAlloc = 0;
    nConnectedness = nConnectednessIn;
    CPLAssert( nConnectedness == 4 || nConnectedness == 8 );
}

template<class DataType, class EqualityTest>
GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::~GDALRasterPolygonEnumeratorT(){
    Clear();
}

template<class DataType, class EqualityTest> 
void GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::Clear(){
    CPLFree( panPolyIdMap );
    CPLFree( panPolyValue );

    panPolyIdMap = NULL;
    panPolyValue = NULL;

    nNextPolygonId = 0;
    nPolyAlloc = 0;
}

template<class DataType, class EqualityTest>
void GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::MergePolygon( int nSrcId, int nDstIdInit ){
    int nDstIdFinal = nDstIdInit;
    while( panPolyIdMap[nDstIdFinal] != nDstIdFinal )
        nDstIdFinal = panPolyIdMap[nDstIdFinal];

    int nDstIdCur = nDstIdInit;
    while( panPolyIdMap[nDstIdCur] != nDstIdCur ){
        int nNextDstId = panPolyIdMap[nDstIdCur];
        panPolyIdMap[nDstIdCur] = nDstIdFinal;
        nDstIdCur = nNextDstId;
    }

    while( panPolyIdMap[nSrcId] != nSrcId ){
        int nNextSrcId = panPolyIdMap[nSrcId];
        panPolyIdMap[nSrcId] = nDstIdFinal;
        nSrcId = nNextSrcId;
    }
    panPolyIdMap[nSrcId] = nDstIdFinal;
}


template<class DataType, class EqualityTest>
int GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::NewPolygon(DataType nValue ){
    const int nPolyId = nNextPolygonId;

    if( nNextPolygonId >= nPolyAlloc ){
        nPolyAlloc = nPolyAlloc * 2 + 20;
        panPolyIdMap = static_cast<GInt32 *>(CPLRealloc(panPolyIdMap,nPolyAlloc*sizeof(GInt32)));
        panPolyValue = static_cast<DataType *>(CPLRealloc(panPolyValue,nPolyAlloc*sizeof(DataType)));
    }

    nNextPolygonId++;

    panPolyIdMap[nPolyId] = nPolyId;
    panPolyValue[nPolyId] = nValue;

    return nPolyId;
}

template<class DataType, class EqualityTest>
void GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::CompleteMerges(){
    int iPoly;
    int nFinalPolyCount = 0;

    for( iPoly = 0; iPoly < nNextPolygonId; iPoly++ ){
        int nId = panPolyIdMap[iPoly];
        while( nId != panPolyIdMap[nId] ){
            nId = panPolyIdMap[nId];
        }

        int nIdCur = panPolyIdMap[iPoly];
        panPolyIdMap[iPoly] = nId;
        while( nIdCur != panPolyIdMap[nIdCur] ){
            int nNextId = panPolyIdMap[nIdCur];
            panPolyIdMap[nIdCur] = nId;
            nIdCur = nNextId;
        }

        if( panPolyIdMap[iPoly] == iPoly )
            nFinalPolyCount++;
    }
}

template<class DataType, class EqualityTest>
void GDALRasterPolygonEnumeratorT<DataType,EqualityTest>::ProcessLine(
    DataType *panLastLineVal, DataType *panThisLineVal,
    GInt32 *panLastLineId,  GInt32 *panThisLineId,
    int nXSize ){
    int i;
    EqualityTest eq;

    if( panLastLineVal == NULL ){
        for( i=0; i < nXSize; i++ ){
            if( panThisLineVal[i] == GP_NODATA_MARKER ){
                panThisLineId[i] = -1;
            }else if( i == 0 || !(eq.operator()(panThisLineVal[i], panThisLineVal[i-1])) ){
                panThisLineId[i] = NewPolygon( panThisLineVal[i] );
            } else {
                panThisLineId[i] = panThisLineId[i - 1];
            }
        }

        return;
    }

    for( i = 0; i < nXSize; i++ ){
        if( panThisLineVal[i] == GP_NODATA_MARKER ){
            panThisLineId[i] = -1;
        }
        else if( i > 0 && eq.operator()(panThisLineVal[i], panThisLineVal[i-1]) ){
            panThisLineId[i] = panThisLineId[i-1];

            if( eq.operator()(panLastLineVal[i], panThisLineVal[i])
                && (panPolyIdMap[panLastLineId[i]]
                    != panPolyIdMap[panThisLineId[i]]) ){
                MergePolygon( panLastLineId[i], panThisLineId[i] );
            }

            if( nConnectedness == 8
                && eq.operator()(panLastLineVal[i-1], panThisLineVal[i])
                && (panPolyIdMap[panLastLineId[i-1]]
                    != panPolyIdMap[panThisLineId[i]]) ){
                MergePolygon( panLastLineId[i-1], panThisLineId[i] );
            }

            if( nConnectedness == 8 && i < nXSize-1
                && eq.operator()(panLastLineVal[i+1], panThisLineVal[i])
                && (panPolyIdMap[panLastLineId[i+1]]
                    != panPolyIdMap[panThisLineId[i]]) ){
                MergePolygon( panLastLineId[i+1], panThisLineId[i] );
            }
        }
        else if( eq.operator()(panLastLineVal[i], panThisLineVal[i]) ){
            panThisLineId[i] = panLastLineId[i];
        }
        else if( i > 0 && nConnectedness == 8 && eq.operator()(panLastLineVal[i-1], panThisLineVal[i]) ){
            panThisLineId[i] = panLastLineId[i-1];

            if( i < nXSize-1 && eq.operator()(panLastLineVal[i+1], panThisLineVal[i])
                && (panPolyIdMap[panLastLineId[i+1]]
                != panPolyIdMap[panThisLineId[i]]) ){
                MergePolygon( panLastLineId[i+1], panThisLineId[i] );
            }
        }
        else if( i < nXSize-1 && nConnectedness == 8 && eq.operator()(panLastLineVal[i+1], panThisLineVal[i]) ){
            panThisLineId[i] = panLastLineId[i+1];
        }
        else{
            panThisLineId[i] = NewPolygon( panThisLineVal[i] );
        }
    }
}

template class GDALRasterPolygonEnumeratorT<GInt32, IntEqualityTest>;