#include "gdal_alg.h"
#define GP_NODATA_MARKER -51502112
#define MAX_ULPS 10

struct IntEqualityTest {
    bool operator()(GInt32 a, GInt32 b) { return a == b; }
};

template<class DataType, class EqualityTest> class GDALRasterPolygonEnumeratorT{
private:
    void     MergePolygon( int nSrcId, int nDstId );
    int      NewPolygon( DataType nValue );

public:

    GInt32   *panPolyIdMap;
    DataType   *panPolyValue;

    int      nNextPolygonId;
    int      nPolyAlloc;

    int      nConnectedness;

public:
    explicit GDALRasterPolygonEnumeratorT( int nConnectedness=4 );
            ~GDALRasterPolygonEnumeratorT();

    void     ProcessLine( DataType *panLastLineVal, DataType *panThisLineVal,
                          GInt32 *panLastLineId,  GInt32 *panThisLineId,
                          int nXSize );

    void     CompleteMerges();

    void     Clear();
};

typedef GDALRasterPolygonEnumeratorT<GInt32, IntEqualityTest> GDALRasterPolygonEnumerator;