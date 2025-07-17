#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <cstring>
#include <spatialindex/SpatialIndex.h>
#include "proj_api.h"
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <gdal_alg.h>
#include "gdal_alg_priv.h"
#include "cpl_vsi.h"
#include "cpl_json.h"
#include <json-c/json.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LineString.h>
#include <geos/geom/LineSegment.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/linearref/LengthIndexedLine.h>
#include <geos/linearref/LengthIndexOfPoint.h>
#include <geos/algorithm/Length.h>
#include <geos/algorithm/Angle.h>
#include <geos/algorithm/MinimumBoundingCircle.h>
#include <geos/algorithm/MinimumDiameter.h>
#include <geos/algorithm/distance/DistanceToPoint.h>
#include <geos/operation/distance/DistanceOp.h>
#include <geos/operation/buffer/BufferOp.h>
#include "interpolation.h"
#include "logUtil.h"
#include <numeric>
#if defined(_WIN32) || defined(_WIN64)
#include <direct.h>
#include <io.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

using namespace std;
using namespace geos;
using namespace geos::geom;
using namespace geos::io;
using namespace geos::linearref;
using namespace geos::algorithm;
using namespace geos::operation::distance;
using namespace geos::operation::buffer;
using namespace SpatialIndex;

typedef struct {
	string link_id;
	std::unique_ptr<LineString> line;
	set<string> in;
	set<string> out;
}link_topo;

typedef struct {
	vector<CoordinateArraySequence*>* lane_markings;
	vector<CoordinateArraySequence*>* stop_lines;
	vector<CoordinateArraySequence*>* cross_walks;
	vector<CoordinateArraySequence*>* road_marks;
	vector<CoordinateArraySequence*>* aois;
}road_features;

typedef struct {
	string link_id;
	string name;
	double length;
	string free_speed;
	string capacity;
	string link_type_name;
	string link_type;
	string allowed_uses;
}osm_road;

class mapbuilding {
public:
	mapbuilding();
	~mapbuilding();

public:
	void getLanemarking(const char* pszShapePath,const char* pszName);

	void matching();

	void parse_mmt(const char* pszOriginPath, const char* pszDir);

	void addFields(const char* pszShapePath);

	void parseOSM(const char* pszPath);

private:
	void initGDAL();

	char** splitLine(const char* pszString, char chDelimiter);

	unique_ptr<LineString> coordinateProjection(unique_ptr<geom::LineString>& line);

	void build_osm_graph(const char* pszFilePath, map<string, link_topo*>& graph_topo);

	double transition_probability(string& link_id1, string& link_id2, unique_ptr<geom::Point>& pt1, unique_ptr<geom::Point>& pt2, map<string, link_topo*>& graph_topo);

	bool toRaster(const char* pszShapePath, const char* pszImagePath, double resolution);

	void compareNeighbour(int nPolyId1, int nPolyId2, int* panPolyIdMap, int*, std::vector<int>& anPolySizes, std::vector<int>& anBigNeighbour);

	CPLErr GPMaskImageData(GDALRasterBandH hMaskBand, GByte* pabyMaskLine, int iY, int nXSize, GInt32* panImageLine);

	bool sieve(GDALRasterBandH hSrcBand, GDALRasterBandH hMaskBand, GDALRasterBandH hDstBand, int nSizeThreshold, int nConnectedness);

	void fillPolygon(const char* pszSrcImagePath, const char* pszDstImagePath, int minPixelCount);

	bool deleteTwig(const char* pszImagePath, double resolution);

	void get_eight_neighborhood(unsigned char* pPixelValue, int width, int x, int y, set<string>& label);

	void utm_coordinate(double* geoTransform, int i, int j, Coordinate& point, double resolution);

	bool is_vertex(unsigned char* pPixelValue, int nWidth, int x, int y);

	bool allow_delete(set<string>& diff);

	bool have_neighborhood(set<string>& diff);

	bool vectorization(const char* pszImagePath, const char* pstShpPath, double resolution);

	void interpolator(const char* pszSrcPath, const char* pszDstPath, int zone);

	int reproject(const char* pszSrcPath, const char* pszDstPath);

	OGRLineString* geometryProjection(OGRLineString* line,int zone);

	LineString* geometryProjectionGeos(OGRLineString* line, int zone);

	OGRLineString* geometryProjection(string& wkt, int zone);

	OGRGeometry* geometryProjectionGeos(Geometry* geosGeom, int zone);

	OGRLineString* geometryProjection(vector<coordinate_ex*>& coordinate_list, int zone);

	bool erosion(void* pPixelValue, unsigned long lHeight, unsigned long nWidth);

	bool skeleton(const char* pszSrcImagePath, const char* pszDstImagePath);

	void read_ddld_landmark(const char* pszPath, vector<map<GInt64, vector<CoordinateArraySequence*>>>& features/*map<GInt64, vector<CoordinateArraySequence*>>& lanemarkings*/);

	void getLaneMarking(json_object* landmarks, vector<CoordinateArraySequence*>& lines,const char* pszFeatureName,const char* pszKeyName);

	void read_mla_egopose(const char* pszPath, map<GInt64, vector<double>>& egopose);

	void read_sensor_gnss_rtk(const char* pszPath, map<GInt64, int>& gnss_rtk);

	void geometryProjection(Coordinate& pt,int zone);

	double _transform_lat(double x, double y);

	double _transform_lon(double x, double y);

	void wgs84_to_gcj02(double lon, double lat, double& mglon, double& mglat);

	unique_ptr<LineString> wgs84_to_gcj02(const char* pszWKT);

	void wgs84_to_gcj02(const char* pszSrcPath, const char* pszDstPath);

	void initGraph();

	string matching(unique_ptr<LineString>& utm_gnss_line,string& road_name,string& road_class);

	unique_ptr<LineString> densifyLine(LineString* line,double interval);

	void getNodes(json_object* root, map<GIntBig, Coordinate>& nodes);

	void getWays(json_object* root, map<GIntBig, Coordinate>& nodes);

	void addLines(const char* pszPath, json_object* nodes_array, json_object* ways_array, int& node_last_id, int& way_last_id, int subtype);

	int getZone(int lon);

	void buildFeatures(const char* pszLanemarkingPath, vector<Coordinate>& centroids, const char* pszRoadmarkSavePath);

	void clusterFeatures(const char* pszPath, vector<vector<const Geometry*>>& cluster_geoms, double accept_distance);

	GIntBig nearestFeature(GIntBig current_id, map<GIntBig, LineString*>& lines, const vector<const Geometry*>& vec_geom, Envelope& current_enve, double radius, double accept_distance, ISpatialIndex* tree);

	void removeIndex(map<GIntBig, LineString*>& lines, GIntBig fid, ISpatialIndex* tree);

	GInt64 findClosest(GInt64 arr[], int n, GInt64 target);

	int getEgoposeStatus(GInt64* array,map<GInt64, int>& gnss_rtk, GInt64& ts);

	void fixingEgopose(map<GInt64, int>& gnss_rtk, map<GInt64, vector<double>>& egopose);

	json_object* getTags(map<string, string>& way_tags);

	double minAngle(LineString* line);

	CoordinateArraySequence* fittingLine(vector<Coordinate>& points);

	void addOtherOsm(const char* pszBasePath);

	void getUTMZone(map<int, Polygon*>& utmZone);

	Region getRect(Geometry* geom);

	Region getRect(OGRGeometry* geom);

	Geometry* fromOGR(OGRGeometry* ogrGeom);

	OGRGeometry* toOGR(std::unique_ptr<Geometry> geom);

	bool isDirectoryExist(const char* pszDirectory);

	void copyFile(const char* srcPath, const char* dstPath);

	void mergeLayer(vector<string>& utm_zones,const char* pszOutputDir);
public:
	void addPolygonsToOsm(const char* pszPath);

	void addRoadmark(const char* pszPath, int& node_id,int& way_id,int subtype, json_object* root);

	void addLinesToOsm(const char* pszPath);

	void kappa2osm(const char* pszPath);

	void getRoadMark(const char* pszRoadmarkPath);

	void export_gnss_rtk(const char* pszDir,const char* pszSavePath);

	void getCrosswalk(const char* pszPath);

	void getStopLine(const char* pszPath);

	void getAois(const char* pszPath);

	void clipLayer(const char* pszPath, map<int, Polygon*>& utmZone);

	void processingZone(const char* pszOriginPath, const char* pszOutputDir);

	bool splitZone(const char* pszPath,const char* pszOutputDir);

	int getZone(double x, double y);

	void splitEgoposeFile(const char* pszPath, map<int, json_object*>& utmEgoposeFiles);

	void buildMapping(const char* pszInDirectory,const char* pszOutDirectory);
private:
	const GeometryFactory* m_factory;
	WKTReader* m_reader;
	IStorageManager* m_diskfile;
	StorageManager::IBuffer* m_bufferfile;
	ISpatialIndex* m_tree;
	map<string, link_topo*> m_graph_topo;
	map<string, osm_road*> m_osm_roads;
	map<string, string> m_osm_road_class;
	int m_zone;
	map<int, Polygon*> m_utmZone;
};