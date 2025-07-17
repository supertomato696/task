#include "mapbuilding.h"
#define MY_MAX_INT 2147483647

class MyVisitor : public IVisitor {
public:
	size_t m_indexIO{ 0 };
	size_t m_leafIO{ 0 };
	vector<string> m_ids;

public:
	MyVisitor() = default;

	void getIDs(vector<string>& ids) {
		ids = m_ids;
	}

	void visitNode(const INode& n) override {
		if (n.isLeaf()) m_leafIO++;
		else m_indexIO++;
	}

	void visitData(const IData& d) override {
		IShape* pS;
		d.getShape(&pS);

		delete pS;
		// data should be an array of characters representing a Region as a string.
		uint8_t* pData = nullptr;
		uint32_t cLen = 0;
		d.getData(cLen, &pData);
		// do something.
		//string s = reinterpret_cast<char*>(pData);
		//cout << s << endl;
		delete[] pData;

		//cout << d.getIdentifier() << endl;
		char szID[512] = { 0 };
		sprintf(szID, "%lld", d.getIdentifier());

		m_ids.emplace_back(szID);
		// the ID of this data entry is an answer to the query. I will just print it to stdout.
	}

	void visitData(std::vector<const IData*>& v) override {
		cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << endl;
	}
};

mapbuilding::mapbuilding() {
	m_factory = GeometryFactory::getDefaultInstance();
	m_reader = new WKTReader(*m_factory);

	initGDAL();
	initGraph();
	m_osm_road_class["motorway"] = "高速公路";
	m_osm_road_class["trunk"] = "国道/城市快速路";
	m_osm_road_class["primary"] = "省道/主干路";
	m_osm_road_class["secondary"] = "县道/次干路";
	m_osm_road_class["tertiary"] = "乡道/支路";
	m_zone = -1;

	getUTMZone(m_utmZone);
}

mapbuilding::~mapbuilding() {
	delete m_reader;
	m_reader = NULL;

	delete m_tree;
	delete m_bufferfile;
	delete m_diskfile;
	auto iter = m_graph_topo.begin();
	for (; iter != m_graph_topo.end(); ++iter) {
		delete iter->second;
	}

	auto iter_utm = m_utmZone.begin();
	for (; iter_utm != m_utmZone.end(); ++iter_utm) {
		m_factory->destroyGeometry(iter_utm->second);
	}
}
void mapbuilding::getLanemarking(const char* pszShapePath, const char* pszName) {
	if (pszShapePath == NULL) {
		return;
	}
	const char* pszBasePath = CPLGetPath(pszShapePath);

	string projectPath = pszBasePath;
	projectPath += "/project.shp";

	string rasterPath = pszBasePath;
	rasterPath += "/raster.img";

	string fillPath = pszBasePath;
	fillPath += "/fill.img";

	string skeletonPath = pszBasePath;
	skeletonPath += "/skeleton.img";

	string vectorizationPath = pszBasePath;
	vectorizationPath += "/vectorization.txt";

	string interpolationPath = pszBasePath;
	interpolationPath += pszName;
	GDALDriver::QuietDelete(interpolationPath.c_str());

	double resolution = 0.4;
	logUtil::log("reproject");
	int zone = reproject(pszShapePath, projectPath.c_str());
	if (zone == -1) {
		return;
	}

	logUtil::log("toraster");
	toRaster(projectPath.c_str(), rasterPath.c_str(), resolution);
	
	logUtil::log("fillPolygon");
	fillPolygon(rasterPath.c_str(), fillPath.c_str(), 60);
	
	logUtil::log("skeleton");
	skeleton(fillPath.c_str(), skeletonPath.c_str());
	
	logUtil::log("deleteTwig");
	deleteTwig(skeletonPath.c_str(),resolution);
	
	logUtil::log("vectorization");
	vectorization(skeletonPath.c_str(), vectorizationPath.c_str(), resolution);
	
	logUtil::log("interpolator");
	interpolator(vectorizationPath.c_str(), interpolationPath.c_str(),m_zone);

	GDALDriver::QuietDelete(projectPath.c_str());
	GDALDriver::QuietDelete(rasterPath.c_str());
	GDALDriver::QuietDelete(fillPath.c_str());
	GDALDriver::QuietDelete(skeletonPath.c_str());
	remove(vectorizationPath.c_str());
	logUtil::log("cleanup");
}
void mapbuilding::matching() {
	auto gnss_line = m_reader->read<LineString>("LINESTRING (114.31343240115817 22.689669179983557, 114.3132880670756 22.68960312879323, 114.31288197457212 22.68947102641258, 114.31237802845337 22.68931201428773, 114.31173708727317 22.689116307057134, 114.3110667900084 22.688901029103484, 114.3103573512975 22.688675965788303, 114.3097457662019 22.688519400003827)");
	auto utm_gnss_line = coordinateProjection(gnss_line);

	string road_name;
	string road_class;
	string res = matching(utm_gnss_line, road_name, road_class);
	return;
}
void mapbuilding::addFields(const char* pszShapePath) {
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszShapePath, GDAL_OF_VECTOR | GDAL_OF_UPDATE, NULL, NULL, NULL);
	OGRLayer* layer = ds->GetLayer(0);

	OGRFieldDefn nameField("road_name", OFTString);
	layer->CreateField(&nameField);

	OGRFieldDefn classField("road_class", OFTString);
	layer->CreateField(&classField);

	OGRFeature* feature = NULL;
	double interval = 3;
	while ((feature = layer->GetNextFeature()) != NULL) {
		GIntBig fid = feature->GetFID();
		//if (fid != 432) {
		//	continue;
		//}
		OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
		OGRwkbGeometryType geoType = line->getGeometryType();
		if (geoType == wkbLineString) {
			char* szWKT = NULL;
			line->exportToWkt(&szWKT);
			auto gnss_line = m_reader->read<LineString>(szWKT);
			auto utm_gnss_line = coordinateProjection(gnss_line);
			auto road_line = densifyLine(utm_gnss_line.get(), interval);
			//string wkt = road_line->getCoordinatesRO()->toString();

			string road_name;
			string road_class;
			string res = matching(road_line, road_name, road_class);
			feature->SetField("road_name", road_name.c_str());
			feature->SetField("road_class", road_class.c_str());
			layer->SetFeature(feature);
			CPLFree(szWKT);
		}
		OGRFeature::DestroyFeature(feature);
	}
	GDALClose(ds);
}
void mapbuilding::initGDAL() {
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	CPLSetConfigOption("SHAPE_ENCODING", "");
#ifdef _WIN32
	CPLSetConfigOption("GDAL_DATA", "D:/gdal-2.4.4/data");
#else
	char szPath[512] = { 0 };
	CPLGetExecPath(szPath, 512);
	string basePath = CPLGetPath(szPath);
	basePath += "/3rd_bin/share/gdal";
	CPLSetConfigOption("GDAL_DATA", basePath.c_str());
#endif
	GDALAllRegister();
}

char** mapbuilding::splitLine(const char* pszString, char chDelimiter) {
	char* pszToken = static_cast<char*>(VSI_CALLOC_VERBOSE(10, 1));
	if (pszToken == nullptr)
		return nullptr;

	int nTokenMax = 10;
	char** papszRetList = nullptr;
	int nListSize = 0;
	int nListAlloc = 0;

	while (pszString != nullptr && *pszString != '\0') {
		bool bInString = false;
		int nTokenLen = 0;

		for (; *pszString != '\0'; pszString++) {
			if (!bInString && *pszString == chDelimiter) {
				pszString++;
				break;
			}

			if (*pszString == '"') {
				if (!bInString || pszString[1] != '"') {
					bInString = !bInString;
					continue;
				} else {
					pszString++;
				}
			}

			if (nTokenLen >= nTokenMax - 2) {
				nTokenMax = nTokenMax * 2 + 10;
				char* pszTokenNew = static_cast<char*>(VSI_REALLOC_VERBOSE(pszToken, nTokenMax));
				if (pszTokenNew == nullptr) {
					VSIFree(pszToken);
					CSLDestroy(papszRetList);
					return nullptr;
				}
				pszToken = pszTokenNew;
			}

			pszToken[nTokenLen] = *pszString;
			nTokenLen++;
		}

		pszToken[nTokenLen] = '\0';
		if (nListSize + 1 >= nListAlloc) {
			nListAlloc = 10 + 2 * nListAlloc;
			char** papszRetListNew = static_cast<char**>(VSI_REALLOC_VERBOSE(papszRetList, (nListAlloc + 1) * sizeof(char*)));
			if (papszRetListNew == nullptr) {
				CSLDestroy(papszRetList);
				VSIFree(pszToken);
				return nullptr;
			}
			papszRetList = papszRetListNew;
		}

		papszRetList[nListSize] = VSI_STRDUP_VERBOSE(pszToken);
		if (papszRetList[nListSize] == nullptr) {
			CSLDestroy(papszRetList);
			VSIFree(pszToken);
			return nullptr;
		}
		nListSize++;
		papszRetList[nListSize] = nullptr;

		if (*pszString == '\0' && *(pszString - 1) == chDelimiter) {
			papszRetList[nListSize] = VSI_STRDUP_VERBOSE("");
			if (papszRetList[nListSize] == nullptr) {
				CSLDestroy(papszRetList);
				VSIFree(pszToken);
				return nullptr;
			}
			nListSize++;
			papszRetList[nListSize] = nullptr;
			break;
		}
	}

	VSIFree(pszToken);
	return papszRetList;
}

unique_ptr<LineString> mapbuilding::coordinateProjection(unique_ptr<geom::LineString>& line) {
	size_t pointNums = line->getNumPoints();
	if (pointNums < 2) {
		return NULL;
	}
	double* x_array = new double[pointNums];
	double* y_array = new double[pointNums];
	for (size_t i = 0; i < pointNums; i++) {
		x_array[i] = line->getCoordinateN(i).x * DEG_TO_RAD;
		y_array[i] = line->getCoordinateN(i).y * DEG_TO_RAD;
	}

	int zone = int(31 + (line->getCoordinateN(0).x / 6.0));
	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pj_transform(pj_latlong, pj_utm, pointNums, 1, x_array, y_array, NULL);

	CoordinateArraySequence* cs = new CoordinateArraySequence();
	for (size_t i = 0; i < pointNums; i++) {
		cs->add(Coordinate(x_array[i], y_array[i]));
	}

	delete[]x_array;
	delete[]y_array;
	pj_free(pj_latlong);
	pj_free(pj_utm);
	return unique_ptr<LineString>(m_factory->createLineString(cs));
}

void mapbuilding::build_osm_graph(const char* pszFilePath,map<string, link_topo*>& graph_topo) {
	//string strDstPath = CPLGetPath(pszFilePath);
	//strDstPath += "/link_gcj02.csv";
	//wgs84_to_gcj02(pszFilePath, strDstPath.c_str());
	
	std::ifstream file("D:\\zhongbao\\mmt\\osm\\link_gcj02.csv");
	if (!file.is_open()) {
		return;
	}
	string record;
	int lineNum = 0;

	map<string, vector<string>> start_relo;
	map<string, vector<string>> end_relo;
	map<string, vector<string>> link_relo;

	while (std::getline(file, record)) {
		if (lineNum++ == 0) {
			continue;
		}
		char** papszRetList = splitLine(record.c_str(), ',');
		int nFields = CSLCount(papszRetList);
		char* szWKT = papszRetList[12];
		char* link_id = papszRetList[1];
		char* from_node_id = papszRetList[3];
		char* to_node_id = papszRetList[4];
		string full_node = from_node_id;
		full_node += ",";
		full_node += to_node_id;

		osm_road* osm_feature = new osm_road();
		osm_feature->link_id = papszRetList[1];
		osm_feature->name = papszRetList[0];
		osm_feature->length = atof(papszRetList[6]) ;
		osm_feature->free_speed = papszRetList[8];
		osm_feature->capacity = papszRetList[9];
		osm_feature->link_type_name = papszRetList[10];
		osm_feature->link_type = papszRetList[11];
		osm_feature->allowed_uses = papszRetList[13];
		m_osm_roads[papszRetList[1]] = osm_feature;

		auto line = m_reader->read<LineString>(szWKT);

		const CoordinateSequence* cs = line->getCoordinatesRO();
		string startPoint = cs->front().toString();
		string endPoint = cs->back().toString();

		vector<string> vec;
		vec.emplace_back(startPoint);
		vec.emplace_back(endPoint);
		vec.emplace_back(full_node);
		link_relo[link_id] = vec;
		auto start_iter = start_relo.find(startPoint);
		if (start_iter == start_relo.end()) {
			vector<string> vec;
			vec.emplace_back(link_id);
			start_relo[startPoint] = vec;
		} else {
			start_iter->second.emplace_back(link_id);
		}

		auto end_iter = end_relo.find(endPoint);
		if (end_iter == end_relo.end()) {
			vector<string> vec;
			vec.emplace_back(link_id);
			end_relo[endPoint] = vec;
		} else {
			end_iter->second.emplace_back(link_id);
		}

		auto utm_line = coordinateProjection(line);
		link_topo* topo = new link_topo();
		topo->link_id = link_id;
		topo->line = std::move(utm_line);

		graph_topo[link_id] = topo;

		CSLDestroy(papszRetList);
	}

	auto link_iter = link_relo.begin();
	for (; link_iter != link_relo.end(); ++link_iter) {
		string startPoint = link_iter->second[0];
		string endPoint = link_iter->second[1];
		string full_node = link_iter->second[2];

		{
			auto end_iter = end_relo.find(startPoint);
			vector<string> vec;
			if (end_iter != end_relo.end()) {
				for (size_t i = 0; i < end_iter->second.size(); i++) {
					string link_id = end_iter->second[i];
					auto find_iter = link_relo.find(link_id);
					if (find_iter != link_relo.end()) {
						string temp_full_node = find_iter->second[2];
						size_t index = temp_full_node.find_first_of(",");
						string from_node = temp_full_node.substr(0, index);
						string to_node = temp_full_node.substr(index + 1, temp_full_node.length() - index);
						string reverse_node = to_node + "," + from_node;
						if (reverse_node != full_node) {
							vec.emplace_back(link_id);
						}
					}
				}
			}

			auto iter = graph_topo.find(link_iter->first);
			if (iter != graph_topo.end()) {
				std::copy(vec.begin(), vec.end(), std::inserter(iter->second->in, iter->second->in.begin()));
			}
		}

		{
			auto start_iter = start_relo.find(endPoint);
			vector<string> vec;
			if (start_iter != start_relo.end()) {
				for (size_t i = 0; i < start_iter->second.size(); i++) {
					string link_id = start_iter->second[i];
					auto find_iter = link_relo.find(link_id);
					if (find_iter != link_relo.end()) {
						string temp_full_node = find_iter->second[2];
						size_t index = temp_full_node.find_first_of(",");
						string from_node = temp_full_node.substr(0, index);
						string to_node = temp_full_node.substr(index + 1, temp_full_node.length() - index);
						string reverse_node = to_node + "," + from_node;
						if (reverse_node != full_node) {
							vec.emplace_back(link_id);
						}
					}
				}
			}

			auto iter = graph_topo.find(link_iter->first);
			if (iter != graph_topo.end()) {
				//iter->second->out = vec;
				std::copy(vec.begin(), vec.end(), std::inserter(iter->second->out, iter->second->out.begin()));
			}
		}
	}

	file.close();
}

double mapbuilding::transition_probability(string& link_id1, string& link_id2, unique_ptr<geom::Point>& pt1, unique_ptr<geom::Point>& pt2, map<string, link_topo*>& graph_topo) {
	double tp = 0;
	if (link_id1 == link_id2) {
		auto iter1 = graph_topo.find(link_id1);
		LengthIndexedLine lil(iter1->second->line.get());
		double index1 = lil.project(Coordinate(pt1->getX(), pt1->getY()));
		double index2 = lil.project(Coordinate(pt2->getX(), pt2->getY()));
		double route_dist = index2 - index1;
		double line_dist = pt1->distance(pt2.get());
		tp = exp(-0.1 * abs(route_dist - line_dist) / 30.2);
	} else {
		auto iter1 = graph_topo.find(link_id1);
		auto iter2 = graph_topo.find(link_id2);
		if (iter1->second->out.count(link_id2) == 1) {
			CoordinateArraySequence* cs1 = (CoordinateArraySequence*)(iter1->second->line->getCoordinates().release());
			cs1->add(iter1->second->line->getCoordinatesRO(), true, true);
			LineString* newLine = m_factory->createLineString(cs1);

			LengthIndexedLine lil(newLine);
			double index1 = lil.project(Coordinate(pt1->getX(), pt1->getY()));
			double index2 = lil.project(Coordinate(pt2->getX(), pt2->getY()));
			double route_dist = index2 - index1;
			double line_dist = pt1->distance(pt2.get());
			tp = exp(-0.1 * abs(route_dist - line_dist) / 30.2);
		}
	}
	return tp;
}

bool mapbuilding::toRaster(const char* pszShapePath, const char* pszImagePath, double resolution) {
	GDALDataset* pVectorDataset = (GDALDataset*)GDALOpenEx(pszShapePath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);

	if (pVectorDataset == NULL) {
		return false;
	}

	OGRLayer* pLayer = pVectorDataset->GetLayer(0);
	OGRSpatialReference* pSpatialRef = pLayer->GetSpatialRef();

	if (pSpatialRef == NULL) {
		GDALClose(pVectorDataset);
		return false;
	}

	OGREnvelope extent;
	pLayer->GetExtent(&extent);
	extent.MinX -= 10;
	extent.MinY -= 10;

	extent.MaxX += 10;
	extent.MaxY += 10;

	int width = (int)ceil((extent.MaxX - extent.MinX) / resolution);
	int height = (int)ceil((extent.MaxY - extent.MinY) / resolution);

	char* pszProjection = NULL;
	pSpatialRef->exportToWkt(&pszProjection);

	unsigned char* pPixelValue = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	if (pPixelValue == NULL) {
		CPLFree(pszProjection);
		GDALClose(pVectorDataset);
		char szMsg[512] = {0};
		sprintf(szMsg, "to raster: memory allocation failed,the size is %d x %d", width, height);
		logUtil::log(szMsg);
		return false;
	}

	//char szDataSet[256] = { 0 };
	//sprintf(szDataSet, "MEM:::DATAPOINTER=%d,PIXELS=%d,LINES=%d", pPixelValue, width, height);

	GDALDriver* pDriver = GetGDALDriverManager()->GetDriverByName("HFA");
	GDALDataset* pRasterDataSet = pDriver->Create(pszImagePath, width, height, 1, GDT_Byte, NULL);

	if (pRasterDataSet == NULL) {
		CPLFree(pszProjection);
		free(pPixelValue);
		GDALClose(pVectorDataset);
		return false;
	}

	double geoTransform[6] = { 0 };
	geoTransform[0] = extent.MinX;
	geoTransform[1] = resolution;
	geoTransform[2] = 0;
	geoTransform[3] = extent.MaxY;
	geoTransform[4] = 0;
	geoTransform[5] = -resolution;

	pRasterDataSet->SetGeoTransform(geoTransform);
	pRasterDataSet->SetProjection(pszProjection);

	int* pBandList = new int[1];
	pBandList[0] = 1;

	double* pdfBurnValues = new double[1];
	pdfBurnValues[0] = 255;

	OGRLayerH* pBurnLayer = new OGRLayerH[1];
	pBurnLayer[0] = (OGRLayerH)pLayer;

	char** papszOptions = NULL;
	papszOptions = CSLSetNameValue(papszOptions, "CHUNKSIZE", "1");
	//papszOptions = CSLSetNameValue(papszOptions, "ALL_TOUCHED", "TRUE");

	CPLErr err = GDALRasterizeLayers((GDALDatasetH)pRasterDataSet, 1, pBandList, 1, pBurnLayer, NULL, NULL, pdfBurnValues, papszOptions, NULL, NULL);
	if (err != CE_None) {
		CSLDestroy(papszOptions);
		CPLFree(pszProjection);
		free(pPixelValue);
		delete[] pBurnLayer;
		delete[] pBandList;
		delete[] pdfBurnValues;
		GDALClose(pVectorDataset);
		GDALClose(pRasterDataSet);
		return false;
	}

	CSLDestroy(papszOptions);
	CPLFree(pszProjection);
	free(pPixelValue);
	delete[] pBurnLayer;
	delete[] pBandList;
	delete[] pdfBurnValues;
	GDALClose(pVectorDataset);
	GDALClose(pRasterDataSet);

	return true;
}

void mapbuilding::compareNeighbour(int nPolyId1, int nPolyId2, int* panPolyIdMap, int*, std::vector<int>& anPolySizes, std::vector<int>& anBigNeighbour) {
	if (nPolyId1 < 0 || nPolyId2 < 0) {
		return;
	}

	nPolyId1 = panPolyIdMap[nPolyId1];
	nPolyId2 = panPolyIdMap[nPolyId2];

	if (nPolyId1 == nPolyId2) {
		return;
	}

	if (anBigNeighbour[nPolyId1] == -1
		|| anPolySizes[anBigNeighbour[nPolyId1]] < anPolySizes[nPolyId2])
		anBigNeighbour[nPolyId1] = nPolyId2;

	if (anBigNeighbour[nPolyId2] == -1
		|| anPolySizes[anBigNeighbour[nPolyId2]] < anPolySizes[nPolyId1])
		anBigNeighbour[nPolyId2] = nPolyId1;
}

CPLErr mapbuilding::GPMaskImageData(GDALRasterBandH hMaskBand, GByte* pabyMaskLine, int iY, int nXSize, GInt32* panImageLine) {
	const CPLErr eErr = GDALRasterIO(hMaskBand, GF_Read, 0, iY, nXSize, 1, pabyMaskLine, nXSize, 1, GDT_Byte, 0, 0);
	if (eErr == CE_None) {
		for (int i = 0; i < nXSize; i++) {
			if (pabyMaskLine[i] == 0) {
				panImageLine[i] = GP_NODATA_MARKER;
			}
		}
	}

	return eErr;
}

bool mapbuilding::sieve(GDALRasterBandH hSrcBand, GDALRasterBandH hMaskBand, GDALRasterBandH hDstBand, int nSizeThreshold, int nConnectedness) {
	if (hSrcBand == NULL || hDstBand == NULL) {
		return false;
	}

	int nXSize = GDALGetRasterBandXSize(hSrcBand);
	int nYSize = GDALGetRasterBandYSize(hSrcBand);
	GInt32* panLastLineVal = static_cast<GInt32*>(VSI_MALLOC2_VERBOSE(sizeof(GInt32), nXSize));
	GInt32* panThisLineVal = static_cast<GInt32*>(VSI_MALLOC2_VERBOSE(sizeof(GInt32), nXSize));
	GInt32* panLastLineId = static_cast<GInt32*>(VSI_MALLOC2_VERBOSE(sizeof(GInt32), nXSize));
	GInt32* panThisLineId = static_cast<GInt32*>(VSI_MALLOC2_VERBOSE(sizeof(GInt32), nXSize));
	GInt32* panThisLineWriteVal = static_cast<GInt32*>(VSI_MALLOC2_VERBOSE(sizeof(GInt32), nXSize));
	GByte* pabyMaskLine = hMaskBand != NULL ? static_cast<GByte*>(VSI_MALLOC_VERBOSE(nXSize)) : NULL;
	if (panLastLineVal == NULL || panThisLineVal == NULL ||
		panLastLineId == NULL || panThisLineId == NULL ||
		panThisLineWriteVal == NULL ||
		(hMaskBand != NULL && pabyMaskLine == NULL)) {
		CPLFree(panThisLineId);
		CPLFree(panLastLineId);
		CPLFree(panThisLineVal);
		CPLFree(panLastLineVal);
		CPLFree(panThisLineWriteVal);
		CPLFree(pabyMaskLine);
		return false;
	}

	GDALRasterPolygonEnumerator oFirstEnum(nConnectedness);
	std::vector<int> anPolySizes;

	CPLErr eErr = CE_None;
	for (int iY = 0; eErr == CE_None && iY < nYSize; iY++) {
		eErr = GDALRasterIO(hSrcBand, GF_Read, 0, iY, nXSize, 1, panThisLineVal, nXSize, 1, GDT_Int32, 0, 0);

		if (eErr == CE_None && hMaskBand != NULL)
			eErr = GPMaskImageData(hMaskBand, pabyMaskLine, iY, nXSize, panThisLineVal);

		if (iY == 0)
			oFirstEnum.ProcessLine(NULL, panThisLineVal, NULL, panThisLineId, nXSize);
		else
			oFirstEnum.ProcessLine(panLastLineVal, panThisLineVal, panLastLineId, panThisLineId, nXSize);

		if (oFirstEnum.nNextPolygonId > static_cast<int>(anPolySizes.size()))
			anPolySizes.resize(oFirstEnum.nNextPolygonId);

		for (int iX = 0; iX < nXSize; iX++) {
			const int iPoly = panThisLineId[iX];

			if (iPoly >= 0 && anPolySizes[iPoly] < MY_MAX_INT)
				anPolySizes[iPoly] += 1;
		}

		std::swap(panLastLineVal, panThisLineVal);
		std::swap(panLastLineId, panThisLineId);
	}

	oFirstEnum.CompleteMerges();

	for (int iPoly = 0; oFirstEnum.panPolyIdMap != NULL && iPoly < oFirstEnum.nNextPolygonId; iPoly++) {
		if (oFirstEnum.panPolyIdMap[iPoly] != iPoly) {
			GIntBig nSize = anPolySizes[oFirstEnum.panPolyIdMap[iPoly]];
			nSize += anPolySizes[iPoly];
			if (nSize > MY_MAX_INT)
				nSize = MY_MAX_INT;

			anPolySizes[oFirstEnum.panPolyIdMap[iPoly]] = static_cast<int>(nSize);
			anPolySizes[iPoly] = 0;
		}
	}

	GDALRasterPolygonEnumerator oSecondEnum(nConnectedness);

	std::vector<int> anBigNeighbour;
	anBigNeighbour.resize(anPolySizes.size());

	for (int iPoly = 0; iPoly < static_cast<int>(anPolySizes.size()); iPoly++)
		anBigNeighbour[iPoly] = -1;

	for (int iY = 0; eErr == CE_None && iY < nYSize; iY++) {
		eErr = GDALRasterIO(hSrcBand, GF_Read, 0, iY, nXSize, 1, panThisLineVal, nXSize, 1, GDT_Int32, 0, 0);

		if (eErr == CE_None && hMaskBand != NULL)
			eErr = GPMaskImageData(hMaskBand, pabyMaskLine, iY, nXSize, panThisLineVal);

		if (eErr != CE_None)
			continue;

		if (iY == 0)
			oSecondEnum.ProcessLine(NULL, panThisLineVal, NULL, panThisLineId, nXSize);
		else
			oSecondEnum.ProcessLine(panLastLineVal, panThisLineVal, panLastLineId, panThisLineId, nXSize);

		for (int iX = 0; iX < nXSize; iX++) {
			if (iY > 0) {
				compareNeighbour(panThisLineId[iX],
					panLastLineId[iX],
					oFirstEnum.panPolyIdMap,
					oFirstEnum.panPolyValue,
					anPolySizes, anBigNeighbour);

				if (iX > 0 && nConnectedness == 8)
					compareNeighbour(panThisLineId[iX],
						panLastLineId[iX - 1],
						oFirstEnum.panPolyIdMap,
						oFirstEnum.panPolyValue,
						anPolySizes, anBigNeighbour);

				if (iX < nXSize - 1 && nConnectedness == 8)
					compareNeighbour(panThisLineId[iX],
						panLastLineId[iX + 1],
						oFirstEnum.panPolyIdMap,
						oFirstEnum.panPolyValue,
						anPolySizes, anBigNeighbour);
			}

			if (iX > 0)
				compareNeighbour(panThisLineId[iX],
					panThisLineId[iX - 1],
					oFirstEnum.panPolyIdMap,
					oFirstEnum.panPolyValue,
					anPolySizes, anBigNeighbour);
		}

		std::swap(panLastLineVal, panThisLineVal);
		std::swap(panLastLineId, panThisLineId);
	}

	int nFailedMerges = 0;
	int nIsolatedSmall = 0;
	int nSieveTargets = 0;

	for (int iPoly = 0; oFirstEnum.panPolyIdMap != NULL && oFirstEnum.panPolyValue != NULL && iPoly < static_cast<int>(anPolySizes.size()); iPoly++) {
		if (oFirstEnum.panPolyIdMap[iPoly] != iPoly)
			continue;

		if (oFirstEnum.panPolyValue[iPoly] == GP_NODATA_MARKER)
			continue;

		if (anPolySizes[iPoly] >= nSizeThreshold) {
			anBigNeighbour[iPoly] = -1;
			continue;
		}

		nSieveTargets++;
		if (anBigNeighbour[iPoly] == -1) {
			nIsolatedSmall++;
			continue;
		}

		std::set<int> oSetVisitedPoly;
		oSetVisitedPoly.insert(iPoly);

		int iFinalId = iPoly;
		bool bFoundBigEnoughPoly = false;
		while (true) {
			iFinalId = anBigNeighbour[iFinalId];
			if (iFinalId < 0) {
				break;
			}

			if (anPolySizes[iFinalId] >= nSizeThreshold) {
				bFoundBigEnoughPoly = true;
				break;
			}

			if (oSetVisitedPoly.find(iFinalId) != oSetVisitedPoly.end())
				break;
			oSetVisitedPoly.insert(iFinalId);
		}

		if (!bFoundBigEnoughPoly) {
			nFailedMerges++;
			anBigNeighbour[iPoly] = -1;
			continue;
		}

		int iPolyCur = iPoly;
		while (anBigNeighbour[iPolyCur] != iFinalId) {
			int iNextPoly = anBigNeighbour[iPolyCur];
			anBigNeighbour[iPolyCur] = iFinalId;
			iPolyCur = iNextPoly;
		}
	}

	oSecondEnum.Clear();

	for (int iY = 0; oFirstEnum.panPolyIdMap != NULL && eErr == CE_None && iY < nYSize; iY++) {
		eErr = GDALRasterIO(hSrcBand, GF_Read, 0, iY, nXSize, 1, panThisLineVal, nXSize, 1, GDT_Int32, 0, 0);

		memcpy(panThisLineWriteVal, panThisLineVal, 4 * nXSize);

		if (eErr == CE_None && hMaskBand != NULL)
			eErr = GPMaskImageData(hMaskBand, pabyMaskLine, iY, nXSize, panThisLineVal);

		if (eErr != CE_None)
			continue;

		if (iY == 0)
			oSecondEnum.ProcessLine(NULL, panThisLineVal, NULL, panThisLineId, nXSize);
		else
			oSecondEnum.ProcessLine(panLastLineVal, panThisLineVal, panLastLineId, panThisLineId, nXSize);

		for (int iX = 0; iX < nXSize; iX++) {
			int iThisPoly = panThisLineId[iX];
			if (iThisPoly >= 0) {
				iThisPoly = oFirstEnum.panPolyIdMap[iThisPoly];

				if (anBigNeighbour[iThisPoly] != -1) {
					panThisLineWriteVal[iX] = 255;
				}
			}
		}

		eErr = GDALRasterIO(hDstBand, GF_Write, 0, iY, nXSize, 1, panThisLineWriteVal, nXSize, 1, GDT_Int32, 0, 0);

		std::swap(panLastLineVal, panThisLineVal);
		std::swap(panLastLineId, panThisLineId);
	}

	CPLFree(panThisLineId);
	CPLFree(panLastLineId);
	CPLFree(panThisLineVal);
	CPLFree(panLastLineVal);
	CPLFree(panThisLineWriteVal);
	CPLFree(pabyMaskLine);

	return true;
}

void mapbuilding::fillPolygon(const char* pszSrcImagePath, const char* pszDstImagePath, int minPixelCount) {
	if (pszSrcImagePath == NULL || pszDstImagePath == NULL || minPixelCount < 1) {
		return;
	}
	GDALDataset* pSrcDS = (GDALDataset*)GDALOpen(pszSrcImagePath, GA_ReadOnly);
	if (pSrcDS == NULL) {
		return;
	}

	GDALRasterBandH hSrcBand = (GDALRasterBandH)pSrcDS->GetRasterBand(1);
	int width = pSrcDS->GetRasterXSize();
	int height = pSrcDS->GetRasterYSize();

	GDALDriver* pDriver = GetGDALDriverManager()->GetDriverByName("HFA");
	char** papszOptions = NULL;
	GDALDataset* pDstDS = pDriver->Create(pszDstImagePath, width, height, 1, GDT_Byte, papszOptions);
	if (pDstDS == NULL) {
		GDALClose(pSrcDS);
		return;
	}

	double geoTransform[6] = { 0 };
	pSrcDS->GetGeoTransform(geoTransform);
	pDstDS->SetProjection(pSrcDS->GetProjectionRef());
	pDstDS->SetGeoTransform(geoTransform);

	GDALRasterBandH pDstBand = (GDALRasterBandH)pDstDS->GetRasterBand(1);
	sieve(hSrcBand, NULL, pDstBand, minPixelCount, 4);

	GDALClose(pSrcDS);
	GDALClose(pDstDS);
}

bool mapbuilding::deleteTwig(const char* pszImagePath, double resolution) {
	GDALDataset* pDataSet = (GDALDataset*)GDALOpen(pszImagePath, GA_Update);

	if (pDataSet == NULL) {
		return false;
	}

	double adfGeoTransform[6] = { 0 };
	pDataSet->GetGeoTransform(adfGeoTransform);
	//int x = (120548.024 - adfGeoTransform[0]) / adfGeoTransform[1];
	//int y = (2504562.511 - adfGeoTransform[3]) / adfGeoTransform[5];

	int width = pDataSet->GetRasterXSize();
	int height = pDataSet->GetRasterYSize();

	unsigned char* pPixelValue = (unsigned char*)malloc(sizeof(unsigned char) * width * height);

	if (pPixelValue == NULL) {
		GDALClose(pDataSet);
		char szMsg[512] = { 0 };
		sprintf(szMsg, "deleteTwig: memory allocation failed,the size is %d x %d", width, height);
		logUtil::log(szMsg);
		return false;
	}

	memset(pPixelValue, 0, width * height);
	pDataSet->RasterIO(GF_Read, 0, 0, width, height, pPixelValue, width, height, GDT_Byte, 1, NULL, 0, 0, 0);

	vector<string> needling_pixel;

	for (int j = 10; j < height - 10; j++) {
		for (int i = 10; i < width - 10; i++) {
			if (pPixelValue[i + j * width] == 255) {
				set<string> label;
				set<string> diff;
				string pre_location = "";
				get_eight_neighborhood(pPixelValue, width, i, j, label);
				if (label.size() == 1) {
					char szLocation[512] = { 0 };
					sprintf(szLocation, "%d,%d", i, j);

					Coordinate pt;
					utm_coordinate(adfGeoTransform, i, j, pt, resolution);
					CoordinateArraySequence* cs = new CoordinateArraySequence();
					cs->add(pt);

					vector<string> temp_needling;
					temp_needling.emplace_back(szLocation);

					int cur_i = 0;
					int cur_j = 0;
					pre_location = szLocation;
					sscanf((*label.begin()).c_str(), "%d,%d", &cur_i, &cur_j);

					while (true) {
						set<string> label1;
						set<string> diff1;
						get_eight_neighborhood(pPixelValue, width, cur_i, cur_j, label1);
						if (label1.count(pre_location) == 1) {
							label1.erase(label1.find(pre_location));
						}

						Coordinate tempPoint;
						utm_coordinate(adfGeoTransform, cur_i, cur_j, tempPoint, resolution);

						char szLocation[512] = { 0 };
						sprintf(szLocation, "%d,%d", cur_i, cur_j);

						if (label1.size() == 1) {
							sscanf((*label1.begin()).c_str(), "%d,%d", &cur_i, &cur_j);
							cs->add(tempPoint);
							temp_needling.emplace_back(szLocation);
						} else if (label1.size() >= 2) {
							if (is_vertex(pPixelValue, width, cur_i, cur_j)) {
								if (allow_delete(label1)) {
									cs->add(tempPoint);
									temp_needling.emplace_back(szLocation);
								}

								if (Length::ofLine(cs) < 25) {
									needling_pixel.insert(needling_pixel.end(), temp_needling.begin(), temp_needling.end());
								}
								break;
							} else {
								cs->add(tempPoint);
								temp_needling.emplace_back(szLocation);
								auto iter = label1.begin();
								for (; iter != label1.end(); ++iter) {
									int x = 0;
									int y = 0;
									string location = *iter;
									sscanf(location.c_str(), "%d,%d", &x, &y);
									if (is_vertex(pPixelValue, width, x, y)) {
										cur_i = x;
										cur_j = y;
										break;
									}
								}
							}
						} else {
							break;
						}

						pre_location = szLocation;
					}
					delete cs;
				}

			}
		}
	}
	for (size_t i = 0; i < needling_pixel.size(); i++) {
		int x = 0;
		int y = 0;
		sscanf(needling_pixel[i].c_str(), "%d,%d", &x, &y);
		pPixelValue[x + y * width] = 0;
		Coordinate tempPoint;
		utm_coordinate(adfGeoTransform, x, y, tempPoint, resolution);
	}
	pDataSet->RasterIO(GF_Write, 0, 0, width, height, pPixelValue, width, height, GDT_Byte, 1, 0, 0, 0, 0);
	free(pPixelValue);
	GDALClose(pDataSet);

	return true;
}

void mapbuilding::get_eight_neighborhood(unsigned char* pPixelValue, int width, int x, int y, set<string>& label) {
	if (pPixelValue == NULL) {
		return;
	}
	for (int i = x - 1; i <= x + 1; i++) {
		for (int j = y - 1; j <= y + 1; j++) {
			if (i == x && j == y) {
				continue;
			}

			if (pPixelValue[i + j * width] == 255) {
				char szLocation[512] = { 0 };
				sprintf(szLocation, "%d,%d", i, j);
				label.emplace(szLocation);
			}
		}
	}
}

void mapbuilding::utm_coordinate(double* geoTransform, int i, int j, Coordinate& point, double resolution) {
	point.x = geoTransform[0] + i * geoTransform[1] + j * geoTransform[2] + resolution / 2;
	point.y = geoTransform[3] + i * geoTransform[4] + j * geoTransform[5] - resolution / 2;
}

bool mapbuilding::is_vertex(unsigned char* pPixelValue, int nWidth, int x, int y) {
	set<string> label;
	get_eight_neighborhood(pPixelValue, nWidth, x, y, label);
	return !have_neighborhood(label);
}

bool mapbuilding::allow_delete(set<string>& diff) {
	bool res = false;
	vector<string> vec_diff;

	std::copy(diff.begin(), diff.end(), std::back_inserter(vec_diff));
	for (size_t i = 0; i < vec_diff.size(); i++) {
		int x1 = 0;
		int y1 = 0;
		sscanf(vec_diff[i].c_str(), "%d,%d", &x1, &y1);
		bool res1 = false;
		for (size_t j = 0; j < vec_diff.size(); j++) {
			if (i != j) {
				int x2 = 0;
				int y2 = 0;
				sscanf(vec_diff[j].c_str(), "%d,%d", &x2, &y2);
				if ((x1 == x2 && abs(y1 - y2) == 1) || (y1 == y2 && abs(x1 - x2) == 1) || abs(y1 - y2) == 1 && abs(x1 - x2) == 1) {
					res1 = true;
					break;
				}
			}
		}
		if (res1) {
			res = true;
			break;
		}
	}

	return res;
}

bool mapbuilding::have_neighborhood(set<string>& diff) {
	bool res = false;
	vector<string> vec_diff;

	std::copy(diff.begin(), diff.end(), std::back_inserter(vec_diff));
	for (size_t i = 0; i < vec_diff.size(); i++) {
		int x1 = 0;
		int y1 = 0;
		sscanf(vec_diff[i].c_str(), "%d,%d", &x1, &y1);
		bool res1 = false;
		for (size_t j = 0; j < vec_diff.size(); j++) {
			if (i != j) {
				int x2 = 0;
				int y2 = 0;
				sscanf(vec_diff[j].c_str(), "%d,%d", &x2, &y2);
				if ((x1 == x2 && abs(y1 - y2) == 1) || (y1 == y2 && abs(x1 - x2) == 1)) {
					res1 = true;
					break;
				}
			}
		}
		if (res1) {
			res = true;
			break;
		}
	}

	return res;
}

bool mapbuilding::vectorization(const char* pszImagePath, const char* pstShpPath, double resolution) {
	GDALDataset* pDataSet = (GDALDataset*)GDALOpen(pszImagePath, GA_ReadOnly);

	if (pDataSet == NULL) {
		return false;
	}

	double adfGeoTransform[6] = { 0 };
	pDataSet->GetGeoTransform(adfGeoTransform);

	int width = pDataSet->GetRasterXSize();
	int height = pDataSet->GetRasterYSize();

	unsigned char* pPixelValue = (unsigned char*)malloc(sizeof(unsigned char) * width * height);

	if (pPixelValue == NULL) {
		GDALClose(pDataSet);
		char szMsg[512] = { 0 };
		sprintf(szMsg, "vectorization: memory allocation failed,the size is %d x %d", width, height);
		logUtil::log(szMsg);
		return false;
	}

	memset(pPixelValue, 0, width * height);
	pDataSet->RasterIO(GF_Read, 0, 0, width, height, pPixelValue, width, height, GDT_Byte, 1, NULL, 0, 0, 0);

	FILE* fp = fopen(pstShpPath, "w");
	vector<CoordinateArraySequence*> lines;
	set<string> marked;
	set<string> end_vertex;
	set<string> marked_lines;
	int count = 0;

	for (int j = 10; j < height - 10; j++) {
		for (int i = 10; i < width - 10; i++) {
			if (pPixelValue[i + j * width] == 255) {
				set<string> label;
				get_eight_neighborhood(pPixelValue, width, i, j, label);
				char szLocation[512] = { 0 };
				sprintf(szLocation, "%d,%d", i, j);
				if (label.size() >= 3 && is_vertex(pPixelValue, width, i, j)) {
					string pre_location = szLocation;
					string base_location = pre_location;

					Coordinate pt;
					utm_coordinate(adfGeoTransform, i, j, pt,resolution);
					//printf("%f,%f,%d\n",pt.x,pt.y,count++);

					auto iter = label.begin();
					for (; iter != label.end(); ++iter) {
						if (marked.count(*iter) == 1) {
							continue;
						}

						CoordinateArraySequence* cs = new CoordinateArraySequence();
						cs->add(pt);

						int cur_i = 0;
						int cur_j = 0;
						sscanf((*iter).c_str(), "%d,%d", &cur_i, &cur_j);

						int line_nums = 0;
						while (true) {
							line_nums++;
							set<string> label1;
							get_eight_neighborhood(pPixelValue, width, cur_i, cur_j, label1);
							if (label1.count(pre_location) == 1) {
								label1.erase(label1.find(pre_location));
							}

							Coordinate tempPoint;
							utm_coordinate(adfGeoTransform, cur_i, cur_j, tempPoint, resolution);

							char szLocation[512] = { 0 };
							sprintf(szLocation, "%d,%d", cur_i, cur_j);

							if (line_nums == 1) {
								auto iter = label.find(szLocation);
								if (iter != label.end()) {
									set<string> temp = label;
									temp.erase(temp.find(szLocation));

									set<string> difference;
									std::set_difference(label1.begin(), label1.end(), temp.begin(), temp.end(), inserter(difference, difference.begin()));
									label1 = difference;
								}
							}

							if (label1.size() == 0) {
								end_vertex.emplace(szLocation);
								cs->add(tempPoint);
								lines.emplace_back(cs);
								pre_location = base_location;
								break;
							} else if (label1.size() == 1) {
								cs->add(tempPoint);
								pre_location = szLocation;
								sscanf((*label1.begin()).c_str(), "%d,%d", &cur_i, &cur_j);
							} else if (label1.size() == 2) {
								if (is_vertex(pPixelValue, width, cur_i, cur_j)) {
									cs->add(tempPoint);
									lines.emplace_back(cs);
									marked.emplace(pre_location);
									pre_location = base_location;
									break;
								} else {
									bool b1 = false;
									bool b2 = false;
									int x1 = 0;
									int y1 = 0;
									int x2 = 0;
									int y2 = 0;
									{
										string location = *label1.begin();
										sscanf(location.c_str(), "%d,%d", &x1, &y1);
										b1 = is_vertex(pPixelValue, width, x1, y1);
									}

									{
										string location = *(++label1.begin());
										sscanf(location.c_str(), "%d,%d", &x2, &y2);
										b2 = is_vertex(pPixelValue, width, x2, y2);
									}

									if (b1 && !b2) {
										cur_i = x1;
										cur_j = y1;
									} else {
										cur_i = x2;
										cur_j = y2;
									}

									cs->add(tempPoint);
									pre_location = szLocation;
								}
							} else {
								break;
							}
						}
					}
				}

				if (label.size() == 1) {
					Coordinate pt1;
					utm_coordinate(adfGeoTransform, i, j, pt1, resolution);
					marked_lines.emplace(szLocation);
				}
			}
		}
	}

	set<string> start_vertex;
	set<string> line_end_vertex;
	std::set_difference(marked_lines.begin(), marked_lines.end(), end_vertex.begin(), end_vertex.end(), inserter(start_vertex, start_vertex.begin()));
	auto iter = start_vertex.begin();
	for (; iter != start_vertex.end(); ++iter) {
		if (line_end_vertex.count(*iter) == 1) {
			continue;
		}
		int cur_i = 0;
		int cur_j = 0;
		sscanf((*iter).c_str(), "%d,%d", &cur_i, &cur_j);

		Coordinate pt;
		utm_coordinate(adfGeoTransform, cur_i, cur_j, pt, resolution);

		CoordinateArraySequence* cs = new CoordinateArraySequence();
		string pre_location = *iter;

		while (true) {
			set<string> label1;
			get_eight_neighborhood(pPixelValue, width, cur_i, cur_j, label1);
			if (label1.count(pre_location) == 1) {
				label1.erase(label1.find(pre_location));
			}

			Coordinate tempPoint;
			utm_coordinate(adfGeoTransform, cur_i, cur_j, tempPoint, resolution);

			char szLocation[512] = { 0 };
			sprintf(szLocation, "%d,%d", cur_i, cur_j);

			if (label1.size() == 0) {
				line_end_vertex.emplace(szLocation);
				cs->add(tempPoint);
				lines.emplace_back(cs);
				break;
			} else if (label1.size() == 1) {
				cs->add(tempPoint);
				pre_location = szLocation;
				sscanf((*label1.begin()).c_str(), "%d,%d", &cur_i, &cur_j);
			} else {
				line_end_vertex.emplace(szLocation);
				cs->add(tempPoint);
				lines.emplace_back(cs);
				break;
			}
		}
	}

	for (size_t i = 0; i < lines.size(); i++) {
		string wkt = "LINESTRING (";
		for (size_t j = 0; j < lines[i]->getSize(); j++) {
			char szCoordinate[512] = { 0 };
			sprintf(szCoordinate, "%f %f,", lines[i]->getX(j), lines[i]->getY(j));
			wkt += szCoordinate;
		}
		wkt = wkt.substr(0, wkt.length() - 1);
		wkt += ")";
		fprintf(fp, "%s\n", wkt.c_str());
		delete lines[i];
	}

	fclose(fp);
	free(pPixelValue);
	GDALClose(pDataSet);
	return true;
}

void mapbuilding::interpolator(const char* pszSrcPath, const char* pszDstPath,int zone) {
	if (pszSrcPath == NULL || pszDstPath == NULL) {
		return;
	}

	ifstream file(pszSrcPath);
	if (!file.is_open()) {
		return;
	}
	interpolation* obj = new interpolation();

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);

	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* ds = driver->Create(pszDstPath, 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* layer = ds->CreateLayer("detection", &spatialRef, wkbLineString, NULL);
	OGRFieldDefn field("id", OFTInteger);
	layer->CreateField(&field);

	string basePath = CPLGetPath(pszDstPath);
	basePath += "/kappa.shp";
	GDALDataset* ds_kappa = driver->Create(basePath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* layer_kappa = ds_kappa->CreateLayer("kappa", &spatialRef, wkbPoint, NULL);
	OGRFieldDefn field_curvature("curvature", OFTReal);
	layer_kappa->CreateField(&field_curvature);

	string scanLine;
	while (getline(file, scanLine)) {
		vector<coordinate_ex*> coordinate_list;
		obj->multivariateInterpolation(scanLine, coordinate_list);
		OGRLineString* line = geometryProjection(coordinate_list, zone);
		if (line != NULL) {
			for (size_t i = 0; i < line->getNumPoints(); i++) {
				OGRPoint* point = (OGRPoint*)OGRGeometryFactory::createGeometry(wkbPoint);
				point->setX(line->getX(i));
				point->setY(line->getY(i));
				OGRFeature* feature_kappa = OGRFeature::CreateFeature(layer_kappa->GetLayerDefn());
				feature_kappa->SetGeometryDirectly(point);
				feature_kappa->SetField("curvature", coordinate_list[i]->curvature);
				layer_kappa->CreateFeature(feature_kappa);
				OGRFeature::DestroyFeature(feature_kappa);
			}
			OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
			feature->SetGeometryDirectly(line);
			layer->CreateFeature(feature);
			OGRFeature::DestroyFeature(feature);
		}
		for (size_t i = 0; i < coordinate_list.size(); i++) {
			delete coordinate_list[i];
		}
	}
	delete obj;
	GDALClose(ds_kappa);
	GDALClose(ds);
	file.close();
}

int mapbuilding::reproject(const char* pszSrcPath, const char* pszDstPath) {
	if (pszSrcPath == NULL ||pszDstPath == NULL) {
		return -1;
	}
	GDALDataset* pDataset = static_cast<GDALDataset*>(GDALOpenEx(pszSrcPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL));
	if (pDataset == NULL) {
		return -1;
	}
	OGRLayer* pLayer = pDataset->GetLayer(0);
	OGRFeatureDefn* layerDefn = pLayer->GetLayerDefn();

	OGREnvelope extent;
	pLayer->GetExtent(&extent);

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(32600 + m_zone);

	GDALDriver* pDriver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* pDstDS = pDriver->Create(pszDstPath, 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* pDstLayer = pDstDS->CreateLayer("detection", &spatialRef, wkbLineString, NULL);
	for (int i = 0; i < layerDefn->GetFieldCount(); i++) {
		pDstLayer->CreateField(layerDefn->GetFieldDefn(i));
	}

	pLayer->SetAttributeFilter("angle > 170");
	OGRFeature* pFeature = NULL;
	while ((pFeature = pLayer->GetNextFeature()) != NULL) {
		OGRLineString* line = (OGRLineString*)pFeature->GetGeometryRef();
		if (line != NULL) {
			OGRLineString* newLine = geometryProjection(line, m_zone);
			if (newLine != NULL) {
				OGRFeature* pNewFeature = OGRFeature::CreateFeature(pDstLayer->GetLayerDefn());
				pNewFeature->SetGeometryDirectly(newLine);
				pDstLayer->CreateFeature(pNewFeature);
				OGRFeature::DestroyFeature(pNewFeature);
			}
		}

		OGRFeature::DestroyFeature(pFeature);
	}
	GDALClose(pDstDS);
	GDALClose(pDataset);
	return 0;
}

OGRLineString* mapbuilding::geometryProjection(OGRLineString* line, int zone) {
	int pointNums = line->getNumPoints();
	if (pointNums < 2) {
		return NULL;
	}
	double* x_array = new double[pointNums];
	double* y_array = new double[pointNums];
	for (int i = 0; i < pointNums; i++) {
		x_array[i] = line->getX(i) * DEG_TO_RAD;
		y_array[i] = line->getY(i) * DEG_TO_RAD;
	}

	//int zone = int(31 + (line->getX(0) / 6.0));
	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pj_transform(pj_latlong, pj_utm, pointNums, 1, x_array, y_array, NULL);

	OGRLineString* project_line = (OGRLineString*)OGRGeometryFactory::createGeometry(wkbLineString);
	for (size_t i = 0; i < pointNums; i++) {
		project_line->addPoint(x_array[i], y_array[i]);
	}

	delete[]x_array;
	delete[]y_array;
	pj_free(pj_latlong);
	pj_free(pj_utm);
	return project_line;
}

LineString* mapbuilding::geometryProjectionGeos(OGRLineString* line, int zone) {
	int pointNums = line->getNumPoints();
	if (pointNums < 2) {
		return NULL;
	}
	double* x_array = new double[pointNums];
	double* y_array = new double[pointNums];
	for (int i = 0; i < pointNums; i++) {
		x_array[i] = line->getX(i) * DEG_TO_RAD;
		y_array[i] = line->getY(i) * DEG_TO_RAD;
	}

	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pj_transform(pj_latlong, pj_utm, pointNums, 1, x_array, y_array, NULL);

	CoordinateArraySequence* cs = new CoordinateArraySequence();
	for (size_t i = 0; i < pointNums; i++) {
		cs->add(Coordinate(x_array[i], y_array[i]), false);
	}

	delete[]x_array;
	delete[]y_array;
	pj_free(pj_latlong);
	pj_free(pj_utm);
	return m_factory->createLineString(cs);
}

OGRLineString* mapbuilding::geometryProjection(string& wkt, int zone) {
	if (wkt == "") {
		return NULL;
	}
	OGRLineString* newLine = NULL;
	OGRGeometry* geom;
	OGRGeometryFactory::createFromWkt(wkt.c_str(),NULL,&geom);
	if (geom != NULL) {
		OGRLineString* line = (OGRLineString*)geom;
		int pointNums = line->getNumPoints();
		double* x_array = new double[pointNums];
		double* y_array = new double[pointNums];

		for (int i = 0; i < pointNums; i++) {
			x_array[i] = line->getX(i);
			y_array[i] = line->getY(i);
		}

		char szProj[512] = { 0 };
		sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

		projPJ pj_utm, pj_latlong;
		pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
		pj_utm = pj_init_plus(szProj);

		pj_transform(pj_utm, pj_latlong, pointNums, 1, x_array, y_array, NULL);

		newLine = (OGRLineString*)OGRGeometryFactory::createGeometry(wkbLineString);
		for (size_t i = 0; i < pointNums; i++) {
			newLine->addPoint(x_array[i] * RAD_TO_DEG, y_array[i] * RAD_TO_DEG);
		}

		delete[]x_array;
		delete[]y_array;
		pj_free(pj_latlong);
		pj_free(pj_utm);
	}
	return newLine;
}

OGRGeometry* mapbuilding::geometryProjectionGeos(Geometry* geosGeom, int zone) {
	if (geosGeom == NULL) {
		return NULL;
	}
	OGRGeometry* resGeom = NULL;

	GeometryTypeId geomType = geosGeom->getGeometryTypeId();
	double* x_array = NULL;
	double* y_array = NULL;

	std::unique_ptr<CoordinateSequence> cs = geosGeom->getCoordinates();
	int pointNums = cs->getSize();
	x_array = new double [pointNums];
	y_array = new double [pointNums];

	for (int i = 0; i < pointNums; i++) {
		x_array[i] = cs->getX(i);
		y_array[i] = cs->getY(i);
	}

	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pj_transform(pj_utm, pj_latlong, pointNums, 1, x_array, y_array, NULL);

	switch (geomType) {
		case geos::geom::GEOS_POINT: {
			OGRPoint* point = (OGRPoint*)OGRGeometryFactory::createGeometry(wkbPoint);
			point->setX(x_array[0] * RAD_TO_DEG);
			point->setY(y_array[0] * RAD_TO_DEG);
			resGeom = point;
		}
			break;
		case geos::geom::GEOS_LINESTRING: {
			OGRLineString* line = (OGRLineString*)OGRGeometryFactory::createGeometry(wkbLineString);
			for (size_t i = 0; i < pointNums; i++) {
				line->addPoint(x_array[i] * RAD_TO_DEG, y_array[i] * RAD_TO_DEG);
			}
			resGeom = line;
		}
			break;
		case geos::geom::GEOS_POLYGON: {
			OGRPolygon* polygon = (OGRPolygon*)OGRGeometryFactory::createGeometry(wkbPolygon);
			OGRLinearRing* ring = (OGRLinearRing*)OGRGeometryFactory::createGeometry(wkbLinearRing);
			for (size_t i = 0; i < pointNums; i++) {
				ring->addPoint(x_array[i] * RAD_TO_DEG, y_array[i] * RAD_TO_DEG);
			}
			polygon->addRingDirectly(ring);
			resGeom = polygon;
		}
			break;
		default:
			break;
	}

	delete[]x_array;
	delete[]y_array;
	pj_free(pj_latlong);
	pj_free(pj_utm);

	return resGeom;
}

OGRLineString* mapbuilding::geometryProjection(vector<coordinate_ex*>& coordinate_list, int zone) {
	if (coordinate_list.size() < 2) {
		return NULL;
	}
	
	OGRLineString* newLine = NULL;

	int pointNums = (int)coordinate_list.size();
	double* x_array = new double[pointNums];
	double* y_array = new double[pointNums];

	for (int i = 0; i < pointNums; i++) {
		x_array[i] = coordinate_list[i]->x;
		y_array[i] = coordinate_list[i]->y;
	}

	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pj_transform(pj_utm, pj_latlong, pointNums, 1, x_array, y_array, NULL);

	newLine = (OGRLineString*)OGRGeometryFactory::createGeometry(wkbLineString);
	for (size_t i = 0; i < pointNums; i++) {
		newLine->addPoint(x_array[i] * RAD_TO_DEG, y_array[i] * RAD_TO_DEG);
	}

	delete[]x_array;
	delete[]y_array;
	pj_free(pj_latlong);
	pj_free(pj_utm);

	return newLine;
}

bool mapbuilding::erosion(void* pPixelValue, unsigned long lHeight, unsigned long nWidth) {
	char* szF = NULL;
	char* szG = NULL;

	char szN[10] = { 0 };
	char szA[5] = { 0, -1, 1, 0, 0 };
	char szB[5] = { 0, 0, 0, 1, -1 };

	char szNrnd, szCond, szN48, szN26, szN24, szN46, szN68, szN82, szN123, szN345, szN567, szN781;
	short k, shori;

	unsigned long i, j;

	long ii, jj, kk, kk1, kk2, kk3, size;

	size = (long)lHeight * (long)nWidth;
	szG = (char*)malloc(size);

	if (szG == NULL) {
		return false;
	}

	szF = (char*)pPixelValue;

	for (kk = 0l; kk < size; kk++) {
		szG[kk] = szF[kk];
	}

	do {
		shori = 0;
		for (k = 1; k <= 4; k++) {
			for (i = 1; i < lHeight - 1; i++) {
				ii = i + szA[k];

				for (j = 1; j < nWidth - 1; j++) {
					kk = i * nWidth + j;

					if (!szF[kk])
						continue;

					jj = j + szB[k];
					kk1 = ii * nWidth + jj;

					if (szF[kk1])
						continue;

					kk1 = kk - nWidth - 1;
					kk2 = kk1 + 1;
					kk3 = kk2 + 1;
					szN[3] = szF[kk1];
					szN[2] = szF[kk2];
					szN[1] = szF[kk3];
					kk1 = kk - 1;
					kk3 = kk + 1;
					szN[4] = szF[kk1];
					szN[8] = szF[kk3];
					kk1 = kk + nWidth - 1;
					kk2 = kk1 + 1;
					kk3 = kk2 + 1;
					szN[5] = szF[kk1];
					szN[6] = szF[kk2];
					szN[7] = szF[kk3];

					szNrnd = szN[1] + szN[2] + szN[3] + szN[4] + szN[5] + szN[6] + szN[7] + szN[8];
					if (szNrnd <= 1)
						continue;

					szCond = 0;
					szN48 = szN[4] + szN[8];
					szN26 = szN[2] + szN[6];
					szN24 = szN[2] + szN[4];
					szN46 = szN[4] + szN[6];
					szN68 = szN[6] + szN[8];
					szN82 = szN[8] + szN[2];
					szN123 = szN[1] + szN[2] + szN[3];
					szN345 = szN[3] + szN[4] + szN[5];
					szN567 = szN[5] + szN[6] + szN[7];
					szN781 = szN[7] + szN[8] + szN[1];

					if (szN[2] == 1 && szN48 == 0 && szN567 > 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[6] == 1 && szN48 == 0 && szN123 > 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[8] == 1 && szN26 == 0 && szN345 > 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[4] == 1 && szN26 == 0 && szN781 > 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[5] == 1 && szN46 == 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[7] == 1 && szN68 == 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[1] == 1 && szN82 == 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					if (szN[3] == 1 && szN24 == 0) {
						if (!szCond)
							continue;
						szG[kk] = 0;
						shori = 1;
						continue;
					}

					szCond = 1;
					if (!szCond)
						continue;
					szG[kk] = 0;
					shori = 1;
				}
			}

			for (i = 0; i < lHeight; i++) {
				for (j = 0; j < nWidth; j++) {
					kk = i * nWidth + j;
					szF[kk] = szG[kk];
				}
			}
		}
	} while (shori);

	free(szG);

	return true;
}
bool mapbuilding::skeleton(const char* pszSrcImagePath, const char* pszDstImagePath) {
	if (pszSrcImagePath == NULL || pszDstImagePath == NULL) {
		return false;
	}
	GDALDataset* pSrcDataSet = (GDALDataset*)GDALOpen(pszSrcImagePath, GA_ReadOnly);
	if (pSrcDataSet == NULL) {
		return false;
	}

	int width = pSrcDataSet->GetRasterXSize();
	int height = pSrcDataSet->GetRasterYSize();

	unsigned char* pPixelValue = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	if (pPixelValue == NULL) {
		GDALClose(pSrcDataSet);
		char szMsg[512] = { 0 };
		sprintf(szMsg, "skeleton: memory allocation failed,the size is %d x %d", width, height);
		logUtil::log(szMsg);
		return false;
	}

	memset(pPixelValue, 0, width * height);

	pSrcDataSet->RasterIO(GF_Read, 0, 0, width, height, pPixelValue, width, height, GDT_Byte, 1, NULL, 0, 0, 0);

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (pPixelValue[i + j * width] != 0) {
				pPixelValue[i + j * width] = 1;
			}
		}
	}

	erosion(pPixelValue, height, width);

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			if (pPixelValue[i + j * width] == 1) {
				pPixelValue[i + j * width] = 255;
			} else {
				pPixelValue[i + j * width] = 0;
			}
		}
	}

	GDALDriver* pDriver = GetGDALDriverManager()->GetDriverByName("HFA");
	char** papszOptions = NULL;
	GDALDataset* pDstDataSet = pDriver->Create(pszDstImagePath, width, height, 1, GDT_Byte, papszOptions);

	double adfGeoTransform[6] = { 0 };

	pSrcDataSet->GetGeoTransform(adfGeoTransform);
	pDstDataSet->SetProjection(pSrcDataSet->GetProjectionRef());
	pDstDataSet->SetGeoTransform(adfGeoTransform);

	pDstDataSet->RasterIO(GF_Write, 0, 0, width, height, pPixelValue, width, height, GDT_Byte, 1, 0, 0, 0, 0);

	free(pPixelValue);

	GDALClose(pSrcDataSet);
	GDALClose(pDstDataSet);

	return true;
}
void mapbuilding::parse_mmt(const char* pszOriginPath, const char* pszDir) {
	if (pszDir == NULL) {
		return;
	}

	string baseName = CPLGetBasename(pszDir);
	m_zone = atoi(baseName.c_str());
	string basePath = CPLGetPath(pszDir);
	basePath += "/";
	basePath += baseName;
	basePath += "_output";
	
	VSIRmdir(basePath.c_str());

	char** papszRetList = VSIReadDir(pszDir);
	int directory_count = CSLCount(papszRetList);

	char szMsg[512] = { 0 };
	sprintf(szMsg, "directory: %s the number of folders is:%d", pszDir, directory_count - 2);
	logUtil::log(szMsg);

	if (directory_count - 2 == 0) {
		CSLDestroy(papszRetList);
		logUtil::log(szMsg);
		return;
	}

	VSIMkdir(basePath.c_str(), 0755);

	string laneMarkingPath = basePath;
	laneMarkingPath += "/lanemarking.shp";

	string roadBoundaryPath = basePath;
	roadBoundaryPath += "/roadboundary.shp";

	string stoplinePath = basePath;
	stoplinePath += "/stopline.shp";

	string crosswalkPath = basePath;
	crosswalkPath += "/crosswalk.shp";

	string roadmarkPath = basePath;
	roadmarkPath += "/roadmark.shp";

	string aoisPath = basePath;
	aoisPath += "/aois.shp";

	string laneCenterPath = basePath;
	laneCenterPath += "/lanecenter.shp";

	GDALDriver::QuietDelete(laneMarkingPath.c_str());
	GDALDriver::QuietDelete(roadBoundaryPath.c_str());
	GDALDriver::QuietDelete(stoplinePath.c_str());
	GDALDriver::QuietDelete(crosswalkPath.c_str());
	GDALDriver::QuietDelete(roadmarkPath.c_str());
	GDALDriver::QuietDelete(aoisPath.c_str());
	GDALDriver::QuietDelete(laneCenterPath.c_str());

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);

	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* lanemarking_ds = driver->Create(laneMarkingPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* lanemarking_layer = lanemarking_ds->CreateLayer("lanemarking", &spatialRef, wkbLineString, NULL);

	GDALDataset* roadboundary_ds = driver->Create(roadBoundaryPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* roadboundary_layer = roadboundary_ds->CreateLayer("roadboundary", &spatialRef, wkbLineString, NULL);

	GDALDataset* stopline_ds = driver->Create(stoplinePath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* stopline_layer = stopline_ds->CreateLayer("stopline", &spatialRef, wkbLineString, NULL);

	GDALDataset* crosswalk_ds = driver->Create(crosswalkPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* crosswalk_layer = crosswalk_ds->CreateLayer("crosswalk", &spatialRef, wkbLineString, NULL);

	GDALDataset* roadmark_ds = driver->Create(roadmarkPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* roadmark_layer = roadmark_ds->CreateLayer("roadmark", &spatialRef, wkbLineString, NULL);

	GDALDataset* aois_ds = driver->Create(aoisPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* aois_layer = aois_ds->CreateLayer("aois", &spatialRef, wkbLineString, NULL);

	GDALDataset* lanecenter_ds = driver->Create(laneCenterPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* lanecenter_layer = lanecenter_ds->CreateLayer("lanecenter", &spatialRef, wkbLineString, NULL);

	vector<OGRLayer*> layers = { lanemarking_layer,roadboundary_layer,stopline_layer,crosswalk_layer,roadmark_layer,aois_layer,lanecenter_layer};

	OGRFieldDefn field("id", OFTString);
	OGRFieldDefn ts_field("ts", OFTInteger64);
	OGRFieldDefn conf_field("conf", OFTReal);
	OGRFieldDefn angle_field("angle", OFTReal);

	for (size_t i = 0; i < layers.size(); i++) {
		layers[i]->CreateField(&field);
		layers[i]->CreateField(&ts_field);
		layers[i]->CreateField(&conf_field);
		layers[i]->CreateField(&angle_field);
	}

	Envelope env(108.66694972, 118.82836862, 18.78853013, 27.51248001);

	int recordNums = 0;
	for (size_t i = 0; i < directory_count; i++) {
		if (strcmp(papszRetList[i],".") == 0 || strcmp(papszRetList[i], "..") == 0) {
			continue;
		}

		string strLandmark = pszOriginPath;
		strLandmark += "/";
		strLandmark += papszRetList[i];
		strLandmark += "/bev_features/_ddld_landmark.json";

		string strEgopose = pszDir;
		strEgopose += "/";
		strEgopose += papszRetList[i];
		strEgopose += "/_mla_egopose.json";

		string strRtk = pszOriginPath;
		strRtk += "/";
		strRtk += papszRetList[i];
		strRtk += "/position/_sensor_gnss_rtk.json";

		map<GInt64, vector<CoordinateArraySequence*>> lanemarkings;
		map<GInt64, vector<double>> egopose;
		map<GInt64, int> gnss_rtk;

		vector<map<GInt64, vector<CoordinateArraySequence*>>> features;
		read_ddld_landmark(strLandmark.c_str(), features);
		read_mla_egopose(strEgopose.c_str(), egopose);
		read_sensor_gnss_rtk(strRtk.c_str(), gnss_rtk);
		fixingEgopose(gnss_rtk, egopose);

		for (size_t n = 0; n < features.size(); n++) {
			double buffer_radius = 0;
			if (n == 0 || n == 6) {
				buffer_radius = 13;
			} else if (n == 1) {
				buffer_radius = 10;
			}

			auto rem_iter = egopose.begin();
			for (; rem_iter != egopose.end();) {
				if (features[n].find(rem_iter->first) == features[n].end()) {
					rem_iter = egopose.erase(rem_iter);
				} else {
					++rem_iter;
				}
			}

			int index = 0;
			Coordinate pre;
			double length = 0;
			map<GInt64, vector<double>> tempegopose;
			rem_iter = egopose.begin();
			for (; rem_iter != egopose.end(); ++rem_iter) {
				double lon = rem_iter->second[0];
				double lat = rem_iter->second[1];
				Coordinate pt(lon, lat);
				if (!env.contains(pt)) {
					continue;
				}

				geometryProjection(pt, m_zone);

				if (++index == 1) {
					tempegopose[rem_iter->first] = rem_iter->second;
				} else {
					length += pt.distance(pre);
					if (length > 8) {
						tempegopose[rem_iter->first] = rem_iter->second;
						length = 0;
					}
				}
				pre = pt;
			}

			egopose = tempegopose;
			auto iter = egopose.begin();
			for (; iter != egopose.end(); ++iter) {
				auto find_iter = features[n].find(iter->first);
				if (find_iter != features[n].end()) {
					double lon = iter->second[0];
					double lat = iter->second[1];
					double w = iter->second[2];
					double x = iter->second[3];
					double y = iter->second[4];
					double z = iter->second[5];

					Coordinate pt(lon, lat);
					geometryProjection(pt, m_zone);

					std::unique_ptr<geos::geom::Point> point(m_factory->createPoint(pt));
					std::unique_ptr<Geometry> bufferGeom = point->buffer(buffer_radius);

					Eigen::Quaterniond q(w, x, y, z);
					MatrixXd rotation = q.toRotationMatrix();
					for (int j = 0; j < find_iter->second.size(); j++) {
						if (find_iter->second[j]->size() > 1) {
							CoordinateArraySequence* cs = new CoordinateArraySequence();
							for (int m = 0; m < find_iter->second[j]->size(); m++) {
								double x1 = find_iter->second[j]->getAt(m).x;
								double y1 = find_iter->second[j]->getAt(m).y;

								VectorXd A(3);
								VectorXd B(3);
								A(0) = x1;
								A(1) = y1;
								A(2) = 0;
								B(0) = pt.x;
								B(1) = pt.y;
								B(2) = 0;

								double x0 = (rotation * A + B)(0);
								double y0 = (rotation * A + B)(1);
								cs->add(Coordinate(x0, y0), false);
							}
							if (cs->size() > 1) {
								LineString* line = m_factory->createLineString(cs);
								if (n == 0 || n == 1 || n == 6 ) {
									if (bufferGeom->intersects(line)) {
										std::unique_ptr<Geometry> intersection_geom = bufferGeom->intersection(line);
										if (intersection_geom->getGeometryTypeId() == GeometryTypeId::GEOS_LINESTRING) {
											auto coords = intersection_geom->getCoordinates();
											double distance1 = coords->back().distance(coords->front());
											double distance2 = Length::ofLine(coords.get());
											double conf = 0;
											if (distance2 != 0) {
												conf = distance1 / distance2;
											}
											double angle = minAngle(dynamic_cast<LineString*>(intersection_geom.get()));
											OGRLineString* new_line = (OGRLineString*)geometryProjectionGeos(intersection_geom.get(), m_zone);
											OGRFeature* feature = OGRFeature::CreateFeature(layers[n]->GetLayerDefn());
											feature->SetGeometryDirectly(new_line);
											feature->SetField("id", papszRetList[i]);
											feature->SetField("ts", iter->first);
											feature->SetField("conf", conf);
											feature->SetField("angle", angle);
											layers[n]->CreateFeature(feature);
											OGRFeature::DestroyFeature(feature);
											if (recordNums++ % 2000 == 0) {
												layers[n]->SyncToDisk();
											}
										}
									}
								} else {
									OGRLineString* new_line = (OGRLineString*)geometryProjectionGeos(line, m_zone);
									OGRFeature* feature = OGRFeature::CreateFeature(layers[n]->GetLayerDefn());
									feature->SetGeometryDirectly(new_line);
									feature->SetField("id", papszRetList[i]);
									feature->SetField("ts", iter->first);
									layers[n]->CreateFeature(feature);
									OGRFeature::DestroyFeature(feature);
									if (recordNums++ % 2000 == 0) {
										layers[n]->SyncToDisk();
									}
								}
								m_factory->destroyGeometry(line);
							} else {
								delete cs;
							}
						}
					}
				}
			}
		}

		for (size_t j = 0; j < features.size(); j++) {
			auto iter = features[j].begin();
			for (; iter != features[j].end(); ++iter) {
				for (size_t m = 0; m < iter->second.size(); m++) {
					delete iter->second[m];
				}
			}
		}
	}

	GDALClose(lanemarking_ds);
	GDALClose(roadboundary_ds);
	GDALClose(stopline_ds);
	GDALClose(crosswalk_ds);
	GDALClose(roadmark_ds);
	GDALClose(aois_ds);
	GDALClose(lanecenter_ds);
	CSLDestroy(papszRetList);

	getCrosswalk(crosswalkPath.c_str());
	getStopLine(stoplinePath.c_str());
	getLanemarking(laneMarkingPath.c_str(), "/lanemarking_res.shp");
	getLanemarking(roadBoundaryPath.c_str(), "/roadboundary_res.shp");
	getRoadMark(roadmarkPath.c_str());
	getAois(aoisPath.c_str());

	//string laneMarkingResPath = pszDstPath;
	//laneMarkingResPath += "/lanemarking_res.shp";

	//string roadmarkResPath = pszDstPath;
	//roadmarkResPath += "/roadmark_res.shp";

	//addLinesToOsm(laneMarkingResPath.c_str());
	//addPolygonsToOsm(roadmarkResPath.c_str());
	//addOtherOsm(pszDstPath);

	logUtil::log("end of parsing mmt files");
}
void mapbuilding::read_ddld_landmark(const char* pszPath, vector<map<GInt64, vector<CoordinateArraySequence*>>>& features/*map<GInt64, vector<CoordinateArraySequence*>>& lanemarkings*/) {
	if (pszPath == NULL) {
		return;
	}

	map<GInt64, vector<CoordinateArraySequence*>> lanemarkings;
	map<GInt64, vector<CoordinateArraySequence*>> road_boundaries;
	map<GInt64, vector<CoordinateArraySequence*>> stop_lines;
	map<GInt64, vector<CoordinateArraySequence*>> cross_walks;
	map<GInt64, vector<CoordinateArraySequence*>> road_marks;
	map<GInt64, vector<CoordinateArraySequence*>> aois;
	map<GInt64, vector<CoordinateArraySequence*>> lanecenter;
	
	json_object* root = json_object_from_file(pszPath);
	if (root != NULL) {
		size_t length = json_object_array_length(root);
		for (size_t i = 0; i < length; i++) {
			json_object* child = json_object_array_get_idx(root, i);
			json_object* meta = json_object_object_get(child, "meta");
			json_object* egopose_relevant_frame = json_object_object_get(meta, "egopose_relevant_frame");
			json_object* timestamp_us = json_object_object_get(egopose_relevant_frame, "timestamp_us");
			int64_t timestamp_us_v = json_object_get_int64(timestamp_us);

			json_object* landmarks = json_object_object_get(child, "landmarks");
			vector<CoordinateArraySequence*> lane_boundaries_lines;
			vector<CoordinateArraySequence*> road_boundaries_lines;
			vector<CoordinateArraySequence*> stop_lines_lines;
			vector<CoordinateArraySequence*> cross_walks_lines;
			vector<CoordinateArraySequence*> road_marks_lines;
			vector<CoordinateArraySequence*> aois_lines;
			vector<CoordinateArraySequence*> lanecenter_lines;

			getLaneMarking(landmarks, lane_boundaries_lines, "lane_boundaries", "line_points");
			getLaneMarking(landmarks, road_boundaries_lines, "road_boundaries", "line_points");
			getLaneMarking(landmarks, stop_lines_lines, "stop_lines", "line_points");
			getLaneMarking(landmarks, cross_walks_lines, "cross_walks", "center_line_points");
			getLaneMarking(landmarks, road_marks_lines, "road_marks", "polygon_points");
			getLaneMarking(landmarks, aois_lines, "aois", "polygon_points");
			getLaneMarking(landmarks, lanecenter_lines, "lane_centers", "line_points");

			lanemarkings[timestamp_us_v] = lane_boundaries_lines;
			road_boundaries[timestamp_us_v] = road_boundaries_lines;
			stop_lines[timestamp_us_v] = stop_lines_lines;
			cross_walks[timestamp_us_v] = cross_walks_lines;
			road_marks[timestamp_us_v] = road_marks_lines;
			aois[timestamp_us_v] = aois_lines;
			lanecenter[timestamp_us_v] = lanecenter_lines;
		}

		features.emplace_back(lanemarkings);
		features.emplace_back(road_boundaries);
		features.emplace_back(stop_lines);
		features.emplace_back(cross_walks);
		features.emplace_back(road_marks);
		features.emplace_back(aois);
		features.emplace_back(lanecenter);

		json_object_put(root);
	}
}

void mapbuilding::getLaneMarking(json_object* landmarks, vector<CoordinateArraySequence*>& lines, const char* pszFeatureName,const char* pszKeyName) {
	json_object* lane_boundaries = json_object_object_get(landmarks, pszFeatureName);
	size_t lane_boundaries_nums = json_object_array_length(lane_boundaries);

	for (size_t j = 0; j < lane_boundaries_nums; j++) {
		json_object* lane_boundaries_obj = json_object_array_get_idx(lane_boundaries, j);
		json_object* line_points = json_object_object_get(lane_boundaries_obj, pszKeyName);

		if (strcmp(pszFeatureName,"road_marks") == 0) {
			set<int> set_types;
			json_object* types = json_object_object_get(lane_boundaries_obj, "types");
			size_t types_nums = json_object_array_length(types);
			for (size_t i = 0; i < types_nums; i++) {
				json_object* child_type = json_object_array_get_idx(types, i);
				int type = json_object_get_int(child_type);
				set_types.emplace(type);
			}

			if (set_types.count(13) == 0) {
				size_t line_points_nums = json_object_array_length(line_points);
				if (line_points_nums > 1) {
					CoordinateArraySequence* cs = new CoordinateArraySequence();
					for (size_t k = 0; k < line_points_nums; k++) {
						json_object* point_obj = json_object_array_get_idx(line_points, k);
						json_object* x_obj = json_object_object_get(point_obj, "x");
						json_object* y_obj = json_object_object_get(point_obj, "y");
						double x = json_object_get_double(x_obj);
						double y = json_object_get_double(y_obj);
						cs->add(Coordinate(x, y), false);
					}
					lines.emplace_back(cs);
				}
			}
		} else {
			size_t line_points_nums = json_object_array_length(line_points);
			if (line_points_nums > 1) {
				CoordinateArraySequence* cs = new CoordinateArraySequence();
				for (size_t k = 0; k < line_points_nums; k++) {
					json_object* point_obj = json_object_array_get_idx(line_points, k);
					json_object* x_obj = json_object_object_get(point_obj, "x");
					json_object* y_obj = json_object_object_get(point_obj, "y");
					double x = json_object_get_double(x_obj);
					double y = json_object_get_double(y_obj);
					cs->add(Coordinate(x, y), false);
				}
				lines.emplace_back(cs);
			}
		}
	}
}

void mapbuilding::read_mla_egopose(const char* pszPath, map<GInt64, vector<double>>& egopose) {
	if (pszPath == NULL) {
		return;
	}
	json_object* root = json_object_from_file(pszPath);
	if (root != NULL) {
		size_t size = json_object_array_length(root);
		for (size_t i = 0; i < size; i++) {
			json_object* child = json_object_array_get_idx(root, i);
			json_object* meta = json_object_object_get(child, "meta");
			json_object* timestamp_us = json_object_object_get(meta, "timestamp_us");
			int64_t timestamp_us_v = json_object_get_int64(timestamp_us);

			json_object* position = json_object_object_get(child, "position");
			json_object* position_global = json_object_object_get(position, "position_global");

			json_object* longitude = json_object_object_get(position_global, "longitude");
			json_object* latitude = json_object_object_get(position_global, "latitude");
			double lon = json_object_get_double(longitude);
			double lat = json_object_get_double(latitude);

			json_object* orientation = json_object_object_get(child, "orientation");
			json_object* quaternion_global = json_object_object_get(orientation, "quaternion_global");

			json_object* quaternion_w = json_object_object_get(quaternion_global, "w");
			json_object* quaternion_x = json_object_object_get(quaternion_global, "x");
			json_object* quaternion_y = json_object_object_get(quaternion_global, "y");
			json_object* quaternion_z = json_object_object_get(quaternion_global, "z");

			double w = json_object_get_double(quaternion_w);
			double x = json_object_get_double(quaternion_x);
			double y = json_object_get_double(quaternion_y);
			double z = json_object_get_double(quaternion_z);

			vector<double> vec{ lon,lat,w,x,y,z };
			egopose[timestamp_us_v] = vec;
		}
		json_object_put(root);
	}
}
void mapbuilding::read_sensor_gnss_rtk(const char* pszPath, map < GInt64, int>& gnss_rtk) {
	if (pszPath == NULL) {
		return;
	}
	json_object* root = json_object_from_file(pszPath);
	if (root != NULL) {
		size_t size = json_object_array_length(root);
		for (size_t i = 0; i < size; i++) {
			json_object* child = json_object_array_get_idx(root, i);
			json_object* meta = json_object_object_get(child, "meta");
			json_object* timestamp_us = json_object_object_get(meta, "timestamp_us");
			int64_t timestamp_us_v = json_object_get_int64(timestamp_us);

			json_object* info = json_object_object_get(child, "info");
			json_object* gnss_data_basic = json_object_object_get(info, "gnss_data_basic");

			json_object* status = json_object_object_get(gnss_data_basic, "status");
			gnss_rtk[timestamp_us_v] = json_object_get_int(status);
		}
		json_object_put(root);
	}
}
void mapbuilding::geometryProjection(Coordinate& pt, int zone) {
	char szProj[512] = { 0 };
	sprintf(szProj, "+proj=utm +zone=%d +datum=WGS84 +units=m +no_defs", zone);

	projPJ pj_utm, pj_latlong;
	pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs");
	pj_utm = pj_init_plus(szProj);

	pt.x *= DEG_TO_RAD;
	pt.y *= DEG_TO_RAD;

	pj_transform(pj_latlong, pj_utm, 1, 1, &pt.x, &pt.y, NULL);

	pj_free(pj_latlong);
	pj_free(pj_utm);
}
double mapbuilding::_transform_lat(double x, double y) {
	double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;
	return ret;
}

double mapbuilding::_transform_lon(double x, double y) {
	double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
	return ret;
}
void mapbuilding::wgs84_to_gcj02(double lon, double lat, double& mglon, double& mglat) {
	double PI = 3.1415926535897932384626;
	double A = 6378245.0;
	double EE = 0.00669342162296594323;

	double dlat = _transform_lat(lon - 105.0, lat - 35.0);
	double dlon = _transform_lon(lon - 105.0, lat - 35.0);
	double radlat = lat / 180.0 * M_PI;
	double magic = sin(radlat);
	magic = 1 - EE * magic * magic;
	double sqrtmagic = sqrt(magic);
	dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * M_PI);
	dlon = (dlon * 180.0) / (A / sqrtmagic * cos(radlat) * M_PI);

	mglat = lat + dlat;
	mglon = lon + dlon;
}
unique_ptr<LineString> mapbuilding::wgs84_to_gcj02(const char* pszWKT) {
	auto line = m_reader->read<LineString>(pszWKT);
	size_t pointNums = line->getNumPoints();
	CoordinateArraySequence* cs = new CoordinateArraySequence();
	for (size_t i = 0; i < pointNums; i++) {
		const Coordinate pt = line->getCoordinateN(i);
		double gcj_lon = 0;
		double gcj_lat = 0;
		wgs84_to_gcj02(pt.x, pt.y, gcj_lon, gcj_lat);
		cs->add(Coordinate(gcj_lon, gcj_lat));
	}

	return unique_ptr<LineString>(m_factory->createLineString(cs));
}
void mapbuilding::wgs84_to_gcj02(const char* pszSrcPath, const char* pszDstPath) {
	std::ifstream file(pszSrcPath);
	string record;
	int lineNum = 0;
	
	FILE* fp = fopen(pszDstPath, "w");
	while (std::getline(file, record)) {
		if (lineNum++ == 0) {
			fprintf(fp, "%s\n", record.c_str());
			continue;
		}
		char** papszRetList = splitLine(record.c_str(), ',');
		int nFields = CSLCount(papszRetList);

		auto line = wgs84_to_gcj02(papszRetList[12]);
		string record;
		for (size_t i = 0; i < nFields; i++) {
			if (i != 12) {
				record += papszRetList[i];
			} else {
				record += "\"";
				record += line->toString();
				record += "\"";
			}
			record += ",";
		}
		record = record.substr(0,record.length() - 1);
		fprintf(fp, "%s\n", record.c_str());
	}
	fclose(fp);
	file.close();
}
unique_ptr<LineString> mapbuilding::densifyLine(LineString* line, double interval) {
	CoordinateArraySequence* cs = new CoordinateArraySequence();
	double length = line->getLength();
	int times = (int)ceil(length / interval);
	LengthIndexedLine lil(line);
	for (int i = 0; i < times; i++) {
		geom::Coordinate pt = lil.extractPoint((double)i * interval);
		cs->add(pt,false);
	}

	return unique_ptr<LineString>(m_factory->createLineString(cs));
}
void mapbuilding::initGraph() {
	const char* pszFilePath = "D:/zhongbao/mmt/osm/link.csv";

	build_osm_graph(pszFilePath, m_graph_topo);

	m_diskfile = StorageManager::createNewMemoryStorageManager();
	m_bufferfile = StorageManager::createNewRandomEvictionsBuffer(*m_diskfile, 10, false);
	double fillFactor = 0.7;
	uint32_t indexCapacity = 10;
	uint32_t leafCapacity = 10;
	uint32_t dimension = 2;
	RTree::RTreeVariant variant = RTree::RV_RSTAR;

	id_type indexIdentifier;
	m_tree = RTree::createNewRTree(*m_bufferfile, fillFactor, indexCapacity, leafCapacity, dimension, variant, indexIdentifier);

	auto iter = m_graph_topo.begin();
	for (; iter != m_graph_topo.end(); ++iter) {
		const Envelope* enve = iter->second->line->getEnvelopeInternal();
		double plow[2] = { 0 };
		double phigh[2] = { 0 };
		plow[0] = enve->getMinX();
		plow[1] = enve->getMinY();
		phigh[0] = enve->getMaxX();
		phigh[1] = enve->getMaxY();

		Region r = Region(plow, phigh, 2);
		m_tree->insertData(0, 0, r, atoll(iter->first.c_str()));
	}
}
string mapbuilding::matching(unique_ptr<LineString>& utm_gnss_line, string& road_name, string& road_class) {
	vector<map<string, double>> eps;
	vector<vector<string>> candinates;
	int pointNums = utm_gnss_line->getNumPoints();
	double sigma = 30;
	double radius = 30;
	for (size_t i = 0; i < pointNums; i++) {
		map<string, double> emission_probability;
		std::unique_ptr<geos::geom::Point> pt = utm_gnss_line->getPointN(i);
		double plow[2] = { 0 };
		double phigh[2] = { 0 };
		plow[0] = pt->getX() - radius;
		plow[1] = pt->getY() - radius;
		phigh[0] = pt->getX() + radius;
		phigh[1] = pt->getY() + radius;
		vector<string> ids;
		MyVisitor vis;
		Region r = Region(plow, phigh, 2);
		m_tree->intersectsWithQuery(r, vis);
		vis.getIDs(ids);

		for (size_t j = 0; j < ids.size(); j++) {
			auto iter1 = m_graph_topo.find(ids[j]);
			if (iter1 != m_graph_topo.end()) {
				double dist = pt->distance(iter1->second->line.get());
				double ep = 1 / (pow(2 * M_PI, 0.5) * sigma) * exp(-pow(dist, 2) / (2 * sigma * sigma));
				char szKey[512] = { 0 };
				sprintf(szKey, "%d_%s", i, ids[j].c_str());
				emission_probability[szKey] = ep;
			}
		}
		eps.emplace_back(emission_probability);
		candinates.emplace_back(ids);
	}

	map<string, string> relation;
	string last_link;
	for (size_t i = 1; i < pointNums; i++) {
		vector<string> v1 = candinates[i - 1];
		vector<string> v2 = candinates[i];
		unique_ptr<geos::geom::Point> pt1 = utm_gnss_line->getPointN(i - 1);
		unique_ptr<geos::geom::Point> pt2 = utm_gnss_line->getPointN(i);

		double max_v2 = 0;
		vector<map<string, string>> v3;
		for (size_t j = 0; j < v2.size(); j++) {
			string link_id2 = v2[j];
			char szKey2[512] = { 0 };
			sprintf(szKey2, "%d_%s", i, link_id2.c_str());
			auto iter2 = eps[i].find(szKey2);
			double max_p = 0;
			string from_link;

			for (size_t k = 0; k < v1.size(); k++) {
				string link_id1 = v1[k];
				char szKey1[512] = { 0 };
				sprintf(szKey1, "%d_%s", i - 1, link_id1.c_str());
				auto iter1 = eps[i - 1].find(szKey1);
				double tp = transition_probability(link_id1, link_id2, pt1, pt2, m_graph_topo);
				double p = iter1->second * tp * iter2->second;
				if (p >= max_p) {
					max_p = p;
					from_link = link_id1;
				}
			}
			char szKey3[512] = { 0 };
			sprintf(szKey3, "%d_%s", i, link_id2.c_str());
			relation[szKey3] = from_link;

			iter2->second = max_p;

			if (iter2->second > max_v2) {
				max_v2 = iter2->second;
				last_link = link_id2;
			}
		}
	}

	vector<string> match_links;
	match_links.emplace_back(last_link);
	for (int i = pointNums - 1; i > 0; i--) {
		char szKey[512] = { 0 };
		sprintf(szKey, "%d_%s", i, last_link.c_str());
		auto iter = relation.find(szKey);
		if (iter != relation.end()) {
			if (!match_links.empty()) {
				if (match_links.back() != iter->second) {
					match_links.emplace_back(iter->second);
				}
			} else {
				match_links.emplace_back(iter->second);
			}
			last_link = iter->second;
		}
	}
	std::reverse(match_links.begin(), match_links.end());
	string match_result = "";

	map<string, double> nameLength;
	for (size_t i = 0; i < match_links.size(); i++) {
		match_result += match_links[i];
		match_result += ",";
		auto iter1 = m_osm_roads.find(match_links[i]);
		if (iter1 != m_osm_roads.end()) {
			if (iter1->second->name != "") {
				auto iter2 = nameLength.find(iter1->second->name);
				if (iter2 == nameLength.end()) {
					nameLength[iter1->second->name] = iter1->second->length;
				} else {
					iter2->second += iter1->second->length;
				}
			}
		}
	}
	if (!nameLength.empty()) {
		std::vector<std::pair<string, double>> sorted_map(nameLength.begin(), nameLength.end());
		std::sort(sorted_map.begin(), sorted_map.end(),
			[](const std::pair<string, double>& a, const std::pair<string, double>& b) {
				return a.second > b.second;
			});
		road_name = sorted_map[0].first;
	}

	{
		map<string, double> typeLength;
		for (size_t i = 0; i < match_links.size(); i++) {
			auto iter1 = m_osm_roads.find(match_links[i]);
			if (iter1->second->link_type_name != "") {
				auto iter2 = typeLength.find(iter1->second->link_type_name);
				if (iter2 == typeLength.end()) {
					typeLength[iter1->second->link_type_name] = iter1->second->length;
				} else {
					iter2->second += iter1->second->length;
				}
			}
		}
		if (!typeLength.empty()) {
			std::vector<std::pair<string, double>> sorted_map(typeLength.begin(), typeLength.end());
			std::sort(sorted_map.begin(), sorted_map.end(),
				[](const std::pair<string, double>& a, const std::pair<string, double>& b) {
					return a.second > b.second;
				});
			string link_type_name = sorted_map[0].first;

			auto find = m_osm_road_class.find(link_type_name);
			if (find != m_osm_road_class.end()) {
				road_class = find->second;
			}
		}
	}

	match_result = match_result.substr(0, match_result.length() - 1);
	return match_result;
}
void mapbuilding::parseOSM(const char* pszPath) {
	json_object* root = json_object_from_file(pszPath);
	if (root != NULL) {
		map<GIntBig, Coordinate> nodes;
		getNodes(root, nodes);
		getWays(root, nodes);
	}
}
void mapbuilding::getNodes(json_object* root, map<GIntBig, Coordinate>& nodes){
	json_object* node = json_object_object_get(root, "node");
	size_t length = json_object_array_length(node);
	for (size_t i = 0; i < length; i++) {
		json_object* child = json_object_array_get_idx(node, i);
		json_object* id = json_object_object_get(child, "id");
		json_object* lon = json_object_object_get(child, "lon");
		json_object* lat = json_object_object_get(child, "lat");
		nodes[json_object_get_int64(id)] = Coordinate(json_object_get_double(lon), json_object_get_double(lat));
	}
}
void mapbuilding::getWays(json_object* root, map<GIntBig, Coordinate>& nodes) {
	json_object* way = json_object_object_get(root, "way");
	size_t length = json_object_array_length(way);
	FILE* fp = fopen("D:\\zhongbao\\数据样例\\way_ground_symbol_osm.txt", "w");
	set<string> s;
	for (size_t i = 0; i < length; i++) {
		json_object* child = json_object_array_get_idx(way, i);
		json_object* id = json_object_object_get(child, "id");
		int way_id = json_object_get_int(id);
		json_object* modelName = json_object_object_get(child, "modelName");
		const char* pszModelName = json_object_get_string(modelName);
		s.insert(pszModelName);
		if (strcmp(pszModelName,"OBJECT_PG") == 0) {
			string wkt = "LINESTRING (";
			CoordinateArraySequence* cs = new CoordinateArraySequence();
			json_object* nd = json_object_object_get(child, "nd");
			size_t ndNums = json_object_array_length(nd);
			for (size_t j = 0; j < ndNums; j++) {
				json_object* childNd = json_object_array_get_idx(nd, j);
				json_object* ref = json_object_object_get(childNd, "ref");
				int64_t node_id = json_object_get_int64(ref);
				auto iter = nodes.find(node_id);
				cs->add(Coordinate(iter->second), false);
				wkt += Coordinate(iter->second).toString();
				wkt += ",";
			}
			wkt = wkt.substr(0, wkt.length() - 1);
			wkt += ")";
			fprintf(fp, "%d#%s\n", way_id, wkt.c_str());
			delete cs;
		}
	}
	fclose(fp);
}
void mapbuilding::addLinesToOsm(const char* pszPath) {
	json_object* root = json_object_new_object();
	json_object* nodes_array = json_object_new_array();
	json_object* ways_array = json_object_new_array();
	json_object* relations_array = json_object_new_array();

	int node_id = 0;
	int relation_id = 0;
	int way_id = 0;

	string laneMarkingPath = pszPath;
	laneMarkingPath += "/lanemarking.shp";

	GDALDataset* ds = (GDALDataset*)GDALOpenEx(laneMarkingPath.c_str(), GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	if (ds != NULL) {
		map<string, string> node_tags = { {"BATCH",""},{"TASKID","0"}, {"SEQ",""}, {"FLAG","1"},{"OPERATOR",""}, {"SDATE",""}, {"ISSPLIT","0"},{"DASHTYPE","6"} };
		map<string, string> way_tags = { {"TASKID",""},{"SEQ",""}, {"FLAG","1"}, {"SDATE",""},{"BATCH",""}, {"SOURCE","8"}, {"OPERATOR",""},{"TOLLFLAG","0"},{"R_LINE","0"},{"IDENTIFIER_TYPE","1"},{"DIRECTION","2"},{"DIVIDER_NO",""} };
		map<string, string> relation_tags = { {"TASKID","0"},{"FLAG","1"}, {"SDATE",""}, {"MAIN_AUX","1"},{"VIRTUAL","0"}, {"OPERATOR",""}, {"MARK_TYPE","1"},{"BATCH",""},{"TYPE","0"},{"DRIVE_RULE","0"},{"GROUP_TYPE","0"},{"CROSS","0"},{"IDENTIFIER_TYPE","1"},{"WIDTH","0"},{"OVERLAY","0"},{"COLOR","1"},{"MATERIAL","1"},{"SOURCE","8"} };

		OGRLayer* layer = ds->GetLayer(0);
		OGRFeature* feature = NULL;
		map<string, int> nodes;

		vector<OGRLineString*> lines;
		while ((feature = layer->GetNextFeature()) != NULL) {
			OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
			if (line != NULL) {
				int numPoints = line->getNumPoints();
				for (size_t i = 0; i < numPoints; i++) {
					double x = line->getX(i);
					double y = line->getY(i);
					Coordinate pt(x, y);
					nodes[pt.toString()] = ++node_id;
					json_object* node_child = json_object_new_object();
					json_object_object_add(node_child, "id", json_object_new_int(node_id));
					json_object_object_add(node_child, "lon", json_object_new_double(x));
					json_object_object_add(node_child, "lat", json_object_new_double(y));
					json_object_object_add(node_child, "modelName", json_object_new_string("DIVIDER_NODE"));
					json_object_object_add(node_child, "tag", getTags(node_tags));
					json_object_object_add(node_child, "z", json_object_new_double(0));
					json_object_array_add(nodes_array, node_child);
				}
				lines.emplace_back((OGRLineString*)line->clone());
			}
			OGRFeature::DestroyFeature(feature);
		}

		size_t lineNums = lines.size();
		for (size_t i = 0; i < lineNums; i++) {
			way_id = i;
			json_object* way_child = json_object_new_object();
			json_object_object_add(way_child, "id", json_object_new_int(way_id));
			json_object_object_add(way_child, "modelName", json_object_new_string("DIVIDER"));
			OGRLineString* line = lines[i];
			int pointNums = line->getNumPoints();
			json_object* nd = json_object_new_array();

			for (size_t j = 0; j < pointNums; j++) {
				double x = line->getX(j);
				double y = line->getY(j);
				Coordinate pt(x, y);
				auto find_iter = nodes.find(pt.toString());
				int node_id = -1;
				if (find_iter != nodes.end()) {
					node_id = find_iter->second;
					json_object* nd_child = json_object_new_object();
					json_object_object_add(nd_child, "ref", json_object_new_int(node_id));
					json_object_array_add(nd, nd_child);

					json_object* member_child_node = json_object_new_object();
					json_object_object_add(member_child_node, "modelName", json_object_new_string("DIVIDER_NODE"));
					json_object_object_add(member_child_node, "ref", json_object_new_int(node_id));
					json_object_object_add(member_child_node, "role", json_object_new_string("DIVIDER_NODE_ID"));
					json_object_object_add(member_child_node, "sequence", json_object_new_int(1));
					json_object_object_add(member_child_node, "type", json_object_new_string("node"));

					json_object* member_child_way = json_object_new_object();
					json_object_object_add(member_child_way, "modelName", json_object_new_string("DIVIDER"));
					json_object_object_add(member_child_way, "ref", json_object_new_int(way_id));
					json_object_object_add(member_child_way, "role", json_object_new_string("DIVIDER_ID"));
					json_object_object_add(member_child_way, "sequence", json_object_new_int(0));
					json_object_object_add(member_child_way, "type", json_object_new_string("way"));

					json_object* members_array = json_object_new_array();
					json_object_array_add(members_array, member_child_node);
					json_object_array_add(members_array, member_child_way);

					relation_id++;
					json_object* relation_child = json_object_new_object();
					json_object_object_add(relation_child, "id", json_object_new_int(relation_id));
					json_object_object_add(relation_child, "member", members_array);
					json_object_object_add(relation_child, "modelName", json_object_new_string("DIVIDER_ATTRIBUTE"));
					json_object_object_add(relation_child, "tag", getTags(relation_tags));
					json_object_array_add(relations_array, relation_child);
				}
			}
			json_object_object_add(way_child, "nd", nd);
			json_object_object_add(way_child, "tag", getTags(way_tags));
			json_object_array_add(ways_array, way_child);
			OGRGeometryFactory::destroyGeometry(line);
		}
		GDALClose(ds);
	}

	json_object_object_add(root, "node", nodes_array);
	json_object_object_add(root, "way", ways_array);
	json_object_object_add(root, "relation", relations_array);

	string stoplinePath = pszPath;
	stoplinePath += "/stopline.shp";

	string roadBoundaryPath = pszPath;
	roadBoundaryPath += "/roadboundary.shp";

	string osmPath = CPLGetPath(pszPath);
	osmPath = CPLGetPath(osmPath.c_str());
	osmPath += "/lane_osm.json";

	addLines(stoplinePath.c_str(), nodes_array, ways_array, node_id, way_id, 3);
	addLines(roadBoundaryPath.c_str(), nodes_array, ways_array, node_id, way_id, 4);
	json_object_to_file_ext(osmPath.c_str(), root, JSON_C_TO_STRING_PRETTY);
	json_object_put(root);
}

void mapbuilding::kappa2osm(const char* pszPath) {
	json_object* root = json_object_new_array();

	json_object* relations_array = json_object_new_array();
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	OGRLayer* layer = ds->GetLayer(0);

	OGRFeature* feature = NULL;
	while ((feature = layer->GetNextFeature()) != NULL) {
		OGRPoint* pt = (OGRPoint*)feature->GetGeometryRef();
		if (pt != NULL) {
			double x = pt->getX();
			double y = pt->getY();
			double curvature = feature->GetFieldAsDouble("curvature");
			json_object* node = json_object_new_object();
			json_object_object_add(node, "curvature", json_object_new_double(curvature));
			json_object_object_add(node, "lon", json_object_new_double(x));
			json_object_object_add(node, "lat", json_object_new_double(y));
			json_object_array_add(root, node);
		}
		OGRFeature::DestroyFeature(feature);
	}

	json_object_to_file_ext("D:\\zhongbao\\数据样例\\adas.json", root, JSON_C_TO_STRING_PRETTY);
	json_object_put(root);
	GDALClose(ds);
}
void mapbuilding::addLines(const char* pszPath, json_object* nodes_array, json_object* ways_array, int& node_last_id, int& way_last_id, int subtype) {
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	if (ds == NULL) {
		return;
	}

	char szSubtype[512] = { 0 };
	sprintf(szSubtype, "%d", subtype);
	map<string, string> way_tags = { {"BATCH",""},{"SDATE",""}, {"TASKID","0"}, {"OPERATOR","1"},{"SEQ",""}, {"FLAG","1"}, {"SOURCE","8"},{"MATERIAL","7"},{"SUBTYPE",szSubtype},{"WIDTH","-1"},{"COLOR","0"},{"TYPE","2"} };
	map<string, string> node_tags = { {"BATCH",""},{"SDATE",""}, {"TASKID","0"}, {"OPERATOR",""},{"SEQ",""}, {"FLAG","1"}, {"SOURCE","8"} };
	map<string, string> relation_tags = { {"TASKID","0"},{"FLAG","1"}, {"SDATE",""}, {"MAIN_AUX","1"},{"VIRTUAL","0"}, {"OPERATOR",""}, {"MARK_TYPE","1"},{"BATCH",""},{"TYPE","0"},{"DRIVE_RULE","0"},{"GROUP_TYPE","0"},{"CROSS","0"},{"IDENTIFIER_TYPE","1"},{"WIDTH","0"},{"OVERLAY","0"},{"COLOR","1"},{"MATERIAL","1"},{"SOURCE","8"} };

	OGRLayer* layer = ds->GetLayer(0);
	OGRFeature* feature = NULL;
	map<string, int> nodes;

	vector<OGRLineString*> lines;
	while ((feature = layer->GetNextFeature()) != NULL) {
		OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
		if (line != NULL) {
			int numPoints = line->getNumPoints();
			for (size_t i = 0; i < numPoints; i++) {
				double x = line->getX(i);
				double y = line->getY(i);
				Coordinate pt(x, y);
				nodes[pt.toString()] = ++node_last_id;
				json_object* node_child = json_object_new_object();
				json_object_object_add(node_child, "id", json_object_new_int(node_last_id));
				json_object_object_add(node_child, "lat", json_object_new_double(y));
				json_object_object_add(node_child, "lon", json_object_new_double(x));
				json_object_object_add(node_child, "modelName", json_object_new_string("OBJECT_PL_NODE"));
				json_object_object_add(node_child, "tag", getTags(node_tags));
				json_object_object_add(node_child, "z", json_object_new_double(0));
				json_object_array_add(nodes_array, node_child);
			}
			lines.emplace_back((OGRLineString*)line->clone());
		}
		OGRFeature::DestroyFeature(feature);
	}

	size_t lineNums = lines.size();
	for (size_t i = 0; i < lineNums; i++) {
		++way_last_id;
		json_object* way_child = json_object_new_object();
		json_object_object_add(way_child, "id", json_object_new_int(way_last_id));
		json_object_object_add(way_child, "modelName", json_object_new_string("OBJECT_PL"));
		OGRLineString* line = lines[i];
		int pointNums = line->getNumPoints();
		json_object* nd = json_object_new_array();
		for (size_t j = 0; j < pointNums; j++) {
			double x = line->getX(j);
			double y = line->getY(j);
			Coordinate pt(x, y);
			auto find_iter = nodes.find(pt.toString());
			int node_id = -1;
			if (find_iter != nodes.end()) {
				node_id = find_iter->second;
				json_object* nd_child = json_object_new_object();
				json_object_object_add(nd_child, "ref", json_object_new_int(node_id));
				json_object_array_add(nd, nd_child);
			}
		}
		json_object_object_add(way_child, "nd", nd);
		json_object_object_add(way_child, "tag",getTags(way_tags));
		json_object_array_add(ways_array, way_child);
		OGRGeometryFactory::destroyGeometry(line);
	}
	GDALClose(ds);
}
void mapbuilding::addRoadmark(const char* pszPath, int& node_id, int& way_id, int subtype,json_object* root) {
	char szSubtype[512] = { 0 };
	sprintf(szSubtype, "%d", subtype);
	map<string, string> way_tags = { {"BATCH",""},{"SDATE",""}, {"TASKID","0"}, {"OPERATOR","1"},{"SEQ",""}, {"FLAG","1"}, {"SOURCE","8"},{"NAME",""},{"ALIAS",""},{"COLOR","1"},{"MATERIAL","5"},{"TYPE","3"},{"SUBTYPE",szSubtype},{"CONT","0"},{"SHAPE","1"} };
	map<string, string> node_tags = { {"BATCH",""},{"SDATE",""}, {"TASKID","0"}, {"OPERATOR",""},{"SEQ",""}, {"FLAG","1"}, {"SOURCE","8"} };

	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	if (ds == NULL) {
		return;
	}
	OGRLayer* layer = ds->GetLayer(0);
	OGRFeature* feature = NULL;
	map<string, int> nodes;

	vector<OGRLineString*> lines;

	json_object* nodes_array = NULL;
	json_object* ways_array = NULL;
	json_object* relations_array = NULL;

	nodes_array = json_object_object_get(root, "node");
	if (nodes_array == NULL) {
		nodes_array = json_object_new_array();
		json_object_object_add(root, "node", nodes_array);
	}
	
	ways_array = json_object_object_get(root, "way");
	if (ways_array == NULL) {
		ways_array = json_object_new_array();
		json_object_object_add(root, "way", ways_array);
	}

	relations_array = json_object_object_get(root, "relation");
	if (relations_array == NULL) {
		relations_array = json_object_new_array();
		json_object_object_add(root, "relation", relations_array);
	}

	while ((feature = layer->GetNextFeature()) != NULL) {
		OGRPolygon* polygon = (OGRPolygon*)feature->GetGeometryRef();
		OGRLinearRing* line = polygon->getExteriorRing();
		if (line != NULL) {
			int numPoints = line->getNumPoints();
			for (size_t i = 0; i < numPoints; i++) {
				double x = line->getX(i);
				double y = line->getY(i);
				Coordinate pt(x, y);
				nodes[pt.toString()] = ++node_id;
				json_object* node_child = json_object_new_object();
				json_object_object_add(node_child, "id", json_object_new_int(node_id));
				json_object_object_add(node_child, "lat", json_object_new_double(y));
				json_object_object_add(node_child, "lon", json_object_new_double(x));
				json_object_object_add(node_child, "modelName", json_object_new_string("OBJECT_PG_NODE"));
				json_object_object_add(node_child, "tag", getTags(node_tags));
				json_object_object_add(node_child, "z", json_object_new_double(0));
				json_object_array_add(nodes_array, node_child);
			}
			lines.emplace_back((OGRLineString*)line->clone());
		}
		OGRFeature::DestroyFeature(feature);
	}

	size_t lineNums = lines.size();
	for (size_t i = 0; i < lineNums; i++) {
		way_id = i;
		json_object* way_child = json_object_new_object();
		json_object_object_add(way_child, "id", json_object_new_int(way_id));
		json_object_object_add(way_child, "modelName", json_object_new_string("OBJECT_PG"));
		OGRLineString* line = lines[i];
		int pointNums = line->getNumPoints();
		json_object* nd = json_object_new_array();

		for (size_t j = 0; j < pointNums; j++) {
			double x = line->getX(j);
			double y = line->getY(j);
			Coordinate pt(x, y);
			auto find_iter = nodes.find(pt.toString());
			int node_id = -1;
			if (find_iter != nodes.end()) {
				node_id = find_iter->second;
				json_object* nd_child = json_object_new_object();
				json_object_object_add(nd_child, "ref", json_object_new_int(node_id));
				json_object_array_add(nd, nd_child);
			}
		}
		json_object_object_add(way_child, "nd", nd);
		json_object_object_add(way_child, "tag", getTags(way_tags));
		json_object_array_add(ways_array, way_child);
		OGRGeometryFactory::destroyGeometry(line);
	}

	GDALClose(ds);
}
int mapbuilding::getZone(int lon) {
	return int(31 + (lon / 6.0));
}
void mapbuilding::clusterFeatures(const char* pszPath, vector<vector<const Geometry*>>& cluster_geoms, double accept_distance) {
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszPath, GDAL_OF_VECTOR | GDAL_OF_UPDATE, NULL, NULL, NULL);
	if (ds == NULL) {
		return;
	}
	OGRLayer* layer = ds->GetLayer(0);
	OGRFeatureDefn* featureDefn = layer->GetLayerDefn();
	set<string> fieldNames;
	for (size_t i = 0; i < featureDefn->GetFieldCount(); i++) {
		OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
		fieldNames.insert(fieldDefn->GetNameRef());
	}
	
	if (fieldNames.count("fid") == 0) {
		OGRFieldDefn fid_field("fid", OFTInteger64);
		layer->CreateField(&fid_field);
	}
	
	if (fieldNames.count("cluster_id") == 0) {
		OGRFieldDefn fid_field("cluster_id", OFTInteger);
		layer->CreateField(&fid_field);
	}

	double fillFactor = 0.7;
	uint32_t indexCapacity = 10;
	uint32_t leafCapacity = 10;
	uint32_t dimension = 2;
	id_type indexIdentifier;

	IStorageManager* diskfile = StorageManager::createNewMemoryStorageManager();
	StorageManager::IBuffer* bufferfile = StorageManager::createNewRandomEvictionsBuffer(*diskfile, 10, false);
	ISpatialIndex* tree = RTree::createNewRTree(*bufferfile, fillFactor, indexCapacity, leafCapacity, dimension, RTree::RV_RSTAR, indexIdentifier);

	map<GIntBig, LineString*> lines;

	OGRFeature* feature = NULL;
	while ((feature = layer->GetNextFeature()) != NULL) {
		GIntBig fid = feature->GetFID();
		OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
		if (line != NULL) {
			LineString* utm_line = geometryProjectionGeos(line, m_zone);
			const Envelope* enve = utm_line->getEnvelopeInternal();
			double plow[2] = { 0 };
			double phigh[2] = { 0 };
			plow[0] = enve->getMinX();
			plow[1] = enve->getMinY();
			phigh[0] = enve->getMaxX();
			phigh[1] = enve->getMaxY();

			Region r = Region(plow, phigh, 2);
			tree->insertData(0, 0, r, fid);
			lines[fid] = utm_line;
		}
		feature->SetField("fid", fid);
		layer->SetFeature(feature);
		OGRFeature::DestroyFeature(feature);
	}

	vector<vector<GIntBig>> vecs;
	set<GIntBig> labeled;

	auto iter = lines.begin();
	for (; iter != lines.end(); ++iter) {
		GIntBig current_id = iter->first;
		if (labeled.count(current_id) == 1) {
			continue;
		}
		LineString* current_line = iter->second;
		Envelope current_enve = *(current_line->getEnvelopeInternal());

		vector<GIntBig> vec_id = { current_id };
		vector<const Geometry*> vec_geom = { current_line };

		removeIndex(lines, current_id, tree);

		while (true) {
			GIntBig search_fid = nearestFeature(current_id, lines, vec_geom, current_enve, 3, /*0.08*/accept_distance, tree);
			if (search_fid != -1) {
				vec_id.emplace_back(search_fid);
				removeIndex(lines, search_fid, tree);
				auto search_iter = lines.find(search_fid);
				const Envelope* search_enve = search_iter->second->getEnvelopeInternal();
				vec_geom.emplace_back(search_iter->second);
				current_enve.expandToInclude(search_enve);
			} else {
				vecs.emplace_back(vec_id);
				cluster_geoms.emplace_back(vec_geom);
				for (size_t i = 0; i < vec_id.size(); i++) {
					labeled.emplace(vec_id[i]);
				}
				break;
			}
		}
	}

	for (size_t i = 0; i < vecs.size(); i++) {
		for (size_t j = 0; j < vecs[i].size(); j++) {
			OGRFeature* feature = layer->GetFeature(vecs[i][j]);
			feature->SetField("cluster_id", (int)i);
			layer->SetFeature(feature);
			OGRFeature::DestroyFeature(feature);
		}
	}

	delete tree;
	delete bufferfile;
	delete diskfile;

	GDALClose(ds);
}

void mapbuilding::getRoadMark(const char* pszRoadmarkPath){
	if (pszRoadmarkPath == NULL) {
		return;
	}
	string basePath = CPLGetPath(pszRoadmarkPath);
	string roadmarkSavePath = basePath;
	roadmarkSavePath += "/roadmark_res.shp";
	GDALDriver::QuietDelete(roadmarkSavePath.c_str());

	string lanemarkingPath = basePath;
	lanemarkingPath += "/lanemarking_res.shp";

	vector<vector<const Geometry*>> cluster_geoms;
	clusterFeatures(pszRoadmarkPath, cluster_geoms,0.08);

	vector<Coordinate> centroids;
	for (size_t i = 0; i < cluster_geoms.size(); i++) {
		MultiLineString* multiline = m_factory->createMultiLineString(cluster_geoms[i]);
		Coordinate pt;
		multiline->getCentroid(pt);
		centroids.emplace_back(pt);
		m_factory->destroyGeometry(multiline);

		for (size_t j = 0; j < cluster_geoms[i].size(); j++) {
			m_factory->destroyGeometry((Geometry*)cluster_geoms[i][j]);
		}
	}

	buildFeatures(lanemarkingPath.c_str(), centroids, roadmarkSavePath.c_str());
}

void mapbuilding::buildFeatures(const char* pszLanemarkingPath, vector<Coordinate>& centroids, const char* pszRoadmarkSavePath) {
	if (centroids.empty()) {
		return;
	}
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszLanemarkingPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	if (ds == NULL) {
		return;
	}

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);
	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* roadmark_ds = driver->Create(pszRoadmarkSavePath, 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* roadmark_layer = roadmark_ds->CreateLayer("roadmark", &spatialRef, wkbPolygon, NULL);

	OGRFieldDefn field("id", OFTInteger);
	roadmark_layer->CreateField(&field);

	double fillFactor = 0.7;
	uint32_t indexCapacity = 10;
	uint32_t leafCapacity = 10;
	uint32_t dimension = 2;
	id_type indexIdentifier;

	IStorageManager* diskfile = StorageManager::createNewMemoryStorageManager();
	StorageManager::IBuffer* bufferfile = StorageManager::createNewRandomEvictionsBuffer(*diskfile, 10, false);
	ISpatialIndex* tree = RTree::createNewRTree(*bufferfile, fillFactor, indexCapacity, leafCapacity, dimension, RTree::RV_RSTAR, indexIdentifier);

	OGRLayer* layer = ds->GetLayer(0);

	map<GIntBig, LineString*> lines;
	OGRFeature* feature = NULL;
	while ((feature = layer->GetNextFeature()) != NULL) {
		GIntBig fid = feature->GetFID();
		OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
		if (line != NULL) {
			LineString* utm_line = geometryProjectionGeos(line,m_zone);
			const Envelope* enve = utm_line->getEnvelopeInternal();
			double plow[2] = { 0 };
			double phigh[2] = { 0 };
			plow[0] = enve->getMinX();
			plow[1] = enve->getMinY();
			phigh[0] = enve->getMaxX();
			phigh[1] = enve->getMaxY();

			Region r = Region(plow, phigh, 2);
			tree->insertData(0, 0, r, fid);
			lines[fid] = utm_line;
		}
		OGRFeature::DestroyFeature(feature);
	}

	for (size_t i = 0; i < centroids.size(); i++) {
		map<GIntBig, double> distancePair;
		Coordinate pt = centroids[i];
		geos::geom::Point* point = m_factory->createPoint(pt);
		double plow[2] = { 0 };
		double phigh[2] = { 0 };
		plow[0] = pt.x - 3;
		plow[1] = pt.y - 3;
		phigh[0] = pt.x + 3;
		phigh[1] = pt.y + 3;

		Region r = Region(plow, phigh, 2);
		vector<string> ids;
		MyVisitor vis;
		tree->intersectsWithQuery(r, vis);
		vis.getIDs(ids);
		
		if (ids.size() > 1) {
			for (size_t i = 0; i < ids.size(); i++) {
				GIntBig fid = atoll(ids[i].c_str());
				auto iter = lines.find(fid);
				if (iter != lines.end()) {
					distancePair[fid] = point->distance(iter->second);
				}
			}

			std::vector<std::pair<GIntBig, double>> sorted_map(distancePair.begin(), distancePair.end());
			std::sort(sorted_map.begin(), sorted_map.end(),
				[](const std::pair<GIntBig, double>& a, const std::pair<GIntBig, double>& b) {
					return a.second < b.second;});

			if (sorted_map[0].second < 2.5 && sorted_map[1].second < 2.5) {
				LineString* line1 = lines.find(sorted_map[0].first)->second;
				LineString* line2 = lines.find(sorted_map[1].first)->second;

				std::unique_ptr<geom::CoordinateSequence> cs1 = DistanceOp::nearestPoints(point,line1);
				std::unique_ptr<geom::CoordinateSequence> cs2 = DistanceOp::nearestPoints(point,line2);

				if (cs1->getSize() == 2 && cs2->getSize() == 2) {
					const Coordinate pt1 = cs1->getAt(1); 
					const Coordinate pt2 = cs2->getAt(1);
					geos::geom::LineSegment ls(pt1,pt2);
					Coordinate midPoint;
					ls.midPoint(midPoint);
					double angle = ls.angle();
					angle -= M_PI_2;

					double length = 2.5;
					CoordinateArraySequence* cs = new CoordinateArraySequence();
					if (!ls.isHorizontal()) {
						double x1 = midPoint.x - length * cos(angle);
						double y1 = midPoint.y - length * sin(angle);
						double x2 = midPoint.x + length * cos(angle);
						double y2 = midPoint.y + length * sin(angle);
						cs->add(Coordinate(x1, y1), false);
						cs->add(Coordinate(x2, y2), false);
					} else {
						double x1 = midPoint.x;
						double y1 = midPoint.x + length;
						double x2 = midPoint.x;
						double y2 = midPoint.x - length;
						cs->add(Coordinate(x1, y1), false);
						cs->add(Coordinate(x2, y2), false);
					}

					LineString* line = m_factory->createLineString(cs);
					std::unique_ptr<geom::Geometry> bufferGeom = BufferOp::bufferOp(line, 0.3, 
						BufferParameters::DEFAULT_QUADRANT_SEGMENTS,BufferParameters::CAP_FLAT);
					OGRGeometry* ogrGeom = geometryProjectionGeos(bufferGeom.get(), m_zone);
					if (ogrGeom != NULL) {
						OGRFeature* newFeature = OGRFeature::CreateFeature(roadmark_layer->GetLayerDefn());
						newFeature->SetGeometryDirectly(ogrGeom);
						roadmark_layer->CreateFeature(newFeature);
						OGRFeature::DestroyFeature(newFeature);
					}
					m_factory->destroyGeometry(line);
				}
			}
		}
		m_factory->destroyGeometry(point);
	}

	auto iter = lines.begin();
	for (; iter != lines.end(); ++iter) {
		m_factory->destroyGeometry(iter->second);
	}

	delete tree;
	delete bufferfile;
	delete diskfile;

	GDALClose(roadmark_ds);
	GDALClose(ds);
}
void mapbuilding::removeIndex(map<GIntBig, LineString*>& lines, GIntBig fid, ISpatialIndex* tree) {
	auto iter = lines.find(fid);
	if (iter != lines.end()) {
		const Envelope* enve = iter->second->getEnvelopeInternal();
		double plow[2] = { 0 };
		double phigh[2] = { 0 };
		plow[0] = enve->getMinX();
		plow[1] = enve->getMinY();
		phigh[0] = enve->getMaxX();
		phigh[1] = enve->getMaxY();			
		Region r = Region(plow, phigh, 2);
		tree->deleteData(r, fid);
	}
}

GIntBig mapbuilding::nearestFeature(GIntBig current_id,map<GIntBig, LineString*>& lines, const vector<const Geometry*>& vec_geom, Envelope& current_enve, double radius, double accept_distance, ISpatialIndex* tree) {
	GIntBig res_fid = -1;

	MultiLineString* multiline = m_factory->createMultiLineString(vec_geom);
	const Envelope* enve = multiline->getEnvelopeInternal();

	double plow[2] = { 0 };
	double phigh[2] = { 0 };
	plow[0] = enve->getMinX() - radius;
	plow[1] = enve->getMinY() - radius;
	phigh[0] = enve->getMaxX() + radius;
	phigh[1] = enve->getMaxY() + radius;

	vector<string> ids;
	MyVisitor vis;
	Region r = Region(plow, phigh, 2);
	tree->intersectsWithQuery(r, vis);
	vis.getIDs(ids);

	double min = 100;
	GIntBig search_fid = -1;
	for (size_t i = 0; i < ids.size(); i++) {
		GIntBig fid = atoll(ids[i].c_str());
		if (fid != current_id) {
			auto iter = lines.find(fid);
			if (iter != lines.end()) {
				double dist = multiline->distance(iter->second);
				if (dist < min) {
					min = dist;
					search_fid = fid;
				}
			}
		}
	}

	if (min < accept_distance) {
		res_fid = search_fid;
	}

	m_factory->destroyGeometry(multiline);

	return res_fid;
}

GInt64 mapbuilding::findClosest(GInt64 arr[], int n, GInt64 target) {
	int left = 0, right = n - 1;
	while (left < right) {
		if (abs(arr[left] - target) <= abs(arr[right] - target)) {
			right--;
		} else {
			left++;
		}
	}
	return arr[left];
}
int mapbuilding::getEgoposeStatus(GInt64* array,map<GInt64, int>& gnss_rtk, GInt64& ts) {
	int status = -1;
	if (gnss_rtk.empty()) {
		return status;
	}

	GInt64 find_ts = findClosest(array, gnss_rtk.size(),ts);

	auto iter = gnss_rtk.find(find_ts);
	if (iter != gnss_rtk.end()) {
		status = iter->second;
	}

	return status;
}
void mapbuilding::fixingEgopose(map<GInt64, int>& gnss_rtk, map<GInt64, vector<double>>& egopose) {
	if (gnss_rtk.empty()) {
		return;
	}

	GInt64* array = new GInt64[gnss_rtk.size()];
	{
		auto iter = gnss_rtk.begin();
		int index = 0;
		for (; iter != gnss_rtk.end(); ++iter) {
			array[index++] = iter->first;
		}
	}

	auto iter = egopose.begin();
	for (; iter != egopose.end();) {
		GInt64 ts = iter->first;
		int status = getEgoposeStatus(array,gnss_rtk, ts);
		if (status != 4) {
			iter = egopose.erase(iter);
		} else {
			++iter;
		}
	}

	delete[]array;
}
void mapbuilding::export_gnss_rtk(const char* pszDir, const char* pszSavePath) {
	char** papszRetList = VSIReadDir(pszDir);
	int nFields = CSLCount(papszRetList);
	FILE* fp = fopen(pszSavePath, "w");

	for (size_t i = 0; i < nFields; i++) {
		if (strcmp(papszRetList[i], ".") == 0 || strcmp(papszRetList[i], "..") == 0) {
			continue;
		}

		string strRtk = pszDir;
		strRtk += "\\";
		strRtk += papszRetList[i];
		strRtk += "\\_sensor_gnss_rtk.json";

		json_object* root = json_object_from_file(strRtk.c_str());
		if (root != NULL) {
			size_t size = json_object_array_length(root);
			for (size_t k = 0; k < size; k++) {
				json_object* child = json_object_array_get_idx(root, k);
				json_object* meta = json_object_object_get(child, "meta");
				json_object* timestamp_us = json_object_object_get(meta, "timestamp_us");
				int64_t timestamp_us_v = json_object_get_int64(timestamp_us);

				json_object* info = json_object_object_get(child, "info");
				json_object* gnss_data_basic = json_object_object_get(info, "gnss_data_basic");

				json_object* status = json_object_object_get(gnss_data_basic, "status");
				json_object* lon = json_object_object_get(gnss_data_basic, "lon");
				json_object* lat = json_object_object_get(gnss_data_basic, "lat");
				double x = json_object_get_double(lon);
				double y = json_object_get_double(lat);

				x = x / M_PI * 180;
				y = y / M_PI * 180;
				fprintf(fp, "%s,%lld,%f,%f,%d\n", papszRetList[i], timestamp_us_v, x, y, json_object_get_int(status));
			}
			json_object_put(root);
		}
	}

	fclose(fp);
	CSLDestroy(papszRetList);
}
void mapbuilding::addPolygonsToOsm(const char* pszPath) {
	int node_id = 0;
	int way_id = 0;

	string crosswalkPath = pszPath;
	crosswalkPath += "/crosswalk.shp";

	string roadmarkPath = pszPath;
	roadmarkPath += "/roadmark.shp";

	string aoisPath = pszPath;
	aoisPath += "/aois.shp";

	string osmPath = CPLGetPath(pszPath);
	osmPath = CPLGetPath(osmPath.c_str());
	osmPath += "/ground_symbol_osm.json";

	json_object* root = json_object_new_object();

	addRoadmark(roadmarkPath.c_str(), node_id, way_id, 2, root);
	addRoadmark(crosswalkPath.c_str(), node_id, way_id, 7, root);
	addRoadmark(aoisPath.c_str(), node_id, way_id, 0, root);

	json_object_to_file_ext(osmPath.c_str(), root, JSON_C_TO_STRING_PRETTY);
	json_object_put(root);
}
json_object* mapbuilding::getTags(map<string, string>& way_tags) {
	json_object* way_tags_array = json_object_new_array();

	auto iter = way_tags.begin();
	for (; iter != way_tags.end(); ++iter) {
		json_object* child = json_object_new_object();
		json_object_object_add(child, "k", json_object_new_string(iter->first.c_str()));
		json_object_object_add(child, "v", json_object_new_string(iter->second.c_str()));
		json_object_array_add(way_tags_array, child);
	}

	return way_tags_array;
}
void mapbuilding::getCrosswalk(const char* pszPath) {
	if (pszPath == NULL) {
		return;
	}
	string basePath = CPLGetPath(pszPath);
	basePath += "/";
	basePath += "crosswalk_res.shp";
	GDALDriver::QuietDelete(basePath.c_str());

	vector<vector<const Geometry*>> cluster_geoms;
	clusterFeatures(pszPath, cluster_geoms,0.08);
	if (cluster_geoms.empty()) {
		return;
	}

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);
	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* ds = driver->Create(basePath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* layer = ds->CreateLayer("crosswalk", &spatialRef, wkbPolygon, NULL);

	OGRFieldDefn area_field("area", OFTReal);
	layer->CreateField(&area_field);

	for (size_t i = 0; i < cluster_geoms.size(); i++) {
		MultiLineString* multiline = m_factory->createMultiLineString(cluster_geoms[i]);
		std::unique_ptr<geom::Geometry> miniRect = MinimumDiameter::getMinimumRectangle(multiline);
		double area = miniRect->getArea();
		if (area < 490 && area > 30) {
			OGRGeometry* geom = geometryProjectionGeos(miniRect.get(), m_zone);
			if (geom != NULL) {
				OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
				feature->SetField("area", area);
				feature->SetGeometryDirectly(geom);
				layer->CreateFeature(feature);
				OGRFeature::DestroyFeature(feature);
			}
		}
		m_factory->destroyGeometry(multiline);

		for (size_t j = 0; j < cluster_geoms[i].size(); j++) {
			m_factory->destroyGeometry((Geometry*)cluster_geoms[i][j]);
		}
	}
	GDALClose(ds);
}
void mapbuilding::getAois(const char* pszPath) {
	if (pszPath == NULL) {
		return;
	}
	string aoisPath = CPLGetPath(pszPath);
	aoisPath += "/";
	aoisPath += "aois_res.shp";
	GDALDriver::QuietDelete(aoisPath.c_str());

	vector<vector<const Geometry*>> cluster_geoms;
	clusterFeatures(pszPath, cluster_geoms, 0.08);
	if (cluster_geoms.empty()) {
		return;
	}

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);
	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* ds = driver->Create(aoisPath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* layer = ds->CreateLayer("aois", &spatialRef, wkbPolygon, NULL);

	OGRFieldDefn area_field("area", OFTReal);
	layer->CreateField(&area_field);

	for (size_t i = 0; i < cluster_geoms.size(); i++) {
		MultiLineString* multiline = m_factory->createMultiLineString(cluster_geoms[i]);
		std::unique_ptr<Geometry> convexHull = multiline->convexHull();
		double area = convexHull->getArea();
		if (area < 5000 && area > 50) {
			OGRGeometry* geom = geometryProjectionGeos(convexHull.get(), m_zone);
			if (geom != NULL) {
				OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
				feature->SetField("area", area);
				feature->SetGeometryDirectly(geom);
				layer->CreateFeature(feature);
				OGRFeature::DestroyFeature(feature);
			}
		}
		m_factory->destroyGeometry(multiline);

		for (size_t j = 0; j < cluster_geoms[i].size(); j++) {
			m_factory->destroyGeometry((Geometry*)cluster_geoms[i][j]);
		}
	}
	GDALClose(ds);
}
double mapbuilding::minAngle(LineString* line) {
	double angle = 360;
	if (line == NULL) {
		return angle;
	}

	size_t pointNums = line->getNumPoints();
	for (size_t i = 1; i < pointNums - 1; i++) {
		const Coordinate pt1 = line->getCoordinateN(i - 1);
		const Coordinate pt2 = line->getCoordinateN(i);
		const Coordinate pt3 = line->getCoordinateN(i + 1);
		double temp = Angle::toDegrees(Angle::angleBetween(pt1, pt2, pt3)) ;
		angle = std::min(temp, angle);
	}
	return angle;
}
CoordinateArraySequence* mapbuilding::fittingLine(vector<Coordinate>& points) {
	if (points.size() < 2) {
		return NULL;
	}
	
	int n = points.size();
	double sumX = 0;
	double sumY = 0;
	double sumXY = 0.0;
	double sumX2 = 0.0;

	for (int i = 0; i < n; i++) {
		sumX += points[i].x;
		sumY += points[i].y;
		sumXY += points[i].x * points[i].y;
		sumX2 += points[i].x * points[i].x;
	}

	double meanX = sumX / n;
	double meanY = sumY / n;

	double a = (sumXY - n * meanX * meanY) / (sumX2 - n * meanX * meanX);
	double b = meanY - a * meanX;

	/*
	MatrixXd A(n, 2);
	VectorXd l(n);

	for (size_t i = 0; i < n; i++) {
		cout << std::fixed << points[i].toString() << endl;
		A(i, 0) = points[i].x;
		A(i, 1) = 1;
		l(i) = -points[i].y;
	}
	MatrixXd At = A.transpose();
	MatrixXd N = At * A;
	MatrixXd U = At * l;
	VectorXd solution = -N.inverse() * U;
	*/
	
	CoordinateArraySequence* cs = new CoordinateArraySequence();
	for (size_t i = 0; i < n; i++) {
		double y = points[i].x * a + b;
		cs->add(Coordinate(points[i].x, y),false);
	}
	return cs;
}
void mapbuilding::getStopLine(const char* pszPath) {
	if (pszPath == NULL) {
		return;
	}
	string basePath = CPLGetPath(pszPath);
	basePath += "/";
	basePath += "stopline_res.shp";
	GDALDriver::QuietDelete(basePath.c_str());

	vector<vector<const Geometry*>> cluster_geoms;
	clusterFeatures(pszPath, cluster_geoms, 2);
	if (cluster_geoms.empty()) {
		return;
	}

	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);
	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	GDALDataset* ds = driver->Create(basePath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
	OGRLayer* layer = ds->CreateLayer("stopline", &spatialRef, wkbLineString, NULL);

	OGRFieldDefn length_field("length", OFTReal);
	layer->CreateField(&length_field);

	for (size_t i = 0; i < cluster_geoms.size(); i++) {
		vector<Coordinate> points;
		MultiLineString* multiline = m_factory->createMultiLineString(cluster_geoms[i]);
		for (size_t j = 0; j < cluster_geoms[i].size(); j++) {
			std::unique_ptr<CoordinateSequence> cs = cluster_geoms[i][j]->getCoordinates();
			points.emplace_back(cs->front());
			points.emplace_back(cs->back());
		}

		CoordinateArraySequence* cs = fittingLine(points);
		if (cs != NULL) {
			MultiPoint * mutiPoint = m_factory->createMultiPoint(*cs);
			MinimumBoundingCircle bmc(mutiPoint);
			std::unique_ptr<geom::Geometry> diameter = bmc.getMaximumDiameter();
			if (diameter->getGeometryTypeId() == GeometryTypeId::GEOS_LINESTRING) {
				OGRGeometry* geom = geometryProjectionGeos(diameter.get(), m_zone);
				if (geom != NULL) {
					double length = diameter->getLength();
					if (length > 10) {
						OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
						feature->SetField("length", length);
						feature->SetGeometryDirectly(geom);
						layer->CreateFeature(feature);
						OGRFeature::DestroyFeature(feature);
					} else {
						OGRGeometryFactory::destroyGeometry(geom);
					}
				}
			}
			m_factory->destroyGeometry(mutiPoint);
			delete cs;
		}
		m_factory->destroyGeometry(multiline);

		for (size_t j = 0; j < cluster_geoms[i].size(); j++) {
			m_factory->destroyGeometry((Geometry*)cluster_geoms[i][j]);
		}
	}

	GDALClose(ds);
}
void mapbuilding::addOtherOsm(const char* pszBasePath) {
	const char* pszRecord = "{\"node\":[],\"relation\":[],\"way\":[]}";
	string poleOsmPath = CPLGetPath(pszBasePath);
	poleOsmPath = CPLGetPath(poleOsmPath.c_str());
	poleOsmPath += "/pole_osm.json";

	string signOsmPath = CPLGetPath(pszBasePath);
	signOsmPath = CPLGetPath(signOsmPath.c_str());
	signOsmPath += "/sign_osm.json";

	FILE* fp1 = fopen(poleOsmPath.c_str(), "w");
	fputs(pszRecord, fp1);

	FILE* fp2 = fopen(signOsmPath.c_str(), "w");
	fputs(pszRecord, fp2);

	fclose(fp1);
	fclose(fp2);
}
void mapbuilding::getUTMZone(map<int, Polygon*>& utmZone) {
	double min_y = 17.45029903;
	double max_y = 54.57743866;

	int umt_zone = 43;
	for (double i = 72; i <= 132; i += 6) {
		double min_x = i;
		double max_x = i + 6;
		CoordinateArraySequence* cs = new CoordinateArraySequence();
		cs->add(Coordinate(min_x, min_y), true);
		cs->add(Coordinate(min_x, max_y), true);
		cs->add(Coordinate(max_x, max_y), true);
		cs->add(Coordinate(max_x, min_y), true);
		cs->add(Coordinate(min_x, min_y), true);
		LinearRing* ring = m_factory->createLinearRing(cs);
		Polygon* polygon = m_factory->createPolygon(ring,NULL);
		utmZone[umt_zone++] = polygon;
	}
}
void mapbuilding::clipLayer(const char* pszPath, map<int, Polygon*>& utmZone) {
	GDALDataset* ds = (GDALDataset*)GDALOpenEx(pszPath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
	if (ds == NULL) {
		return;
	}

	string basePath = CPLGetPath(pszPath);

	OGRLayer* layer = ds->GetLayer(0);
	const char* pszLayerName = layer->GetName();
	OGRFeatureDefn* featureDefn = layer->GetLayerDefn();
	int fieldCount = featureDefn->GetFieldCount();

	double fillFactor = 0.7;
	uint32_t indexCapacity = 10;
	uint32_t leafCapacity = 10;
	uint32_t dimension = 2;
	id_type indexIdentifier;

	IStorageManager* diskfile = StorageManager::createNewMemoryStorageManager();
	StorageManager::IBuffer* bufferfile = StorageManager::createNewRandomEvictionsBuffer(*diskfile, 10, false);
	ISpatialIndex* tree = RTree::createNewRTree(*bufferfile, fillFactor, indexCapacity, leafCapacity, dimension, RTree::RV_RSTAR, indexIdentifier);

	OGRFeature* feature = NULL;
	while ((feature = layer->GetNextFeature()) != NULL) {
		GIntBig fid = feature->GetFID();
		OGRLineString* line = (OGRLineString*)feature->GetGeometryRef();
		if (line != NULL) {
			Region r = getRect(line);
			tree->insertData(0, 0, r, fid);
		}
		OGRFeature::DestroyFeature(feature);
	}

	auto iter = utmZone.begin();
	for (; iter != utmZone.end(); ++iter) {
		Region rect = getRect(iter->second);
		vector<string> ids;
		MyVisitor vis;
		tree->intersectsWithQuery(rect, vis);
		vis.getIDs(ids);
		if (!ids.empty()) {
			char szCommand[512] = { 0 };
			char szDirectory[512] = { 0 };
#if defined(_WIN32) || defined(_WIN64)
			sprintf(szCommand, "mkdir %s\\%d", basePath.c_str(), iter->first);
			sprintf(szDirectory, "%s\\%d", basePath.c_str(), iter->first);
#else
			sprintf(szCommand, "mkdir %s/%d", basePath, iter->first);
			sprintf(szDirectory, "%s/%d", basePath.c_str(), iter->first);
#endif
			if (!isDirectoryExist(szDirectory)) {
				system(szCommand);
			}

			char szShpPath[512] = { 0 };
			sprintf(szShpPath, "%s/%d/%s.shp", basePath.c_str(), iter->first, pszLayerName);

			GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
			GDALDataset* child_ds = driver->Create(szShpPath, 0, 0, 0, GDT_Unknown, NULL);
			OGRLayer* child_layer = child_ds->CreateLayer(pszLayerName, layer->GetSpatialRef(), layer->GetGeomType(), NULL);

			int* field_array = new int[fieldCount];
			for (size_t i = 0; i < fieldCount; i++) {
				child_layer->CreateField(featureDefn->GetFieldDefn(i));
				field_array[i] = i;
			}

			for (size_t i = 0; i < ids.size(); i++) {
				feature = layer->GetFeature(atoll(ids[i].c_str()));
				OGRGeometry* ogrGeom = feature->GetGeometryRef();
				Geometry* geom = fromOGR(ogrGeom);
				if (geom != NULL) {
					try {
						if (geom->intersects(iter->second)) {
							std::unique_ptr<Geometry> res = geom->intersection(iter->second);
							GeometryTypeId geomType = res->getGeometryTypeId();
							if (geomType == GeometryTypeId::GEOS_LINESTRING || geomType == GeometryTypeId::GEOS_MULTILINESTRING) {
								OGRGeometry* ogrGeom = toOGR(std::move(res));
								if (ogrGeom != NULL) {
									OGRFeature* child_feature = OGRFeature::CreateFeature(featureDefn);
									child_feature->SetGeometryDirectly(ogrGeom);
									child_feature->SetFieldsFrom(feature, field_array);
									child_layer->CreateFeature(child_feature);
									OGRFeature::DestroyFeature(child_feature);
								}
							}
						}
					} catch (...) {
					}

					m_factory->destroyGeometry(geom);
				}		
				OGRFeature::DestroyFeature(feature);
			}
			delete[]field_array;
			GDALClose(child_ds);
		}
	}

	delete tree;
	delete bufferfile;
	delete diskfile;
	GDALClose(ds);
}
Region mapbuilding::getRect(Geometry* geom) {
	const Envelope* enve = geom->getEnvelopeInternal();
	double plow[2] = { 0 };
	double phigh[2] = { 0 };
	plow[0] = enve->getMinX();
	plow[1] = enve->getMinY();
	phigh[0] = enve->getMaxX();
	phigh[1] = enve->getMaxY();

	return Region(plow, phigh, 2);
}
Region mapbuilding::getRect(OGRGeometry* geom) {
	OGREnvelope enve;
	geom->getEnvelope(&enve);
	double plow[2] = { 0 };
	double phigh[2] = { 0 };
	plow[0] = enve.MinX;
	plow[1] = enve.MinY;
	phigh[0] = enve.MaxX;
	phigh[1] = enve.MaxY;

	return Region(plow, phigh, 2);
}
Geometry* mapbuilding::fromOGR(OGRGeometry* ogrGeom) {
	Geometry* geom = NULL;
	if (ogrGeom == NULL) {
		return NULL;
	}
	OGRwkbGeometryType geomType = ogrGeom->getGeometryType();
	if (geomType == wkbLineString) {
		OGRLineString* line = (OGRLineString*)ogrGeom;
		int pointNums = line->getNumPoints();
		if (pointNums > 1) {
			CoordinateArraySequence* cs = new CoordinateArraySequence();
			for (size_t i = 0; i < pointNums; i++) {
				cs->add(Coordinate(line->getX(i), line->getY(i)));
			}
			geom = m_factory->createLineString(cs);
		}
	}

	return geom;
}	
OGRGeometry* mapbuilding::toOGR(std::unique_ptr<Geometry> geom) {
	OGRGeometry* ogrGeom = NULL;
	string wkt = geom->toString();
	OGRGeometryFactory::createFromWkt(wkt.c_str(), NULL, &ogrGeom);
	return ogrGeom;
}
bool mapbuilding::isDirectoryExist(const char* pszDirectory) {	
	VSIStatBufL statInfo;
	int res = VSIStatExL(pszDirectory, &statInfo, VSI_STAT_EXISTS_FLAG);
	return res == 0;

	/*
#if defined(_WIN32) || defined(_WIN64)
	if (_access(pszDirectory, 0) != 0) {
		res = false;
	}
#else
	if (access(pszOutDirectory, F_OK) != 0) {
		res = false;
	}
#endif

	return res;
	*/
}
void mapbuilding::processingZone(const char* pszOriginPath,const char* pszOutputDir) {
	if (pszOriginPath == NULL || pszOutputDir == NULL) {
		return;
	}

	vector<string> utm_zones;

	char** papszRetList = VSIReadDir(pszOutputDir);
	int directory_count = CSLCount(papszRetList);
	for (size_t i = 0; i < directory_count; i++) {
		if (strcmp(papszRetList[i], ".") == 0 || strcmp(papszRetList[i], "..") == 0) {
			continue;
		}

		string filePath = pszOutputDir;
		filePath += "/";
		filePath += papszRetList[i];

		parse_mmt(pszOriginPath, filePath.c_str());
		utm_zones.emplace_back(papszRetList[i]);
	}

	mergeLayer(utm_zones, pszOutputDir);

	string outputPath = pszOutputDir;
	outputPath += "/output";

	addLinesToOsm(outputPath.c_str());
	addPolygonsToOsm(outputPath.c_str());
	addOtherOsm(outputPath.c_str());

	CSLDestroy(papszRetList);
	//VSIRmdirRecursive(pszOutputDir);
}
void mapbuilding::mergeLayer(vector<string>& utm_zones, const char* pszOutputDir) {
	string outputPath = pszOutputDir;
	outputPath += "/output";
	VSIRmdirRecursive(outputPath.c_str());
	VSIMkdir(outputPath.c_str(), 0755);

	map<string, GDALDataset*> dstDs = { {"roadboundary",NULL},{"lanemarking",NULL},{"stopline",NULL},{"roadmark",NULL},{"crosswalk",NULL},{"aois",NULL} };

	GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	OGRSpatialReference spatialRef;
	spatialRef.importFromEPSG(4326);

	auto iter = dstDs.begin();
	for (; iter != dstDs.end(); ++iter) {
		string filePath = outputPath;
		filePath += "/";
		filePath += iter->first;
		filePath += ".shp";
		OGRwkbGeometryType layerType = wkbLineString;
		if (iter->first == "roadmark" || iter->first == "crosswalk" || iter->first == "aois") {
			layerType = wkbPolygon;
		}

		GDALDataset* ds = driver->Create(filePath.c_str(), 0, 0, 0, GDT_Unknown, NULL);
		OGRLayer* layer = ds->CreateLayer(iter->first.c_str(), &spatialRef, layerType, NULL);
		if (iter->first == "crosswalk" || iter->first == "aois") {
			OGRFieldDefn area_field("area", OFTReal);
			layer->CreateField(&area_field);
		} else if (iter->first == "stopline") {
			OGRFieldDefn length_field("length", OFTReal);
			layer->CreateField(&length_field);
		} else {
			OGRFieldDefn field("id", OFTInteger);
			layer->CreateField(&field);
		}
		iter->second = ds;
	}

	for (size_t i = 0; i < utm_zones.size(); i++) {
		auto iter = dstDs.begin();
		for (; iter != dstDs.end(); ++iter) {
			char szFilePath[512] = {0};
			sprintf(szFilePath, "%s/%s_output/%s_res.shp", pszOutputDir, utm_zones[i].c_str(), iter->first.c_str());
			GDALDataset* ds = (GDALDataset*)GDALOpenEx(szFilePath, GDAL_OF_VECTOR | GDAL_OF_READONLY, NULL, NULL, NULL);
			if (ds != NULL) {
				OGRLayer* layer = ds->GetLayer(0);
				OGRFeature* feature = NULL;
				while ((feature = layer->GetNextFeature()) != NULL) {
					OGRLayer* dstLayer = iter->second->GetLayer(0);
					OGRFeature* dstFeature = OGRFeature::CreateFeature(dstLayer->GetLayerDefn());
					dstFeature->SetFrom(feature);
					dstLayer->CreateFeature(dstFeature);
					OGRFeature::DestroyFeature(dstFeature);
					OGRFeature::DestroyFeature(feature);
				}
				GDALClose(ds);
			}
		}
	}

	iter = dstDs.begin();
	for (; iter != dstDs.end(); ++iter) {
		GDALClose(iter->second);
	}
}
bool mapbuilding::splitZone(const char* pszPath, const char* pszOutputDir) {
	char** papszRetList = VSIReadDir(pszPath);
	int directory_count = CSLCount(papszRetList);
	if (directory_count - 2 == 0) {
		char szMsg[512] = {0};
		sprintf(szMsg, "the %s is empty", pszPath);
		logUtil::log(szMsg);
		CSLDestroy(papszRetList);
		return false;
	}

	char szMsg[512] = { 0 };
	sprintf(szMsg, "directory: %s the number of folders is:%d", pszPath, directory_count - 2);
	logUtil::log(szMsg);

	for (size_t i = 0; i < directory_count; i++) {
		if (strcmp(papszRetList[i], ".") == 0 || strcmp(papszRetList[i], "..") == 0) {
			continue;
		}

		string strEgopose = pszPath;
		strEgopose += "/";
		strEgopose += papszRetList[i];
		strEgopose += "/position/_mla_egopose.json";

		map<int, json_object*> utmEgoposeFiles;
		splitEgoposeFile(strEgopose.c_str(), utmEgoposeFiles);

		auto iter = utmEgoposeFiles.begin();
		for (; iter != utmEgoposeFiles.end(); ++iter) {
			char dstDir[512] = { 0 };
			char jsonFilePath[512] = { 0 };
			char szUtmDir[512] = { 0 };
			sprintf(szUtmDir, "%s/%d", pszOutputDir, iter->first);
			if (!isDirectoryExist(szUtmDir)) {
				VSIMkdir(szUtmDir, 0755);
			}
			sprintf(dstDir, "%s/%d/%s", pszOutputDir, iter->first, papszRetList[i]);
			sprintf(jsonFilePath, "%s/%d/%s/_mla_egopose.json", pszOutputDir, iter->first, papszRetList[i]);
			VSIMkdir(dstDir, 0755);
			json_object_to_file_ext(jsonFilePath, iter->second, JSON_C_TO_STRING_PRETTY);
			json_object_put(iter->second);
		}
	}

	CSLDestroy(papszRetList);
	return true;
}
void mapbuilding::copyFile(const char* srcPath, const char* dstPath) {
	ifstream src(srcPath, std::ios::binary);
	if (!src.is_open()) {
		return;
	}
	ofstream dst(dstPath, std::ios::binary);
	dst << src.rdbuf();
	src.close();
	dst.close();
}
void mapbuilding::splitEgoposeFile(const char* pszPath, map<int, json_object*>& utmEgoposeFiles) {
	json_object* root = json_object_from_file(pszPath);
	if (root != NULL) {
		size_t size = json_object_array_length(root);
		for (size_t i = 0; i < size; i++) {
			json_object* child = json_object_array_get_idx(root, i);
			json_object* position = json_object_object_get(child, "position");
			json_object* position_global = json_object_object_get(position, "position_global");
			json_object* longitude = json_object_object_get(position_global, "longitude");
			json_object* latitude = json_object_object_get(position_global, "latitude");
			double lon = json_object_get_double(longitude);
			double lat = json_object_get_double(latitude);
			int zone = getZone(lon, lat);
			if (zone != -1) {
				auto iter = utmEgoposeFiles.find(zone);
				if (iter == utmEgoposeFiles.end()) {
					json_object* child_file = json_object_new_array();
					json_object* child_new = NULL;
					json_object_deep_copy(child, &child_new, NULL);
					json_object_array_add(child_file, child_new);
					utmEgoposeFiles[zone] = child_file;
				} else {
					json_object* child_new = NULL;
					json_object_deep_copy(child, &child_new, NULL);
					json_object_array_add(iter->second, child_new);
				}
			}
		}
		json_object_put(root);
	}
}
int mapbuilding::getZone(double x, double y) {
	int zone = -1;
	geos::geom::Point* point = m_factory->createPoint(Coordinate(x, y));
	auto iter = m_utmZone.begin();
	for (; iter != m_utmZone.end(); ++iter) {
		if (iter->second->contains(point)) {
			zone = iter->first;
			break;
		}
	}
	m_factory->destroyGeometry(point);
	return zone;
}
void mapbuilding::buildMapping(const char* pszInDirectory, const char* pszOutDirectory) {
	if (!splitZone(pszInDirectory, pszOutDirectory)) {
		logUtil::log("processing failed");
		return;
	}

	processingZone(pszInDirectory, pszOutDirectory);
	logUtil::log("end of job");
}