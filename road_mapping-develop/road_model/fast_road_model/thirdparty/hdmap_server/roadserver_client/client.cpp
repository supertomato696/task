// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

#include <atomic>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <thread>
#include <experimental/filesystem>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "json/json.h"
#include "brpc/channel.h"
#include <brpc/policy/gzip_compress.h>

#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "proxy/link_proxy.h"
#include "proxy/tile_proxy.h"
#include "proj_helper.h"

DEFINE_string(branch, "test_beijing_full", "branch");
DEFINE_int32(method, 3, "3 for new format file, 2 for server, 1 for local file");
DEFINE_string(file, "/mnt/d/docker_tmp/20210308.csv"/*"/mnt/d/code/20210621.csv"*/, "input file");
DEFINE_string(path, "/mnt/d/code/1101/", "");
DEFINE_string(server, "172.21.207.124:8083", "IP Address of server");
DEFINE_string(roadlist, "http://autopilot-roads.biyadi.com/lx1/api/v1/hdmapmt/getroadslist?size=30000&page=", "Server address for road list");
DEFINE_int32(timeout_ms, 600000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_int64(reqver, 0, "max version retrieved");
DEFINE_string(basedir, "./", "base dir for download/upload files");

using namespace RoadPB;

struct range {
    int64_t range_id;
    int64_t feature_id;
    int range_type;
    double start_position;
    double end_position;

    void parse(const char* buf) {
        char* ptr = (char*)buf;
        range_id = strtoll(buf, &ptr, 10);
        ptr++;
        feature_id = strtoll(ptr, &ptr, 10);
        ptr++; 
        range_type= strtol(ptr, &ptr, 10);
        ptr++;
        start_position = strtod(ptr, &ptr);
        ptr++;
        end_position = strtod(ptr, &ptr);
    }
};

std::unordered_map<std::string, int> key_map = { {"right_lanes", 1}, {"left_lanes", 2}, {"link_kind", 3},
                                                    {"net_grade", 4}, {"pavement_info", 5}, {"lane_count", 6},
                                                    {"link_type", 7}, {"passing_type", 8}, {"if_urban", 9},
                                                    {"grade", 10}, {"road_grade", 11}, {"multiply_digitized", 12} };
struct link_attribue {
    int64_t link_id;
    int64_t range_id;
    int key;
    int start_pos_value;
    int end_pos_value;
    double attribute_length;
    double attribute_coverage;

    void parse(const char* buf) {
        char* ptr = (char*)buf;
        link_id = strtoll(buf, &ptr, 10);
        ptr++;
        range_id = strtoll(ptr, &ptr, 10);
        ptr++;
        auto p = strchr(ptr, ',');
        std::string ks(ptr, p - ptr);
        if (key_map.find(ks) == key_map.end()) {
            LOG(ERROR) << "Unknown attribute_key " << ks << " found: " << buf;
            key = 0;
        }
        else {
            key = key_map[ks];
        }
        ptr = p + 1;
        start_pos_value = strtol(ptr, &ptr, 10);
        ptr++;
        end_pos_value = strtol(ptr, &ptr, 10);
        ptr++;
        attribute_length = strtod(ptr, &ptr);
        ptr++;
        attribute_coverage = strtod(ptr, &ptr);
    }
};
struct link_name {
    int64_t name_id;
    int64_t link_id;
    int64_t range_id;
    std::string name;
    int name_type;
    int name_seq;
    int name_language_code;

    void parse(const char* buf) {
        char* ptr = (char*)buf;
        name_id = strtoll(buf, &ptr, 10);
        ptr++;
        link_id = strtoll(ptr, &ptr, 10);
        ptr++;
        range_id = strtoll(ptr, &ptr, 10);
        ptr++;
        auto p = strchr(ptr, ',');
        name.assign(ptr, p);
        ptr = p + 1;
        name_type = strtol(ptr, &ptr, 10);
        ptr++;
        name_seq = strtol(ptr, &ptr, 10);
        ptr++;
        name_language_code = strtol(ptr, &ptr, 10);
    }
};
struct link_csv {
    int64_t link_id;
    std::deque<std::pair<double, double>> geoms;
    char* fuzzy_z;
    int64_t link_snode;
    int64_t link_enode;
    int direction;
    char* transition_area_flag;
    int toll_area;
    int reversible_flag;
    int theoretical_flag;
    double link_length;
    int open_status;
    char* open_time;
    char* cond_id;
    int province_code;

    void parse(const char* buf) {
        char* ptr = (char*)buf;
        link_id = strtoll(buf, &ptr, 10);
        ptr++;
        char* stp = strchr(ptr, '(');
        char* edp = strchr(ptr, ')');
        ptr = stp + 1;
        int tid = -1;
        while (ptr && ptr < edp) {
            double x = strtod(ptr, &ptr);
            ptr++;
            double y = strtod(ptr, &ptr);
            ptr++;
            geoms.push_back(std::make_pair(x, y));
        }
        fuzzy_z = strchr(ptr, ',') + 1;
        ptr = strchr(fuzzy_z, ',') + 1;
        link_snode = strtoll(ptr, &ptr, 10);
        ptr++;
        link_enode = strtoll(ptr, &ptr, 10);
        ptr++;
        direction = strtol(ptr, &ptr, 10);
        transition_area_flag = ptr;
        ptr = strchr(ptr, ',') + 1;
        toll_area = strtol(ptr, &ptr, 10);
        ptr++;
        reversible_flag = strtol(ptr, &ptr, 10);
        ptr++;
        theoretical_flag = strtol(ptr, &ptr, 10);
        ptr++;
        link_length = strtod(ptr, &ptr) / 100;
        ptr++;
        open_status = strtol(ptr, &ptr, 10);
        ptr++;
        open_time = ptr;
        ptr = strchr(ptr, ',') + 1;
        cond_id = ptr;
        ptr = strchr(ptr, ',') + 1;
        province_code = strtol(ptr, &ptr, 10);
    }
};

class LinkExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LinkProxy> {
public:
    virtual data_access_engine::LinkProxy* create() {
        return new data_access_engine::LinkExtProxy();
    }
    virtual data_access_engine::LinkProxy* create(data_access_engine::FeatureProxyBase* p) {
        return new data_access_engine::LinkExtProxy(p);
    }
};

int main(int argc, char *argv[]) {
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);
    auto conf = data_access_engine::ConfigAddress::get_instance();
    conf->set_road_server_address(FLAGS_server);
    conf->set_road_server_download_branch(FLAGS_branch);
    conf->set_road_server_upload_branch(FLAGS_branch);
    auto mgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    mgr->init_tiles_by_id({1234567});
    delete data_access_engine::FeatureProxyBase::register_creator<data_access_engine::LinkProxy>(new LinkExtProxyCreator());
    auto proj = data_access_engine::DAEInterface::get_instance()->get_projection_helper();
    proj->set_destination_Proj4_string("+proj=geocent +datum=WGS84");
    data_access_engine::TileInfoPtr tmp_tile(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::TileInfoProxy>()); 
    tmp_tile->relink_parent();
    std::vector<std::pair<double, double>> ranges = { {116.0005,39.6723},{117.919,41.0151},
                                                      {120.8743,30.5918},{122.1185,31.9149},
                                                      {113.066,22.462},{114.093,23.971},
                                                      {113.6975,22.1365},{114.6588,22.966},
                                                      {106.1005,29.1905},{106.9038,29.8942}
                                                    };
    /*std::vector<const char*> branchs = { "road_beijingbeiwuhuanliuhuanqujian2_1629294104", "road_beijingshunyiqu2_1629290255", "road_beijingbeiwuhuanliuhuanqujian1_1629292656", "beijingdongwuhuanwaitongzhou3chongpao", "road_beijingshunyiqu1_1629272214", "road_beijingdongwuhuanwaitongzhou4_1629175757", "beijingdongwuhuanwaitongzhou2chongpao", "road_beijingbeiwuhuanchongpao_1628558340", "road_beijingdongwuhuanwaitongzhouqu_1628574469", "road_beijingbeiwuhuanshujuyanzheng_1628253062", "road_beijingnandongsanhuan_1628248868", "road_beijingdongwuhuanwaizhulu_1628469752", "road_beijingxisanhuan_1628232410", "road_beijingdongwuhuan_1628158620", "road_beijingbeisanhuan_1628159260", "road_wuhuan01_1628069603", "road_beijingdongnanwuhuan_1627975578", "road_beijingnanwuhuan_1627974808", "road_beijingxiwuhuan1_1627973977", "road_beijingxibeifangxiangwuhuan_1627973407", "road_beijingzhengbeifangxiangwuhuan2_1627554348", "road_beijingdongbeifangxiangwuhuan_1627552725", "road_beijingzhengbiefangxiangwuhuan1_1627554086"};
    std::vector<std::vector<int>> tidss = {
        {329546536,329546537,329546539,329546542,329546543,329546628,329546629,329546640,329546642,329546648,329546649,329546650,329546651,329546672,329546673,329546676,329546677,329546720,329546721,329546699,329546702,329546700,329546701,329546695,329546706,329546674,329546675,329546680,329546682,329568528,329568530,329568536,329568537,329568539,329568542,329568540,329568538,329568543,329568586,329568587,329568590,329568591,329568560,329568561,329568563,329568569,329568572,329568571,329568570,329568568,329568573,329568575,329568616,329568618,329568619,329568622,329568623,329568634,329568635,329568657,329568659,329568665,329568667,329568689,329568697,329568699,329568691,329569041,329568525,329568519,329568518,329568516,329546670,329546668,329546665,329546659,329546662,329546657,329546635,329546634,329546632,329546626,329546624,329546538},
        {329570120,329570122,329570079,329570067,329570073,329570075,329570097,329570101,329570100,329570099,329570102,329570105,329570107,329570108,329570109,329570111,329570154,329570193,329570240,329570245,329570159,329570157,329570151,329570149,329570127,329570125,329570136,329570138,329570180,329570094,329570091,329570177,329570089,329570092,329570083,329570086,329570084,329570081,329570059,329570062,329570060,329570057,329570051,329570054,329570052,329570053,329570064,329570049,329569707,329569706,329569533,329569535,329569532,329569529,329569523,329569522,329569511,329569508,329569487,329569498,329569509,329569499,329569502,329569505,329569483,329569486,329569480,329569482,329569437,329569431,329569430,329569428,329569425,329569342,329569343,329569386,329569384,329569341,329569378,329569335,329569333,329569376},
        {329546599,329546616,329546618,329546619,329546705,329546707,329546710,329546708,329546711,329547392,329547394,329547393,329547395,329547398,329547396,329546709,329547397,329547444,329547446,329547443,329547441,329547442,329547431,329547430,329547427,329547425,329547424,329546741,329547402,329546719,329546716,329546717,329547436,329547438,329569284,329569286,329569292,329569293,329569295,329569317,329569319,329569325,329569336,329569338,329569339,329569424,329569413,329569412,329569316,329569313,329569312,329568629,329569290,329568607,329568606,329569409,329569408,329568725,329568639,329568638,329568724},
        {329546795,329546798,329546799,329546793,329546797,329546810,329546808,329546809,329546812,329546813,329546856,329546807,329546815,329546858,329546901,329546903,329546909,329546911,329546933,329546935,329546941,329546940,329546934,329546978,329546979,329546985,329546988,329546989,329547000,329547001,329547002,329547003,329547006,329547007,329547178,329547179,329547182,329547183,329547194,329547195,329547198,329547199,329547242,329547243,329547246,329547247,329547258,329547259,329547262,329547263,329590954,329590955,329590958,329590959,329590970,329590971,329590974,329590975,329591018,329591019,329591022,329591364,329591361,329591360},
        {329569578,329569579,329569582,329569583,329569594,329569595,329569598,329569596,329569593,329569592,329569587,329569585,329569597,329569640,329569641,329569646,329569644,329569647,329569733,329569744,329569745,329569746,329569735,329569741,329569752,329569754,329569776,329569765,329569777,329569780,329569680,329569682,329569688,329569677,329569676,329569673,329569672,329569674,329569503,329569501,329569495,329569493,329569696,329569698,329569699,329569702,329569700,329569701,329569712,329569714,329569713,329569716,329569717,329569760,329569761,329569764,329569718,329569715,329569721,329569720,329569724,329569725,329569727,329570069,329569723,329570065,329570068,329570112,329570114,329570113,329570115,329570071,329570116,329570117,329569778,329569784,329569786,329570128,329570130,329570119},
        {329547191,329547234,329547232,329547189,329547167,329547240,329547166,329547164,329547161,329547155,329547158,329547154,329547159,329547202,329547203,329547143,329547142,329547140,329547152,329547137,329547051,329547066,329547064,329547049,329547052,329547046,329547047,329547053,329547058,329547059,329547054,329547206,329547207,329547218,329547219,329547217,329547220,329547222,329547221,329547223,329590914,329590915,329590918,329590921,329590925,329590937,329590924,329590936},
        {329546763,329546766,329546767,329546778,329546779,329546782,329546783,329546801,329546804,329546805,329546848,329546849,329546852,329546853,329546842,329546843,329546864,329546865,329546868,329546869,329547040,329547041,329547044,329547045,329547023,329547034,329547032,329547056,329547057,329547035,329547060,329547038,329547036,329547061,329547039,329547106,329547104,329547082,329547107,329547105,329547083,329547110,329547111,329547122,329547123,329547086,329547087,329547098,329547099,329547102,329547103,329590794,329547128,329547129,329547132,329547133,329590824,329590826,329590827,329590830,329590831,329590842,329590843,329590846,329590847,329590890,329590891,329590894,329590895,329590816,329590817,329590820,329590821,329590832,329590798,329590799,329590822,329590828,329590834,329590835,329590838,329590839,329590882,329590888,329590889,329590892,329590893},
        {329546301,329546295,329546344,329546338,329546345,329546339,329546342,329546343,329546354,329546355,329546358,329546359,329546357,329546530,329546528,329546531,329546529,329546532,329546533,329546546,329546544,329546522,329546545,329546523,329546548,329546526,329546524,329546525,329546568,329546569,329546519,329546518,329546562,329546560,329546561,329546516,329546517,329546563,329546219,329546222,329546220,329546221,329546215,329546226,329546213,329546224,329546202,329546203,329546201,329546195,329546198,329546196,329546197},
        {329546755,329546758,329546759,329546770,329546771,329546774,329546775,329546818,329546819,329546822,329546823,329546839,329546837,329546838,329546836,329546835,329546834,329546841,329546844,329547008,329546832,329546833,329546821,329546845,329547009,329547012,329547013,329547024,329547025,329547026,329547027,329547030,329547031,329547074,329547075,329547076,329547078,329547077,329545710,329547073,329547084,329547085,329547096,329547097,329547100,329547101,329590792,329590793,329590796,329590791,329590797,329547081,329546761,329546764,329546765,329546776,329546777,329546780,329546781,329546824,329547017,329547019,329547016},
        {329546343,329546342,329546526,329546516,329546519,329546561,329546545,329546571,329546221,329546215,329546532,329546357,329546295},
        {329541497,329541500,329541502,329541503,329544234,329544235,329544238,329544239,329544250,329544251,329544254,329544255,329544298,329544299,329544302,329544388,329544303,329544389,329544314,329544400,329544401,329544404,329544405,329544576,329544407,329544578,329544584,329544579,329544585,329544587,329544586,329544609,329544611,329544617,329544619,329545985,329545987,329545986,329545993,329545995,329546017,329546019,329546025,329546027,329546113},
        {329545388,329545389,329545400,329545401,329545404,329545405,329545448,329545449,329545452,329545464,329545453,329545465,329545468,329545469,329545640,329545641,329545350,329545351,329545362,329545363,329545366,329545367,329545410,329545411,329545414,329545415,329545426,329545425,329545428,329545342,329545341,329545343,329545429,329545512,329545514,329545513,329545510,329545516,329545511,329545522,329545523,329545528,329545529,329545530,329545616,329545602,329545600,329545608,329545609,329545611,329545614,329545615,329545637,329545639,329545650,329545644,329545645,329545656,329545657,329545651,329545617,329545619,329545521,329545524,329545526,329545525,329545625,329545627,329545628,329545630,329545631,329545652,329545653,329545674,329545654,329545660,329545661,329545704,329545705,329545708,329545709,329545720,329545722,329545711,329545675,329545672,329545673,329545676,329545677,329545721,329545715,329545723,329545726,329545643,329545658,329545251,329545249,329545252,329545253,329545264,329545231,329545242,329545243,329545246,329545244,329545245,329545238,329545237,329545239,329545280,329545282,329545281,329545283,329545288,329545290,329545289,329545291,329545313,329545316,329545319,329545318,329545336,329545337,329545339,329545424,329545292,329545293,329545304,329545305,329545308,329545309,329545480,329545481,329545484,329545485,329545496,329545498,329545520,329545490,329545284,329544942,329544943,329544954,329544952,329544953,329544874,329544875,329545217,329544878,329544877,329544879,329544888,329544889,329544882,329544883,329544886,329544887,329544930,329544931,329544934,329544935,329544946,329544947,329544950,329547092,329547093,329590784,329590785,329590788,329590789,329589423,329589422,329590800,329590801,329589435,329589438,329589439,329589482,329589420,329589414,329589412,329589413,329589391,329589389,329589400,329589394,329589392,329589306,329545688,329545682,329545683,329545686,329545687,329589376,329545685,329545488,329545146,329545144,329545138,329545136,329545125,329545103,329545101,329545112,329545113,329545116,329545117,329545160,329545161,329545163,329545162,329545164,329545166,329545167,329545178,329544948,329544949,329544951,329545122,329545123,329545120,329545098,329545099,329545097,329545100,329545094,329545092,329545095,329545093,329545104,329545058,329545059,329545062,329545056,329545057,329545060,329545038,329545039,329545037,329545018,329545019,329545017,329545020,329545015,329545021,329545014,329545009,329545008,329544997,329544974,329544996,329544971},
        {329543057,329543059,329542971,329542969,329542961,329542963,329542939,329542937,329542931,329542929,329541563,329541561,329541555,329541553,329541531,329541529,329541532,329541523,329541526,329541524,329541438,329541439,329541437,329541480,329541481,329541484,329541485,329541496},
        {329546752,329546754,329546760,329546762,329546101,329546784,329546103,329546786,329546109,329546111},
        {329543065,329543067,329543070,329543071,329543114,329543115,329543118,329543119,329543130,329543131,329543134,329543135,329545866,329545867,329545889,329545870,329545892,329545871,329545893,329545882,329545904,329545883,329545905,329545908,329545909,329545952,329545953,329545956,329545957,329545968,329545969,329545948,329545950,329545949,329545946,329545947,329545943,329546120,329546114,329546115},
        {329546568,329546339,329546525,329546569,329546201,329546528,329546530,329546529,329546531,329546338,329546517,329546562,329546563,329546544,329546219,329546220,329546222,329546202,329546195},
        {329544122,329544123,329544126,329544468,329544469,329544512,329544513,329544515,329544518,329544519,329544525,329544536,329544538,329544539,329544561,329544564,329544565,329544567,329545250,329545256,329545257,329545259,329545344,329545345,329545346,329545347,329545352,329545353,329545354,329545355,329545377,329545376,329545379,329545378,329545384,329545385,329545386},
        {329540822,329540823,329540821,329540992,329540993,329540996,329540998,329540999,329541010,329541016,329541011,329541017,329541020,329541014,329541015,329541058,329541059,329541062,329541063,329541061,329541074,329541075,329541078,329541079,329543810,329543808,329543809,329543723,329543721,329543718,329543719,329543730,329543736,329543737,329543740,329543741,329543786,329543784,329543787,329543873,329543876,329543878,329543884,329543885,329543898,329543887,329543899,329543902,329543903,329544074,329544075,329544097,329544100,329544102,329544103,329544109,329544120},
        {329542832,329542810,329542811,329542808,329542802,329542800,329542714,329542703,329542789,329542701,329542695,329542693,329542671,329542669,329542663,329542661,329541295,329541293,329541287,329541284,329541286,329541285,329541262,329541261,329541263,329541255,329541264,329541266,329541265,329541267,329541178,329541177,329541179,329541172,329541174,329541173,329541175,329541171,329541260,329541151,329541194,329541193,329541195,329541196,329541191,329541190,329541197,329541189,329540858,329540857,329540859,329541200,329540851,329540849,329540830,329540827,329540828},
        {329543457,329543456,329543263,329543285,329543262,329543260,329543257,329543256,329543245,329543244,329543235,329543241,329543234,329543238,329543191,329543188,329543190,329543185,329542843,329542841,329542835,329542833},
        {329546300,329546294,329546297,329546291,329546296,329546290,329546285,329546279,329546278,329546284,329546281,329546275,329546274,329546280,329543543,329543549,329543548,329543542,329543539,329543545,329543544,329543538,329543527,329543533,329543532,329543526,329543523,329543528,329543522,329543479,329543476,329543478,329543472,329543473,329543475,329543461,329543460,329543529},
        {329546524,329546526,329546525,329546527,329546548,329546516,329546518,329546517,329546519,329546560,329546562,329546561,329546563,329546544,329546546,329546545,329546547,329546522,329546523,329546568,329546570,329546569,329546571,329546219,329546220,329546222,329546221,329546223,329546213,329546215,329546224,329546226,329546202,329546203,329546201,329546195,329546198,329546196,329546197},
        {329546533,329546532,329546529,329546528,329546530,329546531,329546356,329546358,329546357,329546359,329546354,329546355,329546342,329546343,329546338,329546339,329546295,329546301,329546344,329546345}
    };
    {
        std::vector<int> types = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
        for (size_t i = 0; i < branchs.size(); i++) {
            auto& br = branchs[i];
            auto& tids = tidss[i];
            conf->set_road_server_download_branch(br);
            conf->set_road_server_upload_branch("bj010");
            int64_t read_ver = 0;
            mgr->merge_tiles(tids, types, "merger", 0, 1, read_ver);
        }
        return 0;
    }*/
    std::vector<double> link_length(5);
    if (FLAGS_method == 2) {
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::LinkProxy> > id2link(1024000);
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::NodeProxy> > id2node(1024000);
        std::unordered_map<int64_t, std::pair<int64_t, int64_t> > lid2nid(1024000);
        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        mgr->set_current_version(timestamp);
        int page = 1;
        int total = 0;
        int step = 5;
                
        do {
            std::unordered_set<int> tid_set(102400); 
            std::string url = FLAGS_roadlist + std::to_string(page);
            LOG(INFO) << "Load " << url;
            brpc::Channel channel;
            brpc::ChannelOptions options;
            options.connect_timeout_ms = 600 * 1000;
            options.timeout_ms = 600 * 1000;
            options.protocol = brpc::PROTOCOL_HTTP;  // or brpc::PROTOCOL_H2
            if (channel.Init(url.c_str(), &options) != 0) {
                LOG(ERROR) << "Fail to initialize channel to " << url;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            brpc::Controller cntl;
            cntl.http_request().uri() = url;
            channel.CallMethod(NULL, &cntl, NULL, NULL, NULL/*done*/);
            
            auto resp = cntl.response_attachment().to_string();
            JSONCPP_STRING err;
            Json::Value root;
            Json::CharReaderBuilder builder;
            const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
            if (!reader->parse(resp.c_str(), resp.c_str() + resp.length(), &root, &err))  {
                LOG(ERROR) << "Fail to parse response from " << url << " : " << err;
                continue;
            }
            page++;
            total = root["data"]["page_num"].asInt();
            auto ls = root["data"]["list"];
            LOG(INFO) << "Finish load " << url << " : " << root["data"]["current_page"] << " / " << total << " links:" << ls.size();
            for (int i = 0; i < ls.size(); ++i) {
                auto jr = ls[i];
                auto link = tmp_tile->links().add();
                int64_t lid = jr["road_id"].asInt64();                
                link->set_link_type(jr["road_grade"].asInt());
                link->set_road_class(jr["road_class"].asInt());
                link->set_direction(jr["is_single_dir"].asInt());
                //link->set_lane_count(jr["lane_count"].asInt());
                auto name = link->mutable_names()->add();
                name->set_name(jr["name"].asString());
                char* ptr = (char*)jr["geometry"].asCString();
                char* stp = strchr(ptr, '(');
                char* edp = strchr(ptr, ')');
                ptr = stp + 1;
                int tid = -1;
                std::unordered_set<int> tids;
                while (ptr && ptr < edp) {
                    double x = strtod(ptr, &ptr);
                    ptr++;
                    double y = strtod(ptr, &ptr);
                    ptr++;
                    auto pt = link->mutable_geom()->mutable_pts()->add();
                    pt->set_x(x);
                    pt->set_y(y);
                    pt->set_z(0);
                    data_access_engine::Vector3D pos = *pt;
                    int t = mgr->WGS84_to_tileID(pos);
                    if (t <= 0) {
                        continue;
                    }
                    if (tid <= 0) {
                        tid = t;
                    }
                    tid_set.insert(t);
                }
                if (tid <= 0 || !link->geom() || link->geom()->pts().empty()) {
                    LOG(ERROR) << "Link without geom found: " << lid;
                    continue;
                }
                link->mutable_id()->set_value(tid, RoadPB::FeatureID::LINK, lid, timestamp);

                auto pres = jr["pre_adjacency"].asString();
                auto nexts = jr["next_adjacency"].asString();                
                if (id2link.count(lid) > 0) {
                    //LOG(INFO) << "Duplicate link found " << lid;
                    auto tl = id2link[lid];
                    if (tl->geom()->pts().size() < link->geom()->pts().size()) {
                        LOG(ERROR) << "Replace link " << lid << " from " << tl->geom()->pts().size() << " to " << link->geom()->pts().size() << " points!";
                        tl->mutable_geom()->mutable_pts()->clear();
                        for (auto& pt : link->geom()->pts()) {
                            data_access_engine::Vector3D p = *pt;
                            tl->mutable_geom()->mutable_pts()->add()->set(p);
                        }
                    }
                    else if (tl->geom()->pts().size() > link->geom()->pts().size()) {
                        LOG(ERROR) << "Discard link " << lid << " from " << tl->geom()->pts().size() << " to " << link->geom()->pts().size() << " points!";;
                    }
                    continue;
                }
                id2link[lid] = link;
                
                auto sp = link->global_stp_pos(mgr);
                auto ep = link->global_edp_pos(mgr);
                if (!pres.empty()) {
                    std::vector<int64_t> ids;
                    ptr = (char*)pres.c_str();
                    stp = strchr(ptr, '[');
                    edp = strchr(ptr, ']');
                    ptr = stp + 1;
                    
                    std::shared_ptr<data_access_engine::NodeProxy> node;
                    int64_t nodeid = -1;
                    if (lid2nid.find(lid) != lid2nid.end()) {
                        auto& pr = lid2nid[lid];
                        if (pr.first > 0 && id2node[pr.first]) {
                            auto n = id2node[pr.first];
                            auto np = n->global_pos(mgr);
                            if ((np - sp).Length() < 1) {
                                node = n;
                                nodeid = pr.first;
                            }
                        }
                        if (!node && pr.second > 0 && id2node[pr.second]) {
                            auto n = id2node[pr.second];
                            auto np = n->global_pos(mgr);
                            if ((np - sp).Length() < 1) {
                                node = n;
                                nodeid = pr.second;
                            }
                        }
                        if (!node && pr.first > 0 && pr.second > 0) {
                            auto p1 = id2node[pr.first]->global_pos(mgr);
                            auto p2 = id2node[pr.second]->global_pos(mgr);
                            //LOG(INFO) << "Link " << lid << " (" << sp << ")-(" << ep << ") connect to node (" << p1 << ")-(" << p2 << ")";
                            if ((p1 - ep).Length() >= 1) {
                                //LOG(INFO) << "Link " << lid << " reset edp " << pr.first << " (" << p1 << ") for edp (" << ep << ")";
                                pr.first = 0;                                
                            }
                            if ((p2 - ep).Length() >= 1) {
                                //LOG(INFO) << "Link " << lid << " reset edp " << pr.second << " (" << p2 << ") for edp (" << ep << ")";
                                pr.second = 0;                                
                            }
                        }
                    }
                    while (ptr && ptr < edp) {
                        ptr = strchr(ptr, '"');
                        ptr += 1;
                        int64_t id = strtoll(ptr, &ptr, 10);
                        ptr += 1;
                        ids.push_back(id);
                        if (lid2nid.find(id) == lid2nid.end()) {
                            continue;
                        }
                        auto& pr = lid2nid[id];
                        if (!node && pr.first > 0 && id2node[pr.first]) {
                            auto n = id2node[pr.first];
                            auto np = n->global_pos(mgr);
                            if ((np - sp).Length() < 1) {
                                node = n;
                                nodeid = pr.first;
                            }
                        }
                        if (!node && pr.second > 0 && id2node[pr.second]) {
                            auto n = id2node[pr.second];
                            auto np = n->global_pos(mgr);
                            if ((np - sp).Length() < 1) {
                                node = n;
                                nodeid = pr.second;
                            }
                        }
                        if (!node && pr.first > 0 && pr.second > 0) {
                            auto p1 = id2node[pr.first]->global_pos(mgr);
                            auto p2 = id2node[pr.second]->global_pos(mgr);
                            //LOG(INFO) << "Link " << lid << " stp(" << sp << ") to prev link " << id << " disconnect with node (" 
                            //    << p1 << ")/" << pr.first << " - (" << p2 << ")/" << pr.second;                            
                        }
                    }
                    if (!node) {
                        node = tmp_tile->nodes().add();
                        auto nid = node->mutable_id();
                        nodeid = lid * 2;
                        nid->set_id(nodeid);
                        nid->set_type(RoadPB::FeatureID::NODE);
                        nid->set_tileid(link->id()->tileid());
                        nid->set_version(timestamp);
                        node->mutable_geom()->set_x(link->geom()->pts()[0]->x());
                        node->mutable_geom()->set_y(link->geom()->pts()[0]->y());
                        node->mutable_geom()->set_z(0);
                        id2node[nodeid] = node;                        
                    }
                    else {
                        tid_set.insert(node->id()->tileid());
                    }
                    link->mutable_snode_id()->set(node->id());
                    if (node->link_ids().find(link->id()) == node->link_ids().end()) {
                        node->mutable_link_ids()->push_back(link->id());
                    }
                    else {
                        LOG(ERROR) << "Duplicated link " << link->id()->to_string() << " found in " << node->id()->to_string();
                    }
                    if (!node->is_valid()) {
                        LOG(ERROR) << "Invalid node found " << node->id()->to_string();
                    }
                    ids.push_back(lid);
                    for (auto id : ids) {
                        auto& pr = lid2nid[id];
                        if (pr.first == nodeid || pr.second == nodeid) {

                        }
                        else if (pr.first <= 0) {
                            pr.first = nodeid;
                        }
                        else if (pr.second <= 0) {
                            pr.second = nodeid;
                        }
                        else {
                            //LOG(INFO) << "Link " << lid << " stp " << nodeid << " to link " << id << " connect to neither nodes " << pr.first << '/' << pr.second;
                        }
                    }
                }
                if (!nexts.empty()) {
                    std::vector<int64_t> ids;
                    ptr = (char*)nexts.c_str();
                    stp = strchr(ptr, '[');
                    edp = strchr(ptr, ']');
                    ptr = stp + 1;
                    
                    std::shared_ptr<data_access_engine::NodeProxy> node;
                    int64_t nodeid = -1;
                    if (lid2nid.find(lid) != lid2nid.end()) {
                        auto& pr = lid2nid[lid];
                        if (pr.first > 0 && id2node[pr.first]) {
                            auto n = id2node[pr.first];
                            auto np = n->global_pos(mgr);
                            if ((np - ep).Length() < 1) {
                                node = n;
                                nodeid = pr.first;
                            }
                        }
                        if (!node && pr.second > 0 && id2node[pr.second]) {
                            auto n = id2node[pr.second];
                            auto np = n->global_pos(mgr);
                            if ((np - ep).Length() < 1) {
                                node = n;
                                nodeid = pr.second;
                            }
                        }
                        if (!node && pr.first > 0 && pr.second > 0) {
                            auto p1 = id2node[pr.first]->global_pos(mgr);
                            auto p2 = id2node[pr.second]->global_pos(mgr);
                            //LOG(INFO) << "Link " << lid << " (" << sp << ")-(" << ep << ") connect to node (" << p1 << ")-(" << p2 << ")";
                            if ((p1 - sp).Length() >= 1) {
                                //LOG(INFO) << "Link " << lid << " reset stp " << pr.first << " (" << p1 << ") for edp (" << ep << ")";
                                pr.first = 0;                                
                            }
                            if ((p2 - sp).Length() >= 1) {
                                //LOG(INFO) << "Link " << lid << " reset stp " << pr.second << " (" << p2 << ") for edp (" << ep << ")";
                                pr.second = 0;                                
                            }
                        }
                    }
                    while (ptr && ptr < edp) {
                        ptr = strchr(ptr, '"');
                        ptr += 1;
                        int64_t id = strtoll(ptr, &ptr, 10);
                        ptr += 1;
                        ids.push_back(id);
                        auto& pr = lid2nid[id];
                        if (!node && pr.first > 0 && id2node[pr.first]) {
                            auto n = id2node[pr.first];
                            auto np = n->global_pos(mgr);
                            if ((np - ep).Length() < 1) {
                                node = n;
                                nodeid = pr.first;
                            }
                        }
                        if (!node && pr.second > 0 && id2node[pr.second]) {
                            auto n = id2node[pr.second];
                            auto np = n->global_pos(mgr);
                            if ((np - ep).Length() < 1) {
                                node = n;
                                nodeid = pr.second;
                            }
                        }
                        if (!node && pr.first > 0 && pr.second > 0) {
                            auto p1 = id2node[pr.first]->global_pos(mgr);
                            auto p2 = id2node[pr.second]->global_pos(mgr);
                            //LOG(INFO) << "Link " << lid << " edp(" << ep << ") to next link " << id << " disconnect with node (" 
                            //    << p1 << ")/" << pr.first << " - (" << p2 << ")/" << pr.second;    
                        }
                    }
                    if (!node) {
                        node = tmp_tile->nodes().add();
                        auto nid = node->mutable_id();
                        nodeid = lid * 2 + 1;
                        nid->set_id(nodeid);
                        nid->set_type(RoadPB::FeatureID::NODE);
                        nid->set_version(timestamp);
                        auto pt = link->geom()->pts().back();
                        node->mutable_geom()->set_x(pt->x());
                        node->mutable_geom()->set_y(pt->y());
                        node->mutable_geom()->set_z(0);
                        data_access_engine::Vector3D pos = *pt;
                        int t = mgr->WGS84_to_tileID(pos);
                        if (t > 0) {
                            nid->set_tileid(t);
                        }
                        else {
                            nid->set_tileid(link->id()->tileid());
                        }
                        id2node[nodeid] = node;
                    }
                    else {
                        tid_set.insert(node->id()->tileid());
                    }
                    link->mutable_enode_id()->set(node->id());
                    if (node->link_ids().find(link->id()) == node->link_ids().end()) {
                        node->mutable_link_ids()->push_back(link->id());
                    }
                    else {
                        LOG(ERROR) << "Duplicated link " << link->id()->to_string() << " found in " << node->id()->to_string();
                    }
                    if (!node->is_valid()) {
                        LOG(ERROR) << "Invalid node found " << node->id()->to_string();
                    }
                    ids.push_back(lid);
                    for (auto id : ids) {
                        auto& pr = lid2nid[id];
                        if (pr.first == nodeid || pr.second == nodeid) {

                        }
                        else if (pr.first <= 0) {
                            pr.first = nodeid;
                        }
                        else if (pr.second <= 0) {
                            pr.second = nodeid;
                        }
                        else {
                            //LOG(INFO) << "Link " << lid << " edp " << nodeid << " to link " << id << " connect to neither nodes " << pr.first << '/' << pr.second;
                        }
                    }
                }
                if (!link->is_valid()) {
                    LOG(ERROR) << "Invalid link found " << link->id()->to_string();
                }
            }
            if (page % step == 0 || page >= total) {
                LOG(INFO) << "Merge and commit @ page " << page;
                std::vector<int> tids(tid_set.begin(), tid_set.end());
                mgr->clear_all();
                mgr->init_tiles_by_id({1234567});
                proj->set_destination_Proj4_string("+proj=geocent +datum=WGS84");
                mgr->set_current_version(timestamp);
                data_access_engine::RoadTileDownloadParam param;
                param.judge_editable = false;
                param.editor_name = "sd_converter";
                mgr->load_tiles_by_id(tids, &param, 4);
                mgr->filter_invalids();
                data_access_engine::ID2TileMap tile_map;
                mgr->get_road_tiles(tile_map);
                for (auto& tit : tile_map) {
                    for (auto& tl : tit.second->links()) {
                        if (id2link.find(tl->id()->id()) != id2link.end()) {
                            auto link = id2link[tl->id()->id()];
                            if (!link || !link->is_valid()) {
                                continue;
                            }                            
                            if (!tl->geom() || tl->geom()->pts().size() < link->geom()->pts().size()) {
                                LOG(ERROR) << "Replace link " << tl->id()->id() << " to " << link->geom()->pts().size() << " points!";
                                tl->mutable_geom()->mutable_pts()->clear();
                                for (auto& pt : link->geom()->pts()) {
                                    data_access_engine::Vector3D p = *pt;
                                    tl->mutable_geom()->mutable_pts()->add()->set(p);
                                }
                                
                            }
                            else if (tl->geom()->pts().size() > link->geom()->pts().size()) {
                                LOG(ERROR) << "Discard link " << link->id()->id() << " from " << tl->geom()->pts().size() << " to " << link->geom()->pts().size() << " points!";;
                            }
                            if (!tl->snode_id()) {
                                tl->mutable_snode_id()->set(link->snode_id());
                            }
                            if (!tl->enode_id()) {
                                tl->mutable_enode_id()->set(link->enode_id());
                            }
                            id2link[tl->id()->id()].reset();
                        }
                    }
                    for (auto& node : tit.second->nodes()) {
                        if (!node || !node->is_valid()) {
                            continue;
                        }
                        auto pp = node->global_pos(mgr);
                        size_t linksize = node->link_ids().size();
                        for (size_t li = 0; li < linksize; li++) {
                            auto lid = node->link_ids()[li];
                            if (!lid || lid2nid.count(lid->id()) <= 0) {
                                continue;
                            }
                            auto& pr = lid2nid[lid->id()];
                            if (pr.first > 0 && id2node[pr.first]) {
                                auto& n = id2node[pr.first];
                                auto np = n->global_pos(mgr);
                                if ((pp - np).Length() < 1) {
                                    LOG(INFO) << "Merge node " << n->id()->id() << " with old node " << node->id()->id();
                                    for (auto& l : *n->mutable_link_ids()) {
                                        if (!node->link_ids().find(*l)) {
                                            node->mutable_link_ids()->push_back(l);
                                        }
                                        if (l.proxy()) {
                                            auto link = l.proxy();
                                            if (link->snode_id()->is_equal(*n->id())) {
                                                link->mutable_snode_id()->set(node->id());
                                            }
                                            else if (link->enode_id()->is_equal(*n->id())) {
                                                link->mutable_enode_id()->set(node->id());
                                            }
                                        }
                                    }
                                    n.reset();
                                    continue;
                                }
                            }
                            if (pr.second > 0 && id2node[pr.second]) {
                                auto& n = id2node[pr.second];
                                auto np = n->global_pos(mgr);
                                if ((pp - np).Length() < 1) {
                                    LOG(INFO) << "Merge node " << n->id()->id() << " with old node " << node->id()->id();                                    
                                    for (auto& l : *n->mutable_link_ids()) {
                                        if (!node->link_ids().find(*l)) {
                                            node->mutable_link_ids()->push_back(l);
                                        }
                                        if (l.proxy()) {
                                            auto link = l.proxy();
                                            if (link->snode_id()->is_equal(*n->id())) {
                                                link->mutable_snode_id()->set(node->id());
                                            }
                                            else if (link->enode_id()->is_equal(*n->id())) {
                                                link->mutable_enode_id()->set(node->id());
                                            }
                                        }
                                    }
                                    n.reset();
                                    continue;
                                }
                            }
                        }
                    }
                }
                for (auto& lit : id2link) {
                    auto link = lit.second;
                    if (!link) {
                        continue;
                    }
                    link->mutable_id()->set_version(mgr->current_version());
                    if (!link->is_valid()) {
                        LOG(ERROR) << "Invalid link " << link->id()->to_string();
                    }
                    auto tile = mgr->get_road_tile(link->id()->tileid());
                    tile->links().push_back(link);
                    link->correct_content(mgr);
                }
                for (auto& nit : id2node) {
                    auto node = nit.second;
                    if (!node) {
                        continue;
                    }
                    auto tile = mgr->get_road_tile(node->id()->tileid());
                    bool bf = false;
                    for (auto& n : tile->nodes()) {
                        if (node->id()->is_equal(*n->id())) {
                            bf = true;
                            break;
                        }
                    }
                    if (!bf) {
                        node->mutable_id()->set_version(mgr->current_version());
                        if (!node->is_valid()) {
                            LOG(ERROR) << "Invalid node " << node->id()->to_string();
                        }
                        tile->nodes().push_back(node);
                        node->remake_proxy(mgr);
                        node->correct_content(mgr);
                    }
                }
                mgr->filter_invalids();
                data_access_engine::TileInfoList tiles;
                mgr->get_changed_tile_data(tiles);
                if (tiles.empty()) {
                    LOG(INFO) << "No change detected for page " << page;
                    break;
                }
                mgr->upload_tiles("dumper", tiles, -32, 8);
                id2link.clear();
                id2node.clear();
                lid2nid.clear();
                mgr->clear_all();                
                tmp_tile->relink_parent();
                mgr->init_tiles_by_id({1234567});
                mgr->set_current_version(timestamp);
                proj->set_destination_Proj4_string("+proj=geocent +datum=WGS84");                
            }
        } while (page < total);
        LOG(INFO) << "Finish load " << page << " / " << total << " pages"; 
    }
    else if (FLAGS_method == 1) {
        std::ifstream ifs(FLAGS_file);
        char buf[40960] = { 0 };
        ifs.getline(buf, 40960);
        //std::map<int, std::set<int64_t> > tid2link;
        //std::map<int, std::set<int64_t> > tid2node;
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::LinkProxy> > id2link(2048000);
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::NodeProxy> > id2node(2048000);
        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        mgr->set_current_version(timestamp);
        while (!ifs.eof()) {
            ifs.getline(buf, 40960);
            char* ptr = buf;
            int64_t lid = strtol(buf, &ptr, 10);
            if (lid <= 0) {
                continue;
            }
            std::shared_ptr<data_access_engine::LinkProxy> link(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::LinkProxy>());
            link->make_ref_record();
            id2link[lid] = link;
            ptr++;
            double link_len = strtod(ptr, &ptr);
            link->set_link_length(link_len);
            ptr++;            
            if (*ptr == ',') {
                ptr++;                
            }
            else {
                int type = strtol(ptr, &ptr, 10);
                link->set_link_type(type);
                ptr++;
            }
            if (*ptr == ',') {
                ptr++;
            }
            else {
                int grade = strtol(ptr, &ptr, 10);
                link->set_road_class(grade);
                ptr++;
            }
            if (*ptr == ',') {
                ptr++;
            }
            else {
                int pc = strtol(ptr, &ptr, 10);
                link->set_direction(pc);
                ptr++;
            }
            if (*ptr == ',') {
                ptr++;
            }
            else {
                auto p = strchr(ptr, ',');
                std::string n(ptr, (p - ptr));
                if (!n.empty() && n != "NULL") {
                    auto name = link->mutable_names()->add();
                    name->set_name(n);
                }
                ptr = p + 1;
            }
            if (*ptr == ',') {
                ptr++;
            }
            else {
                int lc = strtol(ptr, &ptr, 10);
                //link->set_lane_count(lc);
                ptr++;
            }
            int64_t sid = strtol(ptr, &ptr, 10);
            ptr++;
            int64_t eid = strtol(ptr, &ptr, 10);
            ptr++;
            char* stp = strchr(ptr, '(');
            char* edp = strchr(ptr, ')');
            ptr = stp + 1;
            int tid = -1;
            std::set<int> tids;
            int range_ind = -1;
            while (ptr && ptr < edp) {
                double x = strtod(ptr, &ptr);
                ptr++;
                double y = strtod(ptr, &ptr);
                ptr++;
                auto pt = link->mutable_geom()->mutable_pts()->add();
                pt->set_x(x);
                pt->set_y(y);
                pt->set_z(0);
                for (int i = 0; i < 5; ++i) {
                    if (x >= ranges[i * 2].first && y >= ranges[i * 2].second && x <= ranges[i * 2 + 1].first && y <= ranges[i * 2 + 1].second) {
                        range_ind = i;
                        break;
                    }
                }                
                data_access_engine::Vector3D pos = *pt;
                int t = mgr->WGS84_to_tileID(pos);
                if (t > 0) {
                    auto tile = mgr->get_road_tile(t);
                    if (tid <= 0) {
                        tile->links().push_back(link);
                        data_access_engine::Vector3D p = *pt;
                        mgr->make_new_id(p, link, tile, true);
                        tid = tile->tile_id();
                        t = tid;
                    }
                    if (tids.count(t) <= 0) {
                        tile->link_refs().push_back(link->id());
                        tids.insert(t);
                    }
                }
            }
            if (range_ind >= 0) {
                link_length[range_ind] += link_len;
            }
            if (tid <= 0) {                
                continue;
            }            
            auto& stnode = id2node[sid];
            if (!stnode || !stnode->id()) {
                stnode.reset(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::NodeProxy>());
                stnode->make_ref_record();
                auto nid = stnode->mutable_id();
                nid->set_id(sid);
                nid->set_type(RoadPB::FeatureID::NODE);
                nid->set_tileid(link->id()->tileid());
                nid->set_version(timestamp);
                stnode->mutable_geom()->set_x(link->geom()->pts()[0]->x());
                stnode->mutable_geom()->set_y(link->geom()->pts()[0]->y());
                stnode->mutable_geom()->set_z(0);
                auto tile = mgr->get_road_tile(link->id()->tileid());
                tile->nodes().push_back(stnode);
            }
            stnode->mutable_link_ids()->push_back(link->id());
            link->mutable_snode_id()->set(stnode->id());
            auto& ednode = id2node[eid];
            if (!ednode || !ednode->id()) {
                ednode.reset(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::NodeProxy>());
                ednode->make_ref_record();
                auto pt = link->geom()->pts().back();
                data_access_engine::Vector3D pos = *pt;
                int t = mgr->WGS84_to_tileID(pos);
                if (t > 0) {
                    auto nid = ednode->mutable_id();
                    nid->set_id(eid);
                    nid->set_type(RoadPB::FeatureID::NODE);
                    nid->set_tileid(t);
                    nid->set_version(timestamp);
                    ednode->mutable_geom()->set_x(pt->x());
                    ednode->mutable_geom()->set_y(pt->y());
                    ednode->mutable_geom()->set_z(0);
                    auto tile = mgr->get_road_tile(t);                    
                    tile->nodes().push_back(ednode);
                }
            }
            ednode->mutable_link_ids()->push_back(link->id());
            link->mutable_enode_id()->set(ednode->id());
        }
        for (int i = 0; i < 5; i++) {
            LOG(INFO) << "Range" << i << " link_length: " << link_length[i];
        }
        mgr->correct_tiles();
        data_access_engine::TileInfoList tiles;
        mgr->get_editable_tile_data(tiles);
        mgr->upload_tiles("dumper", tiles, -1, 4);
    }
    else if (FLAGS_method == 3) {
        char buf[40960] = { 0 };
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::LinkExtProxy> > id2link(2048000);
        std::unordered_map<int64_t, std::shared_ptr<data_access_engine::NodeProxy> > id2node(2048000);
        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        mgr->set_current_version(timestamp);
        {
            std::ifstream ifs(FLAGS_path + "link.csv");
            ifs.getline(buf, 40960);
            while (!ifs.eof()) {
                ifs.getline(buf, 40960);
                if (!buf[0]) {
                    break;
                }
                link_csv t;
                t.parse(buf);
                std::shared_ptr<data_access_engine::LinkExtProxy> link(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::LinkExtProxy>());
                link->make_ref_record();
                id2link[t.link_id] = link;
                link->set_link_length(t.link_length);
                link->set_direction(t.direction);
                link->set_provincecode(t.province_code);
                int tid = -1;
                std::set<int> tids;
                for (auto& p : t.geoms) {
                    auto pt = link->mutable_geom()->mutable_pts()->add();
                    pt->set_x(p.first);
                    pt->set_y(p.second);
                    pt->set_z(0);
                    data_access_engine::Vector3D pos = *pt;
                    int tt = mgr->WGS84_to_tileID(pos);
                    if (tt > 0) {
                        auto tile = mgr->get_road_tile(tt);
                        if (tid <= 0) {
                            tile->links().push_back(link);
                            pos = *pt;
                            auto id = link->mutable_id();
                            id->set_id(t.link_id);
                            id->set_type(RoadPB::FeatureID::LINK);
                            id->set_tileid(tt);
                            id->set_version(timestamp);
                            tid = tt;
                        }
                        if (tids.count(tt) <= 0) {
                            tile->link_refs().push_back(link->id());
                            tids.insert(tt);
                        }
                    }
                }
                if (tid <= 0) {
                    LOG(ERROR) << "no geoms " << buf;
                }
                auto& stnode = id2node[t.link_snode];
                if (!stnode || !stnode->id()) {
                    stnode.reset(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::NodeProxy>());
                    stnode->make_ref_record();
                    auto nid = stnode->mutable_id();
                    nid->set_id(t.link_snode);
                    nid->set_type(RoadPB::FeatureID::NODE);
                    nid->set_tileid(link->id()->tileid());
                    nid->set_version(timestamp);
                    stnode->mutable_geom()->set_x(link->geom()->pts()[0]->x());
                    stnode->mutable_geom()->set_y(link->geom()->pts()[0]->y());
                    stnode->mutable_geom()->set_z(0);
                    auto tile = mgr->get_road_tile(link->id()->tileid());
                    tile->nodes().push_back(stnode);
                }
                stnode->mutable_link_ids()->push_back(link->id());
                link->mutable_snode_id()->set(stnode->id());
                auto& ednode = id2node[t.link_enode];
                if (!ednode || !ednode->id()) {
                    ednode.reset(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::NodeProxy>());
                    ednode->make_ref_record();
                    auto pt = link->geom()->pts().back();
                    data_access_engine::Vector3D pos = *pt;
                    int tt = mgr->WGS84_to_tileID(pos);
                    if (tt > 0) {
                        auto nid = ednode->mutable_id();
                        nid->set_id(t.link_enode);
                        nid->set_type(RoadPB::FeatureID::NODE);
                        nid->set_tileid(tt);
                        nid->set_version(timestamp);
                        ednode->mutable_geom()->set_x(pt->x());
                        ednode->mutable_geom()->set_y(pt->y());
                        ednode->mutable_geom()->set_z(0);
                        auto tile = mgr->get_road_tile(tt);                    
                        tile->nodes().push_back(ednode);
                    }
                }
                ednode->mutable_link_ids()->push_back(link->id());
                link->mutable_enode_id()->set(ednode->id());
            }
        }
        std::unordered_map<int64_t, range> id2range(7000000);
        {
            std::ifstream ifs(FLAGS_path + "range.csv");
            ifs.getline(buf, 40960);
            while (!ifs.eof()) {
                ifs.getline(buf, 40960);
                if (!buf[0]) {
                    break;
                }
                range t;
                t.parse(buf);                
                id2range[t.range_id] = t;
                if (id2link.count(t.feature_id) <= 0) {
                    LOG(ERROR) << "unknown link " << buf;
                }
            }
        }
        {
            std::ifstream ifs(FLAGS_path + "link_range_attribute.csv");
            ifs.getline(buf, 40960);
            while (!ifs.eof()) {
                ifs.getline(buf, 40960);
                if (!buf[0]) {
                    break;
                }
                link_attribue t;
                t.parse(buf);
                if (id2link.count(t.link_id) <= 0) {
                    LOG(ERROR) << "unknown link " << buf;
                }
                auto& link = id2link[t.link_id];
                switch (t.key) {
                case 1:
                    link->set_right_lanes(t.start_pos_value);
                    break;
                case 2:
                    link->set_left_lanes(t.start_pos_value);
                    break;
                case 3:
                    link->set_road_class(t.start_pos_value);
                    break;
                case 4:
                    link->set_net_grade(t.start_pos_value);
                    break;
                case 5:
                    link->set_pavement_info(t.start_pos_value);
                    break;
                case 6:
                    link->set_lane_count(t.start_pos_value);
                    break;
                case 7:
                    link->set_link_type(t.start_pos_value);
                    break;
                case 8:
                    link->set_pass_type(t.start_pos_value);
                    break;
                case 9:
                    link->set_urban_flag(t.start_pos_value);
                    break;
                case 10:
                    link->set_grade(t.start_pos_value);
                    break;
                case 11:
                    link->set_road_grade(t.start_pos_value);
                    break;
                case 12:
                    link->set_multiply_digitized(t.start_pos_value);
                    break;
                }                
            }
        }
        {
            std::ifstream ifs(FLAGS_path + "link_name.csv");
            ifs.getline(buf, 40960);
            while (!ifs.eof()) {
                ifs.getline(buf, 40960);
                if (!buf[0]) {
                    break;
                }
                link_name t;
                t.parse(buf);
                if (id2link.count(t.link_id) <= 0) {
                    LOG(ERROR) << "unknown link " << buf;
                }
                auto& link = id2link[t.link_id];
                auto name = link->mutable_names()->add();
                name->set_name_type(std::to_string(t.name_type));
                name->set_lang_code(std::to_string(t.name_language_code));
                name->set_name(t.name);
            }
        }
        
        mgr->correct_tile_refs();
        data_access_engine::ID2TileMap tm;
        data_access_engine::TileInfoList tiles;
        mgr->get_road_tiles(tm);
        for (auto& tit : tm) {
            tiles.push_back(tit.second);
        }
        mgr->upload_tiles("dumper", tiles, -1, 4);
    }

    LOG(INFO) << "tileserver_client is going to quit";
    return 0;
}
