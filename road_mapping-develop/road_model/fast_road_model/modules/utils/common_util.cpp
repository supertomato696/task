

#include <glob.h>
#include "utils/common_util.h"
#include <boost/filesystem.hpp>
#include "proj_api.h"
#include <cmath>

namespace fsdmap {
namespace utils {
constexpr double PI = 3.1415926535897932384626;

Logger* global_log_ptr = NULL;

int get_memory_by_pid(pid_t pid, std::string name) {
    std::ifstream fin;
	char virtual_filename[32] = {0};
	char vmrss_name[32] = {0};
	int vmrss_num = 0;
	sprintf(virtual_filename, "/proc/%d/status", pid);
	fin.open(virtual_filename, std::ios::in);
	if(!fin.is_open()) {
        LOG_ERROR("open {} failed", virtual_filename);
        return -1;
	}

	// VMRSS line is uncertain
    std::string line;
    while (getline(fin, line)) {
		if (strncmp(line.c_str(), name.c_str(), name.size()) == 0) {
			sscanf(line.c_str(), "%s %d", vmrss_name, &vmrss_num);
			break;
		}
	}
    fin.close();
	return vmrss_num;
}

int get_machine_memory() {
	const char* virtual_filename = "/proc/meminfo";
	FILE* fd = NULL;

	char line[1024] = {0};
	fd = fopen(virtual_filename, "r");
	if(fd == NULL) {
        CLOG_ERROR("open %s failed", virtual_filename);
        return -1;
	}
	char vmrss_name[32];
	int machine_memory = 0;
	fgets(line, sizeof(line), fd);
	sscanf(line, "%s %d", vmrss_name, &machine_memory);
	fclose(fd);
	return machine_memory;
}

std::string get_curr_meminfo(std::string &str) {
    int p_mem = get_memory_by_pid(getpid(), "VmRSS:");
    int p_mem_max = get_memory_by_pid(getpid(), "VmHWM:");
    int system_mem = get_machine_memory();
    p_mem /= (1024 * 1024);
    p_mem_max /= (1024 * 1024);
    system_mem /= (1024 * 1024);
    return utils::fmt("{}:{}:{}", p_mem, p_mem_max, system_mem);
}


int glob_dir_all(const char * dir, std::vector<std::string> &ret, bool need_dir) {
    boost::filesystem::path path(dir);
    for (const auto& iter : boost::filesystem::directory_iterator(path)) {
        if (need_dir) {
            ret.push_back(iter.path().string());
        } else {
            ret.push_back(iter.path().filename().string());
        }
    }
    return fsdmap::SUCC;
}

int glob_dir(const char * dir, std::vector<std::string> &ret) {
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    ResPtr file_ptr([&glob_result](){globfree(&glob_result);});

    // do the glob operation
    int return_value = glob(dir, GLOB_TILDE, NULL, &glob_result);
    if (return_value != 0) {
        // globfree(&glob_result);
        return fsdmap::FAIL;
    }
    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        ret.push_back(std::string(glob_result.gl_pathv[i]));
    }
    // globfree(&glob_result);
    return fsdmap::SUCC;
}

void ThreadPoolProxy::schedule(std::function<void(ProcessBar *bar)> run_fun) {
    ++_bar.num_total;
    ProcessBar* bar = &_bar;
    _thread_pool->schedule([bar, run_fun](){
            run_fun(bar);
            ++bar->num_finish;
            });
}

void ThreadPoolProxy::schedule(std::function<void()> run_fun) {
    ++_bar.num_total;
    ProcessBar* bar = &_bar;
    _thread_pool->schedule([bar, run_fun](){
            run_fun();
            ++bar->num_finish;
            });
}

// static projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");

int utm_2_wgs(int zone, Eigen::Vector3d &utm, Eigen::Vector3d &wgs) {
    wgs = utm;
    std::string str = utils::fmt("+proj=utm +zone={}N +ellps=WGS84 +no_defs", zone);
    projPJ g_utm = pj_init_plus(str.c_str());
    static projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
    if (pj_transform(g_utm, g_pWGS84, 1, 1, &wgs.x(), &wgs.y(), &wgs.z())) {
        return -1;
    }
    wgs.x() *= RAD_TO_DEG;
    wgs.y() *= RAD_TO_DEG;
    return 0;
}

int wgs_2_utm(int zone, Eigen::Vector3d &wgs, Eigen::Vector3d &utm) {
    utm = wgs;
    utm.x() *= DEG_TO_RAD;
    utm.y() *= DEG_TO_RAD;
    std::string str = utils::fmt("+proj=utm +zone={}N +ellps=WGS84 +no_defs", zone);
    projPJ g_utm = pj_init_plus(str.c_str());
    static projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
    if (pj_transform(g_pWGS84, g_utm, 1, 1, &utm.x(), &utm.y(), &utm.z())) {
        return -1;
    }
    return 0;
}

#if 0
// TODO:qzc, change to enu
int utm_2_gcj(int zone, Eigen::Vector3d &utm, Eigen::Vector3d &wgs) {
    wgs = utm;
    std::string str = utils::fmt("+proj=utm +zone={}N +ellps=WGS84 +no_defs", zone);
    projPJ g_utm = pj_init_plus(str.c_str());
    static projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
    if (pj_transform(g_utm, g_pWGS84, 1, 1, &wgs.x(), &wgs.y(), &wgs.z())) {
        return -1;
    }
    wgs.x() *= RAD_TO_DEG;
    wgs.y() *= RAD_TO_DEG;
    return 0;
}

int gcj_2_utm(int utm_num, Eigen::Vector3d &gcj, Eigen::Vector3d &utm) {
    double lon = gcj[0];
    double lat = gcj[1];
    double alt = gcj[2];

    if (-80.0 > lat  || lat > 84.0) {
        LOG_ERROR("error lat is {} ", lat);
        return -1;
    }

    if (-180.0 > lon  || lon > 180.0) {
        LOG_ERROR("error lon is {} ", lon);
        return -1;
    }


    if (1 > utm_num  || utm_num > 60) {
        LOG_ERROR("utm number out of range (must be between 1 and 60) ");
        return -1;
    }

    double K0 = 0.9996;
    double E = 0.00669438;
    double E_P2 = 0.006739496752268451;
    double M1 = 0.9983242984503243;
    double M2 = 0.002514607064228144;
    double M3 = 2.6390466021299826e-06;
    double M4 = 3.418046101696858e-09;
    double R = 6378137;
    double lat_rad = lat * DEG_TO_RAD;
    double lat_sin = std::sin(lat_rad);
    double lat_cos = std::cos(lat_rad);
    double lat_tan = lat_sin / lat_cos;
    double lat_tan2 = lat_tan * lat_tan;
    double lat_tan4 = lat_tan2 * lat_tan2;
    double lon_rad = lon * DEG_TO_RAD;
    double central_lon = (utm_num - 1) * 6 - 180 + 3;
    double central_lon_rad = central_lon * DEG_TO_RAD;
    double n = R / std::sqrt(1 - E * lat_sin * lat_sin);
    double c = E_P2 * lat_cos * lat_cos;
    double a = lat_cos * ((lon_rad - central_lon_rad + PI) % (2 * PI) - PI);
    double a2 = a * a;
    double a3 = a2 * a;
    double a4 = a3 * a;
    double a5 = a4 * a;
    double a6 = a5 * a;
    double m = R * (M1 * lat_rad - M2 * np.sin(2 * lat_rad) +
             M3 * np.sin(4 * lat_rad) - M4 * np.sin(6 * lat_rad));
    utm_x = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) +
                      a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000;
    utm_y = K0 * (m + n * lat_tan *
                  (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) +
                   a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));
    if lat < 0:
        utm_y += 10000000

    return 0;
}
#endif

int creat_dir(const char *dir) {
    boost::filesystem::path path(dir);
    if (!boost::filesystem::is_directory(path)) {
        if (!boost::filesystem::create_directory(path)) {
            return -1;
        }
    }
    return 0;
}

}
}
