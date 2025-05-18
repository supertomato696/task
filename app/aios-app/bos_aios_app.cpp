#include "bos_aios_app.h"
#include <filesystem>
#include <iostream>

#ifdef __x86_64__
#include <pwd.h>
#endif

namespace bos::aios {
    bos_aios_application_t* g_instance = nullptr;

    bos_aios_application_t& get_application() {
        return *g_instance;
    }

    bos_aios_application_t::bos_aios_application_t(int argc, char** argv) : m_work_guard(m_io_context.get_executor()) {
        g_instance = this;
    }

    bos_aios_application_t::~bos_aios_application_t() {
        stop();
    }

    void bos_aios_application_t::run() {
        m_io_context.run();
    }

    void bos_aios_application_t::stop() {
        m_work_guard.reset();
        m_io_context.stop();
    }

    asio::io_context& bos_aios_application_t::get_io_context() {
        return m_io_context;
    }
    int bos_aios_application_t::create_aios_root_dir() {
        std::error_code err;

        std::string aios_root_dir;
#ifdef __x86_64__
        const char *home_dir = std::getenv("HOME");
        if (home_dir == nullptr)
        {
            home_dir = getpwuid(getuid())->pw_dir;
        }
        aios_root_dir = std::string(home_dir) + "/aiosdb/";
#else
        aios_root_dir = AIOS_ROOT_DIR; // 默认路径
#endif

        std::filesystem::create_directories(aios_root_dir,err);
        if (err) {
            std::cout << "can not create rm root dir" << err.message() << std::endl;
            return -1;
        }
        return 0;
    }
}
