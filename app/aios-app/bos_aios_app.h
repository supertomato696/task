#pragma once
#include <asio.hpp>


namespace bos::aios {
    const std::string AIOS_ROOT_DIR = "/data/data/aios/byd/"; // 默认路径
    
    class bos_aios_application_t {
    public:
        bos_aios_application_t(int argc, char** argv);
        ~bos_aios_application_t();
    public:
        void run();
        void stop();
        asio::io_context& get_io_context();
        int create_aios_root_dir();
    private:
        asio::io_context m_io_context;
        asio::executor_work_guard<asio::io_context::executor_type> m_work_guard;
    };

    bos_aios_application_t& get_application();
}




