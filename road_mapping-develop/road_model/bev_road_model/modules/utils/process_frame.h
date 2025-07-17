

#pragma once

#include <string>
#include <unordered_map>
#include <pthread.h>
#include <gflags/gflags.h>
#include "utils/log_util.h"
#include "utils/common_util.h"

namespace fsdmap {
namespace process_frame {

/** 
 * @brief proc�������ص�״̬
 */
enum PROC_STATUS {
    PROC_STATUS_SUCC = 0,    // �ɹ����������һ��proc 
    PROC_STATUS_FAIL = -1,   // ʧ�ܣ�ֱ�ӽ�������
    PROC_STATUS_SUSPEND = 2, // ������Ҫ�����첽
    PROC_STATUS_DISABLE = 3,    // �������������һ��proc 
};

class ProcessFrame;

/**
 * @brief session�Ĺ�����
 */
class SessionManagerBase {
public:
    SessionManagerBase() {};
    virtual ~SessionManagerBase() {};
    virtual const char * name() = 0;
    virtual int static_init(ProcessFrame* process_frame) = 0;
};



/**
 * @brief process��ܻ���
 */
class ProcessFrame {
public:
    ProcessFrame() {};
    virtual ~ProcessFrame() {};
    
	int init() {
		if (load_static_conf() != fsdmap::SUCC) {
            CLOG_ERROR("failed to init");
            return fsdmap::FAIL;
        }
		return fsdmap::SUCC;
	}
    
    /**
     * @brief ����ע��÷���������session�Ľӿ�
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int registe_all_session() = 0;

    /**
     * @brief ע��session�ķ���
     *
     * @param session_mgr session mgr����
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int registe_session(SessionManagerBase* session_mgr) {
        if (session_mgr == NULL) {
            CLOG_ERROR("failed to regist session, ptr is null");
            return fsdmap::FAIL;
        }
        _session_map[session_mgr->name()] = session_mgr;
        return fsdmap::SUCC;
    }

    /**
     * @brief ���ؾ�̬���ýӿڣ���server_frame�ӿڶԽ�
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int load_static_conf() {
        int ret = registe_all_session();
        if (ret != fsdmap::SUCC) {
            CLOG_ERROR("failed to registe session");
            return fsdmap::FAIL;
        }

        for (auto &itr : _session_map) {
            ret = itr.second->static_init(this);
            if (ret != fsdmap::SUCC) {
                CLOG_ERROR("faild to static init session[name=%s]", itr.first.c_str());
                return fsdmap::FAIL;
            }
        }
        return fsdmap::SUCC;
    }


private:
    std::unordered_map<std::string, SessionManagerBase* > _session_map;
};
    
/**
 * @brief process�Ļ��࣬�����˳�ʼ���ʹ����Ľӿ�
 */
template <typename SESSION>
class ProcBase {
public:
    ProcBase() {};
    virtual ~ProcBase() {};
    
    /**
     * @brief ��̬��ʼ��
     *
     * @param process_frame process_frame����
     * @param session_mgr session manager����
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int static_init(ProcessFrame* process_frame, SessionManagerBase* session_mgr) {
        _process_frame = process_frame;
        _session_mgr = session_mgr;
        return fsdmap::SUCC;
    }

    /**
     * @brief ��ȡ���ƽӿ�
     *
     * @return ����
     */
    virtual inline const char * name() = 0;

    /**
     * @brief �����ӿ�
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int clear() {
        return fsdmap::SUCC;
    };
    
    /**
     * @brief proc Ԥ����
     *
     * @param session_data �ô�session�����ݶ���
     *
     * @return �μ�ö��PROC_STATUS
     */
    virtual PROC_STATUS pre_proc(SESSION* session_data) {
        _timer.start();
        return proc(session_data);
    };

    /**
     * @brief proc�����ӿ�
     *
     * @param session_data �ô�session�����ݶ���
     *
     * @return �μ�ö��PROC_STATUS
     */
    virtual PROC_STATUS proc(SESSION* session_data) {
    };

    ProcessFrame* get_frame() {
        return _process_frame;
    }

    SessionManagerBase* get_session_mgr() {
        return _session_mgr;
    }
protected:
    ProcessFrame* _process_frame;
    SessionManagerBase* _session_mgr;
    fsdmap::TimeChecker _timer;
    // bool enable_debug_pos;
};

/**
 * @brief session�Ĺ�����
 */
template <typename SD>
class SessionManager : public SessionManagerBase {
public:
    SessionManager() {};
    virtual ~SessionManager() {};

    virtual int declare_proc() = 0;
    
    /**
     * @brief session������,����ע��Ͷ�ȡzk����
     *
     * @return session������
     */
    virtual const char * name() = 0;
    
    /**
     * @brief ��ȡsession data��ʵ������������������
     */
    virtual std::shared_ptr<SD> get_instance() {
        auto session = std::make_shared<SD>();
        return session;
    }

    /**
     * @brief ��̬��ʼ��
     *
     * @return 0 if SUCC, non-zero if error
     */
    int static_init(ProcessFrame* process_frame) {
        _process_frame = process_frame;
        if (declare_proc() != fsdmap::SUCC) {
            CLOG_ERROR("failed to declare proc [session=%s]", name());
            return fsdmap::FAIL;
        }
        return fsdmap::SUCC;
    }

    /**
     * @brief ����proc�ӿڣ���������procʵ���ĺ���ָ��
     *
     * @param fn ����ָ��
     *
     * @return 0 if SUCC, non-zero if error
     */
    int add_proc(ProcBase<SD> * ins_1) {
        if (ins_1->static_init(_process_frame, this) != fsdmap::SUCC) {
            CLOG_ERROR("failed to static init proc[name=%s]", ins_1->name());
            return fsdmap::FAIL;
        }
        _proc_list_1.push_back(ins_1);
        return fsdmap::SUCC;
    }

    std::vector<ProcBase<SD> * >& get_current_proc_list() {
        return _proc_list_1;
    }

    std::vector<ProcBase<SD> * >& get_idl_proc_list() {
        return _proc_list_1;
    }

private:
    std::vector<ProcBase<SD> * > _proc_list_1;
    bool _proc_toggle;
    ProcessFrame* _process_frame; 
};


/**
 * @brief session�Ļ���
 */
// template <typename SD, typename REQ, typename RES>
template <typename SD>
class SessionDataBase {
public:
    SessionDataBase() {};
    virtual ~SessionDataBase() {};
    
    /**
     * @brief ��ʼ����Ա����, 
     *
     * @param cntl controller
     * @param req req
     * @param res res
     * @oaram done done
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int init() {
        _time_unit = 0;
        current_proc = 0;

        if (pthread_mutex_init(&_mu, NULL) != 0) {
            return fsdmap::FAIL;
        }
        return fsdmap::SUCC;
    }

    /**
     * @brief ��������, ��Ҫ�߼�����
     *        1. ��ȡ��ǰ���õ�procʵ���б�
     *        2. ����ÿ��proc�����ݷ���״̬���д���
     *        3. �ɹ���ִ����һ����ʧ��ֱ���˳���������ʲô��������
     *        4. ���й��̻��¼ִ��λ�ã��´ε��û�Ӽ�¼��λ�ÿ�ʼ������Ϊ��ʵ���첽
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int proc() {
        std::vector<ProcBase<SD> * > &proc_list = _session_mgr->get_current_proc_list();
        PROC_STATUS ret;
        fsdmap::TimeChecker proc_timer(_time_unit);
        proc_timer.start();
        pthread_mutex_lock(&_mu);
        for (int i = current_proc; i < proc_list.size(); i++) {
            ProcBase<SD> &proc = *proc_list[i];
            CLOG_DEBUG("start proc[session=%s, proc=%s]", _session_mgr->name(), proc.name());
            ret = proc.pre_proc((SD*)this);
            current_proc = i + 1;
            auto proc_cost = proc_timer.check();
            if (ret == PROC_STATUS_DISABLE) {
                continue;
            }
            _logger->push_info("{}:{}", proc.name(), proc_cost);
            if (ret == PROC_STATUS_SUSPEND) {
                CLOG_DEBUG("session suspend by proc[session=%s, proc=%s]",
                        _session_mgr->name(), proc.name());
                pthread_mutex_unlock(&_mu);
                return fsdmap::SUCC;
            } else if (ret == PROC_STATUS_FAIL) {
                CLOG_WARN("session stop by proc[session=%s, proc=%s]",
                        _session_mgr->name(), proc.name());
                pthread_mutex_unlock(&_mu);
                close();
                return fsdmap::FAIL;
            } else if (ret != PROC_STATUS_SUCC) {
                CLOG_WARN("session stop by proc[session=%s, proc=%s]",
                        _session_mgr->name(), proc.name());
                pthread_mutex_unlock(&_mu);
                close();
                return ret;
            }; 
            int p_mem = utils::get_memory_by_pid(getpid(), "VmRSS:");
            int p_mem_max = utils::get_memory_by_pid(getpid(), "VmHWM:");
            int system_mem = utils::get_machine_memory();
            p_mem /= (1024 * 1024);
            p_mem_max /= (1024 * 1024);
            system_mem /= (1024 * 1024);

            CLOG_DEBUG("finish proc[session=%s, mem=%d:%d:%d, cost=%ld]",
                    _session_mgr->name(), p_mem, p_mem_max, system_mem, proc_cost);
        }
        pthread_mutex_unlock(&_mu);
        close();
        return fsdmap::SUCC;
    }

    /**
     * @brief �������̽ӿ�
     *
     * @return 0 if SUCC, non-zero if error
     */
    virtual int close() {
        // pthread_mutex_unlock(&_mu);
        pthread_mutex_destroy(&_mu);
        // delete this;
        return fsdmap::SUCC;
    }
    
    /**
     * @brief ��ȡ������־��������������־
     * 
     * @return fsdmap::ServerLog& ��־��������
     */
    virtual fsdmap::utils::Logger* get_logger() {
        return _logger;
    }

    /** 
     * @brief ��ȡ������־ʱ�䵥λ
     *
     */
    virtual void set_log_tile_unit(int unit_level) {
        _time_unit = unit_level;
        _logger->get_timer().set_unit_level(_time_unit);
    }
public:
    SessionManager<SD>* _session_mgr;
    fsdmap::utils::Logger* _logger;
    int current_proc;
    std::string _proc_time_cost_str;
    pthread_mutex_t _mu;
    int return_code;
    std::string return_msg;
    int _time_unit;
};

}
}

#define CHECK_FATAL_PROC(var, sub_proc) do {    \
    if (!(var == fsdmap::SUCC)) {    \
        LOG_ERROR("failed to {} in {}", sub_proc, this->name());    \
        return fsdmap::process_frame::PROC_STATUS_FAIL;    \
    }    \
    LOG_INFO("finish {} {}[cost={}]", this->name(), sub_proc, _timer.check_m()); \
} while (0)


