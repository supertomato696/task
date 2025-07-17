

#pragma once

#include <sys/time.h>
#include <cstddef>

namespace fsdmap {

/**
 * time checker for record time
 */
class TimeChecker {
    struct timeval _start_t;
    struct timeval _check_t_1;
    struct timeval _check_t_2;
    bool _check_toggle;
    int _unit_level;

public:
    TimeChecker() {
        clear();
        _unit_level = 0;
    }
    
    TimeChecker(int unit_level) {
        clear();
        _unit_level = unit_level;
    }

    timeval& get_start() {
        return _start_t;
    }
    timeval& get_last() {
        return _check_toggle ? _check_t_1 : _check_t_2;
    }
    timeval& get_check() {
        return !_check_toggle ? _check_t_1 : _check_t_2;
    }

    void set_unit_level(int unit_level) {
        _unit_level = unit_level;
    }

    long get_total_u() {
        return usec_elapsed(_start_t, get_check());
    }

    long get_total_m() {
        return get_total_u() / 1000l;
    }
    long get_total_s() {
        return get_total_u() / 1000000l;
    }

    long get_total_minute() {
        return get_total_u() / 60000000l;
    }

    long get_total() {
        if (_unit_level == 1) {
            return get_total_u();
        } else if (_unit_level == 2) {
            return get_total_m();
        } else if (_unit_level == 3) {
            return get_total_s();
        } else if (_unit_level == 4) {
            return get_total_minute();
        }
        return get_total_m();
    }

    void clear() {
        _start_t.tv_sec = 0;
        _start_t.tv_usec = 0;
        _check_t_1.tv_sec = 0;
        _check_t_1.tv_usec = 0;
        _check_t_2.tv_sec = 0;
        _check_t_2.tv_usec = 0;
        _check_toggle = true;
    }

    // ���¿�ʼ��ʱ
    const timeval& start() {
        gettimeofday(&_start_t, NULL);
        _check_t_1 = _start_t;
        _check_t_2 = _start_t;
        return _start_t;
    }

    // check time and return the interval between last check and this
    long check_u() {
        _check_toggle = !_check_toggle;
        gettimeofday(&(get_check()), NULL);
        return usec_elapsed(get_last(), get_check());
    }
    
    long check_m() {
        return check_u() / 1000l;
    }

    long check_s() {
        return check_u() / 1000000l;
    }

    long check_minute() {
        return check_u() / 60000000l;
    }

    long check() {
        if (_unit_level == 1) {
            return check_u();
        } else if (_unit_level == 2) {
            return check_m();
        } else if (_unit_level == 3) {
            return check_s();
        } else if (_unit_level == 4) {
            return check_minute();
        }
        return check_m();
    }

    long usec_elapsed(const timeval &s, const timeval &e) {
        return(e.tv_sec - s.tv_sec) * 1000 * 1000l +
            (e.tv_usec - s.tv_usec);
    }

    long msec_elapsed(const timeval &s, const timeval &e) {
        return usec_elapsed(s, e) / 1000l;
    }

    long sec_elapsed(const timeval &s, const timeval &e) {
        return msec_elapsed(s, e) / 1000l;
    }
};

}; //namespace fsdmap

