#pragma once
#include <type_traits>

namespace data_access_engine {

const static double FLOAT_TOLERANCE = 1e-12;

template<typename T>
class Optional
{
    using data_t = typename std::aligned_storage<sizeof(T), std::alignment_of<T>::value>::type;
public:
    Optional() : _has_init(false) {}
    Optional(const T& v) {
        create(v);
    }

    Optional(T&& v) : _has_init(false) {
        create(std::move(v));
    }

    ~Optional() {
        destroy();
    }

    Optional(const Optional& other) : _has_init(false) {
        if (other.is_init()) {
            assign(other);
        }
    }

    Optional(Optional&& other) : _has_init(false) {
        if (other.is_init()) {
            assign(std::move(other));
            other.destroy();
        }
    }

    Optional& operator=(Optional &&other) {
        assign(std::move(other));
        return *this;
    }

    Optional& operator=(const Optional &other) {
        assign(other);
        return *this;
    }

    template<class... Args>
    void emplace(Args&&... args) {
        destroy();
        create(std::forward<Args>(args)...);
    }

    bool is_init() const { return _has_init; }

    explicit operator bool() const {
        return is_init();
    }

    T& get() { return *((T*)(&_data)); }
    T const& get() const { return *((T*)(&_data)); }

    T& operator*() {
        return get();
    }

    T const& operator*() const {
        if (is_init()) {
            return get();
        }
        throw std::exception();
    }

    bool operator == (const Optional<T>& rhs) const {
        return (!bool(*this)) != (!rhs) ? false : (!bool(*this) ? true : (*(*this)) == (*rhs));
    }

    bool operator < (const Optional<T>& rhs) const {
        return !rhs ? false : (!bool(*this) ? true : (*(*this) < (*rhs)));
    }

    bool operator != (const Optional<T>& rhs) const {
        return !(*this == (rhs));
    }

    void reset() {
        destroy();
    }
    T get_value_or(T v) const {
        if (is_init()) {
            return get();
        }
        return v;
    }

private:
    template<class... Args>
    void create(Args&&... args) {
        new (&_data) T(std::forward<Args>(args)...);
        _has_init = true;
    }

    void destroy() {
        if (_has_init) {
            _has_init = false;
            ((T*)(&_data))->~T();
        }
    }

    void assign(const Optional& other) {
        if (other.is_init()) {
            copy(other._data);
            _has_init = true;
        }
        else {
            destroy();
        }
    }

    void assign(Optional&& other) {
        if (other.is_init()) {
            move(std::move(other._data));
            _has_init = true;
            other.destroy();
        }
        else {
            destroy();
        }
    }

    void move(data_t&& val) {
        destroy();
        new (&_data) T(std::move(*((T*)(&val))));
    }

    void copy(const data_t& val) {
        destroy();
        new (&_data) T(*((T*)(&val)));
    }

private:
    bool _has_init;
    data_t _data;
};

template<>
inline bool Optional<float>::operator== (const Optional<float>& rhs) const {
    return (!bool(*this)) != (!rhs) ? false : (!bool(*this) ? true 
        : (*(*this) - (*rhs) > -FLOAT_TOLERANCE && *(*this) - (*rhs) < FLOAT_TOLERANCE));
};

template<>
inline bool Optional<double>::operator== (const Optional<double>& rhs) const {
    return (!bool(*this)) != (!rhs) ? false : (!bool(*this) ? true
        : (*(*this) - (*rhs) > -FLOAT_TOLERANCE && *(*this) - (*rhs) < FLOAT_TOLERANCE));
};

template<>
inline bool Optional<float>::operator!= (const Optional<float>& rhs) const {
    return !operator==(rhs);
};

template<>
inline bool Optional<double>::operator!= (const Optional<double>& rhs) const {
    return !operator==(rhs);
};

}; // data_access_engine