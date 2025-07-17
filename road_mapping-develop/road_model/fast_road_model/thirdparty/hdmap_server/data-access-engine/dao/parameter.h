#ifndef _HDMAP_PARAMETER_H_
#define _HDMAP_PARAMETER_H_
#include <memory>
#include <type_traits>
#include <assert.h>
#include <typeindex>
#include <string>
#include <sstream>
#include <vector>
class Any;
class Parameter {
public:
	template<class ...Args>
	Parameter(Args &&... args) {
		Addp(std::forward<Args>(args)...);
	}
	template<class T>
	void Add(T&& head) {
		m_params.push_back(std::forward<T>(head));
	}
	template <class T>
	void Get(int index, T& res) const{
		assert(index < m_params.size());
		res = (m_params[index]).AnyCast<T>();
	}

	template<class T>
	void Set(int index, T&& val){
		assert(index < m_params.size());
		m_params[index] = std::forward<T>(val);
	}
	int Size() const{ return m_params.size(); }
	int Size() { return m_params.size(); }
private:
	
	void Addp() {}
	template<class T, class ... Rest>
	void Addp(T&& head, Rest&&... rest) {
		m_params.push_back(std::forward<T>(head));
		Addp(std::forward<Rest>(rest)...);

	}

	std::vector<Any> m_params;
};
class Any {
public:
	Any(void) :m_ptr(), m_tpindex(typeid(void)) {}
	Any(const Any & other) :m_ptr(other.Clone()), m_tpindex(other.m_tpindex) {}
	Any(Any && other) :m_ptr(std::move(other.m_ptr)), m_tpindex(other.m_tpindex) {}
	template<class U>
	Any(U && value) : m_ptr(new Derived<typename std::decay<U>::type>(std::forward<U>(value))), m_tpindex(std::type_index(typeid(typename std::decay<U>::type))) {
	}
	Any(const char* value) :m_tpindex(std::type_index(typeid(std::string))), m_ptr(new Derived<std::string>(value)) {}
	bool IsNull() const { return m_ptr == nullptr; }
	template<class U>
	bool IS() const { return m_tpindex == std::type_index(typeid(U)); }
	template<class U>
	U AnyCast() const{
		if (!IS<U>()) {
			throw std::bad_cast();
		}
		auto derived = dynamic_cast<Derived<U>*>(m_ptr.get());
		return derived->m_value;
	}


	template<>
	float AnyCast()  const{
		if (IS<double>()) {
			auto derived = dynamic_cast<Derived<double>*>(m_ptr.get());
			return derived->m_value;
		}
		if (IS<int>()) {
			auto derived = dynamic_cast<Derived<int>*>(m_ptr.get());
			return derived->m_value;
		}
		if (IS<short>()) {
			auto derived = dynamic_cast<Derived<short>*>(m_ptr.get());
			return derived->m_value;
		}
		if (!IS<float>()){
			throw std::bad_cast();
		}
		auto derived = dynamic_cast<Derived<float>*>(m_ptr.get());
		return derived->m_value;
	}
	template<>
	int AnyCast()  const{

		if (IS<unsigned int>()) {
			auto derived = dynamic_cast<Derived<unsigned int>*>(m_ptr.get());
			return derived->m_value;
		}
		if (IS<short>()) {
			auto derived = dynamic_cast<Derived<short>*>(m_ptr.get());
			return derived->m_value;
		}
		if (!IS<int>()){
			std::stringstream strstream_;
			strstream_ << "can not cast int to " << m_tpindex.name();
			assert(false, strstream_.str());
			throw std::bad_cast();
		}
		auto derived = dynamic_cast<Derived<int>*>(m_ptr.get());
		return derived->m_value;
	}
	Any& operator=(const Any& other) {
		if (m_ptr == other.m_ptr)
			return *this;
		m_ptr = other.Clone();
		m_tpindex = other.m_tpindex;
	}
private:
	class Base;
	typedef std::unique_ptr<Base> BasePtr;

	class Base {
	public:
		virtual ~Base() {}
		virtual BasePtr Clone() const = 0;
	};
	template <class T>
	class Derived :public Base
	{
	public:
		template<class U>
		Derived(U && value) :m_value(std::forward<U>(value)) {}
		BasePtr Clone() const {
			return BasePtr(new Derived<T>(m_value));
		}
		T m_value;

	};
	BasePtr Clone()const {
		if (m_ptr != nullptr)
			return m_ptr->Clone();
		return nullptr;
	}
	BasePtr m_ptr;
	std::type_index m_tpindex;
};
#endif