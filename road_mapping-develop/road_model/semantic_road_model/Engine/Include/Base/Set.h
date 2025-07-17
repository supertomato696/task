/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Set.h
简要描述:集合模板类
******************************************************************/

#ifndef ENGINE_BASE_SET_H_
#define ENGINE_BASE_SET_H_

#include "Types.h"
#include <set>
#include <algorithm>
#include <initializer_list>
#include "Array.h"
#include "String.h"

namespace Engine
{
    namespace Base
    {
        // 为了尽最大程度上保证该模板类的性能，所以该的函数内部均不对外部传入的参数合法性进行检查，由使用者自己保证
        template <class T, class _Pr = std::less<T>>
        class Set
        {
            typedef std::set<T, _Pr> _Myt;

        public:
            typedef typename std::set<T, _Pr>::iterator _Iter;
            typedef typename std::set<T, _Pr>::const_iterator _Const_Iter;

            // 默认构造函数
            Set()
            {
            }

            // 析构函数
            ~Set()
            {
                Clear();
            }

            // 复制
            Set(const _Myt &other) : m_setInternel(other)
            {
            }

            // 等号重载符
            const T &operator=(const T &other)
            {
                if (this != &other)
                {
                    m_setInternel = other.m_setInternel;
                }
                return *this;
            }

            // 迭代器范围
            template <class Iter>
            Set(Iter _First, Iter _Last) : m_setInternel(_First, _Last)
            {
            }

            // 初始化列表
            Set(::std::initializer_list<T> il) : m_setInternel(il)
            {
            }

        public:
            // 判断数组是否为空
            // 返回数组是否为空，返回值类型为布尔类型。
            Bool IsEmpty() const
            {
                return m_setInternel.empty();
            }

            // 获取数组内的元素个数
            SizeT GetCount() const
            {
                return (SizeT)m_setInternel.size();
            }

            // 增加一个元素
            // newElement 新增元素
            // 返回新增元素的迭代器位置，以及是否新增成功
            std::pair<_Iter, bool> Add(const T &newElement)
            {
                return m_setInternel.insert(newElement);
            }

            // 增加另一个set
            // 自动去重
            void Add(const Set<T> &src)
            {
                m_setInternel.insert(src.m_setInternel.begin(), src.m_setInternel.end());
            }

            // 增加另一段迭代器范围
            // 自动去重
            template <class Iter>
            void Add(Iter _First, Iter _Last)
            {
                m_setInternel.insert(_First, _Last);
            }

            // 重载增加arr 方便重构
            void Add(const Array<T> &arr)
            {
                m_setInternel.insert(arr.Begin(), arr.End());
            }

            // 查找
            _Iter Find(const T &elem)
            {
                return m_setInternel.find(elem);
            }

            _Const_Iter Find(const T &elem) const
            {
                return m_setInternel.find(elem);
            }

            // 删除迭代器指向的元素
            _Iter Erase(_Iter itr)
            {
                if (itr == m_setInternel.end())
                {
                    return itr;
                }
                return m_setInternel.erase(itr);
            }

            // 删除指定值的元素
            SizeT Erase(const T &elem)
            {
                return (SizeT)m_setInternel.erase(elem);
            }

            // 移除所有元素,同时释放内存空间
            void Clear()
            {
                m_setInternel.clear();
                // m_setInternel.swap(std::set<T, _Pr>());
            }

            void swap(_Myt &other)
            {
                m_setInternel.swap(other);
            }

            // 返回数组迭代器的起始位置
            _Iter Begin()
            {
                return m_setInternel.begin();
            }

            // 返回数组迭代器的起始位置
            _Const_Iter Begin() const
            {
                return m_setInternel.cbegin();
            }

            // 返回数组迭代器的终止位置
            _Iter End()
            {
                return m_setInternel.end();
            }

            // 返回数组迭代器的终止位置
            _Const_Iter End() const
            {
                return m_setInternel.cend();
            }

            // 增加一个设置主键的接口,为了使操作履历能携带PID
            void SetMainKey(const Base::String &key)
            {
                m_sKey = key;
            }

            // 得到主键,用于写履历的时候写入PID
            Base::String GetMainKey() const
            {
                return m_sKey;
            }

        private:
            // 内部采用STL的set实现
            std::set<T, _Pr> m_setInternel;
            Base::String m_sKey; // 主键
        };
    }
}

#endif // ENGINE_BASE_SET_H_