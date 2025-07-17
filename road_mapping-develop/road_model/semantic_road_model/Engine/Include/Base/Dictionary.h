/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Dictionary.h
简要描述:字典类，也即哈希表类

Note:	当前模板类的设计无法使用自定义类型作为KEY值
******************************************************************/

#ifndef ENGINE_BASE_DICTIONARY_H_
#define ENGINE_BASE_DICTIONARY_H_

#include "Types.h"
#include <unordered_map>

namespace Engine
{
    namespace Base
    {
        template <class T, class V>
        class Dictionary
        {
        public:
            typedef typename std::unordered_map<T, V>::const_iterator cst_itr;

        public:
            Dictionary()
                : m_dict()
            {
            }

            ~Dictionary()
            {
                Clear();
            }

            // 复制构造函数,浅拷贝
            Dictionary(const Dictionary &rhs)
                : m_dict(rhs.m_dict)
            {
            }

            // 赋值操作符,浅拷贝
            Dictionary &operator=(const Dictionary &rhs)
            {
                if (this != &rhs)
                {
                    // 先释放当前资源
                    Clear();

                    // 然后进行深拷贝操作
                    m_dict = std::unordered_map<T, V>(rhs.m_dict);
                }

                return *this;
            }

            // 返回元素数量
            SizeT GetCount() const
            {
                return m_dict.size();
            }

            // 如果没有元素,返回true,否则返回false
            Bool IsEmpty() const
            {
                return m_dict.empty();
            }

            // 根据key设置value
            void Set(const T &key, const V &newValue)
            {
                operator[](key) = newValue;
            }

            // 下标操作符,常量版本
            const V &operator[](const T &key) const
            {
                return m_dict.operator[](key);
            }

            // 下标操作符
            V &operator[](const T &key)
            {
                return m_dict.operator[](key);
            }

            // 根据key获取value值,当key不存在时,返回false,否则返回true
            cst_itr Find(const T &key) const
            {
                return m_dict.find(key);
            }

            // 根据key删除
            Bool Remove(const T &key)
            {
                return m_dict.erase(key) > 0;
            }

            // 删除所有元素,并释放内存
            Void Clear()
            {
                m_dict.clear();
                std::unordered_map<T, V> dict;
                m_dict.swap(dict);
            }

            // 获取begin
            cst_itr Begin() const
            {
                return m_dict.begin();
            }

            // 获取end
            cst_itr End() const
            {
                return m_dict.end();
            }

            // 是否是结束位置
            Bool IsEOF(cst_itr itr) const
            {
                return itr == m_dict.end();
            }

        protected:
            std::unordered_map<T, V> m_dict;
        };
    }
}

#endif // ENGINE_BASE_DICTIONARY_H_