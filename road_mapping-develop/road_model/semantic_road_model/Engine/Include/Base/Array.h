/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Array.h
简要描述:数组模板类
******************************************************************/

#ifndef ENGINE_BASE_ARRAY_H_
#define ENGINE_BASE_ARRAY_H_

#include "Types.h"
#include <vector>
#include <algorithm>
#include <functional>
#ifdef _WIN32
#include <xfunctional>
#endif

using namespace std;
namespace Engine
{
    namespace Base
    {
        // 为了尽最大程度上保证该模板类的性能，所以该的函数内部均不对外部传入的参数合法性进行检查，由使用者自己保证
        template <class T>
        class Array
        {
            typedef std::vector<T> _Myt;

        public:
            typedef typename std::vector<T>::iterator _Iter;
            typedef typename std::vector<T>::const_iterator _Const_Iter;     // Write by xueyufei
            typedef typename std::vector<T>::reverse_iterator _Reverse_Iter; // Write by xueyufei

            // 默认构造函数，构造一个长度为零的数组
            Array()
            {
                m_nGrowSize = 0;
            }

            // 析构函数
            ~Array()
            {
                Clear(); // 可能会造成性能下降，但是如果不释放内存，则可能会导致内存一直在膨胀
            }

            // 移动构造
            Array(Array &&that)
                : m_nGrowSize(that.m_nGrowSize)
            {
                m_vectorInternel = std::move(that.m_vectorInternel);
                that.m_nGrowSize = 0;
            }

            // 移动赋值运算符
            Array &operator=(Array &&that)
            {
                if (this != &that)
                {
                    m_vectorInternel.clear();
                    m_vectorInternel = std::move(that.m_vectorInternel);
                    m_nGrowSize = that.m_nGrowSize;
                    that.m_nGrowSize = 0;
                }
                return *this;
            }

            /////add llj 2018-09-13
            Array(const Array &that)
                : m_nGrowSize(that.m_nGrowSize)
            {
                m_vectorInternel = that.m_vectorInternel;
            }
            Array &operator=(const Array &that)
            {
                if (this != &that)
                {
                    m_vectorInternel = that.m_vectorInternel;
                    m_nGrowSize = that.m_nGrowSize;
                }
                return *this;
            }
            /////

            // 构造函数，构造长度为nSize的数组
            // nSize 数组长度，必须大于等于0，函数内部不负责判断
            // explicit 用来放置隐形的从整型转化为数组类型
            explicit Array(SizeT nSize)
            {
                m_nGrowSize = 0;
                m_vectorInternel.resize(nSize);
            }

            // 构造长度为nSize的数组,数组元素的值为elem中的枚举值
            // nSize 数组长度, 必须大于等于0
            // elem 数组元素的默认值
            Array(SizeT nSize, const T &elem)
            {
                m_nGrowSize = 0;
                m_vectorInternel.resize(nSize, elem);
            }

            Array(const _Myt &vect) : m_vectorInternel(vect.size())
            {
                m_nGrowSize = 0;
                std::copy(vect.begin(), vect.end(), m_vectorInternel.begin());
            }

        public:
            // 判断数组是否为空
            // 返回数组是否为空，返回值类型为布尔类型。
            Bool IsEmpty() const
            {
                return m_vectorInternel.empty();
            }

            // 获取数组的长度
            // 返回数组的长度，返回值类型为整型。
            SizeT GetCapacity() const
            {
                return (SizeT)m_vectorInternel.capacity();
            }

            // 设置数组的长度
            // nCapacity 数组的长度,必须大于等于0
            // 如果原有空间不够, 内存空间会增加, 但增加的空间不会调用TYPE的构造函数。
            // 如果原有空间比指定的大, 也不会释放空间。
            void SetCapacity(SizeT nCapacity)
            {
                m_vectorInternel.reserve(nCapacity);
            }

            // 设置自动增长量
            // nGrowSize 自动增长量
            // 设置数组自动增长量后，若数据添加元素后，长度超过数组的容量，会按照自动增长量增加数组容量
            void SetGrowSize(SizeT nGrowSize = 20)
            {
                m_nGrowSize = nGrowSize;
            }

            // 获取自动增长量
            // 返回数组的自动增长量，返回值类型为整型。
            SizeT GetGrowSize() const
            {
                return m_nGrowSize;
            }

            // 获取数组内的元素个数
            SizeT GetCount() const
            {
                return (SizeT)m_vectorInternel.size();
            }

            // 在数组末尾增加一个元素，且数组长度增加1.
            // newElement 新增元素
            // 返回新增元素的索引值
            // 若添加元素后，超过数组的容量，则数组长度会自动按照自动增长量来增长。
            SizeT Add(const T &newElement)
            {
                if ((m_vectorInternel.size() >= m_vectorInternel.capacity()) &&
                    (m_nGrowSize > 0))
                {
                    m_vectorInternel.reserve(m_vectorInternel.capacity() + m_nGrowSize);
                }

                m_vectorInternel.push_back(newElement);

                return (SizeT)m_vectorInternel.size() - 1;
            }

            // 在数组末尾追加元素
            // pData 要追加的元素
            // nSize 要追加的元素个数
            // 返回加入元素的起始索引
            SizeT Add(const T *pData, SizeT nSize)
            {
                SizeT nOldSize = (SizeT)m_vectorInternel.size();
                m_vectorInternel.insert(m_vectorInternel.end(), nSize, *pData);
                return nOldSize;
            }

            // 在数组末尾追加数组。
            // src 要追加的数组
            // 返回追加数组的起始索引，返回值类型为整型。
            SizeT Add(const Array<T> &src)
            {
                SizeT nOldSize = (SizeT)m_vectorInternel.size();
                m_vectorInternel.insert(m_vectorInternel.end(), src.m_vectorInternel.begin(), src.m_vectorInternel.end());
                return nOldSize;
            }

            // 拷贝数组
            // src 传入的数组
            // 复制一个数组的元素到另一个数组中。若原有数组中有值，则会被覆盖。
            void Copy(const Array<T> &src)
            {
                if (this == &src)
                {
                    return;
                }

                *this = src;
            }

            // 得到数组内指定位置元素的引用
            // nIndex 指定的位置
            // 返回该位置元素的引用
            T &ElementAt(SizeT nIndex)
            {
                return m_vectorInternel.at(nIndex);
            }

            // 得到数组中指定位置元素的const引用
            // nIndex 指定的位置
            // 返回该位置元素的const引用
            const T &ElementAt(SizeT nIndex) const
            {
                return m_vectorInternel.at(nIndex);
            }

            // 得到数组内指定位置元素的引用
            // nIndex 指定的位置
            // 返回该位置元素的引用
            T &operator[](SizeT nIndex)
            {
                return m_vectorInternel.at(nIndex);
            }

            // 等号重载符
            const T &operator=(const T &other)
            {
                if (this != &other)
                {
                    m_nGrowSize = other.m_nGrowSize;
                    m_vectorInternel = other.m_vectorInternel;
                }

                return *this;
            }
            // 得到数组中指定位置元素的const引用
            // nIndex 指定的位置
            // 返回该位置元素的const引用
            const T &operator[](SizeT nIndex) const
            {
                return m_vectorInternel.at(nIndex);
            }

            // 得到指定位置元素值
            // nIndex 指定的位置/索引值
            // 返回该位置元素的拷贝
            T GetAt(SizeT nIndex) const
            {
                return m_vectorInternel.at(nIndex);
            }

            // 在指定位置设置元素值(替换原有元素)
            // 指定位置必须在数组长度范围之内
            // nIndex 指定的位置
            // newElement 要设置的元素
            void SetAt(SizeT nIndex, const T &newElement)
            {
                m_vectorInternel.at(nIndex) = newElement;
            }

            // 在指定位置设置元素(替换原有元素)
            // 如果指定位置大于数组长度,数组会自动增长
            // nIndex 指定的位置
            // newElement 要设置的元素
            void SetAtGrow(SizeT nIndex, const T &newElement)
            {
                if (nIndex >= (SizeT)m_vectorInternel.size())
                {
                    m_vectorInternel.resize(nIndex + 1);
                }

                m_vectorInternel.at(nIndex) = newElement;
            }

            // 设置数组内可容纳的元素数量
            // nNewSize 指定的新的元素个数
            // 如果原有空间不够, 内存空间会增加,
            // 增加的空间会调用TYPE的默认构造函数(或等于newElement)。
            // 如果原有空间比指定的大, 不会释放空间,但在指定的空间之后的元素就无效了。
            // 如果要强制释放内存,请调用FreeExtra(),释放多余的内存空间。
            void SetSize(SizeT nNewSize)
            {
                if ((nNewSize > (SizeT)m_vectorInternel.capacity()) &&
                    (m_nGrowSize > 0))
                {
                    m_vectorInternel.reserve(m_vectorInternel.capacity() + (nNewSize / m_nGrowSize + 1) * m_nGrowSize); // 确保能够保证按照自动增长量增长
                }
                m_vectorInternel.resize(nNewSize);
            }

            // 移除数组中指定位置的元素
            // 指定位置必须在数组范围内
            // nIndex 要移除的元素位置
            Void Delete(SizeT nIndex)
            {
                Delete(nIndex, 1);
            }

            // 移除数组中指定位置,指定个数的元素
            // 指定位置必须在数组范围内
            // nIndex 要移除的元素位置
            // nCount 要移除的元素个数,默认为1
            // 返回移除的元素个数
            SizeT Delete(SizeT nIndex, SizeT nCount)
            {
                if (nCount <= 0)
                {
                    return 0;
                }

                SizeT nSize = (SizeT)m_vectorInternel.size();
                if (nIndex >= nSize)
                {
                    return 0;
                }

                SizeT nEnd = nIndex + nCount;
                if (nEnd > nSize)
                {
                    nEnd = nSize;
                }

                m_vectorInternel.erase(m_vectorInternel.begin() + nIndex, m_vectorInternel.begin() + nEnd);
                return nEnd - nIndex;
            }

            // Write by xueyufei
            _Iter Find(const T &element)
            {
                return std::find_if(m_vectorInternel.begin(), m_vectorInternel.end(), std::bind2nd(std::equal_to<T>(), element));
            }

            // 删除迭代器指向的元素
            _Iter Erase(_Iter itr)
            {
                if (itr == m_vectorInternel.end())
                {
                    return itr;
                }
                return m_vectorInternel.erase(itr);
            }

            //************************************
            // 删除数组中所有为element的数据
            // element 要移除的元素
            // 删除的对象个数
            SizeT DeleteAll(const T &element)
            {
                SizeT result = 0;

                typename std::vector<T>::iterator iter = m_vectorInternel.begin();

                while (iter != m_vectorInternel.end())
                {
                    if (*iter == element)
                    {
                        iter = m_vectorInternel.erase(iter);
                        result++;
                    }
                    else
                    {
                        ++iter;
                    }
                }

                return result;
            }

            //************************************
            // Method:    在数组中的指定位置之前插入1个元素
            // Parameter: nIndex
            // Parameter: pNewElement
            // remarks    如果指定位置不在数组范围内，不做任何操作
            //************************************
            void InsertAt(SizeT nIndex, const T &newElement)
            {
                InsertAt(nIndex, newElement, 1);
            }

            //************************************
            // Method:    在数组中的指定位置之前，插入指定个数的元素
            // Parameter: nIndex
            // Parameter: pNewElement
            // remarks    如果指定位置不在数组范围内，不做任何操作
            //************************************
            void InsertAt(SizeT nIndex, const T &newElement, SizeT nCount)
            {
                SizeT nSize = this->GetCount();
                if (nIndex < 0 || nIndex >= nSize)
                {
                    return;
                }
                m_vectorInternel.insert(m_vectorInternel.begin() + nIndex, nCount, newElement);
            }

            //************************************
            // Method:    在数组中的指定位置之前，插入一个新的数组
            // Parameter: nIndex
            // Parameter: pNewElement
            // remarks    如果指定位置不在数组范围内，不做任何操作
            //************************************
            void InsertAt(SizeT nIndex, const Array<T> &newArray)
            {
                SizeT nSize = this->GetCount();
                if (nIndex < 0 || nIndex >= nSize)
                {
                    return;
                }
                m_vectorInternel.insert(m_vectorInternel.begin() + nIndex, newArray.m_vectorInternel.begin(), newArray.m_vectorInternel.end());
            }

            // 移除所有元素,同时释放内存空间
            void Clear()
            {
                m_vectorInternel.clear();
                std::vector<T> vector;
                m_vectorInternel.swap(vector);
            }

            // 移除所有元素,同时不释放内存空间
            void Clean()
            {
                m_vectorInternel.clear();
            }

            // 返回数组的内存起始地址
            T *Data()
            {
                return m_vectorInternel.data();
            }

            // 返回数组迭代器的起始位置
            _Iter Begin()
            {
                return m_vectorInternel.begin();
            }

            // const 迭代器,Write by xueyufei
            _Const_Iter Begin() const
            {
                return m_vectorInternel.begin();
            }

            // 反向迭代器,Write by xueyufei
            _Reverse_Iter RBegin()
            {
                return m_vectorInternel.rbegin();
            }

            // 返回数组迭代器的终止位置
            _Iter End()
            {
                return m_vectorInternel.end();
            }

            // const 迭代器,Write by xueyufei
            _Const_Iter End() const
            {
                return m_vectorInternel.end();
            }

            // 反向迭代器,Write by xueyufei
            _Reverse_Iter REnd()
            {
                return m_vectorInternel.rend();
            }

            // 如果传入的类型T是类对象,需要重载<方法。如String类重载了<方可使用此方法。
            Void Sort()
            {
                std::sort(m_vectorInternel.begin(), m_vectorInternel.end());
            }

            // 反转数组里面的元素顺序
            Void Reverse()
            {
                std::reverse(m_vectorInternel.begin(), m_vectorInternel.end());
            }

        private:
            // 内部采用STL的vector实现
            std::vector<T> m_vectorInternel;

            // 数组的自动增量
            SizeT m_nGrowSize;
        };
    }
}

#endif // ENGINE_BASE_ARRAY_H_