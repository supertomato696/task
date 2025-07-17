/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:String.h
简要描述:字符串类
******************************************************************/

#ifndef ENGINE_BASE_STRING_H_
#define ENGINE_BASE_STRING_H_

#include "Base.h"
#include "Types.h"
#include <string>
#include "CharsetCode.h"
#include "Array.h"

namespace Engine
{
    namespace Base
    {
        class Base_API String
        {
        public:
            String();

            /**
             * @brief 构造函数
             * @param str 外部传入的字符串，字符串中包含了结束符
             */
            String(const Char *str);

            /**
             * @brief 构造函数
             * @param str 外部传入的UTF16字符串，字符串中包含了结束符， 程序内部会自动转化成UTF8编码
             */
            // String(const UInt16* str);

            /**
             * @brief 构造函数
             * @param str 外部传入的字符串，字符串中不包含结束符
             */
            String(const Char *str, Int32 nStart, Int32 nCount);

            /**
             * @brief 构造函数
             * @param num 32位整型数字
             */
            String(const Int32 num);

            /**
             * @brief 构造函数
             * @param num 64位整型数字
             */
            String(const Int64 num);

            /**
             * @brief 构造函数
             * @param num 浮点型数字
             */
            String(const Float num);

            /**
             * @brief 构造函数
             * @param num Double数字
             */
            String(const Double num, const Int32 nPre);

            /**
             * @brief 拷贝构造函数
             * @param str 外部传入的字符串
             */
            String(const String &str); // from another String

            /**
             * @brief 赋值操作符
             * @param str 外部传入的字符串
             */
            String &operator=(const String &str);

            /**
             * @brief 析构函数
             */
            ~String();

        public:
            /**
             * @brief 判断字符串是否为空
             * @return true 表明字符串为空，没有任何字符；false 表明字符串不为空
             */
            Bool IsEmpty() const;

            /**
             * @brief 获取字符串长度，也即字符个数
             * @return 字符串长度
             */
            SizeT Length() const;

            /**
             * @brief 追加字符串
             * @param str 要追加的字符串，字符串中包含结束符；函数将str字符串追加到当前字符串中
             * @return
             */
            Void Append(const Char *str);

            /**
             * @brief 追加字符串中前N个字符
             * @param str 要追加的字符串，字符串中包含结束符；函数将str字符串追加到当前字符串中
             * @param n 要追加的字符串个数
             * @return
             */
            Void Append(const Char *str, int n);

            // 获取内部字符串只读指针
            const Char *GetCStr() const;

            /**
             * @brief 比较字符串类的大小，用于作为STL容器元素时
             * @param rhs 被比较的字符串
             * @return 是否小于被比较的字符串
             */
            Bool operator<(const String &rhs) const;

            /**
             * @brief 连接字符串,Write by xueyufei
             * @param rhs 被连接的字符串
             * @return 当前字符串和rhs的串联后的字符串
             */
            String operator+(const String &rhs);

            /**
             * @brief 连接字符串,Write by yanchl
             * @param rhs 被连接的字符串
             * @return 本身的引用,当前字符串与rhs连接，当前字符串被修改
             */
            String &operator+=(const String &rhs);

            /**
             * @brief 分割字符串
             * @return 分割后放在集合中,头文件中的Array是此函数引入的
             * @write by xueyufei
             */
            Array<String> Splite(String &delim);

            /**
             * @brief 替换字符串
             * @return 本身的引用，将当前字符串中src替换为des，当前字符串被修改
             * @write by chenhuan
             */
            String &ReplaceString(const String &src, const String &des);

            /**
             * @brief 查找字符串位置
             * @return 返回从pos位置第一次找到src的位置，没找到返回-1
             * @write by chenhuan
             */
            SizeT Pos(const String &src, SizeT pos = 0);

            /**
             * @brief 擦除指定位置字符
             * @return 无
             * @write by guohaiqiang
             */
            void Erase(SizeT pos, SizeT cnt);

            /**
             * @brief 判断是否以src字符串开头
             * @return 判断结果
             * @write by chenhuan
             */
            Bool StartWith(const String &src);

            /**
             * @brief 判断是否以src字符串结尾
             * @return 判断结果
             * @write by chenhuan
             */
            Bool EndWith(const String &src);

            /**
             * @brief 去除字符串右侧的空格
             * @return 去除右侧空格后的字符串
             */
            String &RightTrim()
            {
                return RightTrim(' ');
            }

            /**
             * @brief 去除字符串左侧的空格
             * @return 去除左侧空格后的字符串
             */
            String &LeftTrim()
            {
                return LeftTrim(' ');
            }

            /**
             * @brief 去除字符串左侧和右侧的空格
             * @return 去除左侧和右侧空格后的字符串
             */
            String &Trim()
            {
                return Trim(' ');
            }

            /**
             * @brief 去除右侧的指定字符
             * @return 去除右侧的指定字符后的字符串
             */
            String &RightTrim(Char cr)
            {
                UInt64 pos = m_str->find_last_not_of(cr);

                if (pos == m_str->npos) // 说明所有的字符都是空格
                {
                    m_str->erase();
                }
                else if (pos == (this->Length() - 1)) // 说明最右边的字符不是要去除的字符
                {
                }
                else
                {
                    m_str->erase(pos + 1, this->Length() - pos - 1);
                }

                return *this;
            }

            /**
             * @brief 去除左侧的指定字符
             * @return 去除左侧的指定字符后的字符串
             */
            String &LeftTrim(Char cr)
            {
                auto pos = m_str->find_first_not_of(cr);
                if (pos == m_str->npos) // 说明所有的字符都是空格
                {
                    m_str->erase(); //
                }
                else if (pos == 0) // 说明第一个字符都不是要移除的字符
                {
                }
                else
                {
                    m_str->erase(0, pos);
                }

                return *this;
            }

            /**
             * @brief 去除左侧和右侧的指定字符
             * @return 去除左侧和右侧的指定字符后的字符串
             */
            String &Trim(Char cr)
            {
                RightTrim(cr);
                LeftTrim(cr);

                return *this;
            }

            // 转换成ANSI字符串
            // String ToANSI();

            // 获取当前字符串的编码类型
            CharsetCode GetCharsetCode()
            {
                return m_emCC;
            }

            // 转换成整型值
            // The return value is 0 for atoi, if the input cannot be converted to a value of that type.
            Int32 ToInt32() const
            {
                if (m_str == NULL)
                {
                    return 0;
                }

                return atoi(m_str->c_str());
            }

            // 转换成双精度浮点
            Double ToDouble()
            {
                if (m_str == NULL)
                {
                    return 0;
                }

                return atof(m_str->c_str());
            }

        private:
            std::string *m_str;
            CharsetCode m_emCC;
        };

        /// 判断两个字符串内容是否相等
        Base_API Base::Bool operator==(const String &a, const String &b);

        /// 判断两个字符串内容是否不相等
        Base_API Base::Bool operator!=(const String &a, const String &b);

    }
}

#endif // ENGINE_BASE_STRING_H_