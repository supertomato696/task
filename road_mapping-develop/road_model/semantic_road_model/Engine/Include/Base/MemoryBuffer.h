/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MemoryBuffer.h
简要描述:memory buffer. Direct read or write memory.
******************************************************************/
#ifndef ENGINE_BASE_MEMORYBUFFER
#define ENGINE_BASE_MEMORYBUFFER

#include "Base/Types.h"
#include "Base/String.h"

#include <string.h>

namespace Engine
{
    namespace Base
    {
        enum class MemoryBufferPosition
        {
            MB_SEEK_CUR,
            MB_SEEK_END,
            MB_SEEK_SET,
        };

        class MemoryBuffer
        {
        public:
            MemoryBuffer(void)
            {
                m_pszBuffer = NULL; // 内存块指针
                m_i64TotalSize = 0; // 分配内存大小
                m_i64UsedSize = 0;  // 使用内存大小
                m_i64Position = -1; // 当前位置
                m_attached = false; //
            }

            virtual ~MemoryBuffer(void)
            {
                Clear();
            }

        private:
            MemoryBuffer(const void *buffer, Int64 size)
            {
                this->m_pszBuffer = (UChar *)buffer;
                this->m_i64TotalSize = size;
                this->m_i64UsedSize = size;
                this->m_i64Position = 0;
                this->m_attached = true;
            }

            /**
             * @brief	所有的写入函数
             */
        public:
            /**
             * @brief	将数据写入内存缓冲区，如果缓冲区不够将申请更大的区域.
             *			以1024为最小递增单位分配内存.
             *
             * @param ptr       要写入的数据内容
             * @param size      单位长度
             * @param count     个数
             * @param stream     内存缓冲区对象
             *
             * @return			写入的数据字节数
             */
            Int64 Write(const void *buffer, Int64 size, Int64 count);

            Int64 Write(const MemoryBuffer *otherStream);

            Int64 Write(const Double &value);

            Int64 Write(const Float &value);

            Int64 Write(const Int32 &value);

            Int64 Write(const Int64 &value);

            Int64 Write(const UChar &value);

            Int64 Write(const String &value);

            Int64 Write(const Char &value);

            // Int64 Write(const NIwchar& value);

            Int64 Write(const SizeT &value);

            Int64 Write(const UInt32 &value);

            Int64 Write(const UInt16 &value);

            Int64 Write(const UChar *value, Int64 charCount);

            Int64 Write(const Char *value, Int64 charCount);

            // Int64 Write(const NIwchar* value, Int64 charCount);

            /**
             * @brief	所有的读取函数
             */
        public:
            /**
             * @将从内存缓冲区读出数据，如果达到缓冲区末尾，停止读取
             *
             * @param ptr        要读出的数据内容
             * @param size       单位长度
             * @param count      个数
             * @param stream     内存缓冲区对象
             *
             * @return           读出的数据字节数
             */
            Int64 Read(void *buffer, Int64 size, Int64 count);

            Int64 Read(Double &value);

            Int64 Read(Float &value);

            Int64 Read(Int32 &value);

            Int64 Read(Int64 &value);

            Int64 Read(UChar &value);

            Int64 Read(String &value);

            // Int64 Read(String& value);

            Int64 Read(Char &value);

            // Int64 Read(NIwchar& value);

            Int64 Read(UInt16 &value);

            Int64 Read(Char *value, Int64 charCount);

            //	Int64 Read(NIwchar* value, Int64 charCount);

            /**
             * @brief 所有对外开放的辅助函数
             */
        public:
            /**
             * @brief 初始化当前使用内存
             *			将当前指针移到首部，将使用内存大小重置为0，但不改变已分配内存.
             *
             * @param stream     内存缓冲区对象
             *
             * @return           当前读写指针位置
             */
            Bool Reset();

            /**
             * @brief 移动读写指针位置
             *
             * @param stream     内存缓冲区对象
             * @param offset     移动偏移量
             * @param origin     移动意识参考位置
             *
             * @return           移动后指针位置
             */
            Int64 Seek(Int64 offset, MemoryBufferPosition origin);

            /**
             * @brief	使用外部内存初始化一个内存缓冲区，只读性质
             *
             * @param ptr       要写入的数据内容
             * @param size      单位长度
             * @return          内存缓冲区对象
             */
            static MemoryBuffer *Attach(const void *buffer, Int64 size);

            /**
             * @brief	将内存缓冲区分离出来，只读性质
             *
             * @param stream    要detach的内存缓冲区对象
             * @param size      内存长度
             * @return          数据缓存
             */
            void *Detach(Int64 *size);

            /**
             * @brief	增大内存空间容量.
             *			如果新容量小于原有容量,则不改变容量大小.
             *
             * @param stream    内存缓冲区对象.
             * @param newsize   新增加的容量大小
             *
             * @return			增加后现有内存大小.
             */
            Int64 Grow(Int64 newMemSize);

            /**
             * @brief	获取类内部内存指针.
             *
             * @return	 内部内存指针
             */
            UChar *GetBuffer() const;

            /**
             * @brief	获取缓存的实际大小.
             *
             * @return	 内部缓存实际大小
             */
            Int64 GetTotalBufferSize() const;

            /**
             * @brief	获取缓存已使用的空间大小.
             * @return	 缓存已使用的空间大小
             */
            Int64 GetUsedBufferSize() const;

            /**
             * @brief	设置缓存已使用的空间大小.
             * @param size 缓存使用大小
             * @return	 缓存已使用的空间大小
             */
            void SetUsedBufferSize(Int64 size);

            /**
             * @brief	获取内存指针的位置.
             * @return	 内存指针位置
             */
            Int64 GetPosition() const;

            /**
             * @brief	将缓存中的所有内容重置为0，类似于ZeroMemory函数
             */
            Void ZeroBuffer();

            /**
             * @brief	释放缓存已分配内存空间，将所有内部变量重新初始化
             */
            Void Clear();

            /**
             * @brief	判断是否到达缓存的起始位置：Begin of buffer
             * @return
             *		true 已经到达起点;
             *		false 没有到达起点位置
             */
            Bool IsBOB();

            /**
             * @brief	判断是否到达缓存的终点位置：End of buffer
             * @return
             *		true 已经到达终点;
             *		false 没有到达终点位置
             */
            Bool IsEOB();

            /**
             * @brief 所有私有函数
             */
        private:
            /**
             * @brief 私有构造函数
             * @param buffer     外部传入的已分配的内存空间
             * @param size     外部传入的已分配的内存空间的大小
             * @return           是否分配成功
             */
            // MemoryBuffer(const void *buffer, Int64 size);
            /**
             * @brief 分配内存
             * @param size     需要分配的内存大小
             * @return           是否分配成功
             */
            Bool MemAlloc(Int64 size);

        private:
            UChar *m_pszBuffer;   // 内存块指针
            Int64 m_i64TotalSize; // 分配内存大小
            Int64 m_i64UsedSize;  // 使用内存大小
            Int64 m_i64Position;  // 当前位置
            Bool m_attached;      // 是否使用了外部的内存对象，如果是外部内存对象，则该类析构时不负责内存释放。
            //	static Int32 g_basicGrowStepLength; // 当用户写入的数据大于内部已经申请的内存空间时，按照这个数值的倍数来申请内存。默认是1024字节
        };

        const Int32 g_basicGrowStepLength = 1024;

        /**
         * @brief 分配内存
         * @param size     需要分配的内存大小
         * @return           是否分配成功
         */
        Bool MemoryBuffer::MemAlloc(Int64 size)
        {
            // 四字节对齐
            Int64 realSize = (size + g_basicGrowStepLength - 1) / g_basicGrowStepLength * g_basicGrowStepLength;

            m_pszBuffer = (UChar *)malloc(realSize);

            if (m_pszBuffer == NULL)
            {
                return false;
            }

            m_i64TotalSize = realSize;
            m_i64UsedSize = 0;
            m_i64Position = 0;
            m_attached = false;

            return true;
        }

        /**
         * @brief	将数据写入内存缓冲区，如果缓冲区不够将申请更大的区域.
         *			以1024为最小递增单位分配内存.
         *
         * @param ptr       要写入的数据内容
         * @param size      单位长度
         * @param count     个数
         * @param stream     内存缓冲区对象
         *
         * @return			写入的数据字节数
         */
        Int64 MemoryBuffer::Write(const void *buffer, Int64 size, Int64 count)
        {
            Int64 writingBufferSize = size * count;

            if (m_attached) // 如果是外部传入的内存地址
            {
                if (this->m_i64TotalSize < Int64(writingBufferSize + this->m_i64Position))
                {
                    return 0;
                }
            }
            else
            {
                if (m_pszBuffer == NULL)
                {
                    MemAlloc(size * count);
                }

                if (this->m_i64TotalSize < Int64(writingBufferSize + this->m_i64Position))
                {
                    Grow(Int64(writingBufferSize + this->m_i64Position) - m_i64TotalSize);
                }
            }

            memcpy(this->m_pszBuffer + m_i64Position, buffer, writingBufferSize);

            m_i64Position += writingBufferSize;

            if (m_i64Position > m_i64UsedSize)
            {
                m_i64UsedSize = m_i64Position;
            }

            return writingBufferSize;
        }

        Int64 MemoryBuffer::Write(const MemoryBuffer *otherStream)
        {
            return Write(otherStream->GetBuffer(), sizeof(UChar), otherStream->GetUsedBufferSize());
        }

        Int64 MemoryBuffer::Write(const Double &value)
        {
            return Write(&value, sizeof(Double), 1);
        }

        Int64 MemoryBuffer::Write(const Float &value)
        {
            return Write(&value, sizeof(Float), 1);
        }

        Int64 MemoryBuffer::Write(const Int32 &value)
        {
            return Write(&value, sizeof(Int32), 1);
        }

        Int64 MemoryBuffer::Write(const Int64 &value)
        {
            return Write(&value, sizeof(Int64), 1);
        }

        Int64 MemoryBuffer::Write(const UChar &value)
        {
            return Write(&value, sizeof(UChar), 1);
        }

        Int64 MemoryBuffer::Write(const String &value)
        {
            return Write(value.GetCStr(), sizeof(Char), value.Length());
        }

        Int64 MemoryBuffer::Write(const Char &value)
        {
            return Write(&value, sizeof(Char), 1);
        }

        /*	Int64 MemoryBuffer::Write(const NIwchar& value)
            {
            return Write(&value, sizeof(NIwchar), 1);
            }*/

        Int64 MemoryBuffer::Write(const SizeT &value)
        {
            return Write(&value, sizeof(SizeT), 1);
        }

        Int64 MemoryBuffer::Write(const UInt32 &value)
        {
            return Write(&value, sizeof(UInt32), 1);
        }

        Int64 MemoryBuffer::Write(const UInt16 &value)
        {
            return Write(&value, sizeof(UInt16), 1);
        }

        Int64 MemoryBuffer::Write(const UChar *value, Int64 charCount)
        {
            return Write(value, sizeof(UChar), charCount);
        }

        Int64 MemoryBuffer::Write(const Char *value, Int64 charCount)
        {
            return Write(value, sizeof(Char), charCount);
        }

        /*	Int64  MemoryBuffer::Write(const NIwchar* value, Int64 charCount)
            {
            return Write(value, sizeof(NIwchar), charCount);
            }*/

        /**
         * @brief	所有的读取函数
         */

        /**
         * @将从内存缓冲区读出数据，如果达到缓冲区末尾，停止读取
         *
         * @param ptr        要读出的数据内容
         * @param size       单位长度
         * @param count      个数
         * @param stream     内存缓冲区对象
         *
         * @return           读出的数据字节数
         */
        Int64 MemoryBuffer::Read(void *buffer, Int64 size, Int64 count)
        {
            if (NULL == m_pszBuffer ||
                NULL == buffer)
            {
                return 0;
            }

            Int64 readingBufferSize = size * count;

            Int64 currentBufferSize = m_i64UsedSize - m_i64Position;

            if (readingBufferSize > currentBufferSize)
            {
                return 0;
            }

            memcpy(buffer, m_pszBuffer + m_i64Position, readingBufferSize);

            m_i64Position += readingBufferSize;

            return m_i64Position;
        }

        Int64 MemoryBuffer::Read(Double &value)
        {
            return Read(&value, sizeof(Double), 1);
        }

        Int64 MemoryBuffer::Read(Float &value)
        {
            return Read(&value, sizeof(Float), 1);
        }

        Int64 MemoryBuffer::Read(Int32 &value)
        {
            return Read(&value, sizeof(Int32), 1);
        }

        Int64 MemoryBuffer::Read(Int64 &value)
        {
            return Read(&value, sizeof(Int64), 1);
        }

        Int64 MemoryBuffer::Read(UChar &value)
        {
            return Read(&value, sizeof(UChar), 1);
        }

        // Int64 MemoryBuffer::Read(String& value)
        //{
        //	if (m_pszBuffer == NULL ||
        //		m_i64TotalSize == 0 ||
        //		m_i64UsedSize == 0)
        //	{
        //		return 0;
        //	}

        //	value = L"";
        //	Int64 startPos = m_i64Position;
        //	Int64 endPos = -1;
        //	//NIwchar* pStr = (NIwchar*)m_pszBuffer;

        //	for (Int64 i = startPos; i < m_i64UsedSize; i += sizeof(NIwchar))
        //	{
        //		if (*(m_pszBuffer + i) == L'\0' ||
        //			*(m_pszBuffer + i) == L'\n')
        //		{
        //			endPos = i;
        //			break;
        //		}
        //	}

        //	// 说明当前buffer中没有换行符，那么就直接返回所有的字符
        //	if (endPos < 0)
        //	{
        //		value = String((NIwchar*)(m_pszBuffer + startPos), 0, Int64((m_i64UsedSize - startPos)*0.5));

        //		m_i64Position = m_i64UsedSize;
        //	}
        //	else
        //	{
        //		value = String((NIwchar*)(m_pszBuffer + startPos), 0, Int64((endPos - startPos + 1)*0.5));

        //		m_i64Position = endPos + 1;
        //	}
        //	value = value.Trim(L'\n');
        //	value = value.Trim(L'\r');
        //	return value.Length();
        //}

        Int64 MemoryBuffer::Read(String &value)
        {
            if (m_pszBuffer == NULL ||
                m_i64TotalSize == 0 ||
                m_i64UsedSize == 0)
            {
                return 0;
            }

            value = "";
            Int64 startPos = m_i64Position;
            Int64 endPos = -1;
            for (Int64 i = startPos; i < m_i64UsedSize; i++)
            {
                if (*(m_pszBuffer + i) == '\0' ||
                    *(m_pszBuffer + i) == '\n')
                {
                    endPos = i;
                    break;
                }
            }

            // 说明当前buffer中没有换行符，那么就直接返回所有的字符
            if (endPos < 0)
            {
                value = String((Char *)(m_pszBuffer + startPos), 0, m_i64UsedSize - startPos);

                m_i64Position = m_i64UsedSize;
            }
            else
            {
                value = String((Char *)(m_pszBuffer + startPos), 0, endPos - startPos);

                m_i64Position = endPos + 1;
            }

            value = value.Trim('\n');
            value = value.Trim('\r');

            return value.Length();
        }

        Int64 MemoryBuffer::Read(Char &value)
        {
            return Read(&value, sizeof(Char), 1);
        }

        /*		Int64 MemoryBuffer::Read(NIwchar& value)
                {
                return Read(&value, sizeof(NIwchar), 1);
                }
                */
        Int64 MemoryBuffer::Read(UInt16 &value)
        {
            return Read(&value, sizeof(UInt16), 1);
        }

        Int64 MemoryBuffer::Read(Char *value, Int64 charCount)
        {
            return Read(value, sizeof(Char), charCount);
        }

        /**
         * @brief 所有对外开放的辅助函数
         */

        /**
         * @brief 初始化当前使用内存
         *			将当前指针移到首部，将使用内存大小重置为0，但不改变已分配内存.
         *
         * @param stream     内存缓冲区对象
         *
         * @return           当前读写指针位置
         */
        Bool MemoryBuffer::Reset()
        {
            m_i64Position = 0;
            m_i64UsedSize = 0;

            return true;
        }

        /**
         * @brief 移动读写指针位置
         *
         * @param stream     内存缓冲区对象
         * @param offset     移动偏移量
         * @param origin     移动意识参考位置
         *
         * @return           移动后指针位置
         */
        Int64 MemoryBuffer::Seek(Int64 offset, MemoryBufferPosition origin)
        {
            if (m_pszBuffer == 0)
            {
                return 0;
            }

            Int64 startPosition = 0;
            switch (origin)
            {
            case MemoryBufferPosition::MB_SEEK_CUR:
            {
                startPosition = m_i64Position + offset;
            }
            break;
            case MemoryBufferPosition::MB_SEEK_END:
            {
                startPosition = m_i64UsedSize + offset;
            }
            break;
            case MemoryBufferPosition::MB_SEEK_SET:
            {
                startPosition = offset;
            }
            break;
            default:
            {
                return 0;
            }
            break;
            }

            if (startPosition < 0)
            {
                startPosition = 0;
            }
            else if (startPosition > m_i64UsedSize)
            {
                startPosition = m_i64UsedSize;
            }
            else
            {
            }

            m_i64Position = startPosition;

            return m_i64Position;
        }

        /**
         * @brief	使用外部内存初始化一个内存缓冲区，只读性质
         *
         * @param ptr       要写入的数据内容
         * @param size      单位长度
         * @return          内存缓冲区对象
         */
        MemoryBuffer *MemoryBuffer::Attach(const void *buffer, Int64 size)
        {
            if (buffer == NULL ||
                size == 0)
            {
                return NULL;
            }

            MemoryBuffer *result = new MemoryBuffer(buffer, size);

            return result;
        }

        /**
         * @brief	将内存缓冲区分离出来，只读性质
         *
         * @param stream    要detach的内存缓冲区对象
         * @param size      内存长度
         * @return          数据缓存
         */
        void *MemoryBuffer::Detach(Int64 *size)
        {
            if (m_pszBuffer == NULL ||
                !m_attached)
            {
                return NULL;
            }

            m_attached = false;
            void *result = m_pszBuffer;

            if (size != NULL)
            {
                *size = (size_t)m_i64TotalSize;
            }

            m_pszBuffer = NULL;
            m_i64TotalSize = 0;
            m_i64UsedSize = 0;
            m_i64Position = -1;

            return result;
        }

        /**
         * @brief	增大内存空间容量.
         *			如果新容量小于原有容量,则不改变容量大小.
         *
         * @param stream    内存缓冲区对象.
         * @param newsize   新增加的容量大小
         *
         * @return			增加后现有内存大小.
         */
        Int64 MemoryBuffer::Grow(Int64 newMemSize)
        {
            if (m_attached)
            {
                return m_i64TotalSize;
            }

            if (this->m_pszBuffer == NULL)
            {
                MemAlloc(newMemSize);

                return m_i64TotalSize;
            }

            Int64 realSize = (m_i64TotalSize + newMemSize + g_basicGrowStepLength - 1) / g_basicGrowStepLength * g_basicGrowStepLength;

            UChar *buf = (UChar *)realloc(m_pszBuffer, realSize);

            if (NULL != buf)
            {
                m_pszBuffer = buf;
                m_i64TotalSize = realSize;
            }

            return m_i64TotalSize;
        }

        /**
         * @brief	获取类内部内存指针.
         *
         * @return	 内部内存指针
         */
        UChar *MemoryBuffer::GetBuffer() const
        {
            return m_pszBuffer;
        }

        /**
         * @brief	获取缓存的实际大小.
         *
         * @return	 内部缓存实际大小
         */
        Int64 MemoryBuffer::GetTotalBufferSize() const
        {
            return m_i64TotalSize;
        }

        /**
         * @brief	获取缓存已使用的空间大小.
         * @return	 缓存已使用的空间大小
         */
        Int64 MemoryBuffer::GetUsedBufferSize() const
        {
            return m_i64UsedSize;
        }

        /**
         * @brief	设置缓存已使用的空间大小.
         * @param size 缓存使用大小
         * @return	 缓存已使用的空间大小
         */
        void MemoryBuffer::SetUsedBufferSize(Int64 size)
        {
            m_i64UsedSize = size;
        }

        /**
         * @brief	获取内存指针的位置.
         * @return	 内存指针位置
         */
        Int64 MemoryBuffer::GetPosition() const
        {
            return m_i64Position;
        }

        /**
         * @brief	将缓存中的所有内容重置为0，类似于ZeroMemory函数
         */
        Void MemoryBuffer::ZeroBuffer()
        {
            if (m_pszBuffer == NULL)
            {
                return;
            }

            if (m_i64TotalSize <= 0)
            {
                return;
            }

            memset(m_pszBuffer, 0, m_i64TotalSize);
        }

        /**
         * @brief	释放缓存已分配内存空间，将所有内部变量重新初始化
         */
        Void MemoryBuffer::Clear()
        {
            if (NULL != m_pszBuffer &&
                !m_attached)
            {
                free(m_pszBuffer);
            }

            m_pszBuffer = NULL;
            m_i64TotalSize = 0; // 分配内存大小
            m_i64UsedSize = 0;  // 使用内存大小
            m_i64Position = -1; // 当前位置
            m_attached = false; //
        }

        /**
         * @brief	判断是否到达缓存的起始位置：Begin of buffer
         * @return
         *		true 已经到达起点;
         *		false 没有到达起点位置
         */
        Bool MemoryBuffer::IsBOB()
        {
            return m_i64Position == 0;
        }

        /**
         * @brief	判断是否到达缓存的终点位置：End of buffer
         * @return
         *		true 已经到达终点;
         *		false 没有到达终点位置
         */
        Bool MemoryBuffer::IsEOB()
        {
            return m_i64Position >= m_i64UsedSize;
        }
    }
}
#endif // !ENGINE_BASE_MEMORYBUFFER
