/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:List.h
简要描述:列表类
******************************************************************/

#ifndef ENGINE_BASE_LIST_H_
#define ENGINE_BASE_LIST_H_

#include "Types.h"
#include "Macros.h"

namespace Engine
{
    namespace Base
    {
        template <class TYPE>
        class List
        {
        public:
            struct ListNode
            {
                /** The next. */
                ListNode *pNext;
                /** The previous. */
                ListNode *pPrev;
                /** The data. */
                TYPE data;
            };

            typedef ListNode *Position; ///< 位置类型。

        public:
            // Construction
            List()
            {
                m_nCount = 0;
                m_pNodeHead = NULL;
                m_pNodeTail = NULL;
                m_pNodeFree = NULL;
            }

            ~List()
            {
                RemoveAll();
            }

            // Attributes (head and tail)
            // count of elements
            Int32 GetCount() const
            {
                return m_nCount;
            }

            Bool IsEmpty() const
            {
                return m_nCount == 0;
            }

            // peek at head or tail
            TYPE &GetHead()
            {
                return m_pNodeHead->data;
            }

            const TYPE &GetHead() const
            {
                return m_pNodeHead->data;
            }

            TYPE &GetTail()
            {
                return m_pNodeTail->data;
            }

            const TYPE &GetTail() const
            {
                return m_pNodeTail->data;
            }

            // Operations
            // get head or tail (and remove it) - don't call on empty list !
            TYPE RemoveHead()
            {
                ListNode *pOldNode = m_pNodeHead;
                TYPE returnValue = pOldNode->data;

                m_pNodeHead = pOldNode->pNext;
                if (m_pNodeHead != NULL)
                {
                    m_pNodeHead->pPrev = NULL;
                }
                else
                {
                    m_pNodeTail = NULL;
                }
                delete pOldNode;
                --m_nCount;
                return returnValue;
            }

            TYPE RemoveTail()
            {
                ListNode *pOldNode = m_pNodeTail;
                TYPE returnValue = pOldNode->data;

                m_pNodeTail = pOldNode->pPrev;
                if (m_pNodeTail != NULL)
                {
                    m_pNodeTail->pNext = NULL;
                }
                else
                {
                    m_pNodeHead = NULL;
                }
                delete pOldNode;
                --m_nCount;
                return returnValue;
            }

            // add before head or after tail
            Position AddHead(const TYPE &newElement)
            {
                ListNode *pNewNode = new ListNode();
                pNewNode->data = newElement;
                if (m_pNodeHead != NULL)
                {
                    m_pNodeHead->pPrev = pNewNode;
                    pNewNode->pNext = m_pNodeHead;
                }
                else
                {
                    m_pNodeTail = pNewNode;
                }
                m_pNodeHead = pNewNode;
                ++m_nCount;
                return pNewNode;
            }

            Position AddTail(const TYPE &newElement)
            {
                ListNode *pNewNode = new ListNode();
                pNewNode->data = newElement;
                if (m_pNodeTail != NULL)
                {
                    m_pNodeTail->pNext = pNewNode;
                    pNewNode->pPrev = m_pNodeTail;
                }
                else
                {
                    m_pNodeHead = pNewNode;
                }
                m_pNodeTail = pNewNode;
                ++m_nCount;
                return pNewNode;
            }

            // add another list of elements before head or after tail
            Void AddHead(List *pNewList)
            {
                // add a list of same elements to head (maintain order)
                Position pos = pNewList->GetTailPosition();
                while (pos != NULL)
                {
                    AddHead(pNewList->GetPrev(pos));
                }
            }

            Void AddTail(List *pNewList)
            {
                // add a list of same elements
                Position pos = pNewList->GetHeadPosition();
                while (pos != NULL)
                {
                    AddTail(pNewList->GetNext(pos));
                }
            }

            // remove all elements
            Void RemoveAll()
            {
                // destroy elements
                while (m_nCount > 0)
                {
                    RemoveTail();
                }
            }

            // iteration
            Position GetHeadPosition() const
            {
                return m_pNodeHead;
            }

            Position GetTailPosition() const
            {
                return m_pNodeTail;
            }

            // return *Position++
            TYPE &GetNext(Position &rPosition)
            {
                ListNode *pNode = (ListNode *)rPosition;

                rPosition = pNode->pNext;
                return pNode->data;
            }

            // return *Position++
            const TYPE &GetNext(Position &rPosition) const
            {
                ListNode *pNode = (ListNode *)rPosition;

                rPosition = pNode->pNext;
                return pNode->data;
            }

            // return *Position--
            TYPE &GetPrev(Position &rPosition)
            {
                ListNode *pNode = (ListNode *)rPosition;

                rPosition = pNode->pPrev;
                return pNode->data;
            }

            // return *Position--
            const TYPE &GetPrev(Position &rPosition) const
            {
                ListNode *pNode = (ListNode *)rPosition;

                rPosition = pNode->pPrev;
                return pNode->data;
            }

            // getting/modifying an element at a given position
            TYPE &GetAt(Position position)
            {
                ListNode *pNode = position;

                return pNode->data;
            }

            const TYPE &GetAt(Position position) const
            {
                ListNode *pNode = position;

                return pNode->data;
            }

            Void SetAt(Position pos, const TYPE &newElement)
            {
                ListNode *pNode = (ListNode *)pos;

                pNode->data = newElement;
            }

            Void RemoveAt(Position position)
            {
                ListNode *pOldNode = position;

                // remove pOldNode from list
                if (pOldNode == m_pNodeHead)
                {
                    m_pNodeHead = pOldNode->pNext;
                }
                else
                {
                    pOldNode->pPrev->pNext = pOldNode->pNext;
                }
                if (pOldNode == m_pNodeTail)
                {
                    m_pNodeTail = pOldNode->pPrev;
                }
                else
                {
                    pOldNode->pNext->pPrev = pOldNode->pPrev;
                }
                delete pOldNode;
                --m_nCount;
            }

            // inserting before or after a given position
            Position InsertBefore(Position position, const TYPE &newElement)
            {
                // Insert it before position
                ListNode *pOldNode = position;
                ListNode *pNewNode = new ListNode();
                pNewNode->data = newElement;

                if (pOldNode->pPrev != NULL)
                {
                    pNewNode->pPrev = pOldNode->pPrev;
                    pOldNode->pPrev->pNext = pNewNode;
                }
                else
                {
                    m_pNodeHead = pNewNode;
                }

                pOldNode->pPrev = pNewNode;
                pNewNode->pNext = pOldNode;
                ++m_nCount;
                return pNewNode;
            }

            Position InsertAfter(Position position, const TYPE &newElement)
            {
                // Insert it before position
                ListNode *pOldNode = position;

                ListNode *pNewNode = new ListNode();
                pNewNode->data = newElement;

                if (pOldNode->pNext != NULL)
                {
                    pNewNode->pNext = pOldNode->pNext;
                    pOldNode->pNext->pPrev = pNewNode;
                }
                else
                {
                    m_pNodeTail = pNewNode;
                }
                pOldNode->pNext = pNewNode;
                pNewNode->pPrev = pOldNode;
                ++m_nCount;
                return pNewNode;
            }

            // defaults to starting at the HEAD, return NULL if not found
            Position Find(const TYPE &searchValue, Position startAfter = NULL) const
            {
                ListNode *pNode = (ListNode *)startAfter;
                if (pNode == NULL)
                {
                    pNode = m_pNodeHead; // start at head
                }
                else
                {
                    pNode = pNode->pNext; // start after the one specified
                }

                for (; pNode != NULL; pNode = pNode->pNext)
                {
                    if (searchValue == pNode->data)
                    {
                        return pNode;
                    }
                }
                return NULL;
            }

            // get the 'nIndex'th element (may return NULL)
            Position FindIndex(Int32 nIndex) const
            {
                if (nIndex >= m_nCount || nIndex < 0)
                {
                    return NULL; // went too far
                }

                ListNode *pNode = m_pNodeHead;
                while (nIndex--)
                {
                    pNode = pNode->pNext;
                }
                return pNode;
            }

        protected:
            ListNode *m_pNodeHead;
            ListNode *m_pNodeTail;
            Int32 m_nCount;
            ListNode *m_pNodeFree;

        private:
            // 复制构造函数
            List(const List &rhs);

            // 赋值操作符
            List &operator=(const List &rhs);
        };
    }
}

#endif // ENGINE_BASE_LIST_H_