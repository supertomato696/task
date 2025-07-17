#include "RoadTopoBuild.h"

#ifndef _E5969763_DF01_4EBE_AF35_4FD2B9D34236
#define _E5969763_DF01_4EBE_AF35_4FD2B9D34236
namespace hdmap_build
{
    class LinearObject
    {
    public:

        LinearObject(Geometries::LineString*, Base::String);
        LinearObject();
        virtual ~LinearObject();

        Geometries::LineString* m_geometry;
        Base::String m_memo;

        static Base::Bool CompareLength(const LinearObject* p0, const LinearObject* p1);

        Geometries::LineString* Geometry();
        Geometries::LineString* Geometry() const;

        Base::Void Break(const Base::Array<Int32>& indices_mark,Base::Array<LinearObject*>& results);
    public:

        //��ʱ������ memo,
        virtual Base::Void SubStrAttributeCopy(Base::Int32 begin,Base::Int32 end,LinearObject* pSubStr);

        virtual LinearObject* CreateObject(Geometries::LineString* pGeometry,Base::String strMemo);

    };

    class LinearObjDuplicateRemoval:public RoadTopoBuild
    {
    public:
        LinearObjDuplicateRemoval();

        //************************************
        // Method:    RemoveDuplicate
        // FullName:  Navinfo::Engine::PCProcess::LinearObjDuplicateRemoval::RemoveDuplicate
        // Access:    public
        // Returns:   Base::Void
        // Qualifier:
        // Parameter: Base::Array<LinearObject * > & input
        // Parameter: Base::Double tolerance
        // Parameter: Base::Bool compare_opposite
        // Parameter: Base::Double mininum_len ��ͳ�������
        //************************************
        Base::Void  RemoveDuplicate(Base::Array<LinearObject*>& input, Base::Double tolerance, Base::Bool compare_opposite,Base::Double mininum_len);
    };
}


#endif //_E5969763_DF01_4EBE_AF35_4FD2B9D34236