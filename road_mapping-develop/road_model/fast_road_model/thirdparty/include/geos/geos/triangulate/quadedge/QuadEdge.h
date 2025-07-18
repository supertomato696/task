/**********************************************************************
 *
 * GEOS - Geometry Engine Open Source
 * http://geos.osgeo.org
 *
 * Copyright (C) 2012 Excensus LLC.
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the GNU Lesser General Licence as published
 * by the Free Software Foundation.
 * See the COPYING file for more information.
 *
 **********************************************************************
 *
 * Last port: triangulate/quadedge/QuadEdge.java r524
 *
 **********************************************************************/

#ifndef GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H
#define GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H

#include <memory>

#include <geos/triangulate/quadedge/Vertex.h>
#include <geos/geom/LineSegment.h>

namespace geos {
namespace triangulate { //geos.triangulate
namespace quadedge { //geos.triangulate.quadedge

/** \brief
 * A class that represents the edge data structure which implements the quadedge algebra.
 *
 * The quadedge algebra was described in a well-known paper by Guibas and Stolfi,
 * "Primitives for the manipulation of general subdivisions and the computation of Voronoi diagrams",
 * *ACM Transactions on Graphics*, 4(2), 1985, 75-123.
 *
 * Each edge object is part of a quartet of 4 edges, linked via their `_rot` references.
 * Any edge in the group may be accessed using a series of {@link #rot()} operations.
 * Quadedges in a subdivision are linked together via their `next` references.
 * The linkage between the quadedge quartets determines the topology
 * of the subdivision.
 *
 * The edge class does not contain separate information for vertice or faces; a vertex is implicitly
 * defined as a ring of edges (created using the `next` field).
 *
 * @author JTS: David Skea
 * @author JTS: Martin Davis
 * @author Benjamin Campbell
 * */
class GEOS_DLL QuadEdge {
public:
    /** \brief
     * Creates a new QuadEdge quartet from {@link Vertex} o to {@link Vertex} d.
     *
     * @param o the origin Vertex
     * @param d the destination Vertex
     * @return the new QuadEdge* The caller is reponsible for
     *         freeing the returned pointer
     */
    static std::unique_ptr<QuadEdge> makeEdge(const Vertex& o, const Vertex& d);

    /** \brief
     * Creates a new QuadEdge connecting the destination of a to the origin of
     * b, in such a way that all three have the same left face after the
     * connection is complete.
     *
     * Additionally, the data pointers of the new edge are set.
     *
     * @return the new QuadEdge* The caller is reponsible for
     *         freeing the returned pointer
     */
    static std::unique_ptr<QuadEdge> connect(QuadEdge& a, QuadEdge& b);

    /** \brief
     * Splices two edges together or apart.
     *
     * Splice affects the two edge rings around the origins of a and b, and, independently, the two
     * edge rings around the left faces of `a` and `b`.
     * In each case, (i) if the two rings are distinct,
     * Splice will combine them into one, or (ii) if the two are the same ring, Splice will break it
     * into two separate pieces. Thus, Splice can be used both to attach the two edges together, and
     * to break them apart.
     *
     * @param a an edge to splice
     * @param b an edge to splice
     *
     */
    static void splice(QuadEdge& a, QuadEdge& b);

    /** \brief
     * Turns an edge counterclockwise inside its enclosing quadrilateral.
     *
     * @param e the quadedge to turn
     */
    static void swap(QuadEdge& e);

private:
    //// the dual of this edge, directed from right to left
    QuadEdge* _rot;
    Vertex   vertex;			// The vertex that this edge represents
    QuadEdge* next;			  // A reference to a connected edge
    void*   data;
    bool isAlive;
    bool visited;

    /**
     * Quadedges must be made using {@link makeEdge},
     * to ensure proper construction.
     */
    QuadEdge();

public:
    virtual ~QuadEdge();

    /** \brief
     * Free the QuadEdge quartet associated with this QuadEdge by a connect()
     * or makeEdge() call.
     *
     * @note DO NOT call this function on a QuadEdge that was not returned
     * by connect() or makeEdge().
     */
    virtual void free();

    /** \brief
     * Gets the primary edge of this quadedge and its `sym`.
     *
     * The primary edge is the one for which the origin
     * and destination coordinates are ordered
     * according to the standard geom::Coordinate ordering
     *
     * @return the primary quadedge
     */
    const QuadEdge& getPrimary() const;

    /** \brief
     * Sets the external data value for this edge.
     *
     * @param data an object containing external data
     */
    virtual void setData(void* data);

    /** \brief
     * Gets the external data value for this edge.
     *
     * @return the data object
     */
    virtual void* getData();

    /** \brief
     * Marks this quadedge as being deleted.
     *
     * This does not free the memory used by
     * this quadedge quartet, but indicates
     * that this quadedge quartet no longer participates
     * in a subdivision.
     *
     * @note called "delete" in JTS
     *
     */
    void remove();

    /** \brief
     * Tests whether this edge has been deleted.
     *
     * @return `true` if this edge has not been deleted.
     */
    inline bool
    isLive() const
    {
        return isAlive;
    }

    inline bool
    isVisited() const
    {
        return visited;
    }

    inline void
    setVisited(bool v) {
        visited = v;
    }

    /** \brief
     * Sets the connected edge
     *
     * @param p_next edge
     */
    inline void
    setNext(QuadEdge* p_next)
    {
        this->next = p_next;
    }

    /***************************************************************************
     * QuadEdge Algebra
     ***************************************************************************
     */

    /** \brief
     * Gets the dual of this edge, directed from its right to its left.
     *
     * @return the rotated edge
     */
    inline QuadEdge&
    rot() const
    {
        return *_rot;
    }

    /** \brief
     * Gets the dual of this edge, directed from its left to its right.
     *
     * @return the inverse rotated edge.
     */
    inline QuadEdge&
    invRot() const
    {
        return rot().sym();
    }

    /** \brief
     * Gets the edge from the destination to the origin of this edge.
     *
     * @return the sym of the edge
     */
    inline QuadEdge&
    sym() const
    {
        return rot().rot();
    }

    /** \brief
     * Gets the next CCW edge around the origin of this edge.
     *
     * @return the next linked edge.
     */
    inline QuadEdge&
    oNext() const
    {
        return *next;
    }

    /** \brief
     * Gets the next CW edge around (from) the origin of this edge.
     *
     * @return the previous edge.
     */
    inline QuadEdge&
    oPrev() const
    {
        return rot().oNext().rot();
    }

    /** \brief
     * Gets the next CCW edge around (into) the destination of this edge.
     *
     * @return the next destination edge.
     */
    inline QuadEdge&
    dNext() const
    {
        return sym().oNext().sym();
    }

    /** \brief
     * Gets the next CW edge around (into) the destination of this edge.
     *
     * @return the previous destination edge.
     */
    inline QuadEdge&
    dPrev() const
    {
        return invRot().oNext().invRot();
    }

    /** \brief
     * Gets the CCW edge around the left face following this edge.
     *
     * @return the next left face edge.
     */
    inline QuadEdge&
    lNext() const
    {
        return invRot().oNext().rot();
    }

    /** \brief
     * Gets the CCW edge around the left face before this edge.
     *
     * @return the previous left face edge.
     */
    inline QuadEdge&
    lPrev() const
    {
        return oNext().sym();
    }

    /** \brief
     * Gets the edge around the right face ccw following this edge.
     *
     * @return the next right face edge.
     */
    inline QuadEdge&
    rNext()
    {
        return rot().oNext().invRot();
    }

    /** \brief
     * Gets the edge around the right face ccw before this edge.
     *
     * @return the previous right face edge.
     */
    inline QuadEdge&
    rPrev()
    {
        return sym().oNext();
    }

    /***********************************************************************************************
     * Data Access
     **********************************************************************************************/
    /** \brief
     * Sets the vertex for this edge's origin
     *
     * @param o the origin vertex
     */
    inline void
    setOrig(const Vertex& o)
    {
        vertex = o;
    }

    /** \brief
     * Sets the vertex for this edge's destination
     *
     * @param d the destination vertex
     */
    inline void
    setDest(const Vertex& d)
    {
        sym().setOrig(d);
    }

    /** \brief
     * Gets the vertex for the edge's origin
     *
     * @return the origin vertex
     */
    const Vertex&
    orig() const
    {
        return vertex;
    }

    /** \brief
     * Gets the vertex for the edge's destination
     *
     * @return the destination vertex
     */
    const Vertex&
    dest() const
    {
        return sym().orig();
    }

    /** \brief
     * Gets the length of the geometry of this quadedge.
     *
     * @return the length of the quadedge
     */
    inline double
    getLength() const
    {
        return orig().getCoordinate().distance(dest().getCoordinate());
    }

    /** \brief
     * Tests if this quadedge and another have the same line segment geometry,
     * regardless of orientation.
     *
     * @param qe a quadege
     * @return `true` if the quadedges are based on the same line segment regardless of orientation
     */
    bool equalsNonOriented(const QuadEdge& qe) const;

    /** \brief
     * Tests if this quadedge and another have the same line segment geometry
     * with the same orientation.
     *
     * @param qe a quadege
     * @return `true` if the quadedges are based on the same line segment
     */
    bool equalsOriented(const QuadEdge& qe) const;

    /** \brief
     * Creates a {@link geom::LineSegment} representing the
     * geometry of this edge.
     *
     * @return a LineSegment
     */
    std::unique_ptr<geom::LineSegment> toLineSegment() const;
};

} //namespace geos.triangulate.quadedge
} //namespace geos.triangulate
} //namespace goes

#endif //GEOS_TRIANGULATE_QUADEDGE_QUADEDGE_H

