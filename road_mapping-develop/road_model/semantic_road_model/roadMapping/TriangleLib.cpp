#include "TriangleLib.h"
#include "Base/Macros.h"

using namespace Engine;
using namespace Engine::Base;

#define RAD_TO_DEG 57.29578

#define INEXACT 
#define TRIPERBLOCK 4092           /* Number of triangles allocated at once. */
#define SHELLEPERBLOCK 508       /* Number of shell edges allocated at once. */
#define POINTPERBLOCK 4092            /* Number of points allocated at once. */
#define VIRUSPERBLOCK 1020   /* Number of virus triangles allocated at once. */
#define BADSEGMENTPERBLOCK 252
#define BADTRIPERBLOCK 4092
#define SPLAYNODEPERBLOCK 508
#define DEADPOINT -1073741824
#define QIUDUPLICATEPOINT -32767
#define SAMPLEFACTOR 11
#define SAMPLERATE 10
#define SQUAREROOTTWO 1.4142135623730950488016887242096980785696718753769480732
#define ONETHIRD 0.333333333333333333333333333333333333333333333333333333333333

#include <stdio.h>
#include <string.h>
#include <math.h>

#define decode(ptr, triedge)                                                  \
	(triedge).orient = (int) ((UInt64) (ptr) & (UInt64) 3l);      \
	(triedge).tri = (triangle *)                                                \
((UInt64) (ptr) ^ (UInt64) (triedge).orient)

#define encode(triedge)                                                       \
(triangle) ((UInt64) (triedge).tri | (UInt64) (triedge).orient)

#define sym(triedge1, triedge2)                                               \
	ptr = (triedge1).tri[(triedge1).orient];                                    \
decode(ptr, triedge2);

#define symself(triedge)                                                      \
	ptr = (triedge).tri[(triedge).orient];                                      \
decode(ptr, triedge);

#define lnext(triedge1, triedge2)                                             \
	(triedge2).tri = (triedge1).tri;                                            \
(triedge2).orient = plus1mod3[(triedge1).orient]

#define lnextself(triedge)                                                    \
(triedge).orient = plus1mod3[(triedge).orient]

#define lprev(triedge1, triedge2)                                             \
	(triedge2).tri = (triedge1).tri;                                            \
(triedge2).orient = minus1mod3[(triedge1).orient]

#define lprevself(triedge)                                                    \
(triedge).orient = minus1mod3[(triedge).orient]

#define onext(triedge1, triedge2)                                             \
	lprev(triedge1, triedge2);                                                  \
symself(triedge2);

#define onextself(triedge)                                                    \
	lprevself(triedge);                                                         \
symself(triedge);

#define oprev(triedge1, triedge2)                                             \
	sym(triedge1, triedge2);                                                    \
lnextself(triedge2);

#define oprevself(triedge)                                                    \
	symself(triedge);                                                           \
lnextself(triedge);

#define dnext(triedge1, triedge2)                                             \
	sym(triedge1, triedge2);                                                    \
lprevself(triedge2);

#define dnextself(triedge)                                                    \
	symself(triedge);                                                           \
lprevself(triedge);

#define dprev(triedge1, triedge2)                                             \
	lnext(triedge1, triedge2);                                                  \
symself(triedge2);

#define dprevself(triedge)                                                    \
	lnextself(triedge);                                                         \
symself(triedge);

#define rnext(triedge1, triedge2)                                             \
	sym(triedge1, triedge2);                                                    \
	lnextself(triedge2);                                                        \
symself(triedge2);

#define rnextself(triedge)                                                    \
	symself(triedge);                                                           \
	lnextself(triedge);                                                         \
symself(triedge);

#define rprev(triedge1, triedge2)                                             \
	sym(triedge1, triedge2);                                                    \
	lprevself(triedge2);                                                        \
symself(triedge2);

#define rprevself(triedge)                                                    \
	symself(triedge);                                                           \
	lprevself(triedge);                                                         \
symself(triedge);

/* These primitives determine or set the origin, destination, or apex of a   */
/* triangle.                                                                 */

#define org(triedge, pointptr)                                                \
pointptr = (point) (triedge).tri[plus1mod3[(triedge).orient] + 3]

#define dest(triedge, pointptr)                                               \
pointptr = (point) (triedge).tri[minus1mod3[(triedge).orient] + 3]

#define apex(triedge, pointptr)                                               \
pointptr = (point) (triedge).tri[(triedge).orient + 3]

#define setorg(triedge, pointptr)                                             \
(triedge).tri[plus1mod3[(triedge).orient] + 3] = (triangle) pointptr

#define setdest(triedge, pointptr)                                            \
(triedge).tri[minus1mod3[(triedge).orient] + 3] = (triangle) pointptr

#define setapex(triedge, pointptr)                                            \
(triedge).tri[(triedge).orient + 3] = (triangle) pointptr

#define setvertices2null(triedge)                                             \
	(triedge).tri[3] = (triangle) NULL;                                         \
	(triedge).tri[4] = (triangle) NULL;                                         \
(triedge).tri[5] = (triangle) NULL;

/* Bond two triangles together.                                              */

#define bond(triedge1, triedge2)                                              \
	(triedge1).tri[(triedge1).orient] = encode(triedge2);                       \
(triedge2).tri[(triedge2).orient] = encode(triedge1)

/* Dissolve a bond (from one side).  Note that the other triangle will still */
/*   think it's connected to this triangle.  Usually, however, the other     */
/*   triangle is being deleted entirely, or bonded to another triangle, so   */
/*   it doesn't matter.                                                      */

#define dissolve(triedge)                                                     \
(triedge).tri[(triedge).orient] = (triangle) dummytri

/* Copy a triangle/edge handle.                                              */

#define triedgecopy(triedge1, triedge2)                                       \
	(triedge2).tri = (triedge1).tri;                                            \
(triedge2).orient = (triedge1).orient

/* Test for equality of triangle/edge handles.                               */

#define triedgeequal(triedge1, triedge2)                                      \
	(((triedge1).tri == (triedge2).tri) &&                                      \
((triedge1).orient == (triedge2).orient))

/* Primitives to infect or cure a triangle with the virus.  These rely on    */
/*   the assumption that all shell edges are aligned to four-byte boundaries.*/

#define infect(triedge)                                                       \
	(triedge).tri[6] = (triangle)                                               \
((UInt64) (triedge).tri[6] | (UInt64) 2l)

#define uninfect(triedge)                                                     \
	(triedge).tri[6] = (triangle)                                               \
((UInt64) (triedge).tri[6] & ~ (UInt64) 2l)

/* Test a triangle for viral infection.                                      */

#define infected(triedge)                                                     \
(((UInt64) (triedge).tri[6] & (UInt64) 2l) != 0)

/* Check or set a triangle's attributes.                                     */

#define elemattribute(triedge, attnum)                                        \
((double *) (triedge).tri)[elemattribindex + (attnum)]

#define setelemattribute(triedge, attnum, value)                              \
((double *) (triedge).tri)[elemattribindex + (attnum)] = value

/* Check or set a triangle's maximum area bound.                             */

#define areabound(triedge)  ((double *) (triedge).tri)[areaboundindex]

#define setareabound(triedge, value)                                          \
((double *) (triedge).tri)[areaboundindex] = value

/********* Primitives for shell edges                                *********/
/*                                                                           */
/*                                                                           */

/* sdecode() converts a pointer to an oriented shell edge.  The orientation  */
/*   is extracted from the least significant bit of the pointer.  The two    */
/*   least significant bits (one for orientation, one for viral infection)   */
/*   are masked out to produce the real pointer.                             */

#define sdecode(sptr, edge)                                                   \
	(edge).shorient = (int) ((UInt64) (sptr) & (UInt64) 1l);      \
	(edge).sh = (shelle *)                                                      \
((UInt64) (sptr) & ~ (UInt64) 3l)

/* sencode() compresses an oriented shell edge into a single pointer.  It    */
/*   relies on the assumption that all shell edges are aligned to two-byte   */
/*   boundaries, so the least significant bit of (edge).sh is zero.          */

#define sencode(edge)                                                         \
(shelle) ((UInt64) (edge).sh | (UInt64) (edge).shorient)

/* ssym() toggles the orientation of a shell edge.                           */

#define ssym(edge1, edge2)                                                    \
	(edge2).sh = (edge1).sh;                                                    \
(edge2).shorient = 1 - (edge1).shorient

#define ssymself(edge)                                                        \
(edge).shorient = 1 - (edge).shorient

/* spivot() finds the other shell edge (from the same segment) that shares   */
/*   the same origin.                                                        */

#define spivot(edge1, edge2)                                                  \
	sptr = (edge1).sh[(edge1).shorient];                                        \
sdecode(sptr, edge2)

#define spivotself(edge)                                                      \
	sptr = (edge).sh[(edge).shorient];                                          \
sdecode(sptr, edge)

/* snext() finds the next shell edge (from the same segment) in sequence;    */
/*   one whose origin is the input shell edge's destination.                 */

#define snext(edge1, edge2)                                                   \
	sptr = (edge1).sh[1 - (edge1).shorient];                                    \
sdecode(sptr, edge2)

#define snextself(edge)                                                       \
	sptr = (edge).sh[1 - (edge).shorient];                                      \
sdecode(sptr, edge)

/* These primitives determine or set the origin or destination of a shell    */
/*   edge.                                                                   */

#define sorg(edge, pointptr)                                                  \
pointptr = (point) (edge).sh[2 + (edge).shorient]

#define sdest(edge, pointptr)                                                 \
pointptr = (point) (edge).sh[3 - (edge).shorient]

#define setsorg(edge, pointptr)                                               \
(edge).sh[2 + (edge).shorient] = (shelle) pointptr

#define setsdest(edge, pointptr)                                              \
(edge).sh[3 - (edge).shorient] = (shelle) pointptr

/* These primitives read or set a shell marker.  Shell markers are used to   */
/*   hold user boundary information.                                         */

#define mark(edge)  (* (int *) ((edge).sh + 6))

#define setmark(edge, value)                                                  \
* (int *) ((edge).sh + 6) = value

/* Bond two shell edges together.                                            */

#define sbond(edge1, edge2)                                                   \
	(edge1).sh[(edge1).shorient] = sencode(edge2);                              \
(edge2).sh[(edge2).shorient] = sencode(edge1)

/* Dissolve a shell edge bond (from one side).  Note that the other shell    */
/*   edge will still think it's connected to this shell edge.                */

#define sdissolve(edge)                                                       \
(edge).sh[(edge).shorient] = (shelle) dummysh

/* Copy a shell edge.                                                        */

#define shellecopy(edge1, edge2)                                              \
	(edge2).sh = (edge1).sh;                                                    \
(edge2).shorient = (edge1).shorient

/* Test for equality of shell edges.                                         */

#define shelleequal(edge1, edge2)                                             \
	(((edge1).sh == (edge2).sh) &&                                              \
((edge1).shorient == (edge2).shorient))

/********* Primitives for interacting triangles and shell edges      *********/
/*                                                                           */
/*                                                                           */

/* tspivot() finds a shell edge abutting a triangle.                         */

#define tspivot(triedge, edge)                                                \
	sptr = (shelle) (triedge).tri[6 + (triedge).orient];                        \
sdecode(sptr, edge)

/* stpivot() finds a triangle abutting a shell edge.  It requires that the   */
/*   variable `ptr' of type `triangle' be defined.                           */

#define stpivot(edge, triedge)                                                \
	ptr = (triangle) (edge).sh[4 + (edge).shorient];                            \
decode(ptr, triedge)

/* Bond a triangle to a shell edge.                                          */

#define tsbond(triedge, edge)                                                 \
	(triedge).tri[6 + (triedge).orient] = (triangle) sencode(edge);             \
(edge).sh[4 + (edge).shorient] = (shelle) encode(triedge)

/* Dissolve a bond (from the triangle side).                                 */

#define tsdissolve(triedge)                                                   \
(triedge).tri[6 + (triedge).orient] = (triangle) dummysh

/* Dissolve a bond (from the shell edge side).                               */

#define stdissolve(edge)                                                      \
(edge).sh[4 + (edge).shorient] = (shelle) dummytri

/********* Primitives for points                                     *********/
#define pointmark(pt)  ((int *) (pt))[pointmarkindex]

#define setpointmark(pt, value)                                               \
((int *) (pt))[pointmarkindex] = value

#define point2tri(pt)  ((triangle *) (pt))[point2triindex]

#define setpoint2tri(pt, value)                                               \
((triangle *) (pt))[point2triindex] = value

/********* Mesh manipulation primitives end here                     *********/

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TriangleLib::TriangleLib()
{
	plus1mod3[0] = 1, plus1mod3[1] = 2, plus1mod3[2] = 0;
	minus1mod3[0] = 2,minus1mod3[1] = 0, minus1mod3[2] = 1;
	m_strStatistics = "\n";
}

TriangleLib::~TriangleLib()
{
	
}

void TriangleLib::internalerror()
{
}

void TriangleLib::parsecommandline(int argc, char **argv)
{
	int STARTINDEX = 0;
	int i, j;
	
	poly = refine = quality = vararea = fixedarea = regionattrib = convex = 0;
	firstnumber = 1;
	edgesout = voronoi = neighbors = geomview = 0;
	nobound = nopolywritten = nonodewritten = noelewritten = noiterationnum = 0;
	noholes = noexact = 0;
	incremental = sweepline = 0;
	dwyer = 1;
	splitseg = 0;
	docheck = 0;
	nobisect = 0;
	steiner = -1;
	order = 1;
	minangle = 0.0;
	maxarea = -1.0;
	quiet = 0;
	verbose = 0;
	
	for (i = STARTINDEX; i < argc; i++) 
	{
		for (j = STARTINDEX; argv[i][j] != '\0'; j++)
		{
				
			if (argv[i][j] == 'v') {
				voronoi = 1;
			}
				
			if (argv[i][j] == 'N') {
				nonodewritten = 1;
			}
			if (argv[i][j] == 'Q') {
				quiet = 1;
			}
		}
	}

	steinerleft = steiner;
	useshelles = poly || refine || quality || convex;
	goodangle = cos(minangle * PI / 180.0);
	goodangle *= goodangle;
	
	if (!refine && !poly)
	{
		vararea = 0;
	}
	
	if (refine || !poly)
	{
		regionattrib = 0;
	}
}

void TriangleLib::printtriangle(struct triedge *t)
{
	struct triedge printtri;
	struct edge printsh;
	point printpoint;

	String m_strTriangleInfo("\n");
	
	
	m_strTriangleInfo += strBuffer;
	decode(t->tri[0], printtri);
	if (printtri.tri == dummytri) {
		m_strTriangleInfo += "    [0] = Outer space\n";
	} else {
		
		m_strTriangleInfo += strBuffer;
	}
	decode(t->tri[1], printtri);
	if (printtri.tri == dummytri) {
		m_strTriangleInfo += "    [1] = Outer space\n";
	} else {
		
		m_strTriangleInfo += strBuffer;
	}
	decode(t->tri[2], printtri);
	if (printtri.tri == dummytri) {
		m_strTriangleInfo += "    [2] = Outer space\n";
	} else {
		
		m_strTriangleInfo += strBuffer;
	}
	org(*t, printpoint);
	if (printpoint == (point) NULL)
	{
		
		m_strTriangleInfo += strBuffer;
	}
	else
	{
		
		m_strTriangleInfo += strBuffer;
	}
	dest(*t, printpoint);
	if (printpoint == (point) NULL)
	{
		
		m_strTriangleInfo += strBuffer;
	}
	else
	{
		
		m_strTriangleInfo += strBuffer;
	}
	apex(*t, printpoint);
	if (printpoint == (point) NULL)
	{
		
		m_strTriangleInfo += strBuffer;
	}
	else
	{
		
		m_strTriangleInfo += strBuffer;
	}
	if (useshelles) {
		sdecode(t->tri[6], printsh);
		if (printsh.sh != dummysh) 
		{
			
			m_strTriangleInfo += strBuffer;
		}
		sdecode(t->tri[7], printsh);
		if (printsh.sh != dummysh) {
			
			m_strTriangleInfo += strBuffer;
		}
		sdecode(t->tri[8], printsh);
		if (printsh.sh != dummysh) 
		{
			
			m_strTriangleInfo += strBuffer;
		}
		if (vararea) 
		{
			
			m_strTriangleInfo += strBuffer;
		}
	}
}

void TriangleLib::printshelle(struct edge *s)
{
	struct edge printsh;
	struct triedge printtri;
	point printpoint;
	
	String m_strEdgeInfo("\n");

	
	m_strEdgeInfo += strBuffer;
	sdecode(s->sh[0], printsh);
	if (printsh.sh == dummysh) {
		m_strEdgeInfo += " [0] = No shell\n";
	} else {
		
		m_strEdgeInfo += strBuffer;
	}
	sdecode(s->sh[1], printsh);
	if (printsh.sh == dummysh) {
		m_strEdgeInfo += "    [1] = No shell\n";
	} else {
		
		m_strEdgeInfo += strBuffer;
	}
	sorg(*s, printpoint);
	if (printpoint == (point) NULL)
	{
		
		m_strEdgeInfo += strBuffer;
	}
	else
	{
		
		m_strEdgeInfo += strBuffer;
	}
	sdest(*s, printpoint);
	if (printpoint == (point) NULL)
	{
		
		m_strEdgeInfo += strBuffer;
	}
	else
	{
		
		m_strEdgeInfo += strBuffer;
	}
	decode(s->sh[4], printtri);
	if (printtri.tri == dummytri) {
		m_strEdgeInfo += "    [4] = Outer space\n";
	}
	else 
	{
		
		m_strEdgeInfo += strBuffer;
	}
	decode(s->sh[5], printtri);
	if (printtri.tri == dummytri) {
		m_strEdgeInfo += ("[5] = Outer space\n");
	} else 
	{
		
		m_strEdgeInfo += strBuffer;
	}
}

void TriangleLib::poolinit(struct memorypool *pool, int bytecount, int itemcount, enum wordtype wtype, int alignment)
{
	int wordsize;
	
	pool->itemwordtype = wtype;
	wordsize = (pool->itemwordtype == POINTER) ? sizeof(int *) : sizeof(double);
	
	if (alignment > wordsize) {
		pool->alignbytes = alignment;
	} else {
		pool->alignbytes = wordsize;
	}
	if (sizeof(int *) > pool->alignbytes) {
		pool->alignbytes = sizeof(int *);
	}
	pool->itemwords = ((bytecount + pool->alignbytes - 1) / pool->alignbytes)
		* (pool->alignbytes / wordsize);
	pool->itembytes = pool->itemwords * wordsize;
	pool->itemsperblock = itemcount;
	
	/* Allocate a block of items.  Space for `itemsperblock' items and one    */
	/*   pointer (to point to the next block) are allocated, as well as space */
	/*   to ensure alignment of the items.                                    */
	pool->firstblock = (int **) malloc(pool->itemsperblock * pool->itembytes
		+ sizeof(int *) + pool->alignbytes);
	if (pool->firstblock == (int **) NULL) 
	{
		
		return;
	}
	/* Set the next block pointer to NULL. */
	*(pool->firstblock) = (int *) NULL;
	poolrestart(pool);
}

void TriangleLib::poolrestart(struct memorypool *pool)
{
	UInt64 alignptr;
	
	pool->items = 0;
	pool->maxitems = 0;
	
	/* Set the currently active block. */
	pool->nowblock = pool->firstblock;
	/* Find the first item in the pool.  Increment by the size of (int *). */
	alignptr = (UInt64) (pool->nowblock + 1);
	/* Align the item on an `alignbytes'-byte boundary. */
	pool->nextitem = (int *)
		(alignptr + (UInt64) pool->alignbytes
		- (alignptr % (UInt64) pool->alignbytes));
	/* There are lots of unallocated items left in this block. */
	pool->unallocateditems = pool->itemsperblock;
	/* The stack of deallocated items is empty. */
	pool->deaditemstack = (int *) NULL;
}

void TriangleLib::pooldeinit(struct memorypool *pool)
{
	while (pool->firstblock != (int **) NULL) {
		pool->nowblock = (int **) *(pool->firstblock);
		free(pool->firstblock);
		pool->firstblock = pool->nowblock;
	}
}

int * TriangleLib::poolalloc(struct memorypool *pool)
{
	int *newitem;
	int **newblock;
	UInt64 alignptr;
	
	/* First check the linked list of dead items.  If the list is not   */
	/*   empty, allocate an item from the list rather than a fresh one. */
	if (pool->deaditemstack != (int *) NULL) {
		newitem = pool->deaditemstack;               /* Take first item in list. */
		pool->deaditemstack = * (int **) pool->deaditemstack;
	} else {
		/* Check if there are any free items left in the current block. */
		if (pool->unallocateditems == 0) {
			/* Check if another block must be allocated. */
			if (*(pool->nowblock) == (int *) NULL) {
				/* Allocate a new block of items, pointed to by the previous block. */
				newblock = (int **) malloc(pool->itemsperblock * pool->itembytes
					+ sizeof(int *) + pool->alignbytes);
				if (newblock == (int **) NULL) {
					
					return NULL;
				}
				*(pool->nowblock) = (int *) newblock;
				/* The next block pointer is NULL. */
				*newblock = (int *) NULL;
			}
			/* Move to the new block. */
			pool->nowblock = (int **) *(pool->nowblock);
			/* Find the first item in the block.    */
			/*   Increment by the size of (int *). */
			alignptr = (UInt64) (pool->nowblock + 1);
			/* Align the item on an `alignbytes'-byte boundary. */
			pool->nextitem = (int *)
				(alignptr + (UInt64) pool->alignbytes
				- (alignptr % (UInt64) pool->alignbytes));
			/* There are lots of unallocated items left in this block. */
			pool->unallocateditems = pool->itemsperblock;
		}
		/* Allocate a new item. */
		newitem = pool->nextitem;
		/* Advance `nextitem' pointer to next free item in block. */
		if (pool->itemwordtype == POINTER) {
			pool->nextitem = (int *) ((int **) pool->nextitem + pool->itemwords);
		} else {
			pool->nextitem = (int *) ((double *) pool->nextitem + pool->itemwords);
		}
		pool->unallocateditems--;
		pool->maxitems++;
	}
	pool->items++;
	return newitem;
}

/*****************************************************************************/
/*                                                                           */
/*  pooldealloc()   Deallocate space for an item.                            */
/*                                                                           */
/*  The deallocated space is stored in a queue for later reuse.              */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::pooldealloc(struct memorypool *pool, int *dyingitem)
{
	/* Push freshly killed item onto stack. */
	*((int **) dyingitem) = pool->deaditemstack;
	pool->deaditemstack = dyingitem;
	pool->items--;
}

/*****************************************************************************/
/*                                                                           */
/*  traversalinit()   Prepare to traverse the entire list of items.          */
/*                                                                           */
/*  This routine is used in conjunction with traverse().                     */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::traversalinit(struct memorypool *pool)
{
	UInt64 alignptr;
	
	/* Begin the traversal in the first block. */
	pool->pathblock = pool->firstblock;
	/* Find the first item in the block.  Increment by the size of (int *). */
	alignptr = (UInt64) (pool->pathblock + 1);
	/* Align with item on an `alignbytes'-byte boundary. */
	pool->pathitem = (int *)
		(alignptr + (UInt64) pool->alignbytes
		- (alignptr % (UInt64) pool->alignbytes));
	/* Set the number of items left in the current block. */
	pool->pathitemsleft = pool->itemsperblock;
}

int * TriangleLib::traverse(struct memorypool *pool)
{
	int *newitem;
	UInt64 alignptr;
	
	/* Stop upon exhausting the list of items. */
	if (pool->pathitem == pool->nextitem) {
		return (int *) NULL;
	}
	/* Check whether any untraversed items remain in the current block. */
	if (pool->pathitemsleft == 0) {
		/* Find the next block. */
		pool->pathblock = (int **) *(pool->pathblock);
		/* Find the first item in the block.  Increment by the size of (int *). */
		alignptr = (UInt64) (pool->pathblock + 1);
		/* Align with item on an `alignbytes'-byte boundary. */
		pool->pathitem = (int *)
			(alignptr + (UInt64) pool->alignbytes
			- (alignptr % (UInt64) pool->alignbytes));
		/* Set the number of items left in the current block. */
		pool->pathitemsleft = pool->itemsperblock;
	}
	newitem = pool->pathitem;
	/* Find the next item in the block. */
	if (pool->itemwordtype == POINTER) {
		pool->pathitem = (int *) ((int **) pool->pathitem + pool->itemwords);
	} else {
		pool->pathitem = (int *) ((double *) pool->pathitem + pool->itemwords);
	}
	pool->pathitemsleft--;
	return newitem;
}

void TriangleLib::dummyinit(int trianglewords, int shellewords)
{
	UInt64 alignptr;
	
	/* `triwords' and `shwords' are used by the mesh manipulation primitives */
	/*   to extract orientations of triangles and shell edges from pointers. */
	triwords = trianglewords;       /* Initialize `triwords' once and for all. */
	shwords = shellewords;           /* Initialize `shwords' once and for all. */
	
	/* Set up `dummytri', the `triangle' that occupies "outer space". */
	dummytribase = (triangle *) malloc(triwords * sizeof(triangle)
		+ triangles.alignbytes);
	if (dummytribase == (triangle *) NULL) {
		
		return;
	}
	/* Align `dummytri' on a `triangles.alignbytes'-byte boundary. */
	alignptr = (UInt64) dummytribase;
	dummytri = (triangle *)
		(alignptr + (UInt64) triangles.alignbytes
		- (alignptr % (UInt64) triangles.alignbytes));
	
	dummytri[0] = (triangle) dummytri;
	dummytri[1] = (triangle) dummytri;
	dummytri[2] = (triangle) dummytri;
	/* Three NULL vertex points. */
	dummytri[3] = (triangle) NULL;
	dummytri[4] = (triangle) NULL;
	dummytri[5] = (triangle) NULL;
	
	if (useshelles) {
		/* Set up `dummysh', the omnipresent "shell edge" pointed to by any      */
		/*   triangle side or shell edge end that isn't attached to a real shell */
		/*   edge.                                                               */
		dummyshbase = (shelle *) malloc(shwords * sizeof(shelle)
			+ shelles.alignbytes);
		if (dummyshbase == (shelle *) NULL) {
			
			return;
		}
		/* Align `dummysh' on a `shelles.alignbytes'-byte boundary. */
		alignptr = (UInt64) dummyshbase;
		dummysh = (shelle *)
			(alignptr + (UInt64) shelles.alignbytes
			- (alignptr % (UInt64) shelles.alignbytes));
		/* Initialize the two adjoining shell edges to be the omnipresent shell */
		/*   edge.  These will eventually be changed by various bonding         */
		/*   operations, but their values don't really matter, as long as they  */
		/*   can legally be dereferenced.                                       */
		dummysh[0] = (shelle) dummysh;
		dummysh[1] = (shelle) dummysh;
		/* Two NULL vertex points. */
		dummysh[2] = (shelle) NULL;
		dummysh[3] = (shelle) NULL;
		/* Initialize the two adjoining triangles to be "outer space". */
		dummysh[4] = (shelle) dummytri;
		dummysh[5] = (shelle) dummytri;
		/* Set the boundary marker to zero. */
		* (int *) (dummysh + 6) = 0;
		
		/* Initialize the three adjoining shell edges of `dummytri' to be */
		/*   the omnipresent shell edge.                                  */
		dummytri[6] = (triangle) dummysh;
		dummytri[7] = (triangle) dummysh;
		dummytri[8] = (triangle) dummysh;
	}
}

void TriangleLib::initializepointpool()
{
	int pointsize;
	
	/* The index within each point at which the boundary marker is found.  */
	/*   Ensure the point marker is aligned to a sizeof(int)-byte address. */
	pointmarkindex = ((mesh_dim + nextras) * sizeof(double) + sizeof(int) - 1)
		/ sizeof(int);
	pointsize = (pointmarkindex + 1) * sizeof(int);
	if (poly) {
		/* The index within each point at which a triangle pointer is found.   */
		/*   Ensure the pointer is aligned to a sizeof(triangle)-byte address. */
		point2triindex = (pointsize + sizeof(triangle) - 1) / sizeof(triangle);
		pointsize = (point2triindex + 1) * sizeof(triangle);
	}
	/* Initialize the pool of points. */
	poolinit(&points, pointsize, POINTPERBLOCK,
		(sizeof(double) >= sizeof(triangle)) ? FLOATINGPOINT : POINTER, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  initializetrisegpools()   Calculate the sizes of the triangle and shell  */
/*                            edge data structures and initialize their      */
/*                            memory pools.                                  */
/*                                                                           */
/*  This routine also computes the `highorderindex', `elemattribindex', and  */
/*  `areaboundindex' indices used to find values within each triangle.       */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::initializetrisegpools()
{
	int trisize;
	
	/* The index within each triangle at which the extra nodes (above three)  */
	/*   associated with high order elements are found.  There are three      */
	/*   pointers to other triangles, three pointers to corners, and possibly */
	/*   three pointers to shell edges before the extra nodes.                */
	highorderindex = 6 + (useshelles * 3);
	/* The number of bytes occupied by a triangle. */
	trisize = ((order + 1) * (order + 2) / 2 + (highorderindex - 3)) *
		sizeof(triangle);
	/* The index within each triangle at which its attributes are found, */
	/*   where the index is measured in REALs.                           */
	elemattribindex = (trisize + sizeof(double) - 1) / sizeof(double);
	/* The index within each triangle at which the maximum area constraint  */
	/*   is found, where the index is measured in REALs.  Note that if the  */
	/*   `regionattrib' flag is set, an additional attribute will be added. */
	areaboundindex = elemattribindex + eextras + regionattrib;
	/* If triangle attributes or an area bound are needed, increase the number */
	/*   of bytes occupied by a triangle.                                      */
	if (vararea) {
		trisize = (areaboundindex + 1) * sizeof(double);
	} else if (eextras + regionattrib > 0) {
		trisize = areaboundindex * sizeof(double);
	}
	/* If a Voronoi diagram or triangle neighbor graph is requested, make    */
	/*   sure there's room to store an integer index in each triangle.  This */
	/*   integer index can occupy the same space as the shell edges or       */
	/*   attributes or area constraint or extra nodes.                       */
	if ((voronoi || neighbors) &&
		(trisize < 6 * sizeof(triangle) + sizeof(int))) {
		trisize = 6 * sizeof(triangle) + sizeof(int);
	}
	/* Having determined the memory size of a triangle, initialize the pool. */
	poolinit(&triangles, trisize, TRIPERBLOCK, POINTER, 4);
	
	if (useshelles) {
		/* Initialize the pool of shell edges. */
		poolinit(&shelles, 6 * sizeof(triangle) + sizeof(int), SHELLEPERBLOCK,
			POINTER, 4);
		
		/* Initialize the "outer space" triangle and omnipresent shell edge. */
		dummyinit(triangles.itemwords, shelles.itemwords);
	} else {
		/* Initialize the "outer space" triangle. */
		dummyinit(triangles.itemwords, 0);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  triangledealloc()   Deallocate space for a triangle, marking it dead.    */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::triangledealloc(triangle *dyingtriangle)
{
	/* Set triangle's vertices to NULL.  This makes it possible to        */
	/*   detect dead triangles when traversing the list of all triangles. */
	dyingtriangle[3] = (triangle) NULL;
	dyingtriangle[4] = (triangle) NULL;
	dyingtriangle[5] = (triangle) NULL;
	pooldealloc(&triangles, (int *) dyingtriangle);
}

/*****************************************************************************/
/*                                                                           */
/*  triangletraverse()   Traverse the triangles, skipping dead ones.         */
/*                                                                           */
/*****************************************************************************/
triangle * TriangleLib::triangletraverse()
{
	triangle *newtriangle;
	
	do {
		newtriangle = (triangle *) traverse(&triangles);
		if (newtriangle == (triangle *) NULL) {
			return (triangle *) NULL;
		}
	} while (newtriangle[3] == (triangle) NULL);            /* Skip dead ones. */
	return newtriangle;
}

/*****************************************************************************/
/*                                                                           */
/*  shelledealloc()   Deallocate space for a shell edge, marking it dead.    */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::shelledealloc(shelle *dyingshelle)
{
	/* Set shell edge's vertices to NULL.  This makes it possible to */
	/*   detect dead shells when traversing the list of all shells.  */
	dyingshelle[2] = (shelle) NULL;
	dyingshelle[3] = (shelle) NULL;
	pooldealloc(&shelles, (int *) dyingshelle);
}

/*****************************************************************************/
/*                                                                           */
/*  shelletraverse()   Traverse the shell edges, skipping dead ones.         */
/*                                                                           */
/*****************************************************************************/
shelle * TriangleLib::shelletraverse()
{
	shelle *newshelle;
	
	do {
		newshelle = (shelle *) traverse(&shelles);
		if (newshelle == (shelle *) NULL) {
			return (shelle *) NULL;
		}
	} while (newshelle[2] == (shelle) NULL);                /* Skip dead ones. */
	return newshelle;
}

/*****************************************************************************/
/*                                                                           */
/*  pointdealloc()   Deallocate space for a point, marking it dead.          */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::pointdealloc(point dyingpoint)
{
	/* Mark the point as dead.  This makes it possible to detect dead points */
	/*   when traversing the list of all points.                             */
	setpointmark(dyingpoint, DEADPOINT);
	pooldealloc(&points, (int *) dyingpoint);
}

/*****************************************************************************/
/*                                                                           */
/*  pointtraverse()   Traverse the points, skipping dead ones.               */
/*                                                                           */
/*****************************************************************************/
point TriangleLib::pointtraverse()
{
	point newpoint;
	
	do {
		newpoint = (point) traverse(&points);
		if (newpoint == (point) NULL) {
			return (point) NULL;
		}
	} while (pointmark(newpoint) == DEADPOINT);             /* Skip dead ones. */
	return newpoint;
}

/*****************************************************************************/
/*                                                                           */
/*  badsegmentdealloc()   Deallocate space for a bad segment, marking it     */
/*                        dead.                                              */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::badsegmentdealloc(struct edge *dyingseg)
{
	/* Set segment's orientation to -1.  This makes it possible to      */
	/*   detect dead segments when traversing the list of all segments. */
	dyingseg->shorient = -1;
	pooldealloc(&badsegments, (int *) dyingseg);
}

/*****************************************************************************/
/*                                                                           */
/*  badsegmenttraverse()   Traverse the bad segments, skipping dead ones.    */
/*                                                                           */
/*****************************************************************************/
struct edge * TriangleLib::badsegmenttraverse()
{
	struct edge *newseg;
	
	do {
		newseg = (struct edge *) traverse(&badsegments);
		if (newseg == (struct edge *) NULL) {
			return (struct edge *) NULL;
		}
	} while (newseg->shorient == -1);                       /* Skip dead ones. */
	return newseg;
}

/*****************************************************************************/
/*                                                                           */
/*  getpoint()   Get a specific point, by number, from the list.             */
/*                                                                           */
/*  The first point is number 'firstnumber'.                                 */
/*                                                                           */
/*  Note that this takes O(n) time (with a small constant, if POINTPERBLOCK  */
/*  is large).  I don't care to take the trouble to make it work in constant */
/*  time.                                                                    */
/*                                                                           */
/*****************************************************************************/
point TriangleLib::getpoint(int number)
{
	int **getblock;
	point foundpoint;
	UInt64 alignptr;
	int current;
	
	getblock = points.firstblock;
	current = firstnumber;
	/* Find the right block. */
	while (current + points.itemsperblock <= number) {
		getblock = (int **) *getblock;
		current += points.itemsperblock;
	}
	/* Now find the right point. */
	alignptr = (UInt64) (getblock + 1);
	foundpoint = (point) (alignptr + (UInt64) points.alignbytes
		- (alignptr % (UInt64) points.alignbytes));
	while (current < number) {
		foundpoint += points.itemwords;
		current++;
	}
	return foundpoint;
}

/*****************************************************************************/
/*                                                                           */
/*  triangledeinit()   Free all remaining allocated memory.                  */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::triangledeinit()
{
	pooldeinit(&triangles);
	free(dummytribase);
	if (useshelles) {
		pooldeinit(&shelles);
		free(dummyshbase);
	}
	pooldeinit(&points);
#ifndef CDT_ONLY
	if (quality) {
		pooldeinit(&badsegments);
		if ((minangle > 0.0) || vararea || fixedarea) {
			pooldeinit(&badtriangles);
		}
	}
#endif
}

void TriangleLib::maketriangle(struct triedge *newtriedge)
{
	int i;
	
	newtriedge->tri = (triangle *) poolalloc(&triangles);
	/* Initialize the three adjoining triangles to be "outer space". */
	newtriedge->tri[0] = (triangle) dummytri;
	newtriedge->tri[1] = (triangle) dummytri;
	newtriedge->tri[2] = (triangle) dummytri;
	/* Three NULL vertex points. */
	newtriedge->tri[3] = (triangle) NULL;
	newtriedge->tri[4] = (triangle) NULL;
	newtriedge->tri[5] = (triangle) NULL;
	/* Initialize the three adjoining shell edges to be the omnipresent */
	/*   shell edge.                                                    */
	if (useshelles) {
		newtriedge->tri[6] = (triangle) dummysh;
		newtriedge->tri[7] = (triangle) dummysh;
		newtriedge->tri[8] = (triangle) dummysh;
	}
	for (i = 0; i < eextras; i++) {
		setelemattribute(*newtriedge, i, 0.0);
	}
	if (vararea) {
		setareabound(*newtriedge, -1.0);
	}
	
	newtriedge->orient = 0;
}

/*****************************************************************************/
/*                                                                           */
/*  makeshelle()   Create a new shell edge with orientation zero.            */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::makeshelle(struct edge *newedge)
{
	newedge->sh = (shelle *) poolalloc(&shelles);
	/* Initialize the two adjoining shell edges to be the omnipresent */
	/*   shell edge.                                                  */
	newedge->sh[0] = (shelle) dummysh;
	newedge->sh[1] = (shelle) dummysh;
	/* Two NULL vertex points. */
	newedge->sh[2] = (shelle) NULL;
	newedge->sh[3] = (shelle) NULL;
	/* Initialize the two adjoining triangles to be "outer space". */
	newedge->sh[4] = (shelle) dummytri;
	newedge->sh[5] = (shelle) dummytri;
	/* Set the boundary marker to zero. */
	setmark(*newedge, 0);
	
	newedge->shorient = 0;
}

#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))

#define Fast_Two_Sum_Tail(a, b, x, y) \
	bvirt = x - a; \
y = b - bvirt

#define Fast_Two_Sum(a, b, x, y) \
	x = (double) (a + b); \
Fast_Two_Sum_Tail(a, b, x, y)

#define Two_Sum_Tail(a, b, x, y) \
	bvirt = (double) (x - a); \
	avirt = x - bvirt; \
	bround = b - bvirt; \
	around = a - avirt; \
y = around + bround

#define Two_Sum(a, b, x, y) \
	x = (double) (a + b); \
Two_Sum_Tail(a, b, x, y)

#define Two_Diff_Tail(a, b, x, y) \
	bvirt = (double) (a - x); \
	avirt = x + bvirt; \
	bround = bvirt - b; \
	around = a - avirt; \
y = around + bround

#define Two_Diff(a, b, x, y) \
	x = (double) (a - b); \
Two_Diff_Tail(a, b, x, y)

#define Split(a, ahi, alo) \
	c = (double) (splitter * a); \
	abig = (double) (c - a); \
	ahi = c - abig; \
alo = a - ahi

#define Two_Product_Tail(a, b, x, y) \
	Split(a, ahi, alo); \
	Split(b, bhi, blo); \
	err1 = x - (ahi * bhi); \
	err2 = err1 - (alo * bhi); \
	err3 = err2 - (ahi * blo); \
y = (alo * blo) - err3

#define Two_Product(a, b, x, y) \
	x = (double) (a * b); \
Two_Product_Tail(a, b, x, y)

/* Two_Product_Presplit() is Two_Product() where one of the inputs has       */
/*   already been split.  Avoids redundant splitting.                        */

#define Two_Product_Presplit(a, b, bhi, blo, x, y) \
	x = (double) (a * b); \
	Split(a, ahi, alo); \
	err1 = x - (ahi * bhi); \
	err2 = err1 - (alo * bhi); \
	err3 = err2 - (ahi * blo); \
y = (alo * blo) - err3

/* Square() can be done more quickly than Two_Product().                     */

#define Square_Tail(a, x, y) \
	Split(a, ahi, alo); \
	err1 = x - (ahi * ahi); \
	err3 = err1 - ((ahi + ahi) * alo); \
y = (alo * alo) - err3

#define Square(a, x, y) \
	x = (double) (a * a); \
Square_Tail(a, x, y)

/* Macros for summing expansions of various fixed lengths.  These are all    */
/*   unrolled versions of Expansion_Sum().                                   */

#define Two_One_Sum(a1, a0, b, x2, x1, x0) \
	Two_Sum(a0, b , _i, x0); \
Two_Sum(a1, _i, x2, x1)

#define Two_One_Diff(a1, a0, b, x2, x1, x0) \
	Two_Diff(a0, b , _i, x0); \
Two_Sum( a1, _i, x2, x1)

#define Two_Two_Sum(a1, a0, b1, b0, x3, x2, x1, x0) \
	Two_One_Sum(a1, a0, b0, _j, _0, x0); \
Two_One_Sum(_j, _0, b1, x3, x2, x1)

#define Two_Two_Diff(a1, a0, b1, b0, x3, x2, x1, x0) \
	Two_One_Diff(a1, a0, b0, _j, _0, x0); \
Two_One_Diff(_j, _0, b1, x3, x2, x1)


void TriangleLib::exactinit()
{

	double half;
	double check, lastcheck;
	int every_other;
	
	every_other = 1;
	half = 0.5;
	epsilon = 1.0;
	splitter = 1.0;
	check = 1.0;
	/* Repeatedly divide `epsilon' by two until it is too small to add to      */
	/*   one without causing roundoff.  (Also check if the sum is equal to     */
	/*   the previous sum, for machines that round up instead of using exact   */
	/*   rounding.  Not that these routines will work on such machines anyway. */
	
	do {
		lastcheck = check;
		epsilon *= half;
		if (every_other) {
			splitter *= 2.0;
		}
		every_other = !every_other;
		check = 1.0 + epsilon;
	} while ((check != 1.0) && (check != lastcheck));
	splitter += 1.0;
	
	/* Error bounds for orientation and incircle tests. */
	resulterrbound = (3.0 + 8.0 * epsilon) * epsilon;
	ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
	ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
	ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon * epsilon;
	iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
	iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
	iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon * epsilon;
}

/*****************************************************************************/
/*                                                                           */
/*  fast_expansion_sum_zeroelim()   Sum two expansions, eliminating zero     */
/*                                  components from the output expansion.    */
/*                                                                           */
/*  Sets h = e + f.  See my Robust Predicates paper for details.             */
/*                                                                           */
/*  If round-to-even is used (as with IEEE 754), maintains the strongly      */
/*  nonoverlapping property.  (That is, if e is strongly nonoverlapping, h   */
/*  will be also.)  Does NOT maintain the nonoverlapping or nonadjacent      */
/*  properties.                                                              */
/*                                                                           */
/*****************************************************************************/
int TriangleLib::fast_expansion_sum_zeroelim(int elen, double *e, int flen, double *f, double *h) // h cannot be e or f.
{
	double Q;
	INEXACT double Qnew;
	INEXACT double hh;
	INEXACT double bvirt;
	double avirt, bround, around;
	int eindex, findex, hindex;
	double enow, fnow;
	
	enow = e[0];
	fnow = f[0];
	eindex = findex = 0;
	if ((fnow > enow) == (fnow > -enow)) {
		Q = enow;
		enow = e[++eindex];
	} else {
		Q = fnow;
		fnow = f[++findex];
	}
	hindex = 0;
	if ((eindex < elen) && (findex < flen)) {
		if ((fnow > enow) == (fnow > -enow)) {
			Fast_Two_Sum(enow, Q, Qnew, hh);
			enow = e[++eindex];
		} else {
			Fast_Two_Sum(fnow, Q, Qnew, hh);
			fnow = f[++findex];
		}
		Q = Qnew;
		if (hh != 0.0) {
			h[hindex++] = hh;
		}
		while ((eindex < elen) && (findex < flen)) {
			if ((fnow > enow) == (fnow > -enow)) {
				Two_Sum(Q, enow, Qnew, hh);
				enow = e[++eindex];
			} else {
				Two_Sum(Q, fnow, Qnew, hh);
				fnow = f[++findex];
			}
			Q = Qnew;
			if (hh != 0.0) {
				h[hindex++] = hh;
			}
		}
	}
	while (eindex < elen) {
		Two_Sum(Q, enow, Qnew, hh);
		enow = e[++eindex];
		Q = Qnew;
		if (hh != 0.0) {
			h[hindex++] = hh;
		}
	}
	while (findex < flen) {
		Two_Sum(Q, fnow, Qnew, hh);
		fnow = f[++findex];
		Q = Qnew;
		if (hh != 0.0) {
			h[hindex++] = hh;
		}
	}
	if ((Q != 0.0) || (hindex == 0)) {
		h[hindex++] = Q;
	}
	return hindex;
}

/*****************************************************************************/
/*                                                                           */
/*  scale_expansion_zeroelim()   Multiply an expansion by a scalar,          */
/*                               eliminating zero components from the        */
/*                               output expansion.                           */
/*                                                                           */
/*  Sets h = be.  See my Robust Predicates paper for details.                */
/*                                                                           */
/*  Maintains the nonoverlapping property.  If round-to-even is used (as     */
/*  with IEEE 754), maintains the strongly nonoverlapping and nonadjacent    */
/*  properties as well.  (That is, if e has one of these properties, so      */
/*  will h.)                                                                 */
/*                                                                           */
/*****************************************************************************/
int TriangleLib::scale_expansion_zeroelim(int elen, double *e, double b, double *h)// e and h cannot be the same. 
{
	INEXACT double Q, sum;
	double hh;
	INEXACT double product1;
	double product0;
	int eindex, hindex;
	double enow;
	INEXACT double bvirt;
	double avirt, bround, around;
	INEXACT double c;
	INEXACT double abig;
	double ahi, alo, bhi, blo;
	double err1, err2, err3;
	
	Split(b, bhi, blo);
	Two_Product_Presplit(e[0], b, bhi, blo, Q, hh);
	hindex = 0;
	if (hh != 0) {
		h[hindex++] = hh;
	}
	for (eindex = 1; eindex < elen; eindex++) {
		enow = e[eindex];
		Two_Product_Presplit(enow, b, bhi, blo, product1, product0);
		Two_Sum(Q, product0, sum, hh);
		if (hh != 0) {
			h[hindex++] = hh;
		}
		Fast_Two_Sum(product1, sum, Q, hh);
		if (hh != 0) {
			h[hindex++] = hh;
		}
	}
	if ((Q != 0.0) || (hindex == 0)) {
		h[hindex++] = Q;
	}
	return hindex;
}

/*****************************************************************************/
/*                                                                           */
/*  estimate()   Produce a one-word estimate of an expansion's value.        */
/*                                                                           */
/*  See my Robust Predicates paper for details.                              */
/*                                                                           */
/*****************************************************************************/
double TriangleLib::estimate(int elen, double *e)
{
	double Q;
	int eindex;
	
	Q = e[0];
	for (eindex = 1; eindex < elen; eindex++) {
		Q += e[eindex];
	}
	return Q;
}

double TriangleLib::counterclockwiseadapt(point pa, point pb, point pc, double detsum)
{
	INEXACT double acx, acy, bcx, bcy;
	double acxtail, acytail, bcxtail, bcytail;
	INEXACT double detleft, detright;
	double detlefttail, detrighttail;
	double det, errbound;
	double B[4], C1[8], C2[12], D[16];
	INEXACT double B3;
	int C1length, C2length, Dlength;
	double u[4];
	INEXACT double u3;
	INEXACT double s1, t1;
	double s0, t0;
	
	INEXACT double bvirt;
	double avirt, bround, around;
	INEXACT double c;
	INEXACT double abig;
	double ahi, alo, bhi, blo;
	double err1, err2, err3;
	INEXACT double _i, _j;
	double _0;
	
	acx = (double) (pa[0] - pc[0]);
	bcx = (double) (pb[0] - pc[0]);
	acy = (double) (pa[1] - pc[1]);
	bcy = (double) (pb[1] - pc[1]);
	
	Two_Product(acx, bcy, detleft, detlefttail);
	Two_Product(acy, bcx, detright, detrighttail);
	
	Two_Two_Diff(detleft, detlefttail, detright, detrighttail,
		B3, B[2], B[1], B[0]);
	B[3] = B3;
	
	det = estimate(4, B);
	errbound = ccwerrboundB * detsum;
	if ((det >= errbound) || (-det >= errbound)) {
		return det;
	}
	
	Two_Diff_Tail(pa[0], pc[0], acx, acxtail);
	Two_Diff_Tail(pb[0], pc[0], bcx, bcxtail);
	Two_Diff_Tail(pa[1], pc[1], acy, acytail);
	Two_Diff_Tail(pb[1], pc[1], bcy, bcytail);
	
	if ((acxtail == 0.0) && (acytail == 0.0)
		&& (bcxtail == 0.0) && (bcytail == 0.0)) {
		return det;
	}
	
	errbound = ccwerrboundC * detsum + resulterrbound * Absolute(det);
	det += (acx * bcytail + bcy * acxtail)
		- (acy * bcxtail + bcx * acytail);
	if ((det >= errbound) || (-det >= errbound)) {
		return det;
	}
	
	Two_Product(acxtail, bcy, s1, s0);
	Two_Product(acytail, bcx, t1, t0);
	Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
	u[3] = u3;
	C1length = fast_expansion_sum_zeroelim(4, B, 4, u, C1);
	
	Two_Product(acx, bcytail, s1, s0);
	Two_Product(acy, bcxtail, t1, t0);
	Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
	u[3] = u3;
	C2length = fast_expansion_sum_zeroelim(C1length, C1, 4, u, C2);
	
	Two_Product(acxtail, bcytail, s1, s0);
	Two_Product(acytail, bcxtail, t1, t0);
	Two_Two_Diff(s1, s0, t1, t0, u3, u[2], u[1], u[0]);
	u[3] = u3;
	Dlength = fast_expansion_sum_zeroelim(C2length, C2, 4, u, D);
	
	return(D[Dlength - 1]);
}

double TriangleLib::counterclockwise(point pa, point pb, point pc)
{
	double detleft, detright, det;
	double detsum, errbound;
	
	counterclockcount++;
	
	detleft = (pa[0] - pc[0]) * (pb[1] - pc[1]);
	detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
	det = detleft - detright;
	
	if (noexact) {
		return det;
	}
	
	if (detleft > 0.0) {
		if (detright <= 0.0) {
			return det;
		} else {
			detsum = detleft + detright;
		}
	} else if (detleft < 0.0) {
		if (detright >= 0.0) {
			return det;
		} else {
			detsum = -detleft - detright;
		}
	} else {
		return det;
	}
	
	errbound = ccwerrboundA * detsum;
	if ((det >= errbound) || (-det >= errbound)) {
		return det;
	}
	
	return counterclockwiseadapt(pa, pb, pc, detsum);
}

double TriangleLib::incircleadapt(point pa,point pb, point pc, point pd, double permanent)
{
	INEXACT double adx, bdx, cdx, ady, bdy, cdy;
	double det, errbound;
	
	INEXACT double bdxcdy1, cdxbdy1, cdxady1, adxcdy1, adxbdy1, bdxady1;
	double bdxcdy0, cdxbdy0, cdxady0, adxcdy0, adxbdy0, bdxady0;
	double bc[4], ca[4], ab[4];
	INEXACT double bc3, ca3, ab3;
	double axbc[8], axxbc[16], aybc[8], ayybc[16], adet[32];
	int axbclen, axxbclen, aybclen, ayybclen, alen;
	double bxca[8], bxxca[16], byca[8], byyca[16], bdet[32];
	int bxcalen, bxxcalen, bycalen, byycalen, blen;
	double cxab[8], cxxab[16], cyab[8], cyyab[16], cdet[32];
	int cxablen, cxxablen, cyablen, cyyablen, clen;
	double abdet[64];
	int ablen;
	double fin1[1152], fin2[1152];
	double *finnow, *finother, *finswap;
	int finlength;
	
	double adxtail, bdxtail, cdxtail, adytail, bdytail, cdytail;
	INEXACT double adxadx1, adyady1, bdxbdx1, bdybdy1, cdxcdx1, cdycdy1;
	double adxadx0, adyady0, bdxbdx0, bdybdy0, cdxcdx0, cdycdy0;
	double aa[4], bb[4], cc[4];
	INEXACT double aa3, bb3, cc3;
	INEXACT double ti1, tj1;
	double ti0, tj0;
	double u[4], v[4];
	INEXACT double u3, v3;
	double temp8[8], temp16a[16], temp16b[16], temp16c[16];
	double temp32a[32], temp32b[32], temp48[48], temp64[64];
	int temp8len, temp16alen, temp16blen, temp16clen;
	int temp32alen, temp32blen, temp48len, temp64len;
	double axtbb[8], axtcc[8], aytbb[8], aytcc[8];
	int axtbblen, axtcclen, aytbblen, aytcclen;
	double bxtaa[8], bxtcc[8], bytaa[8], bytcc[8];
	int bxtaalen, bxtcclen, bytaalen, bytcclen;
	double cxtaa[8], cxtbb[8], cytaa[8], cytbb[8];
	int cxtaalen, cxtbblen, cytaalen, cytbblen;
	double axtbc[8], aytbc[8], bxtca[8], bytca[8], cxtab[8], cytab[8];
	int axtbclen=0, aytbclen=0, bxtcalen=0, bytcalen=0, cxtablen=0, cytablen=0;
	double axtbct[16], aytbct[16], bxtcat[16], bytcat[16], cxtabt[16], cytabt[16];
	int axtbctlen, aytbctlen, bxtcatlen, bytcatlen, cxtabtlen, cytabtlen;
	double axtbctt[8], aytbctt[8], bxtcatt[8];
	double bytcatt[8], cxtabtt[8], cytabtt[8];
	int axtbcttlen, aytbcttlen, bxtcattlen, bytcattlen, cxtabttlen, cytabttlen;
	double abt[8], bct[8], cat[8];
	int abtlen, bctlen, catlen;
	double abtt[4], bctt[4], catt[4];
	int abttlen, bcttlen, cattlen;
	INEXACT double abtt3, bctt3, catt3;
	double negate;
	
	INEXACT double bvirt;
	double avirt, bround, around;
	INEXACT double c;
	INEXACT double abig;
	double ahi, alo, bhi, blo;
	double err1, err2, err3;
	INEXACT double _i, _j;
	double _0;
	
	adx = (double) (pa[0] - pd[0]);
	bdx = (double) (pb[0] - pd[0]);
	cdx = (double) (pc[0] - pd[0]);
	ady = (double) (pa[1] - pd[1]);
	bdy = (double) (pb[1] - pd[1]);
	cdy = (double) (pc[1] - pd[1]);
	
	Two_Product(bdx, cdy, bdxcdy1, bdxcdy0);
	Two_Product(cdx, bdy, cdxbdy1, cdxbdy0);
	Two_Two_Diff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0, bc3, bc[2], bc[1], bc[0]);
	bc[3] = bc3;
	axbclen = scale_expansion_zeroelim(4, bc, adx, axbc);
	axxbclen = scale_expansion_zeroelim(axbclen, axbc, adx, axxbc);
	aybclen = scale_expansion_zeroelim(4, bc, ady, aybc);
	ayybclen = scale_expansion_zeroelim(aybclen, aybc, ady, ayybc);
	alen = fast_expansion_sum_zeroelim(axxbclen, axxbc, ayybclen, ayybc, adet);
	
	Two_Product(cdx, ady, cdxady1, cdxady0);
	Two_Product(adx, cdy, adxcdy1, adxcdy0);
	Two_Two_Diff(cdxady1, cdxady0, adxcdy1, adxcdy0, ca3, ca[2], ca[1], ca[0]);
	ca[3] = ca3;
	bxcalen = scale_expansion_zeroelim(4, ca, bdx, bxca);
	bxxcalen = scale_expansion_zeroelim(bxcalen, bxca, bdx, bxxca);
	bycalen = scale_expansion_zeroelim(4, ca, bdy, byca);
	byycalen = scale_expansion_zeroelim(bycalen, byca, bdy, byyca);
	blen = fast_expansion_sum_zeroelim(bxxcalen, bxxca, byycalen, byyca, bdet);
	
	Two_Product(adx, bdy, adxbdy1, adxbdy0);
	Two_Product(bdx, ady, bdxady1, bdxady0);
	Two_Two_Diff(adxbdy1, adxbdy0, bdxady1, bdxady0, ab3, ab[2], ab[1], ab[0]);
	ab[3] = ab3;
	cxablen = scale_expansion_zeroelim(4, ab, cdx, cxab);
	cxxablen = scale_expansion_zeroelim(cxablen, cxab, cdx, cxxab);
	cyablen = scale_expansion_zeroelim(4, ab, cdy, cyab);
	cyyablen = scale_expansion_zeroelim(cyablen, cyab, cdy, cyyab);
	clen = fast_expansion_sum_zeroelim(cxxablen, cxxab, cyyablen, cyyab, cdet);
	
	ablen = fast_expansion_sum_zeroelim(alen, adet, blen, bdet, abdet);
	finlength = fast_expansion_sum_zeroelim(ablen, abdet, clen, cdet, fin1);
	
	det = estimate(finlength, fin1);
	errbound = iccerrboundB * permanent;
	if ((det >= errbound) || (-det >= errbound)) {
		return det;
	}
	
	Two_Diff_Tail(pa[0], pd[0], adx, adxtail);
	Two_Diff_Tail(pa[1], pd[1], ady, adytail);
	Two_Diff_Tail(pb[0], pd[0], bdx, bdxtail);
	Two_Diff_Tail(pb[1], pd[1], bdy, bdytail);
	Two_Diff_Tail(pc[0], pd[0], cdx, cdxtail);
	Two_Diff_Tail(pc[1], pd[1], cdy, cdytail);
	if ((adxtail == 0.0) && (bdxtail == 0.0) && (cdxtail == 0.0)
		&& (adytail == 0.0) && (bdytail == 0.0) && (cdytail == 0.0)) {
		return det;
	}
	
	errbound = iccerrboundC * permanent + resulterrbound * Absolute(det);
	det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail)
		- (bdy * cdxtail + cdx * bdytail))
		+ 2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
		+ ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail)
		- (cdy * adxtail + adx * cdytail))
		+ 2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
		+ ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail)
		- (ady * bdxtail + bdx * adytail))
		+ 2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
	if ((det >= errbound) || (-det >= errbound)) {
		return det;
	}
	
	finnow = fin1;
	finother = fin2;
	
	if ((bdxtail != 0.0) || (bdytail != 0.0)
		|| (cdxtail != 0.0) || (cdytail != 0.0)) {
		Square(adx, adxadx1, adxadx0);
		Square(ady, adyady1, adyady0);
		Two_Two_Sum(adxadx1, adxadx0, adyady1, adyady0, aa3, aa[2], aa[1], aa[0]);
		aa[3] = aa3;
	}
	if ((cdxtail != 0.0) || (cdytail != 0.0)
		|| (adxtail != 0.0) || (adytail != 0.0)) {
		Square(bdx, bdxbdx1, bdxbdx0);
		Square(bdy, bdybdy1, bdybdy0);
		Two_Two_Sum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0, bb3, bb[2], bb[1], bb[0]);
		bb[3] = bb3;
	}
	if ((adxtail != 0.0) || (adytail != 0.0)
		|| (bdxtail != 0.0) || (bdytail != 0.0)) {
		Square(cdx, cdxcdx1, cdxcdx0);
		Square(cdy, cdycdy1, cdycdy0);
		Two_Two_Sum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0, cc3, cc[2], cc[1], cc[0]);
		cc[3] = cc3;
	}
	
	if (adxtail != 0.0) {
		axtbclen = scale_expansion_zeroelim(4, bc, adxtail, axtbc);
		temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, 2.0 * adx,
			temp16a);
		
		axtcclen = scale_expansion_zeroelim(4, cc, adxtail, axtcc);
		temp16blen = scale_expansion_zeroelim(axtcclen, axtcc, bdy, temp16b);
		
		axtbblen = scale_expansion_zeroelim(4, bb, adxtail, axtbb);
		temp16clen = scale_expansion_zeroelim(axtbblen, axtbb, -cdy, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	if (adytail != 0.0) {
		aytbclen = scale_expansion_zeroelim(4, bc, adytail, aytbc);
		temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, 2.0 * ady,
			temp16a);
		
		aytbblen = scale_expansion_zeroelim(4, bb, adytail, aytbb);
		temp16blen = scale_expansion_zeroelim(aytbblen, aytbb, cdx, temp16b);
		
		aytcclen = scale_expansion_zeroelim(4, cc, adytail, aytcc);
		temp16clen = scale_expansion_zeroelim(aytcclen, aytcc, -bdx, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	if (bdxtail != 0.0) {
		bxtcalen = scale_expansion_zeroelim(4, ca, bdxtail, bxtca);
		temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, 2.0 * bdx,
			temp16a);
		
		bxtaalen = scale_expansion_zeroelim(4, aa, bdxtail, bxtaa);
		temp16blen = scale_expansion_zeroelim(bxtaalen, bxtaa, cdy, temp16b);
		
		bxtcclen = scale_expansion_zeroelim(4, cc, bdxtail, bxtcc);
		temp16clen = scale_expansion_zeroelim(bxtcclen, bxtcc, -ady, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	if (bdytail != 0.0) {
		bytcalen = scale_expansion_zeroelim(4, ca, bdytail, bytca);
		temp16alen = scale_expansion_zeroelim(bytcalen, bytca, 2.0 * bdy,
			temp16a);
		
		bytcclen = scale_expansion_zeroelim(4, cc, bdytail, bytcc);
		temp16blen = scale_expansion_zeroelim(bytcclen, bytcc, adx, temp16b);
		
		bytaalen = scale_expansion_zeroelim(4, aa, bdytail, bytaa);
		temp16clen = scale_expansion_zeroelim(bytaalen, bytaa, -cdx, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	if (cdxtail != 0.0) {
		cxtablen = scale_expansion_zeroelim(4, ab, cdxtail, cxtab);
		temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, 2.0 * cdx,
			temp16a);
		
		cxtbblen = scale_expansion_zeroelim(4, bb, cdxtail, cxtbb);
		temp16blen = scale_expansion_zeroelim(cxtbblen, cxtbb, ady, temp16b);
		
		cxtaalen = scale_expansion_zeroelim(4, aa, cdxtail, cxtaa);
		temp16clen = scale_expansion_zeroelim(cxtaalen, cxtaa, -bdy, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	if (cdytail != 0.0) {
		cytablen = scale_expansion_zeroelim(4, ab, cdytail, cytab);
		temp16alen = scale_expansion_zeroelim(cytablen, cytab, 2.0 * cdy,
			temp16a);
		
		cytaalen = scale_expansion_zeroelim(4, aa, cdytail, cytaa);
		temp16blen = scale_expansion_zeroelim(cytaalen, cytaa, bdx, temp16b);
		
		cytbblen = scale_expansion_zeroelim(4, bb, cdytail, cytbb);
		temp16clen = scale_expansion_zeroelim(cytbblen, cytbb, -adx, temp16c);
		
		temp32alen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
			temp16blen, temp16b, temp32a);
		temp48len = fast_expansion_sum_zeroelim(temp16clen, temp16c,
			temp32alen, temp32a, temp48);
		finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
			temp48, finother);
		finswap = finnow; finnow = finother; finother = finswap;
	}
	
	if ((adxtail != 0.0) || (adytail != 0.0)) {
		if ((bdxtail != 0.0) || (bdytail != 0.0)
			|| (cdxtail != 0.0) || (cdytail != 0.0)) {
			Two_Product(bdxtail, cdy, ti1, ti0);
			Two_Product(bdx, cdytail, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
			u[3] = u3;
			negate = -bdy;
			Two_Product(cdxtail, negate, ti1, ti0);
			negate = -bdytail;
			Two_Product(cdx, negate, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
			v[3] = v3;
			bctlen = fast_expansion_sum_zeroelim(4, u, 4, v, bct);
			
			Two_Product(bdxtail, cdytail, ti1, ti0);
			Two_Product(cdxtail, bdytail, tj1, tj0);
			Two_Two_Diff(ti1, ti0, tj1, tj0, bctt3, bctt[2], bctt[1], bctt[0]);
			bctt[3] = bctt3;
			bcttlen = 4;
		} else {
			bct[0] = 0.0;
			bctlen = 1;
			bctt[0] = 0.0;
			bcttlen = 1;
		}
		
		if (adxtail != 0.0) {
			temp16alen = scale_expansion_zeroelim(axtbclen, axtbc, adxtail, temp16a);
			axtbctlen = scale_expansion_zeroelim(bctlen, bct, adxtail, axtbct);
			temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, 2.0 * adx,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			if (bdytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, cc, adxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			if (cdytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, bb, -adxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			
			temp32alen = scale_expansion_zeroelim(axtbctlen, axtbct, adxtail,
				temp32a);
			axtbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adxtail, axtbctt);
			temp16alen = scale_expansion_zeroelim(axtbcttlen, axtbctt, 2.0 * adx,
				temp16a);
			temp16blen = scale_expansion_zeroelim(axtbcttlen, axtbctt, adxtail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
		if (adytail != 0.0) {
			temp16alen = scale_expansion_zeroelim(aytbclen, aytbc, adytail, temp16a);
			aytbctlen = scale_expansion_zeroelim(bctlen, bct, adytail, aytbct);
			temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, 2.0 * ady,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			
			
			temp32alen = scale_expansion_zeroelim(aytbctlen, aytbct, adytail,
				temp32a);
			aytbcttlen = scale_expansion_zeroelim(bcttlen, bctt, adytail, aytbctt);
			temp16alen = scale_expansion_zeroelim(aytbcttlen, aytbctt, 2.0 * ady,
				temp16a);
			temp16blen = scale_expansion_zeroelim(aytbcttlen, aytbctt, adytail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
	}
	if ((bdxtail != 0.0) || (bdytail != 0.0)) {
		if ((cdxtail != 0.0) || (cdytail != 0.0)
			|| (adxtail != 0.0) || (adytail != 0.0)) {
			Two_Product(cdxtail, ady, ti1, ti0);
			Two_Product(cdx, adytail, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
			u[3] = u3;
			negate = -cdy;
			Two_Product(adxtail, negate, ti1, ti0);
			negate = -cdytail;
			Two_Product(adx, negate, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
			v[3] = v3;
			catlen = fast_expansion_sum_zeroelim(4, u, 4, v, cat);
			
			Two_Product(cdxtail, adytail, ti1, ti0);
			Two_Product(adxtail, cdytail, tj1, tj0);
			Two_Two_Diff(ti1, ti0, tj1, tj0, catt3, catt[2], catt[1], catt[0]);
			catt[3] = catt3;
			cattlen = 4;
		} else {
			cat[0] = 0.0;
			catlen = 1;
			catt[0] = 0.0;
			cattlen = 1;
		}
		
		if (bdxtail != 0.0) {
			temp16alen = scale_expansion_zeroelim(bxtcalen, bxtca, bdxtail, temp16a);
			bxtcatlen = scale_expansion_zeroelim(catlen, cat, bdxtail, bxtcat);
			temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, 2.0 * bdx,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			if (cdytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, aa, bdxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, cdytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			if (adytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, cc, -bdxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			
			temp32alen = scale_expansion_zeroelim(bxtcatlen, bxtcat, bdxtail,
				temp32a);
			bxtcattlen = scale_expansion_zeroelim(cattlen, catt, bdxtail, bxtcatt);
			temp16alen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, 2.0 * bdx,
				temp16a);
			temp16blen = scale_expansion_zeroelim(bxtcattlen, bxtcatt, bdxtail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
		if (bdytail != 0.0) {
			temp16alen = scale_expansion_zeroelim(bytcalen, bytca, bdytail, temp16a);
			bytcatlen = scale_expansion_zeroelim(catlen, cat, bdytail, bytcat);
			temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, 2.0 * bdy,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			
			
			temp32alen = scale_expansion_zeroelim(bytcatlen, bytcat, bdytail,
				temp32a);
			bytcattlen = scale_expansion_zeroelim(cattlen, catt, bdytail, bytcatt);
			temp16alen = scale_expansion_zeroelim(bytcattlen, bytcatt, 2.0 * bdy,
				temp16a);
			temp16blen = scale_expansion_zeroelim(bytcattlen, bytcatt, bdytail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
	}
	if ((cdxtail != 0.0) || (cdytail != 0.0)) {
		if ((adxtail != 0.0) || (adytail != 0.0)
			|| (bdxtail != 0.0) || (bdytail != 0.0)) {
			Two_Product(adxtail, bdy, ti1, ti0);
			Two_Product(adx, bdytail, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, u3, u[2], u[1], u[0]);
			u[3] = u3;
			negate = -ady;
			Two_Product(bdxtail, negate, ti1, ti0);
			negate = -adytail;
			Two_Product(bdx, negate, tj1, tj0);
			Two_Two_Sum(ti1, ti0, tj1, tj0, v3, v[2], v[1], v[0]);
			v[3] = v3;
			abtlen = fast_expansion_sum_zeroelim(4, u, 4, v, abt);
			
			Two_Product(adxtail, bdytail, ti1, ti0);
			Two_Product(bdxtail, adytail, tj1, tj0);
			Two_Two_Diff(ti1, ti0, tj1, tj0, abtt3, abtt[2], abtt[1], abtt[0]);
			abtt[3] = abtt3;
			abttlen = 4;
		} else {
			abt[0] = 0.0;
			abtlen = 1;
			abtt[0] = 0.0;
			abttlen = 1;
		}
		
		if (cdxtail != 0.0) {
			temp16alen = scale_expansion_zeroelim(cxtablen, cxtab, cdxtail, temp16a);
			cxtabtlen = scale_expansion_zeroelim(abtlen, abt, cdxtail, cxtabt);
			temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, 2.0 * cdx,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			if (adytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, bb, cdxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, adytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			if (bdytail != 0.0) {
				temp8len = scale_expansion_zeroelim(4, aa, -cdxtail, temp8);
				temp16alen = scale_expansion_zeroelim(temp8len, temp8, bdytail,
					temp16a);
				finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp16alen,
					temp16a, finother);
				finswap = finnow; finnow = finother; finother = finswap;
			}
			
			temp32alen = scale_expansion_zeroelim(cxtabtlen, cxtabt, cdxtail,
				temp32a);
			cxtabttlen = scale_expansion_zeroelim(abttlen, abtt, cdxtail, cxtabtt);
			temp16alen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, 2.0 * cdx,
				temp16a);
			temp16blen = scale_expansion_zeroelim(cxtabttlen, cxtabtt, cdxtail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
		if (cdytail != 0.0) {
			temp16alen = scale_expansion_zeroelim(cytablen, cytab, cdytail, temp16a);
			cytabtlen = scale_expansion_zeroelim(abtlen, abt, cdytail, cytabt);
			temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, 2.0 * cdy,
				temp32a);
			temp48len = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp32alen, temp32a, temp48);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp48len,
				temp48, finother);
			finswap = finnow; finnow = finother; finother = finswap;
			
			
			temp32alen = scale_expansion_zeroelim(cytabtlen, cytabt, cdytail,
				temp32a);
			cytabttlen = scale_expansion_zeroelim(abttlen, abtt, cdytail, cytabtt);
			temp16alen = scale_expansion_zeroelim(cytabttlen, cytabtt, 2.0 * cdy,
				temp16a);
			temp16blen = scale_expansion_zeroelim(cytabttlen, cytabtt, cdytail,
				temp16b);
			temp32blen = fast_expansion_sum_zeroelim(temp16alen, temp16a,
				temp16blen, temp16b, temp32b);
			temp64len = fast_expansion_sum_zeroelim(temp32alen, temp32a,
				temp32blen, temp32b, temp64);
			finlength = fast_expansion_sum_zeroelim(finlength, finnow, temp64len,
				temp64, finother);
			finswap = finnow; finnow = finother; finother = finswap;
		}
	}
	
	return finnow[finlength - 1];
}

double TriangleLib::incircle(point pa, point pb, point pc, point pd)
{
	double adx, bdx, cdx, ady, bdy, cdy;
	double bdxcdy, cdxbdy, cdxady, adxcdy, adxbdy, bdxady;
	double alift, blift, clift;
	double det;
	double permanent, errbound;
	
	incirclecount++;
	
	adx = pa[0] - pd[0];
	bdx = pb[0] - pd[0];
	cdx = pc[0] - pd[0];
	ady = pa[1] - pd[1];
	bdy = pb[1] - pd[1];
	cdy = pc[1] - pd[1];
	
	bdxcdy = bdx * cdy;
	cdxbdy = cdx * bdy;
	alift = adx * adx + ady * ady;
	
	cdxady = cdx * ady;
	adxcdy = adx * cdy;
	blift = bdx * bdx + bdy * bdy;
	
	adxbdy = adx * bdy;
	bdxady = bdx * ady;
	clift = cdx * cdx + cdy * cdy;
	
	det = alift * (bdxcdy - cdxbdy)
		+ blift * (cdxady - adxcdy)
		+ clift * (adxbdy - bdxady);
	
	if (noexact) {
		return det;
	}
	
	permanent = (Absolute(bdxcdy) + Absolute(cdxbdy)) * alift
		+ (Absolute(cdxady) + Absolute(adxcdy)) * blift
		+ (Absolute(adxbdy) + Absolute(bdxady)) * clift;
	errbound = iccerrboundA * permanent;
	if ((det > errbound) || (-det > errbound)) {
		return det;
	}
	
	return incircleadapt(pa, pb, pc, pd, permanent);
}

void TriangleLib::triangleinit()
{
	points.maxitems = triangles.maxitems = shelles.maxitems = viri.maxitems =
		badsegments.maxitems = badtriangles.maxitems = splaynodes.maxitems = 0l;
	points.itembytes = triangles.itembytes = shelles.itembytes = viri.itembytes =
		badsegments.itembytes = badtriangles.itembytes = splaynodes.itembytes = 0;
	recenttri.tri = (triangle *) NULL;    /* No triangle has been visited yet. */
	samples = 1;            /* Point location should take at least one sample. */
	checksegments = 0;      /* There are no segments in the triangulation yet. */
	incirclecount = counterclockcount = hyperbolacount = 0;
	circumcentercount = circletopcount = 0;
	randomseed = 1;

	exactinit();                     /* Initialize exact arithmetic constants. */
}

/*****************************************************************************/
/*                                                                           */
/*  randomnation()   Generate a random number between 0 and `choices' - 1.   */
/*                                                                           */
/*  This is a simple linear congruential random number generator.  Hence, it */
/*  is a bad random number generator, but good enough for most randomized    */
/*  geometric algorithms.                                                    */
/*                                                                           */
/*****************************************************************************/
UInt64 TriangleLib::randomnation(unsigned int choices)
{
	randomseed = (randomseed * 1366l + 150889l) % 714025l;
	return randomseed / (714025l / choices + 1);
}

/********* Mesh quality testing routines begin here                  *********/

/*****************************************************************************/
/*                                                                           */
/*  checkmesh()   Test the mesh for topological consistency.                 */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::checkmesh()
{
	struct triedge triangleloop;
	struct triedge oppotri, oppooppotri;
	point triorg, tridest, triapex;
	point oppoorg, oppodest;
	int horrors;
	int saveexact;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	/* Temporarily turn on exact arithmetic if it's off. */
	saveexact = noexact;
	noexact = 0;
	
	horrors = 0;
	/* Run through the list of triangles, checking each one. */
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	while (triangleloop.tri != (triangle *) NULL) {
		/* Check all three edges of the triangle. */
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			org(triangleloop, triorg);
			dest(triangleloop, tridest);
			if (triangleloop.orient == 0) {       /* Only test for inversion once. */
				/* Test if the triangle is flat or inverted. */
				apex(triangleloop, triapex);
				if (counterclockwise(triorg, tridest, triapex) <= 0.0) 
				{
					printtriangle(&triangleloop);
					horrors++;
				}
			}
			/* Find the neighboring triangle on this edge. */
			sym(triangleloop, oppotri);
			if (oppotri.tri != dummytri) {
				/* Check that the triangle's neighbor knows it's a neighbor. */
				sym(oppotri, oppooppotri);
				if ((triangleloop.tri != oppooppotri.tri)
					|| (triangleloop.orient != oppooppotri.orient)) {
					
					if (triangleloop.tri == oppooppotri.tri) {
						
					}
					
					printtriangle(&triangleloop);
					
					printtriangle(&oppotri);
					horrors++;
				}
				/* Check that both triangles agree on the identities */
				/*   of their shared vertices.                       */
				org(oppotri, oppoorg);
				dest(oppotri, oppodest);
				if ((triorg != oppodest) || (tridest != oppoorg)) {
					
					printtriangle(&triangleloop);
					
					printtriangle(&oppotri);
					horrors++;
				}
			}
		}
		triangleloop.tri = triangletraverse();
	}
	if (horrors == 0) {
		if (!quiet) {
			
		}
	} else if (horrors == 1) {
		
	} else {
		
	}
	/* Restore the status of exact arithmetic. */
	noexact = saveexact;
}

/*****************************************************************************/
/*                                                                           */
/*  checkdelaunay()   Ensure that the mesh is (constrained) Delaunay.        */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::checkdelaunay()
{
	struct triedge triangleloop;
	struct triedge oppotri;
	struct edge opposhelle;
	point triorg, tridest, triapex;
	point oppoapex;
	int shouldbedelaunay;
	int horrors;
	int saveexact;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	/* Temporarily turn on exact arithmetic if it's off. */
	saveexact = noexact;
	noexact = 0;
	if (!quiet) {
		
	}
	horrors = 0;
	/* Run through the list of triangles, checking each one. */
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	while (triangleloop.tri != (triangle *) NULL) {
		/* Check all three edges of the triangle. */
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			org(triangleloop, triorg);
			dest(triangleloop, tridest);
			apex(triangleloop, triapex);
			sym(triangleloop, oppotri);
			apex(oppotri, oppoapex);
			/* Only test that the edge is locally Delaunay if there is an   */
			/*   adjoining triangle whose pointer is larger (to ensure that */
			/*   each pair isn't tested twice).                             */
			shouldbedelaunay = (oppotri.tri != dummytri)
				&& (triapex != (point) NULL) && (oppoapex != (point) NULL)
				&& (triangleloop.tri < oppotri.tri);
			if (checksegments && shouldbedelaunay) {
				/* If a shell edge separates the triangles, then the edge is */
				/*   constrained, so no local Delaunay test should be done.  */
				tspivot(triangleloop, opposhelle);
				if (opposhelle.sh != dummysh){
					shouldbedelaunay = 0;
				}
			}
			if (shouldbedelaunay) {
				if (incircle(triorg, tridest, triapex, oppoapex) > 0.0) {
					
					printtriangle(&triangleloop);
					
					printtriangle(&oppotri);
					horrors++;
				}
			}
		}
		triangleloop.tri = triangletraverse();
	}
	if (horrors == 0) {
		if (!quiet) {
			
		}
	} else if (horrors == 1) {
		
	} else {
		
	}
	/* Restore the status of exact arithmetic. */
	noexact = saveexact;
}

void TriangleLib::enqueuebadtri(struct triedge *instri, double angle, point insapex, point insorg, point insdest)
{
	struct badface *newface;
	int queuenumber;
	
	if (verbose > 2) {
		
	}
	/* Allocate space for the bad triangle. */
	newface = (struct badface *) poolalloc(&badtriangles);
	triedgecopy(*instri, newface->badfacetri);
	newface->key = angle;
	newface->faceapex = insapex;
	newface->faceorg = insorg;
	newface->facedest = insdest;
	newface->nextface = (struct badface *) NULL;
	/* Determine the appropriate queue to put the bad triangle into. */
	if (angle > 0.6) {
		queuenumber = (int) (160.0 * (angle - 0.6));
		if (queuenumber > 63) {
			queuenumber = 63;
		}
	} else {
		/* It's not a bad angle; put the triangle in the lowest-priority queue. */
		queuenumber = 0;
	}
	/* Add the triangle to the end of a queue. */
	*queuetail[queuenumber] = newface;
	/* Maintain a pointer to the NULL pointer at the end of the queue. */
	queuetail[queuenumber] = &newface->nextface;
}

struct badface * TriangleLib::dequeuebadtri()
{
	struct badface *result;
	int queuenumber;
	
	/* Look for a nonempty queue. */
	for (queuenumber = 63; queuenumber >= 0; queuenumber--) {
		result = queuefront[queuenumber];
		if (result != (struct badface *) NULL) {
			/* Remove the triangle from the queue. */
			queuefront[queuenumber] = result->nextface;
			/* Maintain a pointer to the NULL pointer at the end of the queue. */
			if (queuefront[queuenumber] == (struct badface *) NULL) {
				queuetail[queuenumber] = &queuefront[queuenumber];
			}
			return result;
		}
	}
	return (struct badface *) NULL;
}

int TriangleLib::checkedge4encroach(struct edge *testedge)
{
	struct triedge neighbortri;
	struct edge testsym;
	struct edge *badedge;
	int addtolist;
	int sides;
	point eorg, edest, eapex;
	triangle ptr;                     /* Temporary variable used by stpivot(). */
	
	addtolist = 0;
	sides = 0;
	
	sorg(*testedge, eorg);
	sdest(*testedge, edest);
	/* Check one neighbor of the shell edge. */
	stpivot(*testedge, neighbortri);
	/* Does the neighbor exist, or is this a boundary edge? */
	if (neighbortri.tri != dummytri) {
		sides++;
		/* Find a vertex opposite this edge. */
		apex(neighbortri, eapex);
		/* Check whether the vertex is inside the diametral circle of the  */
		/*   shell edge.  Pythagoras' Theorem is used to check whether the */
		/*   angle at the vertex is greater than 90 degrees.               */
		if (eapex[0] * (eorg[0] + edest[0]) + eapex[1] * (eorg[1] + edest[1]) >
			eapex[0] * eapex[0] + eorg[0] * edest[0] +
			eapex[1] * eapex[1] + eorg[1] * edest[1]) {
			addtolist = 1;
		}
	}
	/* Check the other neighbor of the shell edge. */
	ssym(*testedge, testsym);
	stpivot(testsym, neighbortri);
	/* Does the neighbor exist, or is this a boundary edge? */
	if (neighbortri.tri != dummytri) {
		sides++;
		/* Find the other vertex opposite this edge. */
		apex(neighbortri, eapex);
		/* Check whether the vertex is inside the diametral circle of the  */
		/*   shell edge.  Pythagoras' Theorem is used to check whether the */
		/*   angle at the vertex is greater than 90 degrees.               */
		if (eapex[0] * (eorg[0] + edest[0]) +
			eapex[1] * (eorg[1] + edest[1]) >
			eapex[0] * eapex[0] + eorg[0] * edest[0] +
			eapex[1] * eapex[1] + eorg[1] * edest[1]) {
			addtolist += 2;
		}
	}
	
	if (addtolist && (!nobisect || ((nobisect == 1) && (sides == 2)))) {
		if (verbose > 2)
		{
			
		}
		/* Add the shell edge to the list of encroached segments. */
		/*   Be sure to get the orientation right.                */
		badedge = (struct edge *) poolalloc(&badsegments);
		if (addtolist == 1) {
			shellecopy(*testedge, *badedge);
		} else {
			shellecopy(testsym, *badedge);
		}
	}
	return addtolist;
}

void TriangleLib::testtriangle(struct triedge *testtri)
{
	struct triedge sametesttri;
	struct edge edge1, edge2;
	point torg, tdest, tapex;
	point anglevertex;
	double dxod, dyod, dxda, dyda, dxao, dyao;
	double dxod2, dyod2, dxda2, dyda2, dxao2, dyao2;
	double apexlen, orglen, destlen;
	double angle;
	double area;
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	org(*testtri, torg);
	dest(*testtri, tdest);
	apex(*testtri, tapex);
	dxod = torg[0] - tdest[0];
	dyod = torg[1] - tdest[1];
	dxda = tdest[0] - tapex[0];
	dyda = tdest[1] - tapex[1];
	dxao = tapex[0] - torg[0];
	dyao = tapex[1] - torg[1];
	dxod2 = dxod * dxod;
	dyod2 = dyod * dyod;
	dxda2 = dxda * dxda;
	dyda2 = dyda * dyda;
	dxao2 = dxao * dxao;
	dyao2 = dyao * dyao;
	/* Find the lengths of the triangle's three edges. */
	apexlen = dxod2 + dyod2;
	orglen = dxda2 + dyda2;
	destlen = dxao2 + dyao2;
	if ((apexlen < orglen) && (apexlen < destlen)) {
		/* The edge opposite the apex is shortest. */
		/* Find the square of the cosine of the angle at the apex. */
		angle = dxda * dxao + dyda * dyao;
		angle = angle * angle / (orglen * destlen);
		anglevertex = tapex;
		lnext(*testtri, sametesttri);
		tspivot(sametesttri, edge1);
		lnextself(sametesttri);
		tspivot(sametesttri, edge2);
	} else if (orglen < destlen) {
		/* The edge opposite the origin is shortest. */
		/* Find the square of the cosine of the angle at the origin. */
		angle = dxod * dxao + dyod * dyao;
		angle = angle * angle / (apexlen * destlen);
		anglevertex = torg;
		tspivot(*testtri, edge1);
		lprev(*testtri, sametesttri);
		tspivot(sametesttri, edge2);
	} else {
		/* The edge opposite the destination is shortest. */
		/* Find the square of the cosine of the angle at the destination. */
		angle = dxod * dxda + dyod * dyda;
		angle = angle * angle / (apexlen * orglen);
		anglevertex = tdest;
		tspivot(*testtri, edge1);
		lnext(*testtri, sametesttri);
		tspivot(sametesttri, edge2);
	}
	/* Check if both edges that form the angle are segments. */
	if ((edge1.sh != dummysh) && (edge2.sh != dummysh)) {
		/* The angle is a segment intersection. */
		if ((angle > 0.9924) && !quiet) {                  /* Roughly 5 degrees. */
			if (angle > 1.0) {
				/* Beware of a floating exception in acos(). */
				angle = 1.0;
			}
			/* Find the actual angle in degrees, for printing. */
			angle = acos(sqrt(angle)) * (180.0 / PI);
			
		}
		/* Don't add this bad triangle to the list; there's nothing that */
		/*   can be done about a small angle between two segments.       */
		angle = 0.0;
	}
	/* Check whether the angle is smaller than permitted. */
	if (angle > goodangle) {
		/* Add this triangle to the list of bad triangles. */
		enqueuebadtri(testtri, angle, tapex, torg, tdest);
		return;
	}
	if (vararea || fixedarea) {
		/* Check whether the area is larger than permitted. */
		area = 0.5 * (dxod * dyda - dyod * dxda);
		if (fixedarea && (area > maxarea)) {
			/* Add this triangle to the list of bad triangles. */
			enqueuebadtri(testtri, angle, tapex, torg, tdest);
		} else if (vararea) {
			/* Nonpositive area constraints are treated as unconstrained. */
			if ((area > areabound(*testtri)) && (areabound(*testtri) > 0.0)) {
				/* Add this triangle to the list of bad triangles. */
				enqueuebadtri(testtri, angle, tapex, torg, tdest);
			}
		}
	}
}

void TriangleLib::makepointmap()
{
	struct triedge triangleloop;
	point triorg;
	
	if (verbose) {
		
	}
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	while (triangleloop.tri != (triangle *) NULL) {
		/* Check all three points of the triangle. */
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			org(triangleloop, triorg);
			setpoint2tri(triorg, encode(triangleloop));
		}
		triangleloop.tri = triangletraverse();
	}
}

enum locateresult TriangleLib::preciselocate(point searchpoint, struct triedge *searchtri)
{
	struct triedge backtracktri;
	point forg, fdest, fapex;
	point swappoint;
	double orgorient, destorient;
	int moveleft;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (verbose > 2) 
	{
		
	}
	/* Where are we? */
	org(*searchtri, forg);
	dest(*searchtri, fdest);
	apex(*searchtri, fapex);
	while (1) {
		if (verbose > 2) 
		{
			
		}
		/* Check whether the apex is the point we seek. */
		if ((fapex[0] == searchpoint[0]) && (fapex[1] == searchpoint[1])) {
			lprevself(*searchtri);
			return ONVERTEX;
		}
		/* Does the point lie on the other side of the line defined by the */
		/*   triangle edge opposite the triangle's destination?            */
		destorient = counterclockwise(forg, fapex, searchpoint);
		/* Does the point lie on the other side of the line defined by the */
		/*   triangle edge opposite the triangle's origin?                 */
		orgorient = counterclockwise(fapex, fdest, searchpoint);
		if (destorient > 0.0) {
			if (orgorient > 0.0) {
				/* Move left if the inner product of (fapex - searchpoint) and  */
				/*   (fdest - forg) is positive.  This is equivalent to drawing */
				/*   a line perpendicular to the line (forg, fdest) passing     */
				/*   through `fapex', and determining which side of this line   */
				/*   `searchpoint' falls on.                                    */
				moveleft = (fapex[0] - searchpoint[0]) * (fdest[0] - forg[0]) +
					(fapex[1] - searchpoint[1]) * (fdest[1] - forg[1]) > 0.0;
			} else {
				moveleft = 1;
			}
		} else {
			if (orgorient > 0.0) {
				moveleft = 0;
			} else {
				/* The point we seek must be on the boundary of or inside this */
				/*   triangle.                                                 */
				if (destorient == 0.0) {
					lprevself(*searchtri);
					return ONEDGE;
				}
				if (orgorient == 0.0) {
					lnextself(*searchtri);
					return ONEDGE;
				}
				return INTRIANGLE;
			}
		}
		
		/* Move to another triangle.  Leave a trace `backtracktri' in case */
		/*   floating-point roundoff or some such bogey causes us to walk  */
		/*   off a boundary of the triangulation.  We can just bounce off  */
		/*   the boundary as if it were an elastic band.                   */
		if (moveleft) {
			lprev(*searchtri, backtracktri);
			fdest = fapex;
		} else {
			lnext(*searchtri, backtracktri);
			forg = fapex;
		}
		sym(backtracktri, *searchtri);
		
		/* Check for walking off the edge. */
		if (searchtri->tri == dummytri) {
			/* Turn around. */
			triedgecopy(backtracktri, *searchtri);
			swappoint = forg;
			forg = fdest;
			fdest = swappoint;
			apex(*searchtri, fapex);
			/* Check if the point really is beyond the triangulation boundary. */
			destorient = counterclockwise(forg, fapex, searchpoint);
			orgorient = counterclockwise(fapex, fdest, searchpoint);
			if ((orgorient < 0.0) && (destorient < 0.0)) {
				return OUTSIDE;
			}
		} else {
			apex(*searchtri, fapex);
		}
	}
}

enum locateresult TriangleLib::locate(point searchpoint, struct triedge *searchtri)
{
	int **sampleblock;
	triangle *firsttri;
	struct triedge sampletri;
	point torg, tdest;
	UInt64 alignptr;
	double searchdist, dist;
	double ahead;
	long sampleblocks, samplesperblock, samplenum;
	long triblocks;
	long i, j;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (verbose > 2) 
	{
		
	}
	/* Record the distance from the suggested starting triangle to the */
	/*   point we seek.                                                */
	org(*searchtri, torg);
	searchdist = (searchpoint[0] - torg[0]) * (searchpoint[0] - torg[0])
		+ (searchpoint[1] - torg[1]) * (searchpoint[1] - torg[1]);
	if (verbose > 2) 
	{
		
	}
	
	/* If a recently encountered triangle has been recorded and has not been */
	/*   deallocated, test it as a good starting point.                      */
	if (recenttri.tri != (triangle *) NULL) {
		if (recenttri.tri[3] != (triangle) NULL) {
			org(recenttri, torg);
			if ((torg[0] == searchpoint[0]) && (torg[1] == searchpoint[1])) {
				triedgecopy(recenttri, *searchtri);
				return ONVERTEX;
			}
			dist = (searchpoint[0] - torg[0]) * (searchpoint[0] - torg[0])
				+ (searchpoint[1] - torg[1]) * (searchpoint[1] - torg[1]);
			if (dist < searchdist) {
				triedgecopy(recenttri, *searchtri);
				searchdist = dist;
				if (verbose > 2)
				{
					
				}
			}
		}
	}
	
	/* The number of random samples taken is proportional to the cube root of */
	/*   the number of triangles in the mesh.  The next bit of code assumes   */
	/*   that the number of triangles increases monotonically.                */
	while (SAMPLEFACTOR * samples * samples * samples < triangles.items) {
		samples++;
	}
	triblocks = (triangles.maxitems + TRIPERBLOCK - 1) / TRIPERBLOCK;
	samplesperblock = 1 + (samples / triblocks);
	sampleblocks = samples / samplesperblock;
	sampleblock = triangles.firstblock;
	sampletri.orient = 0;
	for (i = 0; i < sampleblocks; i++) {
		alignptr = (UInt64) (sampleblock + 1);
		firsttri = (triangle *) (alignptr + (UInt64) triangles.alignbytes
			- (alignptr % (UInt64) triangles.alignbytes));
		for (j = 0; j < samplesperblock; j++) {
			if (i == triblocks - 1) {
				samplenum = randomnation((int)
					(triangles.maxitems - (i * TRIPERBLOCK)));
			} else {
				samplenum = randomnation(TRIPERBLOCK);
			}
			sampletri.tri = (triangle *)
				(firsttri + (samplenum * triangles.itemwords));
			if (sampletri.tri[3] != (triangle) NULL) {
				org(sampletri, torg);
				dist = (searchpoint[0] - torg[0]) * (searchpoint[0] - torg[0])
					+ (searchpoint[1] - torg[1]) * (searchpoint[1] - torg[1]);
				if (dist < searchdist) {
					triedgecopy(sampletri, *searchtri);
					searchdist = dist;
					if (verbose > 2)
					{
						
					}
				}
			}
		}
		sampleblock = (int **) *sampleblock;
	}
	/* Where are we? */
	org(*searchtri, torg);
	dest(*searchtri, tdest);
	/* Check the starting triangle's vertices. */
	if ((torg[0] == searchpoint[0]) && (torg[1] == searchpoint[1])) {
		return ONVERTEX;
	}
	if ((tdest[0] == searchpoint[0]) && (tdest[1] == searchpoint[1])) {
		lnextself(*searchtri);
		return ONVERTEX;
	}
	/* Orient `searchtri' to fit the preconditions of calling preciselocate(). */
	ahead = counterclockwise(torg, tdest, searchpoint);
	if (ahead < 0.0) {
		/* Turn around so that `searchpoint' is to the left of the */
		/*   edge specified by `searchtri'.                        */
		symself(*searchtri);
	} else if (ahead == 0.0) {
		/* Check if `searchpoint' is between `torg' and `tdest'. */
		if (((torg[0] < searchpoint[0]) == (searchpoint[0] < tdest[0]))
			&& ((torg[1] < searchpoint[1]) == (searchpoint[1] < tdest[1]))) {
			return ONEDGE;
		}
	}
	return preciselocate(searchpoint, searchtri);
}

void TriangleLib::insertshelle(struct triedge *tri, int shellemark)
//struct triedge *tri;          /* Edge at which to insert the new shell edge. */
//int shellemark;                            /* Marker for the new shell edge. */
{
	struct triedge oppotri;
	struct edge newshelle;
	point triorg, tridest;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	/* Mark points if possible. */
	org(*tri, triorg);
	dest(*tri, tridest);
	if (pointmark(triorg) == 0) {
		setpointmark(triorg, shellemark);
	}
	if (pointmark(tridest) == 0) {
		setpointmark(tridest, shellemark);
	}
	/* Check if there's already a shell edge here. */
	tspivot(*tri, newshelle);
	if (newshelle.sh == dummysh) {
		/* Make new shell edge and initialize its vertices. */
		makeshelle(&newshelle);
		setsorg(newshelle, tridest);
		setsdest(newshelle, triorg);
		/* Bond new shell edge to the two triangles it is sandwiched between. */
		/*   Note that the facing triangle `oppotri' might be equal to        */
		/*   `dummytri' (outer space), but the new shell edge is bonded to it */
		/*   all the same.                                                    */
		tsbond(*tri, newshelle);
		sym(*tri, oppotri);
		ssymself(newshelle);
		tsbond(oppotri, newshelle);
		setmark(newshelle, shellemark);
		if (verbose > 2) {
			
			printshelle(&newshelle);
		}
	} else {
		if (mark(newshelle) == 0) {
			setmark(newshelle, shellemark);
		}
	}
}

void TriangleLib::flip(struct triedge *flipedge)
//struct triedge *flipedge;                    /* Handle for the triangle abc. */
{
	struct triedge botleft, botright;
	struct triedge topleft, topright;
	struct triedge top;
	struct triedge botlcasing, botrcasing;
	struct triedge toplcasing, toprcasing;
	struct edge botlshelle, botrshelle;
	struct edge toplshelle, toprshelle;
	point leftpoint, rightpoint, botpoint;
	point farpoint;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	/* Identify the vertices of the quadrilateral. */
	org(*flipedge, rightpoint);
	dest(*flipedge, leftpoint);
	apex(*flipedge, botpoint);
	sym(*flipedge, top);
#ifdef SELF_CHECK
	if (top.tri == dummytri) {
		
		lnextself(*flipedge);
		return;
	}
	if (checksegments) {
		tspivot(*flipedge, toplshelle);
		if (toplshelle.sh != dummysh) {
			
			lnextself(*flipedge);
			return;
		}
	}
#endif /* SELF_CHECK */
	apex(top, farpoint);
	
	/* Identify the casing of the quadrilateral. */
	lprev(top, topleft);
	sym(topleft, toplcasing);
	lnext(top, topright);
	sym(topright, toprcasing);
	lnext(*flipedge, botleft);
	sym(botleft, botlcasing);
	lprev(*flipedge, botright);
	sym(botright, botrcasing);
	/* Rotate the quadrilateral one-quarter turn counterclockwise. */
	bond(topleft, botlcasing);
	bond(botleft, botrcasing);
	bond(botright, toprcasing);
	bond(topright, toplcasing);
	
	if (checksegments) {
		/* Check for shell edges and rebond them to the quadrilateral. */
		tspivot(topleft, toplshelle);
		tspivot(botleft, botlshelle);
		tspivot(botright, botrshelle);
		tspivot(topright, toprshelle);
		if (toplshelle.sh == dummysh) {
			tsdissolve(topright);
		} else {
			tsbond(topright, toplshelle);
		}
		if (botlshelle.sh == dummysh) {
			tsdissolve(topleft);
		} else {
			tsbond(topleft, botlshelle);
		}
		if (botrshelle.sh == dummysh) {
			tsdissolve(botleft);
		} else {
			tsbond(botleft, botrshelle);
		}
		if (toprshelle.sh == dummysh) {
			tsdissolve(botright);
		} else {
			tsbond(botright, toprshelle);
		}
	}
	
	/* New point assignments for the rotated quadrilateral. */
	setorg(*flipedge, farpoint);
	setdest(*flipedge, botpoint);
	setapex(*flipedge, rightpoint);
	setorg(top, botpoint);
	setdest(top, farpoint);
	setapex(top, leftpoint);
	if (verbose > 2) {
		
		lnextself(topleft);
		printtriangle(&topleft);
		printtriangle(flipedge);
	}
}

enum insertsiteresult TriangleLib::insertsite(point insertpoint, struct triedge *searchtri, struct edge *splitedge,
											   int segmentflaws, int triflaws)
{
	struct triedge horiz;
	struct triedge top;
	struct triedge botleft, botright;
	struct triedge topleft, topright;
	struct triedge newbotleft, newbotright;
	struct triedge newtopright;
	struct triedge botlcasing, botrcasing;
	struct triedge toplcasing, toprcasing;
	struct triedge testtri;
	struct edge botlshelle, botrshelle;
	struct edge toplshelle, toprshelle;
	struct edge brokenshelle;
	struct edge checkshelle;
	struct edge rightedge;
	struct edge newedge;
	struct edge *encroached;
	point first;
	point leftpoint, rightpoint, botpoint, toppoint, farpoint;
	double attrib;
	double area;
	enum insertsiteresult success;
	enum locateresult intersect;
	int doflip;
	int mirrorflag;
	int i;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;         /* Temporary variable used by spivot() and tspivot(). */
	
	if (verbose > 1) {
		
	}
	if (splitedge == (struct edge *) NULL) {
		/* Find the location of the point to be inserted.  Check if a good */
		/*   starting triangle has already been provided by the caller.    */
		if (searchtri->tri == (triangle *) NULL) {
			/* Find a boundary triangle. */
			horiz.tri = dummytri;
			horiz.orient = 0;
			symself(horiz);
			/* Search for a triangle containing `insertpoint'. */
			intersect = locate(insertpoint, &horiz);
		} else {
			/* Start searching from the triangle provided by the caller. */
			triedgecopy(*searchtri, horiz);
			intersect = preciselocate(insertpoint, &horiz);
		}
	} else {
		/* The calling routine provides the edge in which the point is inserted. */
		triedgecopy(*searchtri, horiz);
		intersect = ONEDGE;
	}
	if (intersect == ONVERTEX) {
		/* There's already a vertex there.  Return in `searchtri' a triangle */
		/*   whose origin is the existing vertex.                            */
		triedgecopy(horiz, *searchtri);
		triedgecopy(horiz, recenttri);
		return DUPLICATEPOINT;
	}
	if ((intersect == ONEDGE) || (intersect == OUTSIDE)) {
		/* The vertex falls on an edge or boundary. */
		if (checksegments && (splitedge == (struct edge *) NULL)) {
			/* Check whether the vertex falls on a shell edge. */
			tspivot(horiz, brokenshelle);
			if (brokenshelle.sh != dummysh) {
				/* The vertex falls on a shell edge. */
				if (segmentflaws) {
					if (nobisect == 0) {
						/* Add the shell edge to the list of encroached segments. */
						encroached = (struct edge *) poolalloc(&badsegments);
						shellecopy(brokenshelle, *encroached);
					} else if ((nobisect == 1) && (intersect == ONEDGE)) {
						/* This segment may be split only if it is an internal boundary. */
						sym(horiz, testtri);
						if (testtri.tri != dummytri) {
							/* Add the shell edge to the list of encroached segments. */
							encroached = (struct edge *) poolalloc(&badsegments);
							shellecopy(brokenshelle, *encroached);
						}
					}
				}
				/* Return a handle whose primary edge contains the point, */
				/*   which has not been inserted.                         */
				triedgecopy(horiz, *searchtri);
				triedgecopy(horiz, recenttri);
				return VIOLATINGPOINT;
			}
		}
		/* Insert the point on an edge, dividing one triangle into two (if */
		/*   the edge lies on a boundary) or two triangles into four.      */
		lprev(horiz, botright);
		sym(botright, botrcasing);
		sym(horiz, topright);
		/* Is there a second triangle?  (Or does this edge lie on a boundary?) */
		mirrorflag = topright.tri != dummytri;
		if (mirrorflag) {
			lnextself(topright);
			sym(topright, toprcasing);
			maketriangle(&newtopright);
		} else {
			/* Splitting the boundary edge increases the number of boundary edges. */
			hullsize++;
		}
		maketriangle(&newbotright);
		
		/* Set the vertices of changed and new triangles. */
		org(horiz, rightpoint);
		dest(horiz, leftpoint);
		apex(horiz, botpoint);
		setorg(newbotright, botpoint);
		setdest(newbotright, rightpoint);
		setapex(newbotright, insertpoint);
		setorg(horiz, insertpoint);
		for (i = 0; i < eextras; i++) {
			/* Set the element attributes of a new triangle. */
			setelemattribute(newbotright, i, elemattribute(botright, i));
		}
		if (vararea) {
			/* Set the area constraint of a new triangle. */
			setareabound(newbotright, areabound(botright));
		}
		if (mirrorflag) {
			dest(topright, toppoint);
			setorg(newtopright, rightpoint);
			setdest(newtopright, toppoint);
			setapex(newtopright, insertpoint);
			setorg(topright, insertpoint);
			for (i = 0; i < eextras; i++) {
				/* Set the element attributes of another new triangle. */
				setelemattribute(newtopright, i, elemattribute(topright, i));
			}
			if (vararea) {
				/* Set the area constraint of another new triangle. */
				setareabound(newtopright, areabound(topright));
			}
		}
		
		/* There may be shell edges that need to be bonded */
		/*   to the new triangle(s).                       */
		if (checksegments) {
			tspivot(botright, botrshelle);
			if (botrshelle.sh != dummysh) {
				tsdissolve(botright);
				tsbond(newbotright, botrshelle);
			}
			if (mirrorflag) {
				tspivot(topright, toprshelle);
				if (toprshelle.sh != dummysh) {
					tsdissolve(topright);
					tsbond(newtopright, toprshelle);
				}
			}
		}
		
		/* Bond the new triangle(s) to the surrounding triangles. */
		bond(newbotright, botrcasing);
		lprevself(newbotright);
		bond(newbotright, botright);
		lprevself(newbotright);
		if (mirrorflag) {
			bond(newtopright, toprcasing);
			lnextself(newtopright);
			bond(newtopright, topright);
			lnextself(newtopright);
			bond(newtopright, newbotright);
		}
		
		if (splitedge != (struct edge *) NULL) {
			/* Split the shell edge into two. */
			setsdest(*splitedge, insertpoint);
			ssymself(*splitedge);
			spivot(*splitedge, rightedge);
			insertshelle(&newbotright, mark(*splitedge));
			tspivot(newbotright, newedge);
			sbond(*splitedge, newedge);
			ssymself(newedge);
			sbond(newedge, rightedge);
			ssymself(*splitedge);
		}
		
#ifdef SELF_CHECK
		if (counterclockwise(rightpoint, leftpoint, botpoint) < 0.0) {
			
		}
		if (mirrorflag) {
			if (counterclockwise(leftpoint, rightpoint, toppoint) < 0.0) {
				
			}
			if (counterclockwise(rightpoint, toppoint, insertpoint) < 0.0) {
				
			}
			if (counterclockwise(toppoint, leftpoint, insertpoint) < 0.0) {
				
				
			}
		}
		if (counterclockwise(leftpoint, botpoint, insertpoint) < 0.0) {
			
			
		}
		if (counterclockwise(botpoint, rightpoint, insertpoint) < 0.0) 
		{
			
		}
#endif /* SELF_CHECK */
		if (verbose > 2) {
			
			printtriangle(&botright);
			if (mirrorflag) {
				
				printtriangle(&topright);
				
				printtriangle(&newtopright);
			}
			
			printtriangle(&newbotright);
		}
		
		/* Position `horiz' on the first edge to check for */
		/*   the Delaunay property.                        */
		lnextself(horiz);
  } else {
	  /* Insert the point in a triangle, splitting it into three. */
	  lnext(horiz, botleft);
	  lprev(horiz, botright);
	  sym(botleft, botlcasing);
	  sym(botright, botrcasing);
	  maketriangle(&newbotleft);
	  maketriangle(&newbotright);
	  
	  /* Set the vertices of changed and new triangles. */
	  org(horiz, rightpoint);
	  dest(horiz, leftpoint);
	  apex(horiz, botpoint);
	  setorg(newbotleft, leftpoint);
	  setdest(newbotleft, botpoint);
	  setapex(newbotleft, insertpoint);
	  setorg(newbotright, botpoint);
	  setdest(newbotright, rightpoint);
	  setapex(newbotright, insertpoint);
	  setapex(horiz, insertpoint);
	  for (i = 0; i < eextras; i++) {
		  /* Set the element attributes of the new triangles. */
		  attrib = elemattribute(horiz, i);
		  setelemattribute(newbotleft, i, attrib);
		  setelemattribute(newbotright, i, attrib);
	  }
	  if (vararea) {
		  /* Set the area constraint of the new triangles. */
		  area = areabound(horiz);
		  setareabound(newbotleft, area);
		  setareabound(newbotright, area);
	  }
	  
	  /* There may be shell edges that need to be bonded */
	  /*   to the new triangles.                         */
	  if (checksegments) {
		  tspivot(botleft, botlshelle);
		  if (botlshelle.sh != dummysh) {
			  tsdissolve(botleft);
			  tsbond(newbotleft, botlshelle);
		  }
		  tspivot(botright, botrshelle);
		  if (botrshelle.sh != dummysh) {
			  tsdissolve(botright);
			  tsbond(newbotright, botrshelle);
		  }
	  }
	  
	  /* Bond the new triangles to the surrounding triangles. */
	  bond(newbotleft, botlcasing);
	  bond(newbotright, botrcasing);
	  lnextself(newbotleft);
	  lprevself(newbotright);
	  bond(newbotleft, newbotright);
	  lnextself(newbotleft);
	  bond(botleft, newbotleft);
	  lprevself(newbotright);
	  bond(botright, newbotright);
	  
#ifdef SELF_CHECK
	  if (counterclockwise(rightpoint, leftpoint, botpoint) < 0.0) {
		  
	  }
	  if (counterclockwise(rightpoint, leftpoint, insertpoint) < 0.0) {
		  
	  }
	  if (counterclockwise(leftpoint, botpoint, insertpoint) < 0.0) {
		  
	  }
	  if (counterclockwise(botpoint, rightpoint, insertpoint) < 0.0) {
		  
	  }
#endif /* SELF_CHECK */
	  if (verbose > 2) {
		  
		  printtriangle(&horiz);
		  
		  printtriangle(&newbotleft);
		  
		  printtriangle(&newbotright);
	  }
  }
  
  /* The insertion is successful by default, unless an encroached */
  /*   edge is found.                                             */
  success = SUCCESSFULPOINT;
  /* Circle around the newly inserted vertex, checking each edge opposite */
  /*   it for the Delaunay property.  Non-Delaunay edges are flipped.     */
  /*   `horiz' is always the edge being checked.  `first' marks where to  */
  /*   stop circling.                                                     */
  org(horiz, first);
  rightpoint = first;
  dest(horiz, leftpoint);
  /* Circle until finished. */
  while (1) {
	  /* By default, the edge will be flipped. */
	  doflip = 1;
	  if (checksegments) {
		  /* Check for a segment, which cannot be flipped. */
		  tspivot(horiz, checkshelle);
		  if (checkshelle.sh != dummysh) {
			  /* The edge is a segment and cannot be flipped. */
			  doflip = 0;
#ifndef CDT_ONLY
			  if (segmentflaws) {
				  /* Does the new point encroach upon this segment? */
				  if (checkedge4encroach(&checkshelle)) {
					  success = ENCROACHINGPOINT;
				  }
			  }
#endif /* not CDT_ONLY */
		  }
	  }
	  if (doflip) {
		  /* Check if the edge is a boundary edge. */
		  sym(horiz, top);
		  if (top.tri == dummytri) {
			  /* The edge is a boundary edge and cannot be flipped. */
			  doflip = 0;
		  } else {
			  /* Find the point on the other side of the edge. */
			  apex(top, farpoint);
			  /* In the incremental Delaunay triangulation algorithm, any of    */
			  /*   `leftpoint', `rightpoint', and `farpoint' could be vertices  */
			  /*   of the triangular bounding box.  These vertices must be      */
			  /*   treated as if they are infinitely distant, even though their */
			  /*   "coordinates" are not.                                       */
			  if ((leftpoint == infpoint1) || (leftpoint == infpoint2)
				  || (leftpoint == infpoint3)) {
				  /* `leftpoint' is infinitely distant.  Check the convexity of */
				  /*   the boundary of the triangulation.  'farpoint' might be  */
				  /*   infinite as well, but trust me, this same condition      */
				  /*   should be applied.                                       */
				  doflip = counterclockwise(insertpoint, rightpoint, farpoint) > 0.0;
			  } else if ((rightpoint == infpoint1) || (rightpoint == infpoint2)
				  || (rightpoint == infpoint3)) {
				  /* `rightpoint' is infinitely distant.  Check the convexity of */
				  /*   the boundary of the triangulation.  'farpoint' might be  */
				  /*   infinite as well, but trust me, this same condition      */
				  /*   should be applied.                                       */
				  doflip = counterclockwise(farpoint, leftpoint, insertpoint) > 0.0;
			  } else if ((farpoint == infpoint1) || (farpoint == infpoint2)
				  || (farpoint == infpoint3)) {
				  /* `farpoint' is infinitely distant and cannot be inside */
				  /*   the circumcircle of the triangle `horiz'.           */
				  doflip = 0;
			  } else {
				  /* Test whether the edge is locally Delaunay. */
				  doflip = incircle(leftpoint, insertpoint, rightpoint, farpoint)
					  > 0.0;
			  }
			  if (doflip) {
				  /* We made it!  Flip the edge `horiz' by rotating its containing */
				  /*   quadrilateral (the two triangles adjacent to `horiz').      */
				  /* Identify the casing of the quadrilateral. */
				  lprev(top, topleft);
				  sym(topleft, toplcasing);
				  lnext(top, topright);
				  sym(topright, toprcasing);
				  lnext(horiz, botleft);
				  sym(botleft, botlcasing);
				  lprev(horiz, botright);
				  sym(botright, botrcasing);
				  /* Rotate the quadrilateral one-quarter turn counterclockwise. */
				  bond(topleft, botlcasing);
				  bond(botleft, botrcasing);
				  bond(botright, toprcasing);
				  bond(topright, toplcasing);
				  if (checksegments) {
					  /* Check for shell edges and rebond them to the quadrilateral. */
					  tspivot(topleft, toplshelle);
					  tspivot(botleft, botlshelle);
					  tspivot(botright, botrshelle);
					  tspivot(topright, toprshelle);
					  if (toplshelle.sh == dummysh) {
						  tsdissolve(topright);
					  } else {
						  tsbond(topright, toplshelle);
					  }
					  if (botlshelle.sh == dummysh) {
						  tsdissolve(topleft);
					  } else {
						  tsbond(topleft, botlshelle);
					  }
					  if (botrshelle.sh == dummysh) {
						  tsdissolve(botleft);
					  } else {
						  tsbond(botleft, botrshelle);
					  }
					  if (toprshelle.sh == dummysh) {
						  tsdissolve(botright);
					  } else {
						  tsbond(botright, toprshelle);
					  }
				  }
				  /* New point assignments for the rotated quadrilateral. */
				  setorg(horiz, farpoint);
				  setdest(horiz, insertpoint);
				  setapex(horiz, rightpoint);
				  setorg(top, insertpoint);
				  setdest(top, farpoint);
				  setapex(top, leftpoint);
				  for (i = 0; i < eextras; i++) {
					  /* Take the average of the two triangles' attributes. */
					  attrib = 0.5 * (elemattribute(top, i) + elemattribute(horiz, i));
					  setelemattribute(top, i, attrib);
					  setelemattribute(horiz, i, attrib);
				  }
				  if (vararea) {
					  if ((areabound(top) <= 0.0) || (areabound(horiz) <= 0.0)) {
						  area = -1.0;
					  } else {
						  /* Take the average of the two triangles' area constraints.    */
						  /*   This prevents small area constraints from migrating a     */
						  /*   long, long way from their original location due to flips. */
						  area = 0.5 * (areabound(top) + areabound(horiz));
					  }
					  setareabound(top, area);
					  setareabound(horiz, area);
				  }
#ifdef SELF_CHECK
				  if (insertpoint != (point) NULL) {
					  if (counterclockwise(leftpoint, insertpoint, rightpoint) < 0.0) {
						  
					  }
					  
					  if (counterclockwise(farpoint, leftpoint, insertpoint) < 0.0) {
						  
					  }
					  if (counterclockwise(insertpoint, rightpoint, farpoint) < 0.0) {
						  
					  }
				  }
#endif /* SELF_CHECK */
				  if (verbose > 2) {
					  
					  lnextself(topleft);
					  printtriangle(&topleft);
					  printtriangle(&horiz);
				  }
				  /* On the next iterations, consider the two edges that were  */
				  /*   exposed (this is, are now visible to the newly inserted */
				  /*   point) by the edge flip.                                */
				  lprevself(horiz);
				  leftpoint = farpoint;
        }
      }
    }
    if (!doflip) {
		/* The handle `horiz' is accepted as locally Delaunay. */
#ifndef CDT_ONLY
		if (triflaws) {
			/* Check the triangle `horiz' for quality. */
			testtriangle(&horiz);
		}
#endif /* not CDT_ONLY */
		/* Look for the next edge around the newly inserted point. */
		lnextself(horiz);
		sym(horiz, testtri);
		/* Check for finishing a complete revolution about the new point, or */
		/*   falling off the edge of the triangulation.  The latter will     */
		/*   happen when a point is inserted at a boundary.                  */
		if ((leftpoint == first) || (testtri.tri == dummytri)) {
			/* We're done.  Return a triangle whose origin is the new point. */
			lnext(horiz, *searchtri);
			lnext(horiz, recenttri);
			return success;
		}
		/* Finish finding the next edge around the newly inserted point. */
		lnext(testtri, horiz);
		rightpoint = leftpoint;
		dest(horiz, leftpoint);
    }
  }
}

void TriangleLib::triangulatepolygon(struct triedge *firstedge, struct triedge *lastedge, int edgecount, int doflip, int triflaws)
{
	struct triedge testtri;
	struct triedge besttri;
	struct triedge tempedge;
	point leftbasepoint, rightbasepoint;
	point testpoint;
	point bestpoint;
	int bestnumber;
	int i;
	triangle ptr;   /* Temporary variable used by sym(), onext(), and oprev(). */
	
	/* Identify the base vertices. */
	apex(*lastedge, leftbasepoint);
	dest(*firstedge, rightbasepoint);
	if (verbose > 2) {
		
	}
	/* Find the best vertex to connect the base to. */
	onext(*firstedge, besttri);
	dest(besttri, bestpoint);
	triedgecopy(besttri, testtri);
	bestnumber = 1;
	for (i = 2; i <= edgecount - 2; i++) {
		onextself(testtri);
		dest(testtri, testpoint);
		/* Is this a better vertex? */
		if (incircle(leftbasepoint, rightbasepoint, bestpoint, testpoint) > 0.0) {
			triedgecopy(testtri, besttri);
			bestpoint = testpoint;
			bestnumber = i;
		}
	}
	if (verbose > 2) 
	{
		
	}
	if (bestnumber > 1) {
		/* Recursively triangulate the smaller polygon on the right. */
		oprev(besttri, tempedge);
		triangulatepolygon(firstedge, &tempedge, bestnumber + 1, 1, triflaws);
	}
	if (bestnumber < edgecount - 2) {
		/* Recursively triangulate the smaller polygon on the left. */
		sym(besttri, tempedge);
		triangulatepolygon(&besttri, lastedge, edgecount - bestnumber, 1,
			triflaws);
		/* Find `besttri' again; it may have been lost to edge flips. */
		sym(tempedge, besttri);
	}
	if (doflip) {
		/* Do one final edge flip. */
		flip(&besttri);
#ifndef CDT_ONLY
		if (triflaws) {
			/* Check the quality of the newly committed triangle. */
			sym(besttri, testtri);
			testtriangle(&testtri);
		}
#endif /* not CDT_ONLY */
	}
	/* Return the base triangle. */
	triedgecopy(besttri, *lastedge);
}

void TriangleLib::deletesite(struct triedge *deltri)
{
	struct triedge countingtri;
	struct triedge firstedge, lastedge;
	struct triedge deltriright;
	struct triedge lefttri, righttri;
	struct triedge leftcasing, rightcasing;
	struct edge leftshelle, rightshelle;
	point delpoint;
	point neworg;
	int edgecount;
	triangle ptr;   /* Temporary variable used by sym(), onext(), and oprev(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	org(*deltri, delpoint);
	if (verbose > 1) 
	{
		
	}
	pointdealloc(delpoint);
	
	/* Count the degree of the point being deleted. */
	onext(*deltri, countingtri);
	edgecount = 1;
	while (!triedgeequal(*deltri, countingtri)) {
#ifdef SELF_CHECK
		if (countingtri.tri == dummytri) {
			
			internalerror();
		}
#endif /* SELF_CHECK */
		edgecount++;
		onextself(countingtri);
	}
	
#ifdef SELF_CHECK
	if (edgecount < 3) {
		
		internalerror();
	}
#endif /* SELF_CHECK */
	if (edgecount > 3) {
		/* Triangulate the polygon defined by the union of all triangles */
		/*   adjacent to the point being deleted.  Check the quality of  */
		/*   the resulting triangles.                                    */
		onext(*deltri, firstedge);
		oprev(*deltri, lastedge);
		triangulatepolygon(&firstedge, &lastedge, edgecount, 0, !nobisect);
	}
	/* Splice out two triangles. */
	lprev(*deltri, deltriright);
	dnext(*deltri, lefttri);
	sym(lefttri, leftcasing);
	oprev(deltriright, righttri);
	sym(righttri, rightcasing);
	bond(*deltri, leftcasing);
	bond(deltriright, rightcasing);
	tspivot(lefttri, leftshelle);
	if (leftshelle.sh != dummysh) {
		tsbond(*deltri, leftshelle);
	}
	tspivot(righttri, rightshelle);
	if (rightshelle.sh != dummysh) {
		tsbond(deltriright, rightshelle);
	}
	
	/* Set the new origin of `deltri' and check its quality. */
	org(lefttri, neworg);
	setorg(*deltri, neworg);
	if (!nobisect) {
		testtriangle(deltri);
	}
	
	/* Delete the two spliced-out triangles. */
	triangledealloc(lefttri.tri);
	triangledealloc(righttri.tri);
}

void TriangleLib::pointsort(point *sortarray, int arraysize)
{
	int left, right;
	int pivot;
	double pivotx, pivoty;
	point temp;
	
	if (arraysize == 2) {
		/* Recursive base case. */
		if ((sortarray[0][0] > sortarray[1][0]) ||
			((sortarray[0][0] == sortarray[1][0]) &&
			(sortarray[0][1] > sortarray[1][1]))) {
			temp = sortarray[1];
			sortarray[1] = sortarray[0];
			sortarray[0] = temp;
		}
		return;
	}
	/* Choose a random pivot to split the array. */
	pivot = (int) randomnation(arraysize);
	pivotx = sortarray[pivot][0];
	pivoty = sortarray[pivot][1];
	/* Split the array. */
	left = -1;
	right = arraysize;
	while (left < right) {
		/* Search for a point whose x-coordinate is too large for the left. */
		do {
			left++;
		} while ((left <= right) && ((sortarray[left][0] < pivotx) ||
			((sortarray[left][0] == pivotx) &&
			(sortarray[left][1] < pivoty))));
		/* Search for a point whose x-coordinate is too small for the right. */
		do {
			right--;
		} while ((left <= right) && ((sortarray[right][0] > pivotx) ||
			((sortarray[right][0] == pivotx) &&
			(sortarray[right][1] > pivoty))));
		if (left < right) {
			/* Swap the left and right points. */
			temp = sortarray[left];
			sortarray[left] = sortarray[right];
			sortarray[right] = temp;
		}
	}
	if (left > 1) {
		/* Recursively sort the left subset. */
		pointsort(sortarray, left);
	}
	if (right < arraysize - 2) {
		/* Recursively sort the right subset. */
		pointsort(&sortarray[right + 1], arraysize - right - 1);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  pointmedian()   An order statistic algorithm, almost.  Shuffles an array */
/*                  of points so that the first `median' points occur        */
/*                  lexicographically before the remaining points.           */
/*                                                                           */
/*  Uses the x-coordinate as the primary key if axis == 0; the y-coordinate  */
/*  if axis == 1.  Very similar to the pointsort() procedure, but runs in    */
/*  randomized linear time.                                                  */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::pointmedian(point *sortarray, int arraysize, int median, int axis)
{
	int left, right;
	int pivot;
	double pivot1, pivot2;
	point temp;
	
	if (arraysize == 2) {
		/* Recursive base case. */
		if ((sortarray[0][axis] > sortarray[1][axis]) ||
			((sortarray[0][axis] == sortarray[1][axis]) &&
			(sortarray[0][1 - axis] > sortarray[1][1 - axis]))) {
			temp = sortarray[1];
			sortarray[1] = sortarray[0];
			sortarray[0] = temp;
		}
		return;
	}
	/* Choose a random pivot to split the array. */
	pivot = (int) randomnation(arraysize);
	pivot1 = sortarray[pivot][axis];
	pivot2 = sortarray[pivot][1 - axis];
	/* Split the array. */
	left = -1;
	right = arraysize;
	while (left < right) {
		/* Search for a point whose x-coordinate is too large for the left. */
		do {
			left++;
		} while ((left <= right) && ((sortarray[left][axis] < pivot1) ||
			((sortarray[left][axis] == pivot1) &&
			(sortarray[left][1 - axis] < pivot2))));
		/* Search for a point whose x-coordinate is too small for the right. */
		do {
			right--;
		} while ((left <= right) && ((sortarray[right][axis] > pivot1) ||
			((sortarray[right][axis] == pivot1) &&
			(sortarray[right][1 - axis] > pivot2))));
		if (left < right) {
			/* Swap the left and right points. */
			temp = sortarray[left];
			sortarray[left] = sortarray[right];
			sortarray[right] = temp;
		}
	}
	/* Unlike in pointsort(), at most one of the following */
	/*   conditionals is true.                             */
	if (left > median) {
		/* Recursively shuffle the left subset. */
		pointmedian(sortarray, left, median, axis);
	}
	if (right < median - 1) {
		/* Recursively shuffle the right subset. */
		pointmedian(&sortarray[right + 1], arraysize - right - 1,
			median - right - 1, axis);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  alternateaxes()   Sorts the points as appropriate for the divide-and-    */
/*                    conquer algorithm with alternating cuts.               */
/*                                                                           */
/*  Partitions by x-coordinate if axis == 0; by y-coordinate if axis == 1.   */
/*  For the base case, subsets containing only two or three points are       */
/*  always sorted by x-coordinate.                                           */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::alternateaxes(point *sortarray, int arraysize, int axis)
{
	int divider;
	
	divider = arraysize >> 1;
	if (arraysize <= 3) {
		/* Recursive base case:  subsets of two or three points will be      */
		/*   handled specially, and should always be sorted by x-coordinate. */
		axis = 0;
	}
	/* Partition with a horizontal or vertical cut. */
	pointmedian(sortarray, arraysize, divider, axis);
	/* Recursively partition the subsets with a cross cut. */
	if (arraysize - divider >= 2) {
		if (divider >= 2) {
			alternateaxes(sortarray, divider, 1 - axis);
		}
		alternateaxes(&sortarray[divider], arraysize - divider, 1 - axis);
	}
}

void TriangleLib::mergehulls(struct triedge *farleft, struct triedge *innerleft, struct triedge *innerright, struct triedge *farright, int axis)
{
	struct triedge leftcand, rightcand;
	struct triedge baseedge;
	struct triedge nextedge;
	struct triedge sidecasing, topcasing, outercasing;
	struct triedge checkedge;
	point innerleftdest;
	point innerrightorg;
	point innerleftapex, innerrightapex;
	point farleftpt, farrightpt;
	point farleftapex, farrightapex;
	point lowerleft, lowerright;
	point upperleft, upperright;
	point nextapex;
	point checkvertex;
	int changemade;
	int badedge;
	int leftfinished, rightfinished;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	dest(*innerleft, innerleftdest);
	apex(*innerleft, innerleftapex);
	org(*innerright, innerrightorg);
	apex(*innerright, innerrightapex);
	/* Special treatment for horizontal cuts. */
	if (dwyer && (axis == 1)) {
		org(*farleft, farleftpt);
		apex(*farleft, farleftapex);
		dest(*farright, farrightpt);
		apex(*farright, farrightapex);
		/* The pointers to the extremal points are shifted to point to the */
		/*   topmost and bottommost point of each hull, rather than the    */
		/*   leftmost and rightmost points.                                */
		while (farleftapex[1] < farleftpt[1]) {
			lnextself(*farleft);
			symself(*farleft);
			farleftpt = farleftapex;
			apex(*farleft, farleftapex);
		}
		sym(*innerleft, checkedge);
		apex(checkedge, checkvertex);
		while (checkvertex[1] > innerleftdest[1]) {
			lnext(checkedge, *innerleft);
			innerleftapex = innerleftdest;
			innerleftdest = checkvertex;
			sym(*innerleft, checkedge);
			apex(checkedge, checkvertex);
		}
		while (innerrightapex[1] < innerrightorg[1]) {
			lnextself(*innerright);
			symself(*innerright);
			innerrightorg = innerrightapex;
			apex(*innerright, innerrightapex);
		}
		sym(*farright, checkedge);
		apex(checkedge, checkvertex);
		while (checkvertex[1] > farrightpt[1]) {
			lnext(checkedge, *farright);
			farrightapex = farrightpt;
			farrightpt = checkvertex;
			sym(*farright, checkedge);
			apex(checkedge, checkvertex);
		}
	}
	/* Find a line tangent to and below both hulls. */
	do {
		changemade = 0;
		/* Make innerleftdest the "bottommost" point of the left hull. */
		if (counterclockwise(innerleftdest, innerleftapex, innerrightorg) > 0.0) {
			lprevself(*innerleft);
			symself(*innerleft);
			innerleftdest = innerleftapex;
			apex(*innerleft, innerleftapex);
			changemade = 1;
		}
		/* Make innerrightorg the "bottommost" point of the right hull. */
		if (counterclockwise(innerrightapex, innerrightorg, innerleftdest) > 0.0) {
			lnextself(*innerright);
			symself(*innerright);
			innerrightorg = innerrightapex;
			apex(*innerright, innerrightapex);
			changemade = 1;
		}
	} while (changemade);
	/* Find the two candidates to be the next "gear tooth". */
	sym(*innerleft, leftcand);
	sym(*innerright, rightcand);
	/* Create the bottom new bounding triangle. */
	maketriangle(&baseedge);
	/* Connect it to the bounding boxes of the left and right triangulations. */
	bond(baseedge, *innerleft);
	lnextself(baseedge);
	bond(baseedge, *innerright);
	lnextself(baseedge);
	setorg(baseedge, innerrightorg);
	setdest(baseedge, innerleftdest);
	/* Apex is intentionally left NULL. */
	if (verbose > 2) {
		
		printtriangle(&baseedge);
	}
	/* Fix the extreme triangles if necessary. */
	org(*farleft, farleftpt);
	if (innerleftdest == farleftpt) {
		lnext(baseedge, *farleft);
	}
	dest(*farright, farrightpt);
	if (innerrightorg == farrightpt) {
		lprev(baseedge, *farright);
	}
	/* The vertices of the current knitting edge. */
	lowerleft = innerleftdest;
	lowerright = innerrightorg;
	/* The candidate vertices for knitting. */
	apex(leftcand, upperleft);
	apex(rightcand, upperright);
	/* Walk up the gap between the two triangulations, knitting them together. */
	while (1) {
		/* Have we reached the top?  (This isn't quite the right question,       */
		/*   because even though the left triangulation might seem finished now, */
		/*   moving up on the right triangulation might reveal a new point of    */
		/*   the left triangulation.  And vice-versa.)                           */
		leftfinished = counterclockwise(upperleft, lowerleft, lowerright) <= 0.0;
		rightfinished = counterclockwise(upperright, lowerleft, lowerright) <= 0.0;
		if (leftfinished && rightfinished) {
			/* Create the top new bounding triangle. */
			maketriangle(&nextedge);
			setorg(nextedge, lowerleft);
			setdest(nextedge, lowerright);
			/* Apex is intentionally left NULL. */
			/* Connect it to the bounding boxes of the two triangulations. */
			bond(nextedge, baseedge);
			lnextself(nextedge);
			bond(nextedge, rightcand);
			lnextself(nextedge);
			bond(nextedge, leftcand);
			if (verbose > 2) {
				
				printtriangle(&baseedge);
			}
			/* Special treatment for horizontal cuts. */
			if (dwyer && (axis == 1)) {
				org(*farleft, farleftpt);
				apex(*farleft, farleftapex);
				dest(*farright, farrightpt);
				apex(*farright, farrightapex);
				sym(*farleft, checkedge);
				apex(checkedge, checkvertex);
				/* The pointers to the extremal points are restored to the leftmost */
				/*   and rightmost points (rather than topmost and bottommost).     */
				while (checkvertex[0] < farleftpt[0]) {
					lprev(checkedge, *farleft);
					farleftapex = farleftpt;
					farleftpt = checkvertex;
					sym(*farleft, checkedge);
					apex(checkedge, checkvertex);
				}
				while (farrightapex[0] > farrightpt[0]) {
					lprevself(*farright);
					symself(*farright);
					farrightpt = farrightapex;
					apex(*farright, farrightapex);
				}
			}
			return;
		}
		/* Consider eliminating edges from the left triangulation. */
		if (!leftfinished) {
			/* What vertex would be exposed if an edge were deleted? */
			lprev(leftcand, nextedge);
			symself(nextedge);
			apex(nextedge, nextapex);
			/* If nextapex is NULL, then no vertex would be exposed; the */
			/*   triangulation would have been eaten right through.      */
			if (nextapex != (point) NULL) {
				/* Check whether the edge is Delaunay. */
				badedge = incircle(lowerleft, lowerright, upperleft, nextapex) > 0.0;
				while (badedge) {
					/* Eliminate the edge with an edge flip.  As a result, the    */
					/*   left triangulation will have one more boundary triangle. */
					lnextself(nextedge);
					sym(nextedge, topcasing);
					lnextself(nextedge);
					sym(nextedge, sidecasing);
					bond(nextedge, topcasing);
					bond(leftcand, sidecasing);
					lnextself(leftcand);
					sym(leftcand, outercasing);
					lprevself(nextedge);
					bond(nextedge, outercasing);
					/* Correct the vertices to reflect the edge flip. */
					setorg(leftcand, lowerleft);
					setdest(leftcand, NULL);
					setapex(leftcand, nextapex);
					setorg(nextedge, NULL);
					setdest(nextedge, upperleft);
					setapex(nextedge, nextapex);
					/* Consider the newly exposed vertex. */
					upperleft = nextapex;
					/* What vertex would be exposed if another edge were deleted? */
					triedgecopy(sidecasing, nextedge);
					apex(nextedge, nextapex);
					if (nextapex != (point) NULL) {
						/* Check whether the edge is Delaunay. */
						badedge = incircle(lowerleft, lowerright, upperleft, nextapex)
							> 0.0;
					} else {
						/* Avoid eating right through the triangulation. */
						badedge = 0;
					}
				}
			}
		}
		/* Consider eliminating edges from the right triangulation. */
		if (!rightfinished) {
			/* What vertex would be exposed if an edge were deleted? */
			lnext(rightcand, nextedge);
			symself(nextedge);
			apex(nextedge, nextapex);
			/* If nextapex is NULL, then no vertex would be exposed; the */
			/*   triangulation would have been eaten right through.      */
			if (nextapex != (point) NULL) {
				/* Check whether the edge is Delaunay. */
				badedge = incircle(lowerleft, lowerright, upperright, nextapex) > 0.0;
				while (badedge) {
					/* Eliminate the edge with an edge flip.  As a result, the     */
					/*   right triangulation will have one more boundary triangle. */
					lprevself(nextedge);
					sym(nextedge, topcasing);
					lprevself(nextedge);
					sym(nextedge, sidecasing);
					bond(nextedge, topcasing);
					bond(rightcand, sidecasing);
					lprevself(rightcand);
					sym(rightcand, outercasing);
					lnextself(nextedge);
					bond(nextedge, outercasing);
					/* Correct the vertices to reflect the edge flip. */
					setorg(rightcand, NULL);
					setdest(rightcand, lowerright);
					setapex(rightcand, nextapex);
					setorg(nextedge, upperright);
					setdest(nextedge, NULL);
					setapex(nextedge, nextapex);
					/* Consider the newly exposed vertex. */
					upperright = nextapex;
					/* What vertex would be exposed if another edge were deleted? */
					triedgecopy(sidecasing, nextedge);
					apex(nextedge, nextapex);
					if (nextapex != (point) NULL) {
						/* Check whether the edge is Delaunay. */
						badedge = incircle(lowerleft, lowerright, upperright, nextapex)
							> 0.0;
					} else {
						/* Avoid eating right through the triangulation. */
						badedge = 0;
					}
				}
			}
		}
		if (leftfinished || (!rightfinished &&
			(incircle(upperleft, lowerleft, lowerright, upperright) > 0.0))) {
			/* Knit the triangulations, adding an edge from `lowerleft' */
			/*   to `upperright'.                                       */
			bond(baseedge, rightcand);
			lprev(rightcand, baseedge);
			setdest(baseedge, lowerleft);
			lowerright = upperright;
			sym(baseedge, rightcand);
			apex(rightcand, upperright);
		} else {
			/* Knit the triangulations, adding an edge from `upperleft' */
			/*   to `lowerright'.                                       */
			bond(baseedge, leftcand);
			lnext(leftcand, baseedge);
			setorg(baseedge, lowerright);
			lowerleft = upperleft;
			sym(baseedge, leftcand);
			apex(leftcand, upperleft);
		}
		if (verbose > 2) {
			
			printtriangle(&baseedge);
		}
  }
}

/*****************************************************************************/
/*                                                                           */
/*  divconqrecurse()   Recursively form a Delaunay triangulation by the      */
/*                     divide-and-conquer method.                            */
/*                                                                           */
/*  Recursively breaks down the problem into smaller pieces, which are       */
/*  knitted together by mergehulls().  The base cases (problems of two or    */
/*  three points) are handled specially here.                                */
/*                                                                           */
/*  On completion, `farleft' and `farright' are bounding triangles such that */
/*  the origin of `farleft' is the leftmost vertex (breaking ties by         */
/*  choosing the highest leftmost vertex), and the destination of            */
/*  `farright' is the rightmost vertex (breaking ties by choosing the        */
/*  lowest rightmost vertex).                                                */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::divconqrecurse(point *sortarray, int vertices, int axis,
								  struct triedge *farleft, struct triedge *farright)
{
	struct triedge midtri, tri1, tri2, tri3;
	struct triedge innerleft, innerright;
	double area;
	int divider;
	
	if (verbose > 2) 
	{
		
	}
	if (vertices == 2) {
		/* The triangulation of two vertices is an edge.  An edge is */
		/*   represented by two bounding triangles.                  */

		maketriangle(farleft);
		setorg(*farleft, sortarray[0]);
		setdest(*farleft, sortarray[1]);
		/* The apex is intentionally left NULL. */
		maketriangle(farright);
		setorg(*farright, sortarray[1]);
		setdest(*farright, sortarray[0]);
		/* The apex is intentionally left NULL. */
		bond(*farleft, *farright);
		lprevself(*farleft);
		lnextself(*farright);
		bond(*farleft, *farright);
		lprevself(*farleft);
		lnextself(*farright);
		bond(*farleft, *farright);
		if (verbose > 2) {
			
			printtriangle(farleft);
			printtriangle(farright);
		}
		/* Ensure that the origin of `farleft' is sortarray[0]. */
		lprev(*farright, *farleft);
		return;
	} else if (vertices == 3) {
		/* The triangulation of three vertices is either a triangle (with */
		/*   three bounding triangles) or two edges (with four bounding   */
		/*   triangles).  In either case, four triangles are created.     */
		maketriangle(&midtri);
		maketriangle(&tri1);
		maketriangle(&tri2);
		maketriangle(&tri3);
		area = counterclockwise(sortarray[0], sortarray[1], sortarray[2]);
		if (area == 0.0) {
			/* Three collinear points; the triangulation is two edges. */
			setorg(midtri, sortarray[0]);
			setdest(midtri, sortarray[1]);
			setorg(tri1, sortarray[1]);
			setdest(tri1, sortarray[0]);
			setorg(tri2, sortarray[2]);
			setdest(tri2, sortarray[1]);
			setorg(tri3, sortarray[1]);
			setdest(tri3, sortarray[2]);
			/* All apices are intentionally left NULL. */
			bond(midtri, tri1);
			bond(tri2, tri3);
			lnextself(midtri);
			lprevself(tri1);
			lnextself(tri2);
			lprevself(tri3);
			bond(midtri, tri3);
			bond(tri1, tri2);
			lnextself(midtri);
			lprevself(tri1);
			lnextself(tri2);
			lprevself(tri3);
			bond(midtri, tri1);
			bond(tri2, tri3);
			/* Ensure that the origin of `farleft' is sortarray[0]. */
			triedgecopy(tri1, *farleft);
			/* Ensure that the destination of `farright' is sortarray[2]. */
			triedgecopy(tri2, *farright);
		} else {
			/* The three points are not collinear; the triangulation is one */
			/*   triangle, namely `midtri'.                                 */
			setorg(midtri, sortarray[0]);
			setdest(tri1, sortarray[0]);
			setorg(tri3, sortarray[0]);
			/* Apices of tri1, tri2, and tri3 are left NULL. */
			if (area > 0.0) {
				/* The vertices are in counterclockwise order. */
				setdest(midtri, sortarray[1]);
				setorg(tri1, sortarray[1]);
				setdest(tri2, sortarray[1]);
				setapex(midtri, sortarray[2]);
				setorg(tri2, sortarray[2]);
				setdest(tri3, sortarray[2]);
			} else {
				/* The vertices are in clockwise order. */
				setdest(midtri, sortarray[2]);
				setorg(tri1, sortarray[2]);
				setdest(tri2, sortarray[2]);
				setapex(midtri, sortarray[1]);
				setorg(tri2, sortarray[1]);
				setdest(tri3, sortarray[1]);
			}
			/* The topology does not depend on how the vertices are ordered. */
			bond(midtri, tri1);
			lnextself(midtri);
			bond(midtri, tri2);
			lnextself(midtri);
			bond(midtri, tri3);
			lprevself(tri1);
			lnextself(tri2);
			bond(tri1, tri2);
			lprevself(tri1);
			lprevself(tri3);
			bond(tri1, tri3);
			lnextself(tri2);
			lprevself(tri3);
			bond(tri2, tri3);
			/* Ensure that the origin of `farleft' is sortarray[0]. */
			triedgecopy(tri1, *farleft);
			/* Ensure that the destination of `farright' is sortarray[2]. */
			if (area > 0.0) {
				triedgecopy(tri2, *farright);
			} else {
				lnext(*farleft, *farright);
			}
		}
		if (verbose > 2) {
			
			printtriangle(&midtri);
			
			printtriangle(&tri1);
			
			printtriangle(&tri2);
			
			printtriangle(&tri3);
		}
		return;
	} else {
		/* Split the vertices in half. */
		divider = vertices >> 1;
		/* Recursively triangulate each half. */
		divconqrecurse(sortarray, divider, 1 - axis, farleft, &innerleft);
		divconqrecurse(&sortarray[divider], vertices - divider, 1 - axis,
			&innerright, farright);
		if (verbose > 1) 
		{
			
		}
		/* Merge the two triangulations into one. */
		mergehulls(farleft, &innerleft, &innerright, farright, axis);
	}
}

long TriangleLib::removeghosts(struct triedge *startghost)
{
	struct triedge searchedge;
	struct triedge dissolveedge;
	struct triedge hulltri;
	struct triedge bordertri;
	struct triedge righttri;
	struct triedge firsttri;
	Bool bFirst;
	Bool bHull;
	point apexpt;
	long hullsize;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (verbose) {
		
	}
	/* Find an edge on the convex hull to start point location from. */
	lprev(*startghost, searchedge);
	symself(searchedge);
	dummytri[0] = encode(searchedge);
	/* Remove the bounding box and count the convex hull edges. */
	triedgecopy(*startghost, dissolveedge);
	triedgecopy(*startghost, firsttri);
	bFirst = false;
	hullsize = 0;
	do {
		lprevself(dissolveedge);
		triedgecopy(dissolveedge, hulltri);
		symself(dissolveedge);
		
		apex(dissolveedge, apexpt);
		if (apexpt != NULL)
		{
			bHull = true;
			
			while (SmallerAngle(dissolveedge))
			{
				lnextself(dissolveedge);
				sym(dissolveedge, bordertri);
				bond(hulltri, bordertri);
				lnextself(hulltri);
				sym(hulltri, righttri);
				
				bond(hulltri, dissolveedge);
				lprevself(dissolveedge);
				bond(dissolveedge, righttri);
				
				setdest(dissolveedge, NULL);
				
				lprevself(dissolveedge);
				triedgecopy(dissolveedge, hulltri);
				symself(dissolveedge);
				
				apex(dissolveedge, apexpt);
				
				if (apexpt == NULL)
				{
					bHull = false;
					break;
				}
			}

			if (bHull)
			{
				hullsize++;
			}
		}
		
		if (bFirst == false)
		{
			bFirst = true;
			lnext(hulltri, firsttri);
		}
		/* Remove a bounding triangle from a convex hull triangle. */
		dissolve(dissolveedge);
		/* Find the next bounding triangle. */
		lprevself(hulltri);
		sym(hulltri, dissolveedge);
		
		/* Delete the bounding triangle. */
		triangledealloc(hulltri.tri);
		
	} while (!triedgeequal(dissolveedge, firsttri));
	return hullsize;
}

Bool TriangleLib::SmallerAngle(struct  triedge& dissolveedge)
{
	point orgpt;
	point destpt;
	point apexpt;
	double dx, dy;
	double dTemp;
	double a, b, c;
	double a2, b2, c2;
	double dA, dB, dC;
	double dTolerance = 0.01;

	org(dissolveedge, orgpt);
	dest(dissolveedge, destpt);
	apex(dissolveedge, apexpt);

	dx = (double)(orgpt[0] - destpt[0]);
	dy = (double)(orgpt[1] - destpt[1]);
	a2 = dx*dx + dy*dy;
	a = sqrt(a2);
	dx = (double)(orgpt[0] - apexpt[0]);
	dy = (double)(orgpt[1] - apexpt[1]);
	b2 = dx*dx + dy*dy;
	b = sqrt(b2);
	dx = (double)(destpt[0] - apexpt[0]);
	dy = (double)(destpt[1] - apexpt[1]);
	c2 = dx*dx + dy*dy;
	c = sqrt(c2);

	dTemp = a*b;
	dA = acos((a2 + b2 - c2) / dTemp / 2) * RAD_TO_DEG;

	if (dA < dTolerance)
	{
		return true;
	}

	dTemp = a*c;
	dB = acos((a2 + c2 - b2) / dTemp / 2) * RAD_TO_DEG;

	if (dB < dTolerance)
	{
		return true;
	}

	dC = 180-dA-dB;
	
	if (dC < dTolerance)
	{
		return true;
	}

	return false;
}


/*****************************************************************************/
/*                                                                           */
/*  divconqdelaunay()   Form a Delaunay triangulation by the divide-and-     */
/*                      conquer method.                                      */
/*                                                                           */
/*  Sorts the points, calls a recursive procedure to triangulate them, and   */
/*  removes the bounding box, setting boundary markers as appropriate.       */
/*                                                                           */
/*****************************************************************************/
long TriangleLib::divconqdelaunay()
{
	point *sortarray;
	struct triedge hullleft, hullright;
	int divider;
	int i, j;
	
	/* Allocate an array of pointers to points for sorting. */
	sortarray = (point *) malloc(inpoints * sizeof(point));
	if (sortarray == (point *) NULL) {
		
		return -1;
	}
	traversalinit(&points);
	for (i = 0; i < inpoints; i++) {
		sortarray[i] = pointtraverse();
	}
	if (verbose) {
		
	}
	/* Sort the points. */
	pointsort(sortarray, inpoints);
	/* Discard duplicate points, which can really mess up the algorithm. */
	i = 0;
	for (j = 1; j < inpoints; j++) {
		if ((sortarray[i][0] == sortarray[j][0])
			&& (sortarray[i][1] == sortarray[j][1])) {
			if (!quiet) 
			{
				
			}
			/*  Commented out - would eliminate point from output .node file, but causes
			a failure if some segment has this point as an endpoint.
			setpointmark(sortarray[j], DEADPOINT);
			*/
			setpointmark(sortarray[j], QIUDUPLICATEPOINT);
		} else {
			i++;
			sortarray[i] = sortarray[j];
		}
	}
	i++;
	if (dwyer) {
		/* Re-sort the array of points to accommodate alternating cuts. */
		divider = i >> 1;
		if (i - divider >= 2) {
			if (divider >= 2) {
				alternateaxes(sortarray, divider, 1);
			}
			alternateaxes(&sortarray[divider], i - divider, 1);
		}
	}
	if (verbose) {
		
	}
	/* Form the Delaunay triangulation. */
	divconqrecurse(sortarray, i, 0, &hullleft, &hullright);
	free(sortarray);
	
	return removeghosts(&hullleft);
}

void TriangleLib::boundingbox()
{
	struct triedge inftri;          /* Handle for the triangular bounding box. */
	double width;
	
	if (verbose) {
		
	}
	/* Find the width (or height, whichever is larger) of the triangulation. */
	width = xmax - xmin;
	if (ymax - ymin > width) {
		width = ymax - ymin;
	}
	if (width == 0.0) {
		width = 1.0;
	}
	/* Create the vertices of the bounding box. */
	infpoint1 = (point) malloc(points.itembytes);
	infpoint2 = (point) malloc(points.itembytes);
	infpoint3 = (point) malloc(points.itembytes);
	if ((infpoint1 == (point) NULL) || (infpoint2 == (point) NULL)
		|| (infpoint3 == (point) NULL)) {
		
		return;
	}
	infpoint1[0] = xmin - 50.0 * width;
	infpoint1[1] = ymin - 40.0 * width;
	infpoint2[0] = xmax + 50.0 * width;
	infpoint2[1] = ymin - 40.0 * width;
	infpoint3[0] = 0.5 * (xmin + xmax);
	infpoint3[1] = ymax + 60.0 * width;
	
	/* Create the bounding box. */
	maketriangle(&inftri);
	setorg(inftri, infpoint1);
	setdest(inftri, infpoint2);
	setapex(inftri, infpoint3);
	/* Link dummytri to the bounding box so we can always find an */
	/*   edge to begin searching (point location) from.           */
	dummytri[0] = (triangle) inftri.tri;
	if (verbose > 2) {
		
		printtriangle(&inftri);
	}
}

long TriangleLib::removebox()
{
	struct triedge deadtri;
	struct triedge searchedge;
	struct triedge checkedge;
	struct triedge nextedge, finaledge, dissolveedge;
	point markorg;
	long hullsize;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (verbose) {
		
	}
	/* Find a boundary triangle. */
	nextedge.tri = dummytri;
	nextedge.orient = 0;
	symself(nextedge);
	/* Mark a place to stop. */
	lprev(nextedge, finaledge);
	lnextself(nextedge);
	symself(nextedge);
	/* Find a triangle (on the boundary of the point set) that isn't */
	/*   a bounding box triangle.                                    */
	lprev(nextedge, searchedge);
	symself(searchedge);
	/* Check whether nextedge is another boundary triangle */
	/*   adjacent to the first one.                        */
	lnext(nextedge, checkedge);
	symself(checkedge);
	if (checkedge.tri == dummytri) {
		/* Go on to the next triangle.  There are only three boundary   */
		/*   triangles, and this next triangle cannot be the third one, */
		/*   so it's safe to stop here.                                 */
		lprevself(searchedge);
		symself(searchedge);
	}
	/* Find a new boundary edge to search from, as the current search */
	/*   edge lies on a bounding box triangle and will be deleted.    */
	dummytri[0] = encode(searchedge);
	hullsize = -2l;
	while (!triedgeequal(nextedge, finaledge)) {
		hullsize++;
		lprev(nextedge, dissolveedge);
		symself(dissolveedge);
		/* If not using a PSLG, the vertices should be marked now. */
		/*   (If using a PSLG, markhull() will do the job.)        */
		if (!poly) {
			/* Be careful!  One must check for the case where all the input   */
			/*   points are collinear, and thus all the triangles are part of */
			/*   the bounding box.  Otherwise, the setpointmark() call below  */
			/*   will cause a bad pointer reference.                          */
			if (dissolveedge.tri != dummytri) {
				org(dissolveedge, markorg);
				if (pointmark(markorg) == 0) {
					setpointmark(markorg, 1);
				}
			}
		}
		/* Disconnect the bounding box triangle from the mesh triangle. */
		dissolve(dissolveedge);
		lnext(nextedge, deadtri);
		sym(deadtri, nextedge);
		/* Get rid of the bounding box triangle. */
		triangledealloc(deadtri.tri);
		/* Do we need to turn the corner? */
		if (nextedge.tri == dummytri) {
			/* Turn the corner. */
			triedgecopy(dissolveedge, nextedge);
		}
	}
	triangledealloc(finaledge.tri);
	
	free(infpoint1);                  /* Deallocate the bounding box vertices. */
	free(infpoint2);
	free(infpoint3);
	
	return hullsize;
}

long TriangleLib::incrementaldelaunay()
{
	struct triedge starttri;
	point pointloop;
	int i;
	
	/* Create a triangular bounding box. */
	boundingbox();
	if (verbose) {
		
	}
	traversalinit(&points);
	pointloop = pointtraverse();
	i = 1;
	while (pointloop != (point) NULL) {
		/* Find a boundary triangle to search from. */
		starttri.tri = (triangle *) NULL;
		if (insertsite(pointloop, &starttri, (struct edge *) NULL, 0, 0) ==
			DUPLICATEPOINT) {
			if (!quiet) 
			{
				
			}
			/*  Commented out - would eliminate point from output .node file.
			setpointmark(pointloop, DEADPOINT);
			*/
		}
		pointloop = pointtraverse();
		i++;
	}
	/* Remove the bounding box. */
	return removebox();
}

void TriangleLib::eventheapinsert(struct event **heap, int heapsize, struct event *newevent)
{
	double eventx, eventy;
	int eventnum;
	int parent;
	int notdone;
	
	eventx = newevent->xkey;
	eventy = newevent->ykey;
	eventnum = heapsize;
	notdone = eventnum > 0;
	while (notdone) {
		parent = (eventnum - 1) >> 1;
		if ((heap[parent]->ykey < eventy) ||
			((heap[parent]->ykey == eventy)
			&& (heap[parent]->xkey <= eventx))) {
			notdone = 0;
		} else {
			heap[eventnum] = heap[parent];
			heap[eventnum]->heapposition = eventnum;
			
			eventnum = parent;
			notdone = eventnum > 0;
		}
	}
	heap[eventnum] = newevent;
	newevent->heapposition = eventnum;
}

void TriangleLib::eventheapify(struct event **heap, int heapsize, int eventnum)
{
	struct event *thisevent;
	double eventx, eventy;
	int leftchild, rightchild;
	int smallest;
	int notdone;
	
	thisevent = heap[eventnum];
	eventx = thisevent->xkey;
	eventy = thisevent->ykey;
	leftchild = 2 * eventnum + 1;
	notdone = leftchild < heapsize;
	while (notdone) {
		if ((heap[leftchild]->ykey < eventy) ||
			((heap[leftchild]->ykey == eventy)
			&& (heap[leftchild]->xkey < eventx))) {
			smallest = leftchild;
		} else {
			smallest = eventnum;
		}
		rightchild = leftchild + 1;
		if (rightchild < heapsize) {
			if ((heap[rightchild]->ykey < heap[smallest]->ykey) ||
				((heap[rightchild]->ykey == heap[smallest]->ykey)
				&& (heap[rightchild]->xkey < heap[smallest]->xkey))) {
				smallest = rightchild;
			}
		}
		if (smallest == eventnum) {
			notdone = 0;
		} else {
			heap[eventnum] = heap[smallest];
			heap[eventnum]->heapposition = eventnum;
			heap[smallest] = thisevent;
			thisevent->heapposition = smallest;
			
			eventnum = smallest;
			leftchild = 2 * eventnum + 1;
			notdone = leftchild < heapsize;
		}
	}
}

void TriangleLib::eventheapdelete(struct event **heap, int heapsize, int eventnum)
{
	struct event *moveevent;
	double eventx, eventy;
	int parent;
	int notdone;
	
	moveevent = heap[heapsize - 1];
	if (eventnum > 0) {
		eventx = moveevent->xkey;
		eventy = moveevent->ykey;
		do {
			parent = (eventnum - 1) >> 1;
			if ((heap[parent]->ykey < eventy) ||
				((heap[parent]->ykey == eventy)
				&& (heap[parent]->xkey <= eventx))) {
				notdone = 0;
			} else {
				heap[eventnum] = heap[parent];
				heap[eventnum]->heapposition = eventnum;
				
				eventnum = parent;
				notdone = eventnum > 0;
			}
		} while (notdone);
	}
	heap[eventnum] = moveevent;
	moveevent->heapposition = eventnum;
	eventheapify(heap, heapsize - 1, eventnum);
}

void TriangleLib::createeventheap(struct event ***eventheap, struct event **events, struct event **freeevents)
{
	point thispoint;
	int maxevents;
	int i;
	
	maxevents = (3 * inpoints) / 2;
	*eventheap = (struct event **) malloc(maxevents * sizeof(struct event *));
	if (*eventheap == (struct event **) NULL) {
		
		return;
	}
	*events = (struct event *) malloc(maxevents * sizeof(struct event));
	if (*events == (struct event *) NULL) {
		
		return;
	}
	traversalinit(&points);
	for (i = 0; i < inpoints; i++) {
		thispoint = pointtraverse();
		(*events)[i].eventptr = (int *) thispoint;
		(*events)[i].xkey = thispoint[0];
		(*events)[i].ykey = thispoint[1];
		eventheapinsert(*eventheap, i, *events + i);
	}
	*freeevents = (struct event *) NULL;
	for (i = maxevents - 1; i >= inpoints; i--) {
		(*events)[i].eventptr = (int *) *freeevents;
		*freeevents = *events + i;
	}
}

int TriangleLib::rightofhyperbola(struct triedge *fronttri, point newsite)
{
	point leftpoint, rightpoint;
	double dxa, dya, dxb, dyb;
	
	hyperbolacount++;
	
	dest(*fronttri, leftpoint);
	apex(*fronttri, rightpoint);
	if ((leftpoint[1] < rightpoint[1])
		|| ((leftpoint[1] == rightpoint[1]) && (leftpoint[0] < rightpoint[0]))) {
		if (newsite[0] >= rightpoint[0]) {
			return 1;
		}
	} else {
		if (newsite[0] <= leftpoint[0]) {
			return 0;
		}
	}
	dxa = leftpoint[0] - newsite[0];
	dya = leftpoint[1] - newsite[1];
	dxb = rightpoint[0] - newsite[0];
	dyb = rightpoint[1] - newsite[1];
	return dya * (dxb * dxb + dyb * dyb) > dyb * (dxa * dxa + dya * dya);
}

double TriangleLib::circletop(point pa, point pb, point pc, double ccwabc)
{
	double xac, yac, xbc, ybc, xab, yab;
	double aclen2, bclen2, ablen2;
	
	circletopcount++;
	
	xac = pa[0] - pc[0];
	yac = pa[1] - pc[1];
	xbc = pb[0] - pc[0];
	ybc = pb[1] - pc[1];
	xab = pa[0] - pb[0];
	yab = pa[1] - pb[1];
	aclen2 = xac * xac + yac * yac;
	bclen2 = xbc * xbc + ybc * ybc;
	ablen2 = xab * xab + yab * yab;
	return pc[1] + (xac * bclen2 - xbc * aclen2 + sqrt(aclen2 * bclen2 * ablen2))
		/ (2.0 * ccwabc);
}

void TriangleLib::check4deadevent(struct triedge *checktri, struct event **freeevents, struct event **eventheap, int *heapsize)
{
	struct event *deadevent;
	point eventpoint;
	int eventnum;
	
	org(*checktri, eventpoint);
	if (eventpoint != (point) NULL) {
		deadevent = (struct event *) eventpoint;
		eventnum = deadevent->heapposition;
		deadevent->eventptr = (int *) *freeevents;
		*freeevents = deadevent;
		eventheapdelete(eventheap, *heapsize, eventnum);
		(*heapsize)--;
		setorg(*checktri, NULL);
	}
}

struct splaynode * TriangleLib::splay(struct splaynode *splaytree, point searchpoint, struct triedge *searchtri)
{
	struct splaynode *child, *grandchild;
	struct splaynode *lefttree, *righttree;
	struct splaynode *leftright;
	point checkpoint;
	int rightofroot, rightofchild;
	
	if (splaytree == (struct splaynode *) NULL) {
		return (struct splaynode *) NULL;
	}
	dest(splaytree->keyedge, checkpoint);
	if (checkpoint == splaytree->keydest) {
		rightofroot = rightofhyperbola(&splaytree->keyedge, searchpoint);
		if (rightofroot) {
			triedgecopy(splaytree->keyedge, *searchtri);
			child = splaytree->rchild;
		} else {
			child = splaytree->lchild;
		}
		if (child == (struct splaynode *) NULL) {
			return splaytree;
		}
		dest(child->keyedge, checkpoint);
		if (checkpoint != child->keydest) {
			child = splay(child, searchpoint, searchtri);
			if (child == (struct splaynode *) NULL) {
				if (rightofroot) {
					splaytree->rchild = (struct splaynode *) NULL;
				} else {
					splaytree->lchild = (struct splaynode *) NULL;
				}
				return splaytree;
			}
		}
		rightofchild = rightofhyperbola(&child->keyedge, searchpoint);
		if (rightofchild) {
			triedgecopy(child->keyedge, *searchtri);
			grandchild = splay(child->rchild, searchpoint, searchtri);
			child->rchild = grandchild;
		} else {
			grandchild = splay(child->lchild, searchpoint, searchtri);
			child->lchild = grandchild;
		}
		if (grandchild == (struct splaynode *) NULL) {
			if (rightofroot) {
				splaytree->rchild = child->lchild;
				child->lchild = splaytree;
			} else {
				splaytree->lchild = child->rchild;
				child->rchild = splaytree;
			}
			return child;
		}
		if (rightofchild) {
			if (rightofroot) {
				splaytree->rchild = child->lchild;
				child->lchild = splaytree;
			} else {
				splaytree->lchild = grandchild->rchild;
				grandchild->rchild = splaytree;
			}
			child->rchild = grandchild->lchild;
			grandchild->lchild = child;
		} else {
			if (rightofroot) {
				splaytree->rchild = grandchild->lchild;
				grandchild->lchild = splaytree;
			} else {
				splaytree->lchild = child->rchild;
				child->rchild = splaytree;
			}
			child->lchild = grandchild->rchild;
			grandchild->rchild = child;
		}
		return grandchild;
	} else {
		lefttree = splay(splaytree->lchild, searchpoint, searchtri);
		righttree = splay(splaytree->rchild, searchpoint, searchtri);
		
		pooldealloc(&splaynodes, (int *) splaytree);
		if (lefttree == (struct splaynode *) NULL) {
			return righttree;
		} else if (righttree == (struct splaynode *) NULL) {
			return lefttree;
		} else if (lefttree->rchild == (struct splaynode *) NULL) {
			lefttree->rchild = righttree->lchild;
			righttree->lchild = lefttree;
			return righttree;
		} else if (righttree->lchild == (struct splaynode *) NULL) {
			righttree->lchild = lefttree->rchild;
			lefttree->rchild = righttree;
			return lefttree;
		} else {
			
			leftright = lefttree->rchild;
			while (leftright->rchild != (struct splaynode *) NULL) {
				leftright = leftright->rchild;
			}
			leftright->rchild = righttree;
			return lefttree;
		}
	}
}

struct splaynode * TriangleLib::splayinsert(struct splaynode *splayroot, struct triedge *newkey, point searchpoint)
{
	struct splaynode *newsplaynode;
	
	newsplaynode = (struct splaynode *) poolalloc(&splaynodes);
	triedgecopy(*newkey, newsplaynode->keyedge);
	dest(*newkey, newsplaynode->keydest);
	if (splayroot == (struct splaynode *) NULL) {
		newsplaynode->lchild = (struct splaynode *) NULL;
		newsplaynode->rchild = (struct splaynode *) NULL;
	} else if (rightofhyperbola(&splayroot->keyedge, searchpoint)) {
		newsplaynode->lchild = splayroot;
		newsplaynode->rchild = splayroot->rchild;
		splayroot->rchild = (struct splaynode *) NULL;
	} else {
		newsplaynode->lchild = splayroot->lchild;
		newsplaynode->rchild = splayroot;
		splayroot->lchild = (struct splaynode *) NULL;
	}
	return newsplaynode;
}

struct splaynode * TriangleLib::circletopinsert(struct splaynode *splayroot, struct triedge *newkey, 
												 point pa,point pb, point pc, double topy)
{
	double ccwabc;
	double xac, yac, xbc, ybc;
	double aclen2, bclen2;
	double searchpoint[2];
	struct triedge dummytri;
	
	ccwabc = counterclockwise(pa, pb, pc);
	xac = pa[0] - pc[0];
	yac = pa[1] - pc[1];
	xbc = pb[0] - pc[0];
	ybc = pb[1] - pc[1];
	aclen2 = xac * xac + yac * yac;
	bclen2 = xbc * xbc + ybc * ybc;
	searchpoint[0] = pc[0] - (yac * bclen2 - ybc * aclen2) / (2.0 * ccwabc);
	searchpoint[1] = topy;
	return splayinsert(splay(splayroot, (point) searchpoint, &dummytri), newkey,
		(point) searchpoint);
}


struct splaynode * TriangleLib::frontlocate(struct splaynode *splayroot, struct triedge *bottommost, point searchpoint,
											 struct triedge *searchtri,int *farright)
{
	int farrightflag;
	triangle ptr;                       /* Temporary variable used by onext(). */
	
	triedgecopy(*bottommost, *searchtri);
	splayroot = splay(splayroot, searchpoint, searchtri);
	
	farrightflag = 0;
	while (!farrightflag && rightofhyperbola(searchtri, searchpoint)) {
		onextself(*searchtri);
		farrightflag = triedgeequal(*searchtri, *bottommost);
	}
	*farright = farrightflag;
	return splayroot;
}

long TriangleLib::sweeplinedelaunay()
{
	struct event **eventheap;
	struct event *events;
	struct event *freeevents;
	struct event *nextevent;
	struct event *newevent;
	struct splaynode *splayroot;
	struct triedge bottommost;
	struct triedge searchtri;
	struct triedge fliptri;
	struct triedge lefttri, righttri, farlefttri, farrighttri;
	struct triedge inserttri;
	point firstpoint, secondpoint;
	point nextpoint, lastpoint;
	point connectpoint;
	point leftpoint, midpoint, rightpoint;
	double lefttest, righttest;
	int heapsize;
	int check4events, farrightflag;
	triangle ptr;   /* Temporary variable used by sym(), onext(), and oprev(). */
	
	poolinit(&splaynodes, sizeof(struct splaynode), SPLAYNODEPERBLOCK, POINTER,
		0);
	splayroot = (struct splaynode *) NULL;
	
	if (verbose) {
		
	}
	createeventheap(&eventheap, &events, &freeevents);
	heapsize = inpoints;
	
	if (verbose) {
		
	}
	maketriangle(&lefttri);
	maketriangle(&righttri);
	bond(lefttri, righttri);
	lnextself(lefttri);
	lprevself(righttri);
	bond(lefttri, righttri);
	lnextself(lefttri);
	lprevself(righttri);
	bond(lefttri, righttri);
	firstpoint = (point) eventheap[0]->eventptr;
	eventheap[0]->eventptr = (int *) freeevents;
	freeevents = eventheap[0];
	eventheapdelete(eventheap, heapsize, 0);
	heapsize--;
	do {
		if (heapsize == 0) {
			
			return -1;
		}
		secondpoint = (point) eventheap[0]->eventptr;
		eventheap[0]->eventptr = (int *) freeevents;
		freeevents = eventheap[0];
		eventheapdelete(eventheap, heapsize, 0);
		heapsize--;
		if ((firstpoint[0] == secondpoint[0])
			&& (firstpoint[1] == secondpoint[1])) 
		{
			
		}
	} while ((firstpoint[0] == secondpoint[0])
		&& (firstpoint[1] == secondpoint[1]));
	setorg(lefttri, firstpoint);
	setdest(lefttri, secondpoint);
	setorg(righttri, secondpoint);
	setdest(righttri, firstpoint);
	lprev(lefttri, bottommost);
	lastpoint = secondpoint;
	while (heapsize > 0) {
		nextevent = eventheap[0];
		eventheapdelete(eventheap, heapsize, 0);
		heapsize--;
		check4events = 1;
		if (nextevent->xkey < xmin) {
			decode(nextevent->eventptr, fliptri);
			oprev(fliptri, farlefttri);
			check4deadevent(&farlefttri, &freeevents, eventheap, &heapsize);
			onext(fliptri, farrighttri);
			check4deadevent(&farrighttri, &freeevents, eventheap, &heapsize);
			
			if (triedgeequal(farlefttri, bottommost)) {
				lprev(fliptri, bottommost);
			}
			flip(&fliptri);
			setapex(fliptri, NULL);
			lprev(fliptri, lefttri);
			lnext(fliptri, righttri);
			sym(lefttri, farlefttri);
			
			if (randomnation(SAMPLERATE) == 0) {
				symself(fliptri);
				dest(fliptri, leftpoint);
				apex(fliptri, midpoint);
				org(fliptri, rightpoint);
				splayroot = circletopinsert(splayroot, &lefttri, leftpoint, midpoint,
					rightpoint, nextevent->ykey);
			}
		} else {
			nextpoint = (point) nextevent->eventptr;
			if ((nextpoint[0] == lastpoint[0]) && (nextpoint[1] == lastpoint[1])) 
			{
				
				check4events = 0;
			} else {
				lastpoint = nextpoint;
				
				splayroot = frontlocate(splayroot, &bottommost, nextpoint, &searchtri,
					&farrightflag);
					/*
					triedgecopy(bottommost, searchtri);
					farrightflag = 0;
					while (!farrightflag && rightofhyperbola(&searchtri, nextpoint)) {
					onextself(searchtri);
					farrightflag = triedgeequal(searchtri, bottommost);
					}
				*/
				
				check4deadevent(&searchtri, &freeevents, eventheap, &heapsize);
				
				triedgecopy(searchtri, farrighttri);
				sym(searchtri, farlefttri);
				maketriangle(&lefttri);
				maketriangle(&righttri);
				dest(farrighttri, connectpoint);
				setorg(lefttri, connectpoint);
				setdest(lefttri, nextpoint);
				setorg(righttri, nextpoint);
				setdest(righttri, connectpoint);
				bond(lefttri, righttri);
				lnextself(lefttri);
				lprevself(righttri);
				bond(lefttri, righttri);
				lnextself(lefttri);
				lprevself(righttri);
				bond(lefttri, farlefttri);
				bond(righttri, farrighttri);
				if (!farrightflag && triedgeequal(farrighttri, bottommost)) {
					triedgecopy(lefttri, bottommost);
				}
				
				if (randomnation(SAMPLERATE) == 0) {
					splayroot = splayinsert(splayroot, &lefttri, nextpoint);
				} else if (randomnation(SAMPLERATE) == 0) {
					lnext(righttri, inserttri);
					splayroot = splayinsert(splayroot, &inserttri, nextpoint);
				}
			}
		}
		nextevent->eventptr = (int *) freeevents;
		freeevents = nextevent;
		
		if (check4events) {
			apex(farlefttri, leftpoint);
			dest(lefttri, midpoint);
			apex(lefttri, rightpoint);
			lefttest = counterclockwise(leftpoint, midpoint, rightpoint);
			if (lefttest > 0.0) {
				newevent = freeevents;
				freeevents = (struct event *) freeevents->eventptr;
				newevent->xkey = xminextreme;
				newevent->ykey = circletop(leftpoint, midpoint, rightpoint,
					lefttest);
				newevent->eventptr = (int *) encode(lefttri);
				eventheapinsert(eventheap, heapsize, newevent);
				heapsize++;
				setorg(lefttri, newevent);
			}
			apex(righttri, leftpoint);
			org(righttri, midpoint);
			apex(farrighttri, rightpoint);
			righttest = counterclockwise(leftpoint, midpoint, rightpoint);
			if (righttest > 0.0) {
				newevent = freeevents;
				freeevents = (struct event *) freeevents->eventptr;
				newevent->xkey = xminextreme;
				newevent->ykey = circletop(leftpoint, midpoint, rightpoint,
					righttest);
				newevent->eventptr = (int *) encode(farrighttri);
				eventheapinsert(eventheap, heapsize, newevent);
				heapsize++;
				setorg(farrighttri, newevent);
			}
		}
  }
  
  pooldeinit(&splaynodes);
  lprevself(bottommost);
  return removeghosts(&bottommost);
}

long TriangleLib::delaunay()
{
	eextras = 0;
	initializetrisegpools();
	
#ifdef REDUCED
	if (!quiet) {
		
	}
	return divconqdelaunay();
#else /* not REDUCED */
	if (!quiet)
	{
		
	}
	if (incremental) {
		return incrementaldelaunay();
	} else if (sweepline) {
		return sweeplinedelaunay();
	} else {
		return divconqdelaunay();
	}
#endif /* not REDUCED */
}

int TriangleLib::reconstruct(int *trianglelist, double *triangleattriblist, double *trianglearealist, int elements,
							  int corners, int attribs, int *segmentlist, int *segmentmarkerlist,int numberofsegments)
{
	int pointindex;
	int attribindex;
	struct triedge triangleloop;
	struct triedge triangleleft;
	struct triedge checktri;
	struct triedge checkleft;
	struct triedge checkneighbor;
	struct edge shelleloop;
	triangle *vertexarray;
	triangle *prevlink;
	triangle nexttri;
	point tdest, tapex;
	point checkdest, checkapex;
	point shorg;
	point killpoint;
	double area;
	int corner[3];
	int end[2];
	int killpointindex;
	int incorners;
	int segmentmarkers = 0;
	int boundmarker;
	int aroundpoint;
	long hullsize;
	int notfound;
	int elementnumber, segmentnumber;
	int i, j;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	inelements = elements;
	incorners = corners;
	if (incorners < 3) {
		
		return -1;

	}
	eextras = attribs;
	
	initializetrisegpools();
	
	/* Create the triangles. */
	for (elementnumber = 1; elementnumber <= inelements; elementnumber++) {
		maketriangle(&triangleloop);
		/* Mark the triangle as living. */
		triangleloop.tri[3] = (triangle) triangleloop.tri;
	}
	
	if (poly) {
		insegments = numberofsegments;
		segmentmarkers = segmentmarkerlist != (int *) NULL;
		
		/* Create the shell edges. */
		for (segmentnumber = 1; segmentnumber <= insegments; segmentnumber++) {
			makeshelle(&shelleloop);
			/* Mark the shell edge as living. */
			shelleloop.sh[2] = (shelle) shelleloop.sh;
		}
	}
	
	pointindex = 0;
	attribindex = 0;
	
	if (!quiet) {
		
	}
	/* Allocate a temporary array that maps each point to some adjacent  */
	/*   triangle.  I took care to allocate all the permanent memory for */
	/*   triangles and shell edges first.                                */
	vertexarray = (triangle *) malloc(points.items * sizeof(triangle));
	if (vertexarray == (triangle *) NULL) {
		
		return -1;
	}
	/* Each point is initially unrepresented. */
	for (i = 0; i < points.items; i++) {
		vertexarray[i] = (triangle) dummytri;
	}
	
	if (verbose) {
		
	}
	/* Read the triangles from the .ele file, and link */
	/*   together those that share an edge.            */
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	elementnumber = firstnumber;
	while (triangleloop.tri != (triangle *) NULL) {
		/* Copy the triangle's three corners. */
		for (j = 0; j < 3; j++) {
			corner[j] = trianglelist[pointindex++];
			if ((corner[j] < firstnumber) || (corner[j] >= firstnumber + inpoints))
			{
				
				return -1;
			}
		}
		
		/* Find out about (and throw away) extra nodes. */
		for (j = 3; j < incorners; j++) {
			
			killpointindex = trianglelist[pointindex++];
			if ((killpointindex >= firstnumber) &&
				(killpointindex < firstnumber + inpoints)) {
				/* Delete the non-corner point if it's not already deleted. */
				killpoint = getpoint(killpointindex);
				if (pointmark(killpoint) != DEADPOINT) {
					pointdealloc(killpoint);
				}
			}
		}
		
		/* Read the triangle's attributes. */
		for (j = 0; j < eextras; j++) {
			setelemattribute(triangleloop, j, triangleattriblist[attribindex++]);
		}
		
		if (vararea) {
			area = trianglearealist[elementnumber - firstnumber];
			setareabound(triangleloop, area);
		}
		
		/* Set the triangle's vertices. */
		triangleloop.orient = 0;
		setorg(triangleloop, getpoint(corner[0]));
		setdest(triangleloop, getpoint(corner[1]));
		setapex(triangleloop, getpoint(corner[2]));
		/* Try linking the triangle to others that share these vertices. */
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			/* Take the number for the origin of triangleloop. */
			aroundpoint = corner[triangleloop.orient];
			/* Look for other triangles having this vertex. */
			nexttri = vertexarray[aroundpoint - firstnumber];
			/* Link the current triangle to the next one in the stack. */
			triangleloop.tri[6 + triangleloop.orient] = nexttri;
			/* Push the current triangle onto the stack. */
			vertexarray[aroundpoint - firstnumber] = encode(triangleloop);
			decode(nexttri, checktri);
			if (checktri.tri != dummytri) {
				dest(triangleloop, tdest);
				apex(triangleloop, tapex);
				/* Look for other triangles that share an edge. */
				do {
					dest(checktri, checkdest);
					apex(checktri, checkapex);
					if (tapex == checkdest) {
						/* The two triangles share an edge; bond them together. */
						lprev(triangleloop, triangleleft);
						bond(triangleleft, checktri);
					}
					if (tdest == checkapex) {
						/* The two triangles share an edge; bond them together. */
						lprev(checktri, checkleft);
						bond(triangleloop, checkleft);
					}
					/* Find the next triangle in the stack. */
					nexttri = checktri.tri[6 + checktri.orient];
					decode(nexttri, checktri);
				} while (checktri.tri != dummytri);
			}
		}
		triangleloop.tri = triangletraverse();
		elementnumber++;
	}
	pointindex = 0;
	
	hullsize = 0;                      /* Prepare to count the boundary edges. */
	if (poly) {
		if (verbose) {
			
		}
		/* Read the segments from the .poly file, and link them */
		/*   to their neighboring triangles.                    */
		boundmarker = 0;
		traversalinit(&shelles);
		shelleloop.sh = shelletraverse();
		segmentnumber = firstnumber;
		while (shelleloop.sh != (shelle *) NULL) {
			end[0] = segmentlist[pointindex++];
			end[1] = segmentlist[pointindex++];
			if (segmentmarkers) {
				boundmarker = segmentmarkerlist[segmentnumber - firstnumber];
			}
			for (j = 0; j < 2; j++) {
				if ((end[j] < firstnumber) || (end[j] >= firstnumber + inpoints)) 
				{
					
					return -1;
				}
			}
			
			/* set the shell edge's vertices. */
			shelleloop.shorient = 0;
			setsorg(shelleloop, getpoint(end[0]));
			setsdest(shelleloop, getpoint(end[1]));
			setmark(shelleloop, boundmarker);
			/* Try linking the shell edge to triangles that share these vertices. */
			for (shelleloop.shorient = 0; shelleloop.shorient < 2;
			shelleloop.shorient++) {
				/* Take the number for the destination of shelleloop. */
				aroundpoint = end[1 - shelleloop.shorient];
				/* Look for triangles having this vertex. */
				prevlink = &vertexarray[aroundpoint - firstnumber];
				nexttri = vertexarray[aroundpoint - firstnumber];
				decode(nexttri, checktri);
				sorg(shelleloop, shorg);
				notfound = 1;
				/* Look for triangles having this edge.  Note that I'm only       */
				/*   comparing each triangle's destination with the shell edge;   */
				/*   each triangle's apex is handled through a different vertex.  */
				/*   Because each triangle appears on three vertices' lists, each */
				/*   occurrence of a triangle on a list can (and does) represent  */
				/*   an edge.  In this way, most edges are represented twice, and */
				/*   every triangle-segment bond is represented once.             */
				while (notfound && (checktri.tri != dummytri)) {
					dest(checktri, checkdest);
					if (shorg == checkdest) {
						/* We have a match.  Remove this triangle from the list. */
						*prevlink = checktri.tri[6 + checktri.orient];
						/* Bond the shell edge to the triangle. */
						tsbond(checktri, shelleloop);
						/* Check if this is a boundary edge. */
						sym(checktri, checkneighbor);
						if (checkneighbor.tri == dummytri) {
							/* The next line doesn't insert a shell edge (because there's */
							/*   already one there), but it sets the boundary markers of  */
							/*   the existing shell edge and its vertices.                */
							insertshelle(&checktri, 1);
							hullsize++;
						}
						notfound = 0;
					}
					/* Find the next triangle in the stack. */
					prevlink = &checktri.tri[6 + checktri.orient];
					nexttri = checktri.tri[6 + checktri.orient];
					decode(nexttri, checktri);
				}
			}
			shelleloop.sh = shelletraverse();
			segmentnumber++;
		}
	}
	
	/* Mark the remaining edges as not being attached to any shell edge. */
	/* Also, count the (yet uncounted) boundary edges.                   */
	for (i = 0; i < points.items; i++) {
		/* Search the stack of triangles adjacent to a point. */
		nexttri = vertexarray[i];
		decode(nexttri, checktri);
		while (checktri.tri != dummytri) {
			/* Find the next triangle in the stack before this */
			/*   information gets overwritten.                 */
			nexttri = checktri.tri[6 + checktri.orient];
			/* No adjacent shell edge.  (This overwrites the stack info.) */
			tsdissolve(checktri);
			sym(checktri, checkneighbor);
			if (checkneighbor.tri == dummytri) {
				insertshelle(&checktri, 1);
				hullsize++;
			}
			decode(nexttri, checktri);
		}
	}
	
	free(vertexarray);
	return hullsize;
}

enum finddirectionresult TriangleLib::finddirection(struct triedge *searchtri, point endpoint)
{
	struct triedge checktri;
	point startpoint;
	point leftpoint, rightpoint;
	double leftccw, rightccw;
	int leftflag, rightflag;
	triangle ptr;           /* Temporary variable used by onext() and oprev(). */
	
	org(*searchtri, startpoint);
	dest(*searchtri, rightpoint);
	apex(*searchtri, leftpoint);
	/* Is `endpoint' to the left? */
	leftccw = counterclockwise(endpoint, startpoint, leftpoint);
	leftflag = leftccw > 0.0;
	/* Is `endpoint' to the right? */
	rightccw = counterclockwise(startpoint, endpoint, rightpoint);
	rightflag = rightccw > 0.0;
	if (leftflag && rightflag) {
		/* `searchtri' faces directly away from `endpoint'.  We could go */
		/*   left or right.  Ask whether it's a triangle or a boundary   */
		/*   on the left.                                                */
		onext(*searchtri, checktri);
		if (checktri.tri == dummytri) {
			leftflag = 0;
		} else {
			rightflag = 0;
		}
	}
	while (leftflag) {
		/* Turn left until satisfied. */
		onextself(*searchtri);
		if (searchtri->tri == dummytri) 
		{
			
			internalerror();
		}
		apex(*searchtri, leftpoint);
		rightccw = leftccw;
		leftccw = counterclockwise(endpoint, startpoint, leftpoint);
		leftflag = leftccw > 0.0;
	}
	while (rightflag) {
		/* Turn right until satisfied. */
		oprevself(*searchtri);
		if (searchtri->tri == dummytri) 
		{
			
			internalerror();
		}
		dest(*searchtri, rightpoint);
		leftccw = rightccw;
		rightccw = counterclockwise(startpoint, endpoint, rightpoint);
		rightflag = rightccw > 0.0;
	}
	if (leftccw == 0.0) {
		return LEFTCOLLINEAR;
	} else if (rightccw == 0.0) {
		return RIGHTCOLLINEAR;
	} else {
		return WITHIN;
	}
}

void TriangleLib::segmentintersection(struct triedge *splittri, struct edge *splitshelle,point endpoint2)
{
	point endpoint1;
	point torg, tdest;
	point leftpoint, rightpoint;
	point newpoint;
	enum insertsiteresult success;
	enum finddirectionresult collinear;
	double ex, ey;
	double tx, ty;
	double etx, ety;
	double split, denom;
	int i;
	triangle ptr;                       /* Temporary variable used by onext(). */
	
	/* Find the other three segment endpoints. */
	apex(*splittri, endpoint1);
	org(*splittri, torg);
	dest(*splittri, tdest);
	/* Segment intersection formulae; see the Antonio reference. */
	tx = tdest[0] - torg[0];
	ty = tdest[1] - torg[1];
	ex = endpoint2[0] - endpoint1[0];
	ey = endpoint2[1] - endpoint1[1];
	etx = torg[0] - endpoint2[0];
	ety = torg[1] - endpoint2[1];
	denom = ty * ex - tx * ey;
	if (denom == 0.0) {
		
		internalerror();
	}
	split = (ey * etx - ex * ety) / denom;
	/* Create the new point. */
	newpoint = (point) poolalloc(&points);
	/* Interpolate its coordinate and attributes. */
	for (i = 0; i < 2 + nextras; i++) {
		newpoint[i] = torg[i] + split * (tdest[i] - torg[i]);
	}
	setpointmark(newpoint, mark(*splitshelle));
	if (verbose > 1)
	{
		
	}
	/* Insert the intersection point.  This should always succeed. */
	success = insertsite(newpoint, splittri, splitshelle, 0, 0);
	if (success != SUCCESSFULPOINT) {
		
		internalerror();
	}
	if (steinerleft > 0) {
		steinerleft--;
	}
	/* Inserting the point may have caused edge flips.  We wish to rediscover */
	/*   the edge connecting endpoint1 to the new intersection point.         */
	collinear = finddirection(splittri, endpoint1);
	dest(*splittri, rightpoint);
	apex(*splittri, leftpoint);
	if ((leftpoint[0] == endpoint1[0]) && (leftpoint[1] == endpoint1[1])) {
		onextself(*splittri);
	} else if ((rightpoint[0] != endpoint1[0]) ||
		(rightpoint[1] != endpoint1[1])) {
		
		internalerror();
	}
}

int TriangleLib::scoutsegment(struct triedge *searchtri, point endpoint2, int newmark)
{
	struct triedge crosstri;
	struct edge crossedge;
	point leftpoint, rightpoint;
	point endpoint1;
	enum finddirectionresult collinear;
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	collinear = finddirection(searchtri, endpoint2);
	dest(*searchtri, rightpoint);
	apex(*searchtri, leftpoint);
	if ( ( fabs(leftpoint[0] - endpoint2[0]) < 1.0e-5 && fabs(leftpoint[1] - endpoint2[1]) < 1.0e-5 ) ||
		 ( fabs(rightpoint[0] - endpoint2[0]) < 1.0e-5  && fabs(rightpoint[1] - endpoint2[1]) < 1.0e-5) ) 
	{
		/* The segment is already an edge in the mesh. */
		if ( fabs(leftpoint[0] - endpoint2[0]) < 1.0e-5 && fabs(leftpoint[1] - endpoint2[1]) < 1.0e-5 ) 
		{
			lprevself(*searchtri);
		}
		/* Insert a shell edge, if there isn't already one there. */
		insertshelle(searchtri, newmark);
		return 1;
	} else if (collinear == LEFTCOLLINEAR) {
		/* We've collided with a point between the segment's endpoints. */
		/* Make the collinear point be the triangle's origin. */
		lprevself(*searchtri);
		insertshelle(searchtri, newmark);
		/* Insert the remainder of the segment. */
		return scoutsegment(searchtri, endpoint2, newmark);
	} else if (collinear == RIGHTCOLLINEAR) {
		/* We've collided with a point between the segment's endpoints. */
		insertshelle(searchtri, newmark);
		/* Make the collinear point be the triangle's origin. */
		lnextself(*searchtri);
		/* Insert the remainder of the segment. */
		return scoutsegment(searchtri, endpoint2, newmark);
	} else {
		lnext(*searchtri, crosstri);
		tspivot(crosstri, crossedge);
		/* Check for a crossing segment. */
		if (crossedge.sh == dummysh) {
			return 0;
		} else {
			org(*searchtri, endpoint1);
			/* Insert a point at the intersection. */
			segmentintersection(&crosstri, &crossedge, endpoint2);
			triedgecopy(crosstri, *searchtri);
			insertshelle(searchtri, newmark);
			/* Insert the remainder of the segment. */
			return scoutsegment(searchtri, endpoint2, newmark);
		}
	}
}

void TriangleLib::conformingedge(point endpoint1, point endpoint2, int newmark)
{
	struct triedge searchtri1, searchtri2;
	struct edge brokenshelle;
	point newpoint;
	point midpoint1, midpoint2;
	enum insertsiteresult success;
	int result1, result2;
	int i;
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (verbose > 2) 
	{
		
	}
	/* Create a new point to insert in the middle of the segment. */
	newpoint = (point) poolalloc(&points);
	/* Interpolate coordinates and attributes. */
	for (i = 0; i < 2 + nextras; i++) {
		newpoint[i] = 0.5 * (endpoint1[i] + endpoint2[i]);
	}
	setpointmark(newpoint, newmark);
	/* Find a boundary triangle to search from. */
	searchtri1.tri = (triangle *) NULL;
	/* Attempt to insert the new point. */
	success = insertsite(newpoint, &searchtri1, (struct edge *) NULL, 0, 0);
	if (success == DUPLICATEPOINT) {
		if (verbose > 2) 
		{
			
		}
		/* Use the point that's already there. */
		pointdealloc(newpoint);
		org(searchtri1, newpoint);
	} else {
		if (success == VIOLATINGPOINT) {
			if (verbose > 2) 
			{
				
			}
			/* By fluke, we've landed right on another segment.  Split it. */
			tspivot(searchtri1, brokenshelle);
			success = insertsite(newpoint, &searchtri1, &brokenshelle, 0, 0);
			if (success != SUCCESSFULPOINT) {
				
				internalerror();
			}
		}
		/* The point has been inserted successfully. */
		if (steinerleft > 0) {
			steinerleft--;
		}
	}
	triedgecopy(searchtri1, searchtri2);
	result1 = scoutsegment(&searchtri1, endpoint1, newmark);
	result2 = scoutsegment(&searchtri2, endpoint2, newmark);
	if (!result1) {
		/* The origin of searchtri1 may have changed if a collision with an */
		/*   intervening vertex on the segment occurred.                    */
		org(searchtri1, midpoint1);
		conformingedge(midpoint1, endpoint1, newmark);
	}
	if (!result2) {
		/* The origin of searchtri2 may have changed if a collision with an */
		/*   intervening vertex on the segment occurred.                    */
		org(searchtri2, midpoint2);
		conformingedge(midpoint2, endpoint2, newmark);
	}
}

void TriangleLib::delaunayfixup(struct triedge *fixuptri, int leftside)
{
	struct triedge neartri;
	struct triedge fartri;
	struct edge faredge;
	point nearpoint, leftpoint, rightpoint, farpoint;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	lnext(*fixuptri, neartri);
	sym(neartri, fartri);
	/* Check if the edge opposite the origin of fixuptri can be flipped. */
	if (fartri.tri == dummytri) {
		return;
	}
	tspivot(neartri, faredge);
	if (faredge.sh != dummysh) {
		return;
	}
	/* Find all the relevant vertices. */
	apex(neartri, nearpoint);
	org(neartri, leftpoint);
	dest(neartri, rightpoint);
	apex(fartri, farpoint);
	/* Check whether the previous polygon vertex is a reflex vertex. */
	if (leftside) {
		if (counterclockwise(nearpoint, leftpoint, farpoint) <= 0.0) {
			/* leftpoint is a reflex vertex too.  Nothing can */
			/*   be done until a convex section is found.     */
			return;
		}
	} else {
		if (counterclockwise(farpoint, rightpoint, nearpoint) <= 0.0) {
			/* rightpoint is a reflex vertex too.  Nothing can */
			/*   be done until a convex section is found.      */
			return;
		}
	}
	if (counterclockwise(rightpoint, leftpoint, farpoint) > 0.0) {
		/* fartri is not an inverted triangle, and farpoint is not a reflex */
		/*   vertex.  As there are no reflex vertices, fixuptri isn't an    */
		/*   inverted triangle, either.  Hence, test the edge between the   */
		/*   triangles to ensure it is locally Delaunay.                    */
		if (incircle(leftpoint, farpoint, rightpoint, nearpoint) <= 0.0) {
			return;
		}
		/* Not locally Delaunay; go on to an edge flip. */
	}        /* else fartri is inverted; remove it from the stack by flipping. */
	flip(&neartri);
	lprevself(*fixuptri);    /* Restore the origin of fixuptri after the flip. */
	/* Recursively process the two triangles that result from the flip. */
	delaunayfixup(fixuptri, leftside);
	delaunayfixup(&fartri, leftside);
}

void TriangleLib::constrainededge(struct triedge *starttri, point endpoint2, int newmark)
{
	struct triedge fixuptri, fixuptri2;
	struct edge fixupedge;
	point endpoint1;
	point farpoint;
	double area;
	int collision;
	int done;
	triangle ptr;             /* Temporary variable used by sym() and oprev(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	org(*starttri, endpoint1);
	lnext(*starttri, fixuptri);
	flip(&fixuptri);
	/* `collision' indicates whether we have found a point directly */
	/*   between endpoint1 and endpoint2.                           */
	collision = 0;
	done = 0;
	do {
		org(fixuptri, farpoint);
		/* `farpoint' is the extreme point of the polygon we are "digging" */
		/*   to get from endpoint1 to endpoint2.                           */
		if ((farpoint[0] == endpoint2[0]) && (farpoint[1] == endpoint2[1])) {
			oprev(fixuptri, fixuptri2);
			/* Enforce the Delaunay condition around endpoint2. */
			delaunayfixup(&fixuptri, 0);
			delaunayfixup(&fixuptri2, 1);
			done = 1;
		} else {
			/* Check whether farpoint is to the left or right of the segment */
			/*   being inserted, to decide which edge of fixuptri to dig     */
			/*   through next.                                               */
			area = counterclockwise(endpoint1, endpoint2, farpoint);
			if (area == 0.0) {
				/* We've collided with a point between endpoint1 and endpoint2. */
				collision = 1;
				oprev(fixuptri, fixuptri2);
				/* Enforce the Delaunay condition around farpoint. */
				delaunayfixup(&fixuptri, 0);
				delaunayfixup(&fixuptri2, 1);
				done = 1;
			} else {
				if (area > 0.0) {         /* farpoint is to the left of the segment. */
					oprev(fixuptri, fixuptri2);
					/* Enforce the Delaunay condition around farpoint, on the */
					/*   left side of the segment only.                       */
					delaunayfixup(&fixuptri2, 1);
					/* Flip the edge that crosses the segment.  After the edge is */
					/*   flipped, one of its endpoints is the fan vertex, and the */
					/*   destination of fixuptri is the fan vertex.               */
					lprevself(fixuptri);
				} else {                 /* farpoint is to the right of the segment. */
					delaunayfixup(&fixuptri, 0);
					/* Flip the edge that crosses the segment.  After the edge is */
					/*   flipped, one of its endpoints is the fan vertex, and the */
					/*   destination of fixuptri is the fan vertex.               */
					oprevself(fixuptri);
				}
				/* Check for two intersecting segments. */
				tspivot(fixuptri, fixupedge);
				if (fixupedge.sh == dummysh) {
					flip(&fixuptri);   /* May create an inverted triangle on the left. */
				} else {
					/* We've collided with a segment between endpoint1 and endpoint2. */
					collision = 1;
					/* Insert a point at the intersection. */
					segmentintersection(&fixuptri, &fixupedge, endpoint2);
					done = 1;
				}
			}
		}
	} while (!done);
	/* Insert a shell edge to make the segment permanent. */
	insertshelle(&fixuptri, newmark);
	/* If there was a collision with an interceding vertex, install another */
	/*   segment connecting that vertex with endpoint2.                     */
	if (collision) {
		/* Insert the remainder of the segment. */
		if (!scoutsegment(&fixuptri, endpoint2, newmark)) {
			constrainededge(&fixuptri, endpoint2, newmark);
		}
	}
}

/*****************************************************************************/
/*                                                                           */
/*  insertsegment()   Insert a PSLG segment into a triangulation.            */
/*                                                                           */
/*****************************************************************************/

void TriangleLib::insertsegment(point endpoint1, point endpoint2, int newmark)
{
	struct triedge searchtri1, searchtri2;
	
	triangle encodedtri = (triangle)NULL;//zhangli
	point checkpoint;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (verbose > 1)
	{
		
	}
	
	checkpoint = (point) NULL;
	if(pointmark(endpoint1) != QIUDUPLICATEPOINT)
		encodedtri = point2tri(endpoint1);
	else
		encodedtri = (triangle) NULL;
	
	if (encodedtri != (triangle) NULL) {
		decode(encodedtri, searchtri1);
		org(searchtri1, checkpoint);
	}
	if (checkpoint != endpoint1) {
		/* Find a boundary triangle to search from. */
		searchtri1.tri = dummytri;
		searchtri1.orient = 0;
		symself(searchtri1);
		/* Search for the segment's first endpoint by point location. */
		if (locate(endpoint1, &searchtri1) != ONVERTEX) {
			
			internalerror();
		}
	}
	/* Remember this triangle to improve subsequent point location. */
	triedgecopy(searchtri1, recenttri);
	/* Scout the beginnings of a path from the first endpoint */
	/*   toward the second.                                   */
	if (scoutsegment(&searchtri1, endpoint2, newmark)) {
		/* The segment was easily inserted. */
		return;
	}
	/* The first endpoint may have changed if a collision with an intervening */
	/*   vertex on the segment occurred.                                      */
	org(searchtri1, endpoint1);
	
	/* Find a triangle whose origin is the segment's second endpoint. */
	checkpoint = (point) NULL;
	
	if(pointmark(endpoint2) != QIUDUPLICATEPOINT)
		encodedtri = point2tri(endpoint2);
	else
		encodedtri = (triangle) NULL;
	
	if (encodedtri != (triangle) NULL) {
		decode(encodedtri, searchtri2);
		org(searchtri2, checkpoint);
	}
	if (checkpoint != endpoint2) {
		/* Find a boundary triangle to search from. */
		searchtri2.tri = dummytri;
		searchtri2.orient = 0;
		symself(searchtri2);
		/* Search for the segment's second endpoint by point location. */
		if (locate(endpoint2, &searchtri2) != ONVERTEX) 
		{
			
			internalerror();
		}
	}
	/* Remember this triangle to improve subsequent point location. */
	triedgecopy(searchtri2, recenttri);
	/* Scout the beginnings of a path from the second endpoint */
	/*   toward the first.                                     */
	if (scoutsegment(&searchtri2, endpoint1, newmark)) {
		/* The segment was easily inserted. */
		return;
	}
	/* The second endpoint may have changed if a collision with an intervening */
	/*   vertex on the segment occurred.                                       */
	org(searchtri2, endpoint2);

#ifndef REDUCED
#ifndef CDT_ONLY
	if (splitseg) {
		/* Insert vertices to force the segment into the triangulation. */
		conformingedge(endpoint1, endpoint2, newmark);
	} else {
#endif /* not CDT_ONLY */
#endif /* not REDUCED */
		/* Insert the segment directly into the triangulation. */
		constrainededge(&searchtri1, endpoint2, newmark);
#ifndef REDUCED
#ifndef CDT_ONLY
	}
#endif /* not CDT_ONLY */
#endif /* not REDUCED */
}

/*****************************************************************************/
/*                                                                           */
/*  markhull()   Cover the convex hull of a triangulation with shell edges.  */
/*                                                                           */
/*****************************************************************************/

void TriangleLib::markhull()
{
	struct triedge hulltri;
	struct triedge nexttri;
	struct triedge starttri;
	triangle ptr;             /* Temporary variable used by sym() and oprev(). */
	
	/* Find a triangle handle on the hull. */
	hulltri.tri = dummytri;
	hulltri.orient = 0;
	symself(hulltri);
	/* Remember where we started so we know when to stop. */
	triedgecopy(hulltri, starttri);
	/* Go once counterclockwise around the convex hull. */
	do {
		/* Create a shell edge if there isn't already one here. */
		insertshelle(&hulltri, 1);
		/* To find the next hull edge, go clockwise around the next vertex. */
		lnextself(hulltri);
		oprev(hulltri, nexttri);
		while (nexttri.tri != dummytri) {
			triedgecopy(nexttri, hulltri);
			oprev(hulltri, nexttri);
		}
	} while (!triedgeequal(hulltri, starttri));
}

/*****************************************************************************/
/*                                                                           */
/*  formskeleton()   Create the shell edges of a triangulation, including    */
/*                   PSLG edges and edges on the convex hull.                */
/*                                                                           */
/*  The PSLG edges are read from a .poly file.  The return value is the      */
/*  number of segments in the file.                                          */
/*                                                                           */
/*****************************************************************************/

int TriangleLib::formskeleton(int *segmentlist, int *segmentmarkerlist, int numberofsegments)
{
	char polyfilename[6];
	int index;
	point endpoint1, endpoint2;
	int segments;
	int segmentmarkers;
	int end1, end2;
	int boundmarker;
	int i;
	
	if (poly) {
		if (!quiet) {
			
		}
		strcpy(polyfilename, "input");
		segments = numberofsegments;
		segmentmarkers = segmentmarkerlist != (int *) NULL;
		index = 0;
		/* If segments are to be inserted, compute a mapping */
		/*   from points to triangles.                       */
		if (segments > 0) {
			if (verbose) {
				
			}
			makepointmap();
		}
		
		boundmarker = 0;
		/* Read and insert the segments. */
		for (i = 1; i <= segments; i++) {
			end1 = segmentlist[index++];
			end2 = segmentlist[index++];
			if (segmentmarkers) {
				boundmarker = segmentmarkerlist[i - 1];
			}
			if ((end1 < firstnumber) || (end1 >= firstnumber + inpoints)) {
				if (!quiet) 
				{
					
				}
			} else if ((end2 < firstnumber) || (end2 >= firstnumber + inpoints)) {
				if (!quiet) {
					
				}
			} else {
				endpoint1 = getpoint(end1);
				endpoint2 = getpoint(end2);
				if ((endpoint1[0] == endpoint2[0]) && (endpoint1[1] == endpoint2[1])) {
					if (!quiet) {
						
					}
				} else {
					insertsegment(endpoint1, endpoint2, boundmarker);
				}
			}
		}
	} else {
		segments = 0;
	}
	if (convex || !poly) {
		/* Enclose the convex hull with shell edges. */
		if (verbose) {
			
		}
		markhull();
	}
	return segments;
}

/********* Segment (shell edge) insertion ends here                  *********/

/********* Carving out holes and concavities begins here             *********/

/*****************************************************************************/
/*                                                                           */
/*  infecthull()   Virally infect all of the triangles of the convex hull    */
/*                 that are not protected by shell edges.  Where there are   */
/*                 shell edges, set boundary markers as appropriate.         */
/*                                                                           */
/*****************************************************************************/

void TriangleLib::infecthull()
{
	struct triedge hulltri;
	struct triedge nexttri;
	struct triedge starttri;
	struct edge hulledge;
	triangle **deadtri;
	point horg, hdest;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (verbose) {
		
	}
	/* Find a triangle handle on the hull. */
	hulltri.tri = dummytri;
	hulltri.orient = 0;
	symself(hulltri);
	/* Remember where we started so we know when to stop. */
	triedgecopy(hulltri, starttri);
	/* Go once counterclockwise around the convex hull. */
	do {
		/* Ignore triangles that are already infected. */
		if (!infected(hulltri)) {
			/* Is the triangle protected by a shell edge? */
			tspivot(hulltri, hulledge);
			if (hulledge.sh == dummysh) {
				/* The triangle is not protected; infect it. */
				infect(hulltri);
				deadtri = (triangle **) poolalloc(&viri);
				*deadtri = hulltri.tri;
			} else {
				/* The triangle is protected; set boundary markers if appropriate. */
				if (mark(hulledge) == 0) {
					setmark(hulledge, 1);
					org(hulltri, horg);
					dest(hulltri, hdest);
					if (pointmark(horg) == 0) {
						setpointmark(horg, 1);
					}
					if (pointmark(hdest) == 0) {
						setpointmark(hdest, 1);
					}
				}
			}
		}
		/* To find the next hull edge, go clockwise around the next vertex. */
		lnextself(hulltri);
		oprev(hulltri, nexttri);
		while (nexttri.tri != dummytri) {
			triedgecopy(nexttri, hulltri);
			oprev(hulltri, nexttri);
		}
	} while (!triedgeequal(hulltri, starttri));
}

void TriangleLib::plague()
{
	struct triedge testtri;
	struct triedge neighbor;
	triangle **virusloop;
	triangle **deadtri;
	struct edge neighborshelle;
	point testpoint;
	point norg, ndest;
	point deadorg, deaddest, deadapex;
	int killorg;
	triangle ptr;             /* Temporary variable used by sym() and onext(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (verbose) {
		
	}
	/* Loop through all the infected triangles, spreading the virus to */
	/*   their neighbors, then to their neighbors' neighbors.          */
	traversalinit(&viri);
	virusloop = (triangle **) traverse(&viri);
	while (virusloop != (triangle **) NULL) {
		testtri.tri = *virusloop;
		/* A triangle is marked as infected by messing with one of its shell */
		/*   edges, setting it to an illegal value.  Hence, we have to       */
		/*   temporarily uninfect this triangle so that we can examine its   */
		/*   adjacent shell edges.                                           */
		uninfect(testtri);
		if (verbose > 2) {
			/* Assign the triangle an orientation for convenience in */
			/*   checking its points.                                */
			testtri.orient = 0;
			org(testtri, deadorg);
			dest(testtri, deaddest);
			apex(testtri, deadapex);
			
		}
		/* Check each of the triangle's three neighbors. */
		for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
			/* Find the neighbor. */
			sym(testtri, neighbor);
			/* Check for a shell between the triangle and its neighbor. */
			tspivot(testtri, neighborshelle);
			/* Check if the neighbor is nonexistent or already infected. */
			if ((neighbor.tri == dummytri) || infected(neighbor)) {
				if (neighborshelle.sh != dummysh) {
					/* There is a shell edge separating the triangle from its */
					/*   neighbor, but both triangles are dying, so the shell */
					/*   edge dies too.                                       */
					shelledealloc(neighborshelle.sh);
					if (neighbor.tri != dummytri) {
						/* Make sure the shell edge doesn't get deallocated again */
						/*   later when the infected neighbor is visited.         */
						uninfect(neighbor);
						tsdissolve(neighbor);
						infect(neighbor);
					}
				}
			} else {                   /* The neighbor exists and is not infected. */
				if (neighborshelle.sh == dummysh) {
					/* There is no shell edge protecting the neighbor, so */
					/*   the neighbor becomes infected.                   */
					if (verbose > 2) {
						org(neighbor, deadorg);
						dest(neighbor, deaddest);
						apex(neighbor, deadapex);
						
					}
					infect(neighbor);
					/* Ensure that the neighbor's neighbors will be infected. */
					deadtri = (triangle **) poolalloc(&viri);
					*deadtri = neighbor.tri;
				} else {               /* The neighbor is protected by a shell edge. */
					/* Remove this triangle from the shell edge. */
					stdissolve(neighborshelle);
					/* The shell edge becomes a boundary.  Set markers accordingly. */
					if (mark(neighborshelle) == 0) {
						setmark(neighborshelle, 1);
					}
					org(neighbor, norg);
					dest(neighbor, ndest);
					if (pointmark(norg) == 0) {
						setpointmark(norg, 1);
					}
					if (pointmark(ndest) == 0) {
						setpointmark(ndest, 1);
					}
				}
			}
		}
		/* Remark the triangle as infected, so it doesn't get added to the */
		/*   virus pool again.                                             */
		infect(testtri);
		virusloop = (triangle **) traverse(&viri);
	}
	
	if (verbose) {
		
	}
	traversalinit(&viri);
	virusloop = (triangle **) traverse(&viri);
	while (virusloop != (triangle **) NULL) {
		testtri.tri = *virusloop;
		
		/* Check each of the three corners of the triangle for elimination. */
		/*   This is done by walking around each point, checking if it is   */
		/*   still connected to at least one live triangle.                 */
		for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
			org(testtri, testpoint);
			/* Check if the point has already been tested. */
			if (testpoint != (point) NULL) {
				killorg = 1;
				/* Mark the corner of the triangle as having been tested. */
				setorg(testtri, NULL);
				/* Walk counterclockwise about the point. */
				onext(testtri, neighbor);
				/* Stop upon reaching a boundary or the starting triangle. */
				while ((neighbor.tri != dummytri)
					&& (!triedgeequal(neighbor, testtri))) {
					if (infected(neighbor)) {
						/* Mark the corner of this triangle as having been tested. */
						setorg(neighbor, NULL);
					} else {
						/* A live triangle.  The point survives. */
						killorg = 0;
					}
					/* Walk counterclockwise about the point. */
					onextself(neighbor);
				}
				/* If we reached a boundary, we must walk clockwise as well. */
				if (neighbor.tri == dummytri) {
					/* Walk clockwise about the point. */
					oprev(testtri, neighbor);
					/* Stop upon reaching a boundary. */
					while (neighbor.tri != dummytri) {
						if (infected(neighbor)) {
							/* Mark the corner of this triangle as having been tested. */
							setorg(neighbor, NULL);
						} else {
							/* A live triangle.  The point survives. */
							killorg = 0;
						}
						/* Walk clockwise about the point. */
						oprevself(neighbor);
					}
				}
				if (killorg) {
					if (verbose > 1) {
						
					}
					pointdealloc(testpoint);
				}
			}
		}
		
		/* Record changes in the number of boundary edges, and disconnect */
		/*   dead triangles from their neighbors.                         */
		for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
			sym(testtri, neighbor);
			if (neighbor.tri == dummytri) {
				/* There is no neighboring triangle on this edge, so this edge    */
				/*   is a boundary edge.  This triangle is being deleted, so this */
				/*   boundary edge is deleted.                                    */
				hullsize--;
			} else {
				/* Disconnect the triangle from its neighbor. */
				dissolve(neighbor);
				/* There is a neighboring triangle on this edge, so this edge */
				/*   becomes a boundary edge when this triangle is deleted.   */
				hullsize++;
			}
		}
		/* Return the dead triangle to the pool of triangles. */
		triangledealloc(testtri.tri);
		virusloop = (triangle **) traverse(&viri);
	}
	/* Empty the virus pool. */
	poolrestart(&viri);
}

/*****************************************************************************/
/*                                                                           */
/*  regionplague()   Spread regional attributes and/or area constraints      */
/*                   (from a .poly file) throughout the mesh.                */
/*                                                                           */
/*  This procedure operates in two phases.  The first phase spreads an       */
/*  attribute and/or an area constraint through a (segment-bounded) region.  */
/*  The triangles are marked to ensure that each triangle is added to the    */
/*  virus pool only once, so the procedure will terminate.                   */
/*                                                                           */
/*  The second phase uninfects all infected triangles, returning them to     */
/*  normal.                                                                  */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::regionplague(double attribute, double area)
{
	struct triedge testtri;
	struct triedge neighbor;
	triangle **virusloop;
	triangle **regiontri;
	struct edge neighborshelle;
	point regionorg, regiondest, regionapex;
	triangle ptr;             /* Temporary variable used by sym() and onext(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (verbose > 1) {
		
	}
	/* Loop through all the infected triangles, spreading the attribute      */
	/*   and/or area constraint to their neighbors, then to their neighbors' */
	/*   neighbors.                                                          */
	traversalinit(&viri);
	virusloop = (triangle **) traverse(&viri);
	while (virusloop != (triangle **) NULL) {
		testtri.tri = *virusloop;
		/* A triangle is marked as infected by messing with one of its shell */
		/*   edges, setting it to an illegal value.  Hence, we have to       */
		/*   temporarily uninfect this triangle so that we can examine its   */
		/*   adjacent shell edges.                                           */
		uninfect(testtri);
		if (regionattrib) {
			/* Set an attribute. */
			setelemattribute(testtri, eextras, attribute);
		}
		if (vararea) {
			/* Set an area constraint. */
			setareabound(testtri, area);
		}
		if (verbose > 2) {
			/* Assign the triangle an orientation for convenience in */
			/*   checking its points.                                */
			testtri.orient = 0;
			org(testtri, regionorg);
			dest(testtri, regiondest);
			apex(testtri, regionapex);
			
		}
		/* Check each of the triangle's three neighbors. */
		for (testtri.orient = 0; testtri.orient < 3; testtri.orient++) {
			/* Find the neighbor. */
			sym(testtri, neighbor);
			/* Check for a shell between the triangle and its neighbor. */
			tspivot(testtri, neighborshelle);
			/* Make sure the neighbor exists, is not already infected, and */
			/*   isn't protected by a shell edge.                          */
			if ((neighbor.tri != dummytri) && !infected(neighbor)
				&& (neighborshelle.sh == dummysh)) {
				if (verbose > 2) {
					org(neighbor, regionorg);
					dest(neighbor, regiondest);
					apex(neighbor, regionapex);
					
				}
				/* Infect the neighbor. */
				infect(neighbor);
				/* Ensure that the neighbor's neighbors will be infected. */
				regiontri = (triangle **) poolalloc(&viri);
				*regiontri = neighbor.tri;
			}
		}
		/* Remark the triangle as infected, so it doesn't get added to the */
		/*   virus pool again.                                             */
		infect(testtri);
		virusloop = (triangle **) traverse(&viri);
	}
	
	/* Uninfect all triangles. */
	if (verbose > 1) {
		
	}
	traversalinit(&viri);
	virusloop = (triangle **) traverse(&viri);
	while (virusloop != (triangle **) NULL) {
		testtri.tri = *virusloop;
		uninfect(testtri);
		virusloop = (triangle **) traverse(&viri);
	}
	/* Empty the virus pool. */
	poolrestart(&viri);
}

/*****************************************************************************/
/*                                                                           */
/*  carveholes()   Find the holes and infect them.  Find the area            */
/*                 constraints and infect them.  Infect the convex hull.     */
/*                 Spread the infection and kill triangles.  Spread the      */
/*                 area constraints.                                         */
/*                                                                           */
/*  This routine mainly calls other routines to carry out all these          */
/*  functions.                                                               */
/*                                                                           */
/*****************************************************************************/

void TriangleLib::carveholes(double *holelist, int holes, double *regionlist, int regions)
{
	struct triedge searchtri;
	struct triedge triangleloop;
	struct triedge *regiontris = NULL;
	triangle **holetri;
	triangle **regiontri;
	point searchorg, searchdest;
	enum locateresult intersect;
	int i;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (!(quiet || (noholes && convex))) 
	{
		
	}
	
	if (regions > 0) {
		/* Allocate storage for the triangles in which region points fall. */
		regiontris = (struct triedge *) malloc(regions * sizeof(struct triedge));
		if (regiontris == (struct triedge *) NULL) {
			
			return;
		}
	}
	
	if (((holes > 0) && !noholes) || !convex || (regions > 0)) {
		/* Initialize a pool of viri to be used for holes, concavities, */
		/*   regional attributes, and/or regional area constraints.     */
		poolinit(&viri, sizeof(triangle *), VIRUSPERBLOCK, POINTER, 0);
	}
	
	if (!convex) {
		/* Mark as infected any unprotected triangles on the boundary. */
		/*   This is one way by which concavities are created.         */
		infecthull();
	}
	
	if ((holes > 0) && !noholes) {
		/* Infect each triangle in which a hole lies. */
		for (i = 0; i < 2 * holes; i += 2) {
			/* Ignore holes that aren't within the bounds of the mesh. */
			if ((holelist[i] >= xmin) && (holelist[i] <= xmax)
				&& (holelist[i + 1] >= ymin) && (holelist[i + 1] <= ymax)) {
				/* Start searching from some triangle on the outer boundary. */
				searchtri.tri = dummytri;
				searchtri.orient = 0;
				symself(searchtri);
				/* Ensure that the hole is to the left of this boundary edge; */
				/*   otherwise, locate() will falsely report that the hole    */
				/*   falls within the starting triangle.                      */
				org(searchtri, searchorg);
				dest(searchtri, searchdest);
				if (counterclockwise(searchorg, searchdest, &holelist[i]) > 0.0) {
					/* Find a triangle that contains the hole. */
					intersect = locate(&holelist[i], &searchtri);
					if ((intersect != OUTSIDE) && (!infected(searchtri))) {
						/* Infect the triangle.  This is done by marking the triangle */
						/*   as infect and including the triangle in the virus pool.  */
						infect(searchtri);
						holetri = (triangle **) poolalloc(&viri);
						*holetri = searchtri.tri;
					}
				}
			}
		}
	}
	
	/* Now, we have to find all the regions BEFORE we carve the holes, because */
	/*   locate() won't work when the triangulation is no longer convex.       */
	/*   (Incidentally, this is the reason why regional attributes and area    */
	/*   constraints can't be used when refining a preexisting mesh, which     */
	/*   might not be convex; they can only be used with a freshly             */
	/*   triangulated PSLG.)                                                   */
	if (regions > 0) 
	{	
		for (i = 0; i < regions; i++) {
			regiontris[i].tri = dummytri;
			/* Ignore region points that aren't within the bounds of the mesh. */
			if ((regionlist[4 * i] >= xmin) && (regionlist[4 * i] <= xmax) &&
				(regionlist[4 * i + 1] >= ymin) && (regionlist[4 * i + 1] <= ymax)) {
				/* Start searching from some triangle on the outer boundary. */
				searchtri.tri = dummytri;
				searchtri.orient = 0;
				symself(searchtri);
				/* Ensure that the region point is to the left of this boundary */
				/*   edge; otherwise, locate() will falsely report that the     */
				/*   region point falls within the starting triangle.           */
				org(searchtri, searchorg);
				dest(searchtri, searchdest);
				if (counterclockwise(searchorg, searchdest, &regionlist[4 * i]) >
					0.0) {
					/* Find a triangle that contains the region point. */
					intersect = locate(&regionlist[4 * i], &searchtri);
					if ((intersect != OUTSIDE) && (!infected(searchtri))) {
						/* Record the triangle for processing after the */
						/*   holes have been carved.                    */
						triedgecopy(searchtri, regiontris[i]);
					}
				}
			}
		}
	}
	
	if (viri.items > 0) {
		/* Carve the holes and concavities. */
		plague();
	}
	/* The virus pool should be empty now. */
	
	if (regions > 0) {
		if (!quiet) {
			if (regionattrib) {
				if (vararea) {
					
				} else {
					
				}
			} else { 
				
			}
		}
		if (regionattrib && !refine) {
			/* Assign every triangle a regional attribute of zero. */
			traversalinit(&triangles);
			triangleloop.orient = 0;
			triangleloop.tri = triangletraverse();
			while (triangleloop.tri != (triangle *) NULL) {
				setelemattribute(triangleloop, eextras, 0.0);
				triangleloop.tri = triangletraverse();
			}
		}
		for (i = 0; i < regions; i++) {
			if (regiontris[i].tri != dummytri) {
				/* Make sure the triangle under consideration still exists. */
				/*   It may have been eaten by the virus.                   */
				if (regiontris[i].tri[3] != (triangle) NULL) {
					/* Put one triangle in the virus pool. */
					infect(regiontris[i]);
					regiontri = (triangle **) poolalloc(&viri);
					*regiontri = regiontris[i].tri;
					/* Apply one region's attribute and/or area constraint. */
					regionplague(regionlist[4 * i + 2], regionlist[4 * i + 3]);
					/* The virus pool should be empty now. */
				}
			}
		}
		if (regionattrib && !refine) {
			/* Note the fact that each triangle has an additional attribute. */
			eextras++;
		}
	}
	
	/* Free up memory. */
	if (((holes > 0) && !noholes) || !convex || (regions > 0)) {
		pooldeinit(&viri);
	}
	if (regions > 0)
	{
		free(regiontris);
	}
}

void TriangleLib::tallyencs()
{
	struct edge edgeloop;
	int dummy;
	
	traversalinit(&shelles);
	edgeloop.shorient = 0;
	edgeloop.sh = shelletraverse();
	while (edgeloop.sh != (shelle *) NULL) {
		/* If the segment is encroached, add it to the list. */
		dummy = checkedge4encroach(&edgeloop);
		edgeloop.sh = shelletraverse();
	}
}

/*****************************************************************************/
/*                                                                           */
/*  precisionerror()  Print an error message for precision problems.         */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::precisionerror()
{

}

/*****************************************************************************/
/*                                                                           */
/*  repairencs()   Find and repair all the encroached segments.              */
/*                                                                           */
/*  Encroached segments are repaired by splitting them by inserting a point  */
/*  at or near their centers.                                                */
/*                                                                           */
/*  `flaws' is a flag that specifies whether one should take note of new     */
/*  encroached segments and bad triangles that result from inserting points  */
/*  to repair existing encroached segments.                                  */
/*                                                                           */
/*  When a segment is split, the two resulting subsegments are always        */
/*  tested to see if they are encroached upon, regardless of the value       */
/*  of `flaws'.                                                              */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::repairencs(int flaws)
{
	struct triedge enctri;
	struct triedge testtri;
	struct edge *encloop;
	struct edge testsh;
	point eorg, edest;
	point newpoint;
	enum insertsiteresult success;
	double segmentlength, nearestpoweroftwo;
	double split;
	int acuteorg, acutedest;
	int dummy;
	int i;
	triangle ptr;                     /* Temporary variable used by stpivot(). */
	shelle sptr;                        /* Temporary variable used by snext(). */
	
	
	while ((badsegments.items > 0) && (steinerleft != 0)) {
		traversalinit(&badsegments);
		encloop = badsegmenttraverse();
		while ((encloop != (struct edge *) NULL) && (steinerleft != 0)) {
			/* To decide where to split a segment, we need to know if the  */
			/*   segment shares an endpoint with an adjacent segment.      */
			/*   The concern is that, if we simply split every encroached  */
			/*   segment in its center, two adjacent segments with a small */
			/*   angle between them might lead to an infinite loop; each   */
			/*   point added to split one segment will encroach upon the   */
			/*   other segment, which must then be split with a point that */
			/*   will encroach upon the first segment, and so on forever.  */
			/* To avoid this, imagine a set of concentric circles, whose   */
			/*   radii are powers of two, about each segment endpoint.     */
			/*   These concentric circles determine where the segment is   */
			/*   split.  (If both endpoints are shared with adjacent       */
			/*   segments, split the segment in the middle, and apply the  */
			/*   concentric shells for later splittings.)                  */
			
			/* Is the origin shared with another segment? */
			stpivot(*encloop, enctri);
			lnext(enctri, testtri);
			tspivot(testtri, testsh);
			acuteorg = testsh.sh != dummysh;
			/* Is the destination shared with another segment? */
			lnextself(testtri);
			tspivot(testtri, testsh);
			acutedest = testsh.sh != dummysh;
			/* Now, check the other side of the segment, if there's a triangle */
			/*   there.                                                        */
			sym(enctri, testtri);
			if (testtri.tri != dummytri) {
				/* Is the destination shared with another segment? */
				lnextself(testtri);
				tspivot(testtri, testsh);
				acutedest = acutedest || (testsh.sh != dummysh);
				/* Is the origin shared with another segment? */
				lnextself(testtri);
				tspivot(testtri, testsh);
				acuteorg = acuteorg || (testsh.sh != dummysh);
			}
			
			sorg(*encloop, eorg);
			sdest(*encloop, edest);
			/* Use the concentric circles if exactly one endpoint is shared */
			/*   with another adjacent segment.                             */
			if (acuteorg ^ acutedest) {
				segmentlength = sqrt((edest[0] - eorg[0]) * (edest[0] - eorg[0])
					+ (edest[1] - eorg[1]) * (edest[1] - eorg[1]));
				/* Find the power of two nearest the segment's length. */
				nearestpoweroftwo = 1.0;
				while (segmentlength > SQUAREROOTTWO * nearestpoweroftwo) {
					nearestpoweroftwo *= 2.0;
				}
				while (segmentlength < (0.5 * SQUAREROOTTWO) * nearestpoweroftwo) {
					nearestpoweroftwo *= 0.5;
				}
				/* Where do we split the segment? */
				split = 0.5 * nearestpoweroftwo / segmentlength;
				if (acutedest) {
					split = 1.0 - split;
				}
			} else {
				/* If we're not worried about adjacent segments, split */
				/*   this segment in the middle.                       */
				split = 0.5;
			}
			
			/* Create the new point. */
			newpoint = (point) poolalloc(&points);
			/* Interpolate its coordinate and attributes. */
			for (i = 0; i < 2 + nextras; i++) {
				newpoint[i] = (1.0 - split) * eorg[i] + split * edest[i];
			//	j ++;
			}
			setpointmark(newpoint, mark(*encloop));
			if (verbose > 1) {
				
			}
			/* Check whether the new point lies on an endpoint. */
			if (((newpoint[0] == eorg[0]) && (newpoint[1] == eorg[1]))
				|| ((newpoint[0] == edest[0]) && (newpoint[1] == edest[1]))) 
			{
				
				precisionerror();
				
				return;
			}
			/* Insert the splitting point.  This should always succeed. */
			success = insertsite(newpoint, &enctri, encloop, flaws, flaws);
			if ((success != SUCCESSFULPOINT) && (success != ENCROACHINGPOINT)) {
				
				internalerror();
			}
			if (steinerleft > 0) {
				steinerleft--;
			}
			/* Check the two new subsegments to see if they're encroached. */
			dummy = checkedge4encroach(encloop);
			snextself(*encloop);
			dummy = checkedge4encroach(encloop);
			
			badsegmentdealloc(encloop);
			encloop = badsegmenttraverse();
    }
  }
}

/*****************************************************************************/
/*                                                                           */
/*  tallyfaces()   Test every triangle in the mesh for quality measures.     */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::tallyfaces()
{
	struct triedge triangleloop;
	
	if (verbose) {
		
	}
	traversalinit(&triangles);
	triangleloop.orient = 0;
	triangleloop.tri = triangletraverse();
	while (triangleloop.tri != (triangle *) NULL) {
		/* If the triangle is bad, enqueue it. */
		testtriangle(&triangleloop);
		triangleloop.tri = triangletraverse();
	}
}

enum circumcenterresult TriangleLib::findcircumcenter(point torg, point tdest, point tapex,
													   point circumcenter,double *xi, double *eta)
{
	double xdo, ydo, xao, yao, xad, yad;
	double dodist, aodist, addist;
	double denominator;
	double dx, dy;
	
	circumcentercount++;
	
	/* Compute the circumcenter of the triangle. */
	xdo = tdest[0] - torg[0];
	ydo = tdest[1] - torg[1];
	xao = tapex[0] - torg[0];
	yao = tapex[1] - torg[1];
	dodist = xdo * xdo + ydo * ydo;
	aodist = xao * xao + yao * yao;
	if (noexact) {
		denominator = 0.5 / (xdo * yao - xao * ydo);
	} else {
		/* Use the counterclockwise() routine to ensure a positive (and */
		/*   reasonably accurate) result, avoiding any possibility of   */
		/*   division by zero.                                          */
		denominator = 0.5 / counterclockwise(tdest, tapex, torg);
		/* Don't count the above as an orientation test. */
		counterclockcount--;
	}
	circumcenter[0] = torg[0] - (ydo * aodist - yao * dodist) * denominator;  
	circumcenter[1] = torg[1] + (xdo * aodist - xao * dodist) * denominator;  
	
	/* To interpolate point attributes for the new point inserted at  */
	/*   the circumcenter, define a coordinate system with a xi-axis, */
	/*   directed from the triangle's origin to its destination, and  */
	/*   an eta-axis, directed from its origin to its apex.           */
	/*   Calculate the xi and eta coordinates of the circumcenter.    */
	dx = circumcenter[0] - torg[0];
	dy = circumcenter[1] - torg[1];
	*xi = (dx * yao - xao * dy) * (2.0 * denominator);
	*eta = (xdo * dy - dx * ydo) * (2.0 * denominator);
	
	xad = tapex[0] - tdest[0];
	yad = tapex[1] - tdest[1];
	addist = xad * xad + yad * yad;
	if ((addist < dodist) && (addist < aodist)) {
		return OPPOSITEORG;
	} else if (dodist < aodist) {
		return OPPOSITEAPEX;
	} else {
		return OPPOSITEDEST;
	}
}

void TriangleLib::splittriangle(struct badface *badtri)
{
	point borg, bdest, bapex;
	point newpoint;
	double xi, eta;
	enum insertsiteresult success;
	enum circumcenterresult shortedge;
	int errorflag;
	int i;

	org(badtri->badfacetri, borg);
	dest(badtri->badfacetri, bdest);
	apex(badtri->badfacetri, bapex);
	/* Make sure that this triangle is still the same triangle it was      */
	/*   when it was tested and determined to be of bad quality.           */
	/*   Subsequent transformations may have made it a different triangle. */
	if ((borg == badtri->faceorg) && (bdest == badtri->facedest) &&
		(bapex == badtri->faceapex)) {
		if (verbose > 1)
		{
			
		}
		errorflag = 0;
		/* Create a new point at the triangle's circumcenter. */
		newpoint = (point) poolalloc(&points);
		shortedge = findcircumcenter(borg, bdest, bapex, newpoint, &xi, &eta);
		/* Check whether the new point lies on a triangle vertex. */
		if (((newpoint[0] == borg[0]) && (newpoint[1] == borg[1]))
			|| ((newpoint[0] == bdest[0]) && (newpoint[1] == bdest[1]))
			|| ((newpoint[0] == bapex[0]) && (newpoint[1] == bapex[1]))) {
			if (!quiet) 
			{
				
				errorflag = 1;
			}
			pointdealloc(newpoint);
		} else {
			for (i = 2; i < 2 + nextras; i++) 
			{
			 	double dd1 = sqrt((newpoint[i-2] - borg[0])*(newpoint[i-2] - borg[0]) +
					(newpoint[i-1] - borg[1])*(newpoint[i-1] - borg[1]));
				double dd2 = sqrt((newpoint[i-2] - bdest[0])*(newpoint[i-2] - bdest[0]) +
					(newpoint[i-1] - bdest[1])*(newpoint[i-1] - bdest[1]));
				double dd3 = sqrt((newpoint[i-2] - bapex[0])*(newpoint[i-2] - bapex[0]) +
					(newpoint[i-1] - bapex[1])*(newpoint[i-1] - bapex[1]));
				double dd4 = (dd1*dd2 + dd1*dd3 + dd2*dd3);
				if(fabs(dd4) < Geometries_EP)
				{
					newpoint[i] = (borg[i] + bdest[i] + bapex[i])/3;
				}
				else
				{
					dd4 = 1/dd4;
					newpoint[i] = (borg[i]*dd2*dd3 + bdest[i]*dd1*dd3+ bapex[i]*dd1*dd2)*dd4;
				}
			}
			/* The new point must be in the interior, and have a marker of zero. */
			setpointmark(newpoint, 0);
			/* Ensure that the handle `badtri->badfacetri' represents the shortest */
			/*   edge of the triangle.  This ensures that the circumcenter must    */
			/*   fall to the left of this edge, so point location will work.       */
			if (shortedge == OPPOSITEORG) {
				lnextself(badtri->badfacetri);
			} else if (shortedge == OPPOSITEDEST) {
				lprevself(badtri->badfacetri);
			}
			/* Insert the circumcenter, searching from the edge of the triangle, */
			/*   and maintain the Delaunay property of the triangulation.        */
			success = insertsite(newpoint, &(badtri->badfacetri),
				(struct edge *) NULL, 1, 1);
			if (success == SUCCESSFULPOINT) {
				if (steinerleft > 0) {
					steinerleft--;
				}
			} else if (success == ENCROACHINGPOINT) {
				/* If the newly inserted point encroaches upon a segment, delete it. */
				deletesite(&(badtri->badfacetri));
			} else if (success == VIOLATINGPOINT) {
				/* Failed to insert the new point, but some segment was */
				/*   marked as being encroached.                        */
				pointdealloc(newpoint);
			} else {                                  /* success == DUPLICATEPOINT */
				/* Failed to insert the new point because a vertex is already there. */
				if (!quiet) 
				{
					
					errorflag = 1;
				}
				pointdealloc(newpoint);
			}
		}
		if (errorflag) {
			if (verbose) {
				
			}
			
			precisionerror(); 
		}
	}
	/* Return the bad triangle to the pool. */
	pooldealloc(&badtriangles, (int *) badtri);
}

/*****************************************************************************/
/*                                                                           */
/*  enforcequality()   Remove all the encroached edges and bad triangles     */
/*                     from the triangulation.                               */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::enforcequality()
{
	int i;

	if (!quiet)
	{

	}
	/* Initialize the pool of encroached segments. */
	poolinit(&badsegments, sizeof(struct edge), BADSEGMENTPERBLOCK, POINTER, 0);
	if (verbose) {

	}
	/* Test all segments to see if they're encroached. */
	tallyencs();
	if (verbose && (badsegments.items > 0)) {

	}
	/* Note that steinerleft == -1 if an unlimited number */
	/*   of Steiner points is allowed.                    */
	while ((badsegments.items > 0) && (steinerleft != 0)) {
		/* Fix the segments without noting newly encroached segments or   */
		/*   bad triangles.  The reason we don't want to note newly       */
		/*   encroached segments is because some encroached segments are  */
		/*   likely to be noted multiple times, and would then be blindly */
		/*   split multiple times.  I should fix that some time.          */
		repairencs(0);
		/* Now, find all the segments that became encroached while adding */
		/*   points to split encroached segments.                         */
		tallyencs();
	}
	/* At this point, if we haven't run out of Steiner points, the */
	/*   triangulation should be (conforming) Delaunay.            */

	/* Next, we worry about enforcing triangle quality. */
	if ((minangle > 0.0) || vararea || fixedarea) {
		/* Initialize the pool of bad triangles. */
		poolinit(&badtriangles, sizeof(struct badface), BADTRIPERBLOCK, POINTER,
			0);
		/* Initialize the queues of bad triangles. */
		for (i = 0; i < 64; i++) {
			queuefront[i] = (struct badface *) NULL;
			queuetail[i] = &queuefront[i];
		}
		/* Test all triangles to see if they're bad. */
		tallyfaces();
		if (verbose) {

		}
		while ((badtriangles.items > 0) && (steinerleft != 0)) {
			/* Fix one bad triangle by inserting a point at its circumcenter. */
			splittriangle(dequeuebadtri());
			/* Fix any encroached segments that may have resulted.  Record */
			/*   any new bad triangles or encroached segments that result. */
			if (badsegments.items > 0) {
				repairencs(1);
			}
		}
	}
	/* At this point, if we haven't run out of Steiner points, the */
	/*   triangulation should be (conforming) Delaunay and have no */
	/*   low-quality triangles.                                    */

	/* Might we have run out of Steiner points too soon? */
	if (!quiet && (badsegments.items > 0) && (steinerleft == 0))
	{

		if (badsegments.items == 1) {

		}
		else {

		}
	}
}

void TriangleLib::highorder()
{
	struct triedge triangleloop, trisym;
	struct edge checkmark;
	point newpoint;
	point torg, tdest;
	int i;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (!quiet) {
		
	}
	/* The following line ensures that dead items in the pool of nodes    */
	/*   cannot be allocated for the extra nodes associated with high     */
	/*   order elements.  This ensures that the primary nodes (at the     */
	/*   corners of elements) will occur earlier in the output files, and */
	/*   have lower indices, than the extra nodes.                        */
	points.deaditemstack = (int *) NULL;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	/* To loop over the set of edges, loop over all triangles, and look at   */
	/*   the three edges of each triangle.  If there isn't another triangle  */
	/*   adjacent to the edge, operate on the edge.  If there is another     */
	/*   adjacent triangle, operate on the edge only if the current triangle */
	/*   has a smaller pointer than its neighbor.  This way, each edge is    */
	/*   considered only once.                                               */
	while (triangleloop.tri != (triangle *) NULL) {
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			sym(triangleloop, trisym);
			if ((triangleloop.tri < trisym.tri) || (trisym.tri == dummytri)) {
				org(triangleloop, torg);
				dest(triangleloop, tdest);
				/* Create a new node in the middle of the edge.  Interpolate */
				/*   its attributes.                                         */
				newpoint = (point) poolalloc(&points);
				for (i = 0; i < 2 + nextras; i++) {
					newpoint[i] = 0.5 * (torg[i] + tdest[i]);
				}
				/* Set the new node's marker to zero or one, depending on */
				/*   whether it lies on a boundary.                       */
				setpointmark(newpoint, trisym.tri == dummytri);
				if (useshelles) {
					tspivot(triangleloop, checkmark);
					/* If this edge is a segment, transfer the marker to the new node. */
					if (checkmark.sh != dummysh) {
						setpointmark(newpoint, mark(checkmark));
					}
				}
				
				/* Record the new node in the (one or two) adjacent elements. */
				triangleloop.tri[highorderindex + triangleloop.orient] =
					(triangle) newpoint;
				if (trisym.tri != dummytri) {
					trisym.tri[highorderindex + trisym.orient] = (triangle) newpoint;
				}
			}
		}
		triangleloop.tri = triangletraverse();
	}
}

void TriangleLib::transfernodes(double *pointlist, double *pointattriblist,
								 int *pointmarkerlist, int numberofpoints,int numberofpointattribs)
{
	point pointloop;
	double x, y;
	int i, j;
	int coordindex;
	int attribindex;
	
	inpoints = numberofpoints;
	mesh_dim = 2;
	nextras = numberofpointattribs;
	readnodefile = 0;
	if (inpoints < 3) {
		
		return;
	}
	initializepointpool();


	/* Read the points. */
	coordindex = 0;
	attribindex = 0;
	for (i = 0; i < inpoints; i++) {
		pointloop = (point) poolalloc(&points);
		/* Read the point coordinates. */
		x = pointloop[0] = pointlist[coordindex++];
		y = pointloop[1] = pointlist[coordindex++];
		/* Read the point attributes. */
		for (j = 0; j < numberofpointattribs; j++) {
			pointloop[2 + j] = pointattriblist[attribindex++];
		}
		if (pointmarkerlist != (int *) NULL) {
			/* Read a point marker. */
			setpointmark(pointloop, pointmarkerlist[i]);
		} else {
			/* If no markers are specified, they default to zero. */
			setpointmark(pointloop, 0);
		}
		x = pointloop[0];
		y = pointloop[1];
		/* Determine the smallest and largest x and y coordinates. */
		if (i == 0) {
			xmin = xmax = x;
			ymin = ymax = y;
		} else {
			xmin = (x < xmin) ? x : xmin;
			xmax = (x > xmax) ? x : xmax;
			ymin = (y < ymin) ? y : ymin;
			ymax = (y > ymax) ? y : ymax;
		}
	}
	
	/* Nonexistent x value used as a flag to mark circle events in sweepline */
	/*   Delaunay algorithm.                                                 */
	xminextreme = 10 * xmin - 9 * xmax;
}

void TriangleLib::writenodes(double **pointlist, double **pointattriblist, int **pointmarkerlist)
{
	double *plist;
	double *palist;
	int *pmlist;
	int coordindex;
	int attribindex;
	point pointloop;
	int pointnumber;
	int i;
	
	if (!quiet) {
		
	}
	/* Allocate memory for output points if necessary. */
	if (*pointlist == (double *) NULL) {
		*pointlist = (double *) malloc(points.items * 2 * sizeof(double));
		if (*pointlist == (double *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for output point attributes if necessary. */
	if ((nextras > 0) && (*pointattriblist == (double *) NULL)) {
		*pointattriblist = (double *) malloc(points.items * nextras * sizeof(double));
		if (*pointattriblist == (double *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for output point markers if necessary. */
	if (!nobound && (*pointmarkerlist == (int *) NULL)) {
		*pointmarkerlist = (int *) malloc(points.items * sizeof(int));
		if (*pointmarkerlist == (int *) NULL) {
			
			return;
		}
	}
	plist = *pointlist;
	palist = *pointattriblist;
	pmlist = *pointmarkerlist;
	coordindex = 0;
	attribindex = 0;
	traversalinit(&points);
	pointloop = pointtraverse();
	pointnumber = firstnumber;
	
	int j = 0; //test

	while (pointloop != (point) NULL) {
		/* X and y coordinates. */
		plist[coordindex++] = pointloop[0];
		plist[coordindex++] = pointloop[1];
		/* Point attributes. */
		for (i = 0; i < nextras; i++) {
			palist[attribindex++] = pointloop[2 + i];
			j ++;//test
		}
		if (!nobound) {
			/* Copy the boundary marker. */
			pmlist[pointnumber - firstnumber] = pointmark(pointloop);
		}
		
		setpointmark(pointloop, pointnumber);
		pointloop = pointtraverse();
		pointnumber++;
	}
}

/*****************************************************************************/
/*                                                                           */
/*  numbernodes()   Number the points.                                       */
/*                                                                           */
/*  Each point is assigned a marker equal to its number.                     */
/*                                                                           */
/*  Used when writenodes() is not called because no .node file is written.   */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::numbernodes()
{
	point pointloop;
	int pointnumber;
	
	traversalinit(&points);
	pointloop = pointtraverse();
	pointnumber = firstnumber;
	while (pointloop != (point) NULL) {
		setpointmark(pointloop, pointnumber);
		pointloop = pointtraverse();
		pointnumber++;
	}
}

/*****************************************************************************/
/*                                                                           */
/*  writeelements()   Write the triangles to an .ele file.                   */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::writeelements(int **trianglelist, double **triangleattriblist)
{
	int *tlist;
	double *talist;
	int pointindex;
	int attribindex;
	struct triedge triangleloop;
	point p1, p2, p3;
	point mid1, mid2, mid3;
	int elementnumber;
	int i;
	
	if (!quiet) {
		
	}
	/* Allocate memory for output triangles if necessary. */
	if (*trianglelist == (int *) NULL) {
		*trianglelist = (int *) malloc(triangles.items *
			((order + 1) * (order + 2) / 2) * sizeof(int));
		if (*trianglelist == (int *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for output triangle attributes if necessary. */
	if ((eextras > 0) && (*triangleattriblist == (double *) NULL)) {
		*triangleattriblist = (double *) malloc(triangles.items * eextras *
			sizeof(double));
		if (*triangleattriblist == (double *) NULL) {
			
			return;
		}
	}
	tlist = *trianglelist;
	talist = *triangleattriblist;
	pointindex = 0;
	attribindex = 0;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	triangleloop.orient = 0;
	elementnumber = firstnumber;
	while (triangleloop.tri != (triangle *) NULL) {
		org(triangleloop, p1);
		dest(triangleloop, p2);
		apex(triangleloop, p3);
		if (order == 1) {
			tlist[pointindex++] = pointmark(p1);
			tlist[pointindex++] = pointmark(p2);
			tlist[pointindex++] = pointmark(p3);
		} else {
			mid1 = (point) triangleloop.tri[highorderindex + 1];
			mid2 = (point) triangleloop.tri[highorderindex + 2];
			mid3 = (point) triangleloop.tri[highorderindex];
			tlist[pointindex++] = pointmark(p1);
			tlist[pointindex++] = pointmark(p2);
			tlist[pointindex++] = pointmark(p3);
			tlist[pointindex++] = pointmark(mid1);
			tlist[pointindex++] = pointmark(mid2);
			tlist[pointindex++] = pointmark(mid3);
		}
		
		for (i = 0; i < eextras; i++) {
			talist[attribindex++] = elemattribute(triangleloop, i);
		}
		
		triangleloop.tri = triangletraverse();
		elementnumber++;
	}
	
}

/*****************************************************************************/
/*                                                                           */
/*  writepoly()   Write the segments and holes to a .poly file.              */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::writepoly(int **segmentlist, int **segmentmarkerlist)
{
	int *slist;
	int *smlist;
	int index;
	struct edge shelleloop;
	point endpoint1, endpoint2;
	int shellenumber;
	
	if (!quiet) {
		
	}
	/* Allocate memory for output segments if necessary. */
	if (*segmentlist == (int *) NULL) {
		*segmentlist = (int *) malloc(shelles.items * 2 * sizeof(int));
		if (*segmentlist == (int *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for output segment markers if necessary. */
	if (!nobound && (*segmentmarkerlist == (int *) NULL)) {
		*segmentmarkerlist = (int *) malloc(shelles.items * sizeof(int));
		if (*segmentmarkerlist == (int *) NULL) {
			
			return;
		}
	}
	slist = *segmentlist;
	smlist = *segmentmarkerlist;
	index = 0;
	traversalinit(&shelles);
	shelleloop.sh = shelletraverse();
	shelleloop.shorient = 0;
	shellenumber = firstnumber;
	while (shelleloop.sh != (shelle *) NULL) {
		sorg(shelleloop, endpoint1);
		sdest(shelleloop, endpoint2);
		/* Copy indices of the segment's two endpoints. */
		slist[index++] = pointmark(endpoint1);
		slist[index++] = pointmark(endpoint2);
		if (!nobound) {
			/* Copy the boundary marker. */
			smlist[shellenumber - firstnumber] = mark(shelleloop);
		}
		shelleloop.sh = shelletraverse();
		shellenumber++;
	}
	
}

/*****************************************************************************/
/*                                                                           */
/*  writeedges()   Write the edges to a .edge file.                          */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::writeedges(int **edgelist, int **edgemarkerlist)
{
	int *elist;
	int *emlist;
	int index;
	struct triedge triangleloop, trisym;
	struct edge checkmark;
	point p1, p2;
	int edgenumber;
	triangle ptr;                         /* Temporary variable used by sym(). */
	shelle sptr;                      /* Temporary variable used by tspivot(). */
	
	if (!quiet) {
		
	}
	/* Allocate memory for edges if necessary. */
	if (*edgelist == (int *) NULL) {
		*edgelist = (int *) malloc(edges * 2 * sizeof(int));
		if (*edgelist == (int *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for edge markers if necessary. */
	if (!nobound && (*edgemarkerlist == (int *) NULL)) {
		*edgemarkerlist = (int *) malloc(edges * sizeof(int));
		if (*edgemarkerlist == (int *) NULL) {
			
			return;
		}
	}
	elist = *edgelist;
	emlist = *edgemarkerlist;
	index = 0;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	edgenumber = firstnumber;
	/* To loop over the set of edges, loop over all triangles, and look at   */
	/*   the three edges of each triangle.  If there isn't another triangle  */
	/*   adjacent to the edge, operate on the edge.  If there is another     */
	/*   adjacent triangle, operate on the edge only if the current triangle */
	/*   has a smaller pointer than its neighbor.  This way, each edge is    */
	/*   considered only once.                                               */
	while (triangleloop.tri != (triangle *) NULL) {
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			sym(triangleloop, trisym);
			if ((triangleloop.tri < trisym.tri) || (trisym.tri == dummytri)) {
				org(triangleloop, p1);
				dest(triangleloop, p2);
				elist[index++] = pointmark(p1);
				elist[index++] = pointmark(p2);
				if (nobound) {
				} else {
					/* Edge number, indices of two endpoints, and a boundary marker. */
					/*   If there's no shell edge, the boundary marker is zero.      */
					if (useshelles) {
						tspivot(triangleloop, checkmark);
						if (checkmark.sh == dummysh) {
							emlist[edgenumber - firstnumber] = 0;
						} else {
							emlist[edgenumber - firstnumber] = mark(checkmark);
						}
					} else {
						emlist[edgenumber - firstnumber] = trisym.tri == dummytri;
					}
				}
				edgenumber++;
			}
		}
		triangleloop.tri = triangletraverse();
	}
	
}

/*****************************************************************************/
/*                                                                           */
/*  writevoronoi()   Write the Voronoi diagram to a .v.node and .v.edge      */
/*                   file.                                                   */
/*                                                                           */
/*  The Voronoi diagram is the geometric dual of the Delaunay triangulation. */
/*  Hence, the Voronoi vertices are listed by traversing the Delaunay        */
/*  triangles, and the Voronoi edges are listed by traversing the Delaunay   */
/*  edges.                                                                   */
/*                                                                           */
/*  WARNING:  In order to assign numbers to the Voronoi vertices, this       */
/*  procedure messes up the shell edges or the extra nodes of every          */
/*  element.  Hence, you should call this procedure last.                    */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::writevoronoi(double **vpointlist, double **vpointattriblist, int **vpointmarkerlist, 
								int **vedgelist,int **vedgemarkerlist, double **vnormlist)
{
	double *plist;
	double *palist;
	int *elist;
	double *normlist;
	int coordindex;
	int attribindex;
	struct triedge triangleloop, trisym;
	point torg, tdest, tapex;
	double circumcenter[2];
	double xi, eta;
	int vnodenumber, vedgenumber;
	int p1, p2;
	int i;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (!quiet) {
		
	}
	/* Allocate memory for Voronoi vertices if necessary. */
	if (*vpointlist == (double *) NULL) {
		*vpointlist = (double *) malloc(triangles.items * 2 * sizeof(double));
		if (*vpointlist == (double *) NULL) {
			
			return;
		}
	}
	/* Allocate memory for Voronoi vertex attributes if necessary. */
	if (*vpointattriblist == (double *) NULL) {
		*vpointattriblist = (double *) malloc(triangles.items * nextras *
			sizeof(double));
		if (*vpointattriblist == (double *) NULL) {
			
			return;
		}
	}
	*vpointmarkerlist = (int *) NULL;
	plist = *vpointlist;
	palist = *vpointattriblist;
	coordindex = 0;
	attribindex = 0;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	triangleloop.orient = 0;
	vnodenumber = firstnumber;
	while (triangleloop.tri != (triangle *) NULL) {
		org(triangleloop, torg);
		dest(triangleloop, tdest);
		apex(triangleloop, tapex);
		findcircumcenter(torg, tdest, tapex, circumcenter, &xi, &eta);
		/* X and y coordinates. */
		plist[coordindex++] = circumcenter[0];
		plist[coordindex++] = circumcenter[1];
		for (i = 2; i < 2 + nextras; i++) {
			/* Interpolate the point attributes at the circumcenter. */
			palist[attribindex++] = torg[i] + xi * (tdest[i] - torg[i])
				+ eta * (tapex[i] - torg[i]);
		}
		* (int *) (triangleloop.tri + 6) = vnodenumber;
		triangleloop.tri = triangletraverse();
		vnodenumber++;
	}
	
	if (!quiet) {
		
	}
	/* Allocate memory for output Voronoi edges if necessary. */
	if (*vedgelist == (int *) NULL) 
	{
		*vedgelist = (int *) malloc(edges * 2 * sizeof(int));
		if (*vedgelist == (int *) NULL) {
			
			return;
		}
	}
	*vedgemarkerlist = (int *) NULL;
	/* Allocate memory for output Voronoi norms if necessary. */
	if (*vnormlist == (double *) NULL) 
	{
		*vnormlist = (double *) malloc(edges * 2 * sizeof(double));
		if (*vnormlist == (double *) NULL) {
			
			return;
		}
	}
	elist = *vedgelist;
	normlist = *vnormlist;
	coordindex = 0;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	vedgenumber = firstnumber;
	/* To loop over the set of edges, loop over all triangles, and look at   */
	/*   the three edges of each triangle.  If there isn't another triangle  */
	/*   adjacent to the edge, operate on the edge.  If there is another     */
	/*   adjacent triangle, operate on the edge only if the current triangle */
	/*   has a smaller pointer than its neighbor.  This way, each edge is    */
	/*   considered only once.                                               */
	while (triangleloop.tri != (triangle *) NULL) {
		for (triangleloop.orient = 0; triangleloop.orient < 3;
		triangleloop.orient++) {
			sym(triangleloop, trisym);
			if ((triangleloop.tri < trisym.tri) || (trisym.tri == dummytri)) {
				/* Find the number of this triangle (and Voronoi vertex). */
				p1 = * (int *) (triangleloop.tri + 6);
				if (trisym.tri == dummytri) {
					org(triangleloop, torg);
					dest(triangleloop, tdest);
					/* Copy an infinite ray.  Index of one endpoint, and -1. */
					elist[coordindex] = p1;
					normlist[coordindex++] = tdest[1] - torg[1];
					elist[coordindex] = -1;
					normlist[coordindex++] = torg[0] - tdest[0];
				} else {
					/* Find the number of the adjacent triangle (and Voronoi vertex). */
					p2 = * (int *) (trisym.tri + 6);
					/* Finite edge.  Write indices of two endpoints. */
					elist[coordindex] = p1;
					normlist[coordindex++] = 0.0;
					elist[coordindex] = p2;
					normlist[coordindex++] = 0.0;
				}
				vedgenumber++;
			}
		}
		triangleloop.tri = triangletraverse();
	}
	
}
void TriangleLib::writeneighbors(int **neighborlist)
{
	int *nlist;
	int index;
	struct triedge triangleloop, trisym;
	int elementnumber;
	int neighbor1, neighbor2, neighbor3;
	triangle ptr;                         /* Temporary variable used by sym(). */
	
	if (!quiet) {
		
	}
	/* Allocate memory for neighbors if necessary. */
	if (*neighborlist == (int *) NULL) {
		*neighborlist = (int *) malloc(triangles.items * 3 * sizeof(int));
		if (*neighborlist == (int *) NULL) {
			
			return;
		}
	}
	nlist = *neighborlist;
	index = 0;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	triangleloop.orient = 0;
	elementnumber = firstnumber;
	while (triangleloop.tri != (triangle *) NULL) {
		* (int *) (triangleloop.tri + 6) = elementnumber;
		triangleloop.tri = triangletraverse();
		elementnumber++;
	}
	* (int *) (dummytri + 6) = -1;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	elementnumber = firstnumber;
	while (triangleloop.tri != (triangle *) NULL) {
		triangleloop.orient = 1;
		sym(triangleloop, trisym);
		neighbor1 = * (int *) (trisym.tri + 6);
		triangleloop.orient = 2;
		sym(triangleloop, trisym);
		neighbor2 = * (int *) (trisym.tri + 6);
		triangleloop.orient = 0;
		sym(triangleloop, trisym);
		neighbor3 = * (int *) (trisym.tri + 6);
		nlist[index++] = neighbor1;
		nlist[index++] = neighbor2;
		nlist[index++] = neighbor3;
		triangleloop.tri = triangletraverse();
		elementnumber++;
	}
	
}

/********* File I/O routines end here                                *********/

/*****************************************************************************/
/*                                                                           */
/*  quality_statistics()   Print statistics about the quality of the mesh.   */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::quality_statistics()
{
	struct triedge triangleloop;
	point p[3];
	double cossquaretable[8];
	double ratiotable[16];
	double dx[3], dy[3];
	double edgelength[3];
	double dotproduct;
	double cossquare;
	double triarea;
	double shortest, longest;
	double trilongest2;
	double smallestarea, biggestarea;
	double triminaltitude2;
	double minaltitude;
	double triaspect2;
	double worstaspect;
	double smallestangle, biggestangle;
	double radconst, degconst;
	int angletable[18];
	int aspecttable[16];
	int aspectindex;
	int tendegree;
	int acutebiggest;
	int i, ii, j, k;
	
	m_strStatistics +="Mesh quality statistics:\n";
	radconst = PI / 18.0;
	degconst = 180.0 / PI;
	for (i = 0; i < 8; i++) {
		cossquaretable[i] = cos(radconst * (double) (i + 1));
		cossquaretable[i] = cossquaretable[i] * cossquaretable[i];
	}
	for (i = 0; i < 18; i++) {
		angletable[i] = 0;
	}
	
	ratiotable[0]  =      1.5;      ratiotable[1]  =     2.0;
	ratiotable[2]  =      2.5;      ratiotable[3]  =     3.0;
	ratiotable[4]  =      4.0;      ratiotable[5]  =     6.0;
	ratiotable[6]  =     10.0;      ratiotable[7]  =    15.0;
	ratiotable[8]  =     25.0;      ratiotable[9]  =    50.0;
	ratiotable[10] =    100.0;      ratiotable[11] =   300.0;
	ratiotable[12] =   1000.0;      ratiotable[13] = 10000.0;
	ratiotable[14] = 100000.0;      ratiotable[15] =     0.0;
	for (i = 0; i < 16; i++) {
		aspecttable[i] = 0;
	}
	
	worstaspect = 0.0;
	minaltitude = xmax - xmin + ymax - ymin;
	minaltitude = minaltitude * minaltitude;
	shortest = minaltitude;
	longest = 0.0;
	smallestarea = minaltitude;
	biggestarea = 0.0;
	worstaspect = 0.0;
	smallestangle = 0.0;
	biggestangle = 2.0;
	acutebiggest = 1;
	
	traversalinit(&triangles);
	triangleloop.tri = triangletraverse();
	triangleloop.orient = 0;
	while (triangleloop.tri != (triangle *) NULL) {
		org(triangleloop, p[0]);
		dest(triangleloop, p[1]);
		apex(triangleloop, p[2]);
		trilongest2 = 0.0;
		
		for (i = 0; i < 3; i++) {
			j = plus1mod3[i];
			k = minus1mod3[i];
			dx[i] = p[j][0] - p[k][0];
			dy[i] = p[j][1] - p[k][1];
			edgelength[i] = dx[i] * dx[i] + dy[i] * dy[i];
			if (edgelength[i] > trilongest2) {
				trilongest2 = edgelength[i];
			}
			if (edgelength[i] > longest) {
				longest = edgelength[i];
			}
			if (edgelength[i] < shortest) {
				shortest = edgelength[i];
			}
		}
		
		triarea = counterclockwise(p[0], p[1], p[2]);
		if (triarea < smallestarea) {
			smallestarea = triarea;
		}
		if (triarea > biggestarea) {
			biggestarea = triarea;
		}
		triminaltitude2 = triarea * triarea / trilongest2;
		if (triminaltitude2 < minaltitude) {
			minaltitude = triminaltitude2;
		}
		triaspect2 = trilongest2 / triminaltitude2;
		if (triaspect2 > worstaspect) {
			worstaspect = triaspect2;
		}
		aspectindex = 0;
		while ((triaspect2 > ratiotable[aspectindex] * ratiotable[aspectindex])
			&& (aspectindex < 15)) {
			aspectindex++;
		}
		aspecttable[aspectindex]++;
		
		for (i = 0; i < 3; i++) {
			j = plus1mod3[i];
			k = minus1mod3[i];
			dotproduct = dx[j] * dx[k] + dy[j] * dy[k];
			cossquare = dotproduct * dotproduct / (edgelength[j] * edgelength[k]);
			tendegree = 8;
			for (ii = 7; ii >= 0; ii--) {
				if (cossquare > cossquaretable[ii]) {
					tendegree = ii;
				}
			}
			if (dotproduct <= 0.0) {
				angletable[tendegree]++;
				if (cossquare > smallestangle) {
					smallestangle = cossquare;
				}
				if (acutebiggest && (cossquare < biggestangle)) {
					biggestangle = cossquare;
				}
			} else {
				angletable[17 - tendegree]++;
				if (acutebiggest || (cossquare > biggestangle)) {
					biggestangle = cossquare;
					acutebiggest = 0;
				}
			}
		}
		triangleloop.tri = triangletraverse();
	}
	
	shortest = sqrt(shortest);
	longest = sqrt(longest);
	minaltitude = sqrt(minaltitude);
	worstaspect = sqrt(worstaspect);
	smallestarea *= 2.0;
	biggestarea *= 2.0;
	if (smallestangle >= 1.0) {
		smallestangle = 0.0;
	} else {
		smallestangle = degconst * acos(sqrt(smallestangle));
	}
	if (biggestangle >= 1.0) {
		biggestangle = 180.0;
	} else {
		if (acutebiggest) {
			biggestangle = degconst * acos(sqrt(biggestangle));
		} else {
			biggestangle = 180.0 - degconst * acos(sqrt(biggestangle));
		}
	}
}
/*****************************************************************************/
/*                                                                           */
/*  statistics()   Print all sorts of cool facts.                            */
/*                                                                           */
/*****************************************************************************/
void TriangleLib::statistics()
{
	if(verbose) 
	{
		quality_statistics();
	}
	
}

void TriangleLib::triangulate(struct triangulateio *in, struct triangulateio *out, char *triswitches,struct triangulateio *vorout)
{
	double *holearray = NULL;
	double *regionarray = NULL;

	triangleinit();

	parsecommandline(1, &triswitches);
	
	transfernodes(in->pointlist, in->pointattributelist, in->pointmarkerlist, in->numberofpoints, in->numberofpointattributes);
	
	if(!nonodewritten)
	{
		if(in->pointlist)
		{
			delete []in->pointlist;
			in->pointlist = NULL;
		}
		if(in->pointattributelist)
		{
			delete []in->pointattributelist;
			in->pointattributelist = NULL;
		}
		if(in->pointmarkerlist)
		{
			delete []in->pointmarkerlist;
			in->pointmarkerlist = NULL;
		}
		in->numberofpoints = 0;
		in->numberofpointattributes= 0;
	}
	
	hullsize = delaunay();

	infpoint1 = (point) NULL;
	infpoint2 = (point) NULL;
	infpoint3 = (point) NULL;
	
	if (useshelles) 
	{
		checksegments = 1;
		
		if (!refine) 
		{		
			insegments = formskeleton(in->segmentlist, in->segmentmarkerlist,
				in->numberofsegments);
			
			if(in->segmentlist)
			{
				delete []in->segmentlist;
				in->segmentlist = NULL;
			}
			if(in->segmentmarkerlist)
			{
				delete []in->segmentmarkerlist;
				in->segmentmarkerlist = NULL;
			}
			in->numberofsegments = 0;
		}
	}
	
	holes = 0;
	regions = 0;
	
	edges = (3l * triangles.items + hullsize) / 2l;
	
	if (order > 1)
	{
		highorder();
	}
	
	out->numberofpoints = points.items;
	out->numberofpointattributes = nextras;
	out->numberoftriangles = triangles.items;
	out->numberofcorners = (order + 1) * (order + 2) / 2;
	out->numberoftriangleattributes = eextras;
	out->numberofedges = edges;
	
	if (useshelles) 
	{
		out->numberofsegments = shelles.items;
	} 
	else 
	{
		out->numberofsegments = hullsize;
	}
	
	if (vorout != (struct triangulateio *) NULL) 
	{
		vorout->numberofpoints = triangles.items;
		vorout->numberofpointattributes = nextras;
		vorout->numberofedges = edges;
	}

	if (nonodewritten || (noiterationnum && readnodefile)) 
	{
		numbernodes();
	}
	else 
	{
		writenodes(&out->pointlist, &out->pointattributelist, &out->pointmarkerlist);
	}
	
	writeelements(&out->trianglelist, &out->triangleattributelist);
	
	if (voronoi) 
	{
		writevoronoi(&vorout->pointlist, &vorout->pointattributelist,
			&vorout->pointmarkerlist, &vorout->edgelist,
			&vorout->edgemarkerlist, &vorout->normlist);
	}
	
	statistics();
	triangledeinit();

	return;
}
