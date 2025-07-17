#ifndef __TRIANGLELIB_H_ 
#define __TRIANGLELIB_H_ 

#include "TriangleStruct.h"
#include "Base/String.h"
#include "Base/Types.h"

using namespace Engine::Base;

class TriangleLib
{
public:
	TriangleLib();
	virtual ~TriangleLib();
private:
	struct memorypool triangles;
	struct memorypool shelles;
	struct memorypool points;
	struct memorypool viri;
	struct memorypool badsegments;
	struct memorypool badtriangles;
	struct memorypool splaynodes;

	struct badface *queuefront[64];
	struct badface **queuetail[64];

	double xmin, xmax, ymin, ymax;
	double xminextreme;
	int inpoints;
	int inelements;
	int insegments;
	int holes;
	int regions;
	long edges;
	int mesh_dim;
	int nextras;
	int eextras;
	long hullsize;
	int triwords;
	int shwords;
	int pointmarkindex;
	int point2triindex;
	int highorderindex;
	int elemattribindex;
	int areaboundindex;
	int checksegments;
	int readnodefile;
	long samples;
	UInt64 randomseed;

	double splitter;
	double epsilon;
	double resulterrbound;
	double ccwerrboundA, ccwerrboundB, ccwerrboundC;
	double iccerrboundA, iccerrboundB, iccerrboundC;

	long incirclecount;
	long counterclockcount;
	long hyperbolacount;
	long circumcentercount;
	long circletopcount;

	int poly, refine, quality, vararea, fixedarea, regionattrib, convex;
	int firstnumber;
	int edgesout, voronoi, neighbors, geomview;
	int nobound, nopolywritten, nonodewritten, noelewritten, noiterationnum;
	int noholes, noexact;
	int incremental, sweepline, dwyer;
	int splitseg;
	int docheck;
	int quiet, verbose;
	int useshelles;
	int order;
	int nobisect;
	int steiner, steinerleft;
	double minangle, goodangle;
	double maxarea;

	point infpoint1, infpoint2, infpoint3;

	triangle *dummytri;
	triangle *dummytribase;

	shelle *dummysh;

	shelle *dummyshbase;

	struct triedge recenttri;
private:
	int plus1mod3[3];
	int minus1mod3[3];
public:
	String m_strStatistics;
	String strBuffer;

private:

	Bool SmallerAngle(struct triedge& dissolveedge);

public:

	void internalerror();
	void parsecommandline(int argc, char **argv);
private:

	void printtriangle(struct triedge *t);
	void printshelle(struct edge *s);
private:

	void poolinit(struct memorypool *pool, int bytecount, int itemcount, enum wordtype wtype, int alignment);
	void poolrestart(struct memorypool *pool);
	void pooldeinit(struct memorypool *pool);
	int * poolalloc(struct memorypool *pool);
	void pooldealloc(struct memorypool *pool, int *dyingitem);
	void traversalinit(struct memorypool *pool);
	int *traverse(struct memorypool *pool);
	void dummyinit(int trianglewords, int shellewords);
	void initializepointpool();
	void initializetrisegpools();
	void triangledealloc(triangle *dyingtriangle);
	triangle *triangletraverse();
	void shelledealloc(shelle *dyingshelle);
	shelle *shelletraverse();
	void pointdealloc(point dyingpoint);
	point pointtraverse();
	void badsegmentdealloc(struct edge *dyingseg);
	struct edge *badsegmenttraverse();
	point getpoint(int number);
	void triangledeinit();
private:

	void maketriangle(struct triedge *newtriedge);
	void makeshelle(struct edge *newedge);
private:

	void exactinit();
	int fast_expansion_sum_zeroelim(int elen, double *e, int flen, double *f, double *h);
	int scale_expansion_zeroelim(int elen, double *e, double b, double *h);
	double estimate(int elen, double *e);

	double counterclockwiseadapt(point pa, point pb, point pc, double detsum);
	double counterclockwise(point pa, point pb, point pc);

	double incircleadapt(point pa, point pb, point pc, point pd, double permanent);
	double incircle(point pa, point pb, point pc, point pd);
private:
	void triangleinit();
	UInt64 randomnation(unsigned int choices);
	void highorder();
private:

	void checkmesh();
	void checkdelaunay();
	void enqueuebadtri(struct triedge *instri, double angle, point insapex, point insorg, point insdest);
	struct badface *dequeuebadtri();
	int checkedge4encroach(struct edge *testedge);
	void testtriangle(struct triedge *testtri);
private:

	void makepointmap();
	enum locateresult preciselocate(point searchpoint, struct triedge *searchtri);
	enum locateresult locate(point searchpoint, struct triedge *searchtri);
private:
	void insertshelle(struct triedge *tri, int shellemark);
	void flip(struct triedge *flipedge);
	enum insertsiteresult insertsite(point insertpoint, struct triedge *searchtri, struct edge *splitedge,
		int segmentflaws, int triflaws);
	void triangulatepolygon(struct triedge *firstedge, struct triedge *lastedge, int edgecount, int doflip, int triflaws);
	void deletesite(struct triedge *deltri);
private:
	void pointsort(point *sortarray, int arraysize);
	void pointmedian(point *sortarray, int arraysize, int median, int axis);
	void alternateaxes(point *sortarray, int arraysize, int axis);
	void mergehulls(struct triedge *farleft, struct triedge *innerleft, struct triedge *innerright, struct triedge *farright, int axis);
	void divconqrecurse(point *sortarray, int vertices, int axis, struct triedge *farleft, struct triedge *farright);
	long divconqdelaunay();
	long removeghosts(struct triedge *startghost);
private:
	void boundingbox();
	long removebox();
	long incrementaldelaunay();
private:
	void eventheapinsert(struct event **heap, int heapsize, struct event *newevent);
	void eventheapify(struct event **heap, int heapsize, int eventnum);
	void eventheapdelete(struct event **heap, int heapsize, int eventnum);
	void createeventheap(struct event ***eventheap, struct event **events, struct event **freeevents);
	int rightofhyperbola(struct triedge *fronttri, point newsite);
	double circletop(point pa, point pb, point pc, double ccwabc);
	void check4deadevent(struct triedge *checktri, struct event **freeevents, struct event **eventheap, int *heapsize);
	struct splaynode *splay(struct splaynode *splaytree, point searchpoint, struct triedge *searchtri);
	struct splaynode *splayinsert(struct splaynode *splayroot, struct triedge *newkey, point searchpoint);
	struct splaynode *circletopinsert(struct splaynode *splayroot, struct triedge *newkey,
		point pa, point pb, point pc, double topy);
	struct splaynode *frontlocate(struct splaynode *splayroot, struct triedge *bottommost, point searchpoint,
	struct triedge *searchtri, int *farright);
	long sweeplinedelaunay();

private:
	long delaunay();
	int reconstruct(int *trianglelist, double *triangleattriblist, double *trianglearealist, int elements,
		int corners, int attribs, int *segmentlist, int *segmentmarkerlist, int numberofsegments);
private:
	enum finddirectionresult finddirection(struct triedge *searchtri, point endpoint);
	void segmentintersection(struct triedge *splittri, struct edge *splitshelle, point endpoint2);
	int scoutsegment(struct triedge *searchtri, point endpoint2, int newmark);
	void conformingedge(point endpoint1, point endpoint2, int newmark);
	void delaunayfixup(struct triedge *fixuptri, int leftside);
	void constrainededge(struct triedge *starttri, point endpoint2, int newmark);
	void insertsegment(point endpoint1, point endpoint2, int newmark);
	void markhull();
	int formskeleton(int *segmentlist, int *segmentmarkerlist, int numberofsegments);
private:
	void infecthull();
	void plague();
	void regionplague(double attribute, double area);
	void carveholes(double *holelist, int holes, double *regionlist, int regions);
private:
	void tallyencs();
	void precisionerror();
	void repairencs(int flaws);
	void tallyfaces();
	enum circumcenterresult findcircumcenter(point torg, point tdest, point tapex,
		point circumcenter, double *xi, double *eta);
	void splittriangle(struct badface *badtri);
	void enforcequality();
private:
	void transfernodes(double *pointlist, double *pointattriblist,
		int *pointmarkerlist, int numberofpoints, int numberofpointattribs);
	void writenodes(double **pointlist, double **pointattriblist, int **pointmarkerlist);
	void numbernodes();
	void writeelements(int **trianglelist, double **triangleattriblist);
	void writepoly(int **segmentlist, int **segmentmarkerlist);
	void writeedges(int **edgelist, int **edgemarkerlist);
	void writevoronoi(double **vpointlist, double **vpointattriblist, int **vpointmarkerlist,
		int **vedgelist, int **vedgemarkerlist, double **vnormlist);
	void writeneighbors(int **neighborlist);
public:
	void quality_statistics();
	void statistics();
	void triangulate(struct triangulateio *in, struct triangulateio *out, char *triswitches = "-Q", struct triangulateio *vorout = NULL);
};

#endif
