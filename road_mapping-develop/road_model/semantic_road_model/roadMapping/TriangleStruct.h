#ifndef ENGINE_PCPROCESS_TRIANGLESTRUCT_H_ 
#define ENGINE_PCPROCESS_TRIANGLESTRUCT_H_ 

enum wordtype {POINTER, FLOATINGPOINT};
enum locateresult {INTRIANGLE, ONEDGE, ONVERTEX, OUTSIDE};
enum insertsiteresult {SUCCESSFULPOINT, ENCROACHINGPOINT, VIOLATINGPOINT,DUPLICATEPOINT};
enum finddirectionresult {WITHIN, LEFTCOLLINEAR, RIGHTCOLLINEAR};
enum circumcenterresult {OPPOSITEORG, OPPOSITEDEST, OPPOSITEAPEX};
typedef double **triangle;
struct triedge 
{
  triangle *tri;
  int orient;                                         
};
typedef double **shelle;                  
struct edge 
{
  shelle *sh;
  int shorient;                           
};
typedef double *point;
struct badsegment 
{
  struct edge encsegment;                 
  point segorg, segdest;                  
  struct badsegment *nextsegment;     
};
struct badface 
{
  struct triedge badfacetri;          
  double key;                         
  point faceorg, facedest, faceapex;  
  struct badface *nextface;           
};
struct event 
{
  double xkey, ykey;                  
  int *eventptr;       
  int heapposition;    
};
struct splaynode 
{
  struct triedge keyedge;                  
  point keydest;            
  struct splaynode *lchild, *rchild;           
};
struct memorypool 
{
  int **firstblock, **nowblock;
  int *nextitem;
  int *deaditemstack;
  int **pathblock;
  int *pathitem;
  enum wordtype itemwordtype;
  int alignbytes;
  int itembytes, itemwords;
  int itemsperblock;
  long items, maxitems;
  int unallocateditems;
  int pathitemsleft;
};

struct triangulateio 
{
  double *pointlist;                                               
  double *pointattributelist;                                      
  int *pointmarkerlist;                                          
  int numberofpoints;                                            
  int numberofpointattributes;                                   

  int *trianglelist;                                             
  double *triangleattributelist;                                 
  double *trianglearealist;                                      
  int *neighborlist;                                             
  int numberoftriangles;                                         
  int numberofcorners;                                           
  int numberoftriangleattributes;                                

  int *segmentlist;                                              
  int *segmentmarkerlist;                                        
  int numberofsegments;                                          

  double *holelist;                        
  int numberofholes;                       

  double *regionlist;                      
  int numberofregions;                     

  int *edgelist;                           
  int *edgemarkerlist;           
  double *normlist;              
  int numberofedges;             
};

#endif

