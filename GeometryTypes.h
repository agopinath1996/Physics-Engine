#pragma once
#include "CommonMathTypes.h"

namespace Geometry
{
    template<typename T>
    struct AABB
    {
        MathCommon::Vector3<T> m_diagonal;

        AABB(){};
        AABB(const T& x_diff,const T& y_diff,const T& z_diff)
        {
            m_diagonal<<x_diff,y_diff,z_diff;
        }
        AABB(const T& x_min,const T& y_min,const T& z_min,const T& x_max,const T& y_max,const T& z_max)
        {
            m_diagonal<<(x_max-x_min),(y_max-y_min),(z_max-z_min);
        }
    };
    template<typename T>
    using AABBList = std::vector<AABB<T>*>;
    struct AdjacencyInfo
{
  // half edge
  struct Edge
  {
    int vert; // pointed to by half edge
    int face; // to the left of half edge
    int next;
    int prev;
    int twin;
    int freeLink;
    bool active;
    Edge(void)
      : vert(-1)
      , face(-1)
      , next(-1)
      , prev(-1)
      , twin(-1)
      , freeLink(-1)
      , active(false)
    { }
  };
 
  struct Vert
  {
    Vec3 position;
    int edge; // a half edge pointing away
    int freeLink;
    bool active;
    Vert(void)
      : edge(-1)
      , freeLink(-1)
      , active(false)
    { }
  };
 
  struct Face
  {
    int edge; // an incident half edge
    int freeLink;
    bool active;
    Face(void) 
      : edge(-1)
      , freeLink(-1)
      , active(false)
    { }
  };
 
  // type defs
  typedef std::vector<Vert> VertList;
  typedef std::vector<Edge> EdgeList;
  typedef std::vector<Face> FaceList;
 
  //constructure
  AdjacencyInfo(void) { Clear(); }
 
  // operations
  int AddVert(const Vec3 &position);
  void RemoveVert(int v);
  int AddFace(int v0, int v1, int v2);
  void RemoveFace(int f);
  void Clear(void);
 
  // utils
  int FindVertEdge(int v) const;
 
  // container for memory management
  VertList m_verts;
  EdgeList m_edges;
  FaceList m_faces;
 
  // free lists
  int m_freeVerts;
  int m_freeEdges;
  int m_freeFaces;
 
  // counters
  uint m_numVerts;
  uint m_numEdges;
  uint m_numFaces;
 
  // edge records
  typedef std::pair<int, int> VertPair;
  typedef std::map<VertPair, int> EdgeMap;
  EdgeMap m_edgeMap;
};
}