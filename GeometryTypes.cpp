#include "GeometryTypes.h"

/* 'T' indicates the individual type and 'Container' is a memory management medium for 'T' 
example: <int,std::vector<int>>*/
template <typename T, typename Container>
int Geometry::Allocate(int &freeList, Container &container)
{
  int index = -1;
  if (freeList < 0)
  {
    // free list empty, just push
    index = static_cast<int>(container.size());
    container.resize(index + 1);
  }
  else
  {
    index = freeList;
 
    // grab free feature    
    T &feature = container[freeList]; 
 
    // free list
    freeList = feature.freeLink;
 
    // remove feature from free list
    feature.freeLink = -1;
  }
  T &feature = container[index];
  feature.active = true;
  return index;
}

template <typename T, typename Container>
static void Geometry::Free(int index, int &freeList, Container &container)
{
  T &feature = container[index];
  feature.freeLink = freeList;
  feature.active = false;
  freeList = index;
}

template <typename T>
int Geometry::HalfEdgeMesh<T>::AddVert(const MathCommon::Vector3<T> &position)
{
  const int v = Geometry::Allocate<Vertex,VertList>(m_freeVerts, m_verts);
  Geometry::HalfEdgeMesh<T>::Vertex &vert = m_verts[v];
 
  // write position
  vert.m_position = position;
 
  // initialize edge index
  vert.m_edge = -1;
 
  ++m_numVerts;
 
  return v;
}

template <typename T>
void Geometry::HalfEdgeMesh<T>::RemoveVert(int v)
{
  Geometry::HalfEdgeMesh<T>::Vertex &vert = m_verts[v];
 
  // remove faces this vert connects to
  const int startEdge = vert.m_edge;
  if (startEdge >= 0)
  {
    int e = startEdge;
    do
    {
      Geometry::HalfEdgeMesh<T>::HalfEdge &edge = m_edges[e];
      Geometry::HalfEdgeMesh<T>::RemoveFace(edge.m_left_face);
      e = m_edges[edge.m_prev].m_twin;
    } while (e != startEdge);
  }
 
  // dispose vert
  if (vert.m_edge >= 0)
    vert.m_edge = -1;
 
  // update free list
  Geometry::Free<Vertex,VertList>(v, m_freeVerts, m_verts);
 
  --m_numVerts;
}

template <typename T>
int Geometry::HalfEdgeMesh<T>::AddFace(int v0, int v1, int v2)
{
  const int faceVerts[3] = { v0, v1, v2 };
   
  // allocate face
  const int f = Allocate<Face,FaceList>(m_freeFaces, m_faces);
   Geometry::HalfEdgeMesh<T>::Face &face = m_faces[f];
 
  // create edges
  {
    // iterate face edges
    int faceEdgeIndices[3] = {-1, -1, -1};
    for (uint i = 2, j = 0; j < 3; i = j++)
    {
      const uint v0 = faceVerts[i];
      const uint v1 = faceVerts[j];
      auto &vert0 = m_verts[v0];
      auto &vert1 = m_verts[v1];
 
      // check existence of half edge pair
      const auto edgeIter = 
        m_edgeMap.find(VertPair(v0, v1));
      const bool edgePairExists = 
        edgeIter != m_edgeMap.end();
      int e01 = -1;
      int e10 = -1;
      if (edgePairExists)
      {
        e01 = edgeIter->second;
        e10 = m_edges[e01].twin;
      }
      else
      {
        // allocate & init half edge pair
        e01 = Allocate<HalfEdge,EdgeList>(m_freeEdges, m_edges);
        e10 = Allocate<HalfEdge,EdgeList>(m_freeEdges, m_edges);
 
        // link twins
        m_edges[e01].twin = e10;
        m_edges[e10].twin = e01;
 
        // record edge existence
        m_edgeMap[VertPair(v0, v1)] = e01;
        m_edgeMap[VertPair(v1, v0)] = e10;
 
        m_numEdges += 2;
        m_numBoundaryEdges += 2;
      } // end of edge allocation
 
      auto &edge01 = m_edges[e01];
      auto &edge10 = m_edges[e10];
 
      // link vert to edges
      if (edge01.vert < 0)
        edge01.vert = v1;
      if (edge10.vert < 0)
        edge10.vert = v0;
 
      // link face to edge
      if (edge01.face < 0)
      {
        edge01.face = f;
        --m_numBoundaryEdges;
      }
       
      // link edge to vert
      if (vert0.edge < 0)
        vert0.edge = e01;
 
      // link edge to face
      if (face.edge < 0)
        face.edge = e01;
 
      // record face edges
      faceEdgeIndices[i] = e01;
    }
 
    // link face edges
    for (unsigned i = 2, j = 0; j < 3; i = j++)
    {
      const int eI = faceEdgeIndices[i];
      const int eJ = faceEdgeIndices[j];
      m_edges[eI].next = eJ;
      m_edges[eJ].prev = eI;
    }
 
  } // end of edge creation
 
  ++m_numFaces;
  return f;
}

template <typename T>
int Geometry::HalfEdgeMesh<T>::FindVertEdge(int v) const
{
  // POSSIBLE OPTIMIZATION: 
  //   this is currently a linear operation
  for (auto &pair : m_edgeMap)
  {
    if (v == pair.first.first)
      return pair.second;
  }
  return -1;
}
 
template <typename T>
void Geometry::HalfEdgeMesh<T>::RemoveFace(int f)
{
  // remove adjacent edges
  const int startEdge = m_faces[f].edge;
 
  int faceVertIndices[3] = {-1, -1, -1};
  int faceEdgeIndices[3] = {-1, -1, -1};
  int e = startEdge;
  int i = 0;
  do
  {
    auto &edge = m_edges[e];
    const int t = edge.m_twin;
    const int n = edge.m_next;
    const int p = edge.m_prev;
    auto &twin = m_edges[t];
    auto &next = m_edges[n];
    auto &prev = m_edges[p];
 
    faceVertIndices[i] = edge.m_vert;
    faceEdgeIndices[i] = e;
    ++i;
 
    // free both edges if twin face does not exist
    if (twin.face < 0)
    {
      m_edgeMap.erase(VertPair(edge.m_vert, prev.m_vert));
      m_edgeMap.erase(VertPair(prev.m_vert, edge.m_vert));
 
      edge.m_twin = -1;
      twin.m_twin = -1;
 
      Free<HalfEdge,EdgeList>(e, m_freeEdges, m_edges);
      Free<HalfEdge,EdgeList>(t, m_freeEdges, m_edges);
 
      m_numEdges -= 2;
      m_numBoundaryEdges -= 2;
    }
 
    ++m_numBoundaryEdges;
 
    e = n;
  } while (e != startEdge);
 
  // unlink everything from edges
  for (const int e : faceEdgeIndices)
  {
    // okay to access data members after fake &quot;free&quot;
    auto &edge = m_edges[e];
    edge.next = -1;
    edge.prev = -1;
    edge.vert = -1;
    edge.face = -1;
  }
 
  // update vert edge
  for (const int v : faceVertIndices)
  {
    auto &vert = m_verts[v];
    vert.m_edge = FindVertEdge(v);
  }
 
  // unlink edge from face
  auto &face = m_faces[f];
  face.m_edge = -1;
 
  // finally, free face
  Free<Face>(f, m_freeFaces, m_faces);
 
  --m_numFaces;
}

template <typename T>
void Geometry::HalfEdgeMesh<T>::Clear()
{
  m_verts.clear();
  m_edges.clear();
  m_faces.clear();
 
  m_numVerts = 0;
  m_numEdges = 0;
  m_numFaces = 0;
 
  m_freeVerts = -1;
  m_freeEdges = -1;
  m_freeFaces = -1;
 
  m_edgeMap.clear();
}

template <typename T>
const MathCommon::Vector3<T> Support(const Geometry::HalfEdgeMesh<T> &mesh, const MathCommon::Vector3<T> &dir)
{
  // grab adjacency information
  auto &adj = mesh;
  auto &verts = adj.m_verts;
  auto &edges = adj.m_edges;
   
  // start from first point
  int bestVertIndex = 0;
  auto bestVert = &verts[bestVertIndex];
  T bestDot = dir.dot(bestVert->m_position);
 
  bool end = false;
  while (!end)
  {
    // loop through all edges leaving best vert
    int e = bestVert->m_half_edge;
    int startEdge = e;
    const float oldBestDot = bestDot;
    do
    {
      auto &edge = edges[e];
 
      // get adjacent vert pointed by edge
      const int vertIndex = edge.m_vertex;
      auto vert = &verts[vertIndex];
 
      const float dot = dir.dot(vert->m_position);
      if (dot >= bestDot + MathCommon::EPSILON)
      {
        // record new best vert
        bestVertIndex = bestVertIndex;
        bestVert = vert;
        bestDot = dot;
      }
 
      // get next out-going edge
      e = edges[edge.m_twin].m_next;
    } while (startEdge != e);
 
    // end if no new best vert is found
    end = (bestDot == oldBestDot);
  }
 
  return bestVert->m_position;
}