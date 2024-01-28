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
    
    template<typename T>
    struct HalfEdgeMesh
    {
        struct HalfEdge
        {
            int m_vertex; // pointed to by half edge
            int m_left_face; // to the left of half edge
            int m_next;
            int m_prev;
            int m_twin;
            int m_freeLink;
            bool m_active;
            
            //Constructor
            HalfEdge()
            : m_vertex(-1)
            , m_left_face(-1)
            , m_next(-1)
            , m_prev(-1)
            , m_twin(-1)
            , m_freeLink(-1)
            , m_active(false)
            { }
        };
 
        template<typename T>
        struct Vertex
        {
            MathCommon::Vector3<T> m_position;
            int m_half_edge; // a half edge pointing away
            int m_freeLink;
            bool m_active;

            //Constructor
            Vertex()
            : m_half_edge(-1)
            , m_freeLink(-1)
            , m_active(false)
            { }
        };
        
        struct Face
        {
            int m_half_edge; // an incident half edge
            int m_freeLink;
            bool m_active;

            //Constructor
            Face() 
            : m_half_edge(-1)
            , m_freeLink(-1)
            , m_active(false)
            { }
        };
        
        // Typedefs for memory management
        typedef std::vector<Vertex<T>> VertList;
        typedef std::vector<HalfEdge> EdgeList;
        typedef std::vector<Face> FaceList;
        
        // Constructor
        AdjacencyInfo() { Clear(); }
        
        // operations
        int AddVert(const Vec3 &position);
        void RemoveVert(int v);
        int AddFace(int v0, int v1, int v2);
        void RemoveFace(int f);
        void Clear(void);
        
        // utils
        int FindVertEdge(int v) const;
        
        // container for memory management
        VertList<T> m_verts;
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