#pragma once
#include <map>
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
    struct Simplex 
    {
        std::array<MathCommon::Vector3<T>, 4> m_points;
        int m_size;

        Simplex(): m_size (0)
        {}

        Simplex& operator=(std::initializer_list<MathCommon::Vector3<T>> list) 
        {
            for (MathCommon::Vector3<T> point : list)
                m_points[m_size++] = point;

            return *this;
        }

        void push_front(MathCommon::Vector3<T> point) 
        {
            m_points = { point, m_points[0], m_points[1], m_points[2] };
            m_size = std::min(m_size + 1, 4);
        }

        MathCommon::Vector3<T>& operator[](int i) { return m_points[i]; }
    };
    
    template<typename T>
    using AABBList = std::vector<AABB<T>*>;
    
    template<typename T>
    struct HalfEdgeMesh
    {
        // Half edge = edge in this implementation
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
            : m_vertex(-1),
            m_left_face(-1),
            m_next(-1),
            m_prev(-1),
            m_twin(-1),
            m_freeLink(-1),
            m_active(false)
            { }
        };

        struct Vertex
        {
            MathCommon::Vector3<T> m_position;
            int m_half_edge; // a half edge pointing away
            int m_freeLink;
            bool m_active;

            //Constructor
            Vertex()
            : m_half_edge(-1),
            m_freeLink(-1),
            m_active(false)
            { }
        };
        
        struct Face
        {
            int m_half_edge; // an incident half edge
            int m_freeLink;
            bool m_active;

            //Constructor
            Face() 
            : m_half_edge(-1),
            m_freeLink(-1),
            m_active(false)
            { }
        };
        
        // Typedefs for memory management
        typedef std::vector<Vertex> VertList;
        typedef std::vector<HalfEdge> EdgeList;
        typedef std::vector<Face> FaceList;
        
        // Constructor
        HalfEdgeMesh() { Clear(); }
        
        // operations
        int AddVert(const MathCommon::Vector3<T> &position);
        void RemoveVert(int v);
        int AddFace(int v0, int v1, int v2);
        void RemoveFace(int f);
        void Clear();
        
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
        uint m_numBoundaryEdges;
        
        // edge records
        typedef std::pair<int, int> VertPair;
        typedef std::map<VertPair, int> EdgeMap;
        EdgeMap m_edgeMap;
    };

    template <typename T, typename Container>
    int Allocate(int &freeList, Container &container);

    template <typename T, typename Container>
    static void Free(int index, int &freeList, Container &container);

    template <typename T>
    const MathCommon::Vector3<T> Support(const Geometry::HalfEdgeMesh<T> &mesh, const MathCommon::Vector3<T> &dir, const T& epsilon);

    template<typename T>
    struct CollisionPoints
    {
        MathCommon::Vector3<T> m_normal;
        T m_penetrationDepth;
        bool m_hasCollision;

        CollisionPoints()
        {
            m_normal<<T(0.0),T(0.0),T(0.0);
            m_penetrationDepth = T(0.0);
            m_hasCollision = false;
        }
    };

    // Helper function to find if tow vectors are in the same direction
    template<typename T>
    bool SameDirection(const MathCommon::Vector3<T>& direction, const MathCommon::Vector3<T>& direction2)
    {
        return (direction.dot(direction2) > 0);
    }

    template<typename T>
    bool Line(Simplex<T>& points, MathCommon::Vector3<T>& direction)
    {
        MathCommon::Vector3<T> a = points[0];
        MathCommon::Vector3<T> b = points[1];

        MathCommon::Vector3<T> ab = b - a;
        MathCommon::Vector3<T> ao =  -a;
    
        if (SameDirection(ab, ao)) 
        {
            direction = (ab.cross(ao)).cross(ab);
        }

        else 
        {
            points = { a };
            direction = ao;
        }

        return false;
    }

    template<typename T>
    bool Triangle(Simplex<T>& points, MathCommon::Vector3<T>& direction)
    {
        MathCommon::Vector3<T> a = points[0];
        MathCommon::Vector3<T> b = points[1];
        MathCommon::Vector3<T> c = points[2];

        MathCommon::Vector3<T> ab = b - a;
        MathCommon::Vector3<T> ac = c - a;
        MathCommon::Vector3<T> ao =   - a;
    
        MathCommon::Vector3<T> abc = ab.cross(ac);
    
        if (SameDirection((abc.cross(ac)).cross(ao)) 
        {
            if (SameDirection(ac, ao)) 
            {
                points = { a, c };
                direction = cross(cross(ac, ao), ac);
            }

            else 
            {
                return Line(points = { a, b }, direction);
            }
        }
    
        else 
        {
            if (SameDirection(ab.cross(abc), ao)) 
            {
                return Line(points = { a, b }, direction);
            }

            else 
            {
                if (SameDirection(abc, ao)) 
                {
                    direction = abc;
                }

                else 
                {
                    points = { a, c, b };
                    direction = -abc;
                }
            }
        }

        return false;
    }

    template<typename T>
    bool Tetrahedron(Simplex<T>& points, MathCommon::Vector3<T>& direction)
    {
        MathCommon::Vector3<T> a = points[0];
        MathCommon::Vector3<T> b = points[1];
        MathCommon::Vector3<T> c = points[2];
        MathCommon::Vector3<T> d = points[3];

        MathCommon::Vector3<T> ab = b - a;
        MathCommon::Vector3<T> ac = c - a;
        MathCommon::Vector3<T> ad = d - a;
        MathCommon::Vector3<T> ao =   - a;
    
        MathCommon::Vector3<T> abc = cross(ab, ac);
        MathCommon::Vector3<T> acd = cross(ac, ad);
        MathCommon::Vector3<T> adb = cross(ad, ab);
    
        if (SameDirection(abc, ao)) 
        {
            return Triangle(points = { a, b, c }, direction);
        }
            
        if (SameDirection(acd, ao)) 
        {
            return Triangle(points = { a, c, d }, direction);
        }
    
        if (SameDirection(adb, ao)) 
        {
            return Triangle(points = { a, d, b }, direction);
        }
    
        return true;
    }

}
