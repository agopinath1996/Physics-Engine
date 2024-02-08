#pragma once
#include "RigidBody.h"
#include "CommonMathTypes.h"

namespace CollisionDetection
{
    template<typename T>
    void EPA(Body::Collider<T> colliderA, Body::Collider<T> colliderB)
    {
        
    // TODO: output from GJK
    MathCommon::Vector3<T> simplex[4]; // global support points
    MathCommon::Vector3<T> simplexA[4]; // local support points for collider A
    MathCommon::Vector3<T> simplexB[4]; // local support points for collider B
    unsigned numVerts;
    
    // simplex declaration from GJK


    // Internal floating point margin
    const T k_epsilon = T(0.00001);
    const T k_epsilonSq = k_epsilon * k_epsilon;
    
    // constant vector representing the origin
    const MathCommon::MathCommon::Vector3<T> k_origin(T(0.0), T(0.0), T(0.0));
    
    // ... GJK stuff ...
    
    // Expand the simplex from GJK into a tetrahedron
    switch (numVerts)
    {
        /*
        As GJK output might have 1 to 4 verteces we need to iteratively expand the simplex
        */
    case 1:
    
        // 6 principal directions
        static const MathCommon::Vector3<T> k_searchDirs[] = 
        {
            /*
                |  1  0  0 |
                | -1  0  0 |
                |  0  1  0 |
                |  0 -1  0 |
                |  0  0  1 |
                |  0  0 -1 |
            */
            MathCommon::Vector3<T>(T(1.0),  T(0.0),  T(0.0)), 
            MathCommon::Vector3<T>(T(-1.0),  T(0.0),  T(0.0)), 
            MathCommon::Vector3<T>(T(0.0),  T(1.0),  T(0.0)), 
            MathCommon::Vector3<T>(T(0.0),  T(-1.0),  T(0.0)), 
            MathCommon::Vector3<T>(T(0.0),  T(0.0),  T(1.0)), 
            MathCommon::Vector3<T>(T(0.0),  T(0.0),  T(-1.0)), 
        };
    
        // iterate until a good search direction is used
        for (const MathCommon::Vector3<T> &searchDir : k_searchDirs)
        {
        CsoSupport(colliderA, colliderB, searchDir, 
                    simplex[1], simplexA[1], simplexB[1]);
    
        // good search direction used, break
        if ((simplex[1] - simplex[0]).squaredNorm() >= k_epsilonSq)
            break;
        }
        // end of case 1
    
    
    case 2:
    
        // 3 principal axes
        static const MathCommon::Vector3<T> k_axes[3] =
        { MathCommon::Vector3<T>(T(1.0), T(0.0), T(0.0)), 
            MathCommon::Vector3<T>(T(0.0), T(1.0), T(0.0)), 
            MathCommon::Vector3<T>(T(0.0), T(0.0), T(1.0)) };
    
        // line direction vector
        const MathCommon::Vector3<T> lineVec = simplex[1] - simplex[0];
        
        // find least significant axis of line direction for initial search direction
        MathCommon::Vector3<T> searchDir = (LeastSignificantComponent(lineVec)).Y;

        // build a rotation matrix of 60 degrees about line vector
        MathCommon::Matrix3<T> rotationMatrix = (Eigen::AngleAxis<T>(MathCommon::PI/3, lineVec)).toRotationMatrix();
    
        // find up to 6 directions perpendicular to the line vector
        // until a good search direction is used
        for (int i = 0; i < 6; ++i)
        {
        CsoSupport(colliderA, colliderB, searchDir, 
                    simplex[2], simplexA[2], simplexB[2]);
    
        // good search direction used, break
        if (simplex[2].squaredNorm() > k_epsilonSq)
            break;
    
        // rotate search direction by 60 degrees
        searchDir = rot * searchDir;
        }
        // end of case 2
    
    
    case 3:
        
        // use triangle normal as search direction
        const MathCommon::Vector3<T> v01 = simplex[1] - simplex[0];
        const MathCommon::Vector3<T> v02 = simplex[2] - simplex[0];
        MathCommon::Vector3<T> searchDir = v01.Cross(v02);
    
        CsoSupport(colliderA, colliderB, searchDir, 
                simplex[3], simplexA[3], simplexB[3]);
        
        // search direction not good, use its opposite direction
        if (simplex[3].LengthSQ &amp;lt; k_epsilonSq)
        {
        searchDir.Negate();
        CsoSupport(colliderA, colliderB, searchDir, 
                    simplex[3], simplexA[3], simplexB[3]);
        }
        // end of case 3
    }
    
    // fix tetrahedron winding
    // so that simplex[0]-simplex[1]-simplex[2] is CCW winding
    const MathCommon::Vector3<T> v30 = simplex[0] - simplex[3];
    const MathCommon::Vector3<T> v31 = simplex[1] - simplex[3];
    const MathCommon::Vector3<T> v32 = simplex[2] - simplex[3];
    const float det = v30.Dot(v31.Cross(v32));
    if (det > T(0.0))
    {
    std::swap(verts [0], verts [1]);
    std::swap(vertsA[0], vertsA[1]);
    std::swap(vertsB[0], vertsB[1]);
    }
        }

    template<typename T>
    MathCommon::Basis3D<T> BasisFromDirection(const MathCommon::Vector3<T>& direction)
    {
        // crreate a basis using the least significant component of a direction
        //Amazing code originally in python but templated here for C++, Erin Catto @ https://box2d.org/
        if (direction[0] >= T(0.57735))
            MathCommon::Vector3<T> t1(direction[1], -direction[0], T(0.0));
        else
            MathCommon::Vector3<T> t1(T(0.0), direction[2], -direction[1]);
    
        //least significant component of a direction
        t1.normalize();


        t2 = (direction.cross(t1)).normalize();
        return MathCommon::Basis3D<T>(direction,t1,t2);
    }
}