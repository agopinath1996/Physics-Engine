#pragma once
#include "RigidBody.h"
#include "GeometryTypes.h"
#include "CommonMathTypes.h"
#include <stddef.h>

namespace CollisionDetection
{
/*
-------------------------------------------------------------Helper functions-------------------------------------------------------------
*/   
    void AddIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& faces, size_t a, size_t b)
    {
        auto reverse = std::find(edges.begin(), edges.end(),std::make_pair(faces[b], faces[a]));
        if (reverse != edges.end()) 
        {
            edges.erase(reverse);
        }
    
        else 
        {
            edges.emplace_back(faces[a], faces[b]);
        }
    }

    template <typename T>
    std::pair<std::vector<MathCommon::Vector4<T>>, size_t> GetFaceNormals(const std::vector<MathCommon::Vector3<T>>& polytope, const std::vector<size_t>& faces)
    {
        //Possible optimization: once a plane is calculated, save it in the normals struct with distanace as well
        std::vector<MathCommon::Vector4<T>> normals;
        size_t minTriangle = 0;
        T  minDistance = MathCommon::INF<T>;

        for (size_t i = 0; i < faces.size(); i += 3) 
        {
            MathCommon::Vector3<T> a = polytope[faces[i    ]];
            MathCommon::Vector3<T> b = polytope[faces[i + 1]];
            MathCommon::Vector3<T> c = polytope[faces[i + 2]];

            MathCommon::Vector3<T> normal = ((b - a).cross(c - a)).normalized();
            T distance = normal.dot(a);

            if (distance < 0) {
                normal   *= -1;
                distance *= -1;
            }

            normals.emplace_back(MathCommon::Vector4<T>()<<normal,distance);

            if (distance < minDistance) {
                minTriangle = i / 3;
                minDistance = distance;
            }
        }

        return { normals, minTriangle };
    }
    
    /*
    Test if the dot product between the face normal (CCW winding assumed) and the vector from any one of the triangle’s vertex to the point is greater than zero.
    */
    template <typename T>
    bool SameDirection(const MathCommon::Vector4<T>&direction, 
                    const MathCommon::Vector3<T>& point, 
                    const std::vector<MathCommon::Vector3<T>>& faceEdges)
    {
        return (direction.dot(point - faceEdges[0]) >=0)
            || (direction.dot(point - faceEdges[1]) >=0)
            || (direction.dot(point - faceEdges[2]) >=0);
    }

    template <typename T>
    bool NextSimplex(Geometry::Simplex<T>& points, MathCommon::Vector3<T>& direction)
    {
        switch (points.size()) 
        {
        case 2: 
            return Geometry::Line(points, direction);
        case 3: 
            return Geometry::Triangle(points, direction);
        case 4: 
            return Geometry::Tetrahedron(points, direction);
        }
    
        // fallthrough case
        return false;
    }

    /*
-------------------------------------------------------------Main function-------------------------------------------------------------
    */

    template<typename T>
    Geometry::CollisionPoints<T> EPA(Body::Collider<T> colliderA, Body::Collider<T> colliderB)
    {
        // Simplex initialization for GJK
        Geometry::Simplex<T> simplex; // global support points
        Geometry::Simplex<T> simplexA; // local support points for collider A
        Geometry::Simplex<T> simplexB; // local support points for collider B
        MathCommon::Vector3<T> support;
        MathCommon::Vector3<T> supportA;
        MathCommon::Vector3<T> supportB;

        Geometry::CollisionPoints<T> collisionPoints;
        unsigned numVerts;

        // Internal floating point margin
        const T k_epsilon = T(0.00001);
        const T k_epsilonSq = k_epsilon * k_epsilon;
        
        // constant vector representing the origin
        const MathCommon::Vector3<T> k_origin(T(0.0), T(0.0), T(0.0));
        
         /*
-------------------------------------------------------------GJK start-------------------------------------------------------------
        */

        // Get initial support point in any direction
	    CSOSupport(colliderA, colliderB, MathCommon::Vector3<T>(T(1.0), T(0.0), T(0.0)), support, supportA, supportB);

        simplex.push_front(support);
        
        // New direction is towards the origin
	    MathCommon::Vector3<T> direction = -support;
        
        while(!collisionPoints.m_hasCollision)
        {
            CSOSupport(colliderA, colliderB, direction, support, supportA, supportB);
    
            if ((support.dot(direction) <= 0)) 
            {
                /*
                Breakout case1 
                Support point is behind th origin in that direction (Origin is outside CSO)
                */
               return collisionPoints;
               break;
            }

            simplex.push_front(support);

            if (NextSimplex(simplex, direction)) 
            {
			    collisionPoints.m_hasCollision = true;
		    }
        }


        /*
-------------------------------------------------------------EPA start-------------------------------------------------------------
        */
        if(simplex.m_size<4)
        {
            MathCommon::Vector3<T> searchDir;
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

            // Expand the simplex from GJK into a tetrahedron
            switch (simplex.m_size)
            {
                /*
                As GJK output might have 1 to 4 verteces we need to iteratively expand the simplex
                NOTE: break statements have been ommitted for case fallthrough
                */
            case 1:
                /*
                Case where GJK found the first support point to be the closest point in simplex to origin
                */

                // iterate until a good search direction is used
                for (const MathCommon::Vector3<T> &searchDir : k_searchDirs)
                {
                    CsoSupport(colliderA, colliderB, searchDir, simplex.m_points[1], simplexA.m_points[1], simplexB.m_points[1]);
                
                    // good search direction used, break
                    if ((simplex.m_points[1] - simplex.m_points[0]).squaredNorm() >= k_epsilonSq)
                        break;
                }
                // end of case 1


            case 2:
                /*
                Case where GJK found the second support point to be the closest point in simplex to origin
                */
                // 3 principal axes
                static const MathCommon::Vector3<T> k_axes[3] = {MathCommon::Vector3<T>(T(1.0), T(0.0), T(0.0)), 
                                                                MathCommon::Vector3<T>(T(0.0), T(1.0), T(0.0)), 
                                                                MathCommon::Vector3<T>(T(0.0), T(0.0), T(1.0))};

                // line direction vector
                const MathCommon::Vector3<T> lineVec = simplex.m_points[1] - simplex.m_points[0];
                
                // find least significant axis of line direction for initial search direction
                searchDir = (MathCommon::BasisFromDirection(lineVec)).Y;

                // build a rotation matrix of 60 degrees about line vector
                MathCommon::Matrix3<T> rotationMatrix = (Eigen::AngleAxis<T>(MathCommon::PI<T>/3, lineVec)).toRotationMatrix();

                // find up to 6 directions perpendicular to the line vector
                // until a good search direction is used
                for (int i = 0; i < 6; ++i)
                {
                    CsoSupport(colliderA, colliderB, searchDir, simplex.m_points[2], simplexA.m_points[2], simplexB.m_points[2]);
                
                    // good search direction used, break
                    if (simplex.m_points[2].squaredNorm() > k_epsilonSq)
                        break;
                
                    // rotate search direction by 60 degrees
                    searchDir = rotationMatrix * searchDir;
                }
                // end of case 2


            case 3:
                /*
                Case where GJK found the second third support point to be the closest point in simplex to origin
                */
                // use triangle normal as search direction
                const MathCommon::Vector3<T> v01 = simplex.m_points[1] - simplex.m_points[0];
                const MathCommon::Vector3<T> v02 = simplex.m_points[2] - simplex.m_points[0];
                searchDir = v01.cross(v02);

                CsoSupport(colliderA, colliderB, searchDir, simplex.m_points[3], simplexA.m_points[3], simplexB.m_points[3]);
                
                // search direction not good, use its opposite direction
                if (simplex.m_points[3].squaredNorm() <= k_epsilonSq)
                {
                    searchDir = -searchDir;
                    CsoSupport(colliderA, colliderB, searchDir, simplex.m_points[3], simplexA.m_points[3], simplexB.m_points[3]);
                }
                // end of case 3
            }
        }
        
        // fix tetrahedron winding
        // so that simplex.m_points[0]-simplex.m_points[1]-simplex.m_points[2] is CCW winding
        const MathCommon::Vector3<T> v30 = simplex.m_points[0] - simplex.m_points[3];
        const MathCommon::Vector3<T> v31 = simplex.m_points[1] - simplex.m_points[3];
        const MathCommon::Vector3<T> v32 = simplex.m_points[2] - simplex.m_points[3];
        const T det = v30.dot(v31.cross(v32));

        if (det > T(0.0))
        {
            std::swap(simplex.m_points[0], simplex.m_points[1]);
            std::swap(simplexA.m_points[0], simplexA.m_points[1]);
            std::swap(simplexB.m_points[0], simplexB.m_points[1]);
        }

        // Main EPA
        std::vector<MathCommon::Vector3<T>> polytope(simplex.begin(), simplex.end());
	    std::vector<size_t> faces = {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2
        };

        // list: vec4(normal, distance), index: min distance
        auto [normals, minFace] = GetFaceNormals(polytope, faces);

        MathCommon::Vector3<T>  minNormal;
        T minDistance = MathCommon::INF<T>;

        while (minDistance == MathCommon::INF<T>) 
        {
            //Get closest normal from origin
            minNormal<<normals[minFace](0),normals[minFace](1),normals[minFace](2);
            minDistance = normals[minFace](3);
            
            // Support point using that normal
            Support(colliderA, colliderB, minNormal, support, supportA, supportB);

            //Distance between face and support point
            T sDistance = minNormal.dot(support);

            if (abs(sDistance - minDistance) > T(0.001)) 
            {
                minDistance = MathCommon::INF<T>;
                
                std::vector<std::pair<size_t, size_t>> uniqueEdges;

                for (size_t i = 0; i < normals.size(); i++) 
                {
                    std::vector<MathCommon::Vector3<T>> facePoints = {polytope[i],polytope[i+1],polytope[i+2]};
                    if (SameDirection(normals[i], support, facePoints)) 
                    {
                        size_t f = i * 3;

                        AddIfUniqueEdge(uniqueEdges, faces, f,     f + 1);
                        AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
                        AddIfUniqueEdge(uniqueEdges, faces, f + 2, f    );

                        faces[f + 2] = faces.back(); faces.pop_back();
                        faces[f + 1] = faces.back(); faces.pop_back();
                        faces[f    ] = faces.back(); faces.pop_back();

                        normals[i] = normals.back(); // pop-erase
                        normals.pop_back();

                        i--;
                    }
                }

                std::vector<size_t> newFaces;
                for (auto [edgeIndex1, edgeIndex2] : uniqueEdges) 
                {
                    newFaces.push_back(edgeIndex1);
                    newFaces.push_back(edgeIndex2);
                    newFaces.push_back(polytope.size());
                }
                
                polytope.push_back(support);

                auto [newNormals, newMinFace] = GetFaceNormals(polytope, newFaces);

                T oldMinDistance = MathCommon::INF<T>;
                
                for (size_t i = 0; i < normals.size(); i++) 
                {
                    if (normals[minFace](3) < oldMinDistance) {
                        oldMinDistance = normals[minFace](3);
                        minFace = i;
                    }
                }
    
                if (newNormals[newMinFace].w < oldMinDistance) 
                {
                    minFace = newMinFace + normals.size();
                }
    
                faces  .insert(faces  .end(), newFaces  .begin(), newFaces  .end());
                normals.insert(normals.end(), newNormals.begin(), newNormals.end());
            }
        }

        collisionPoints.m_normal = minNormal;
        collisionPoints.m_penetrationDepth = minDistance + T(0.001);
        collisionPoints.m_hasCollision = true;

        return collisionPoints;
    }
}