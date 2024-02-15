#pragma once
#include<utility>
#include<list>
#include<vector>
#include "CommonMathTypes.h"
#include "GeometryTypes.h"

namespace Body
{
    template<typename T>
    struct Collider
    {
        /* 
        A collider is a subset of a RigidBody that is capable of collision
        */

        // Mechanical properties
        T m_mass;
        /*Potential for improvement*/
        MathCommon::Matrix3<T> m_localInertiaTensor;
        MathCommon::Vector3<T> m_localCentroid;

        // Geometrical properties
        Geometry::AABB<T> m_aabb;
        Geometry::HalfEdgeMesh<T> m_surface;
        
        MathCommon::Vector3<T> Support(const MathCommon::Vector3<T>& direction)
        {
            MathCommon::Vector3<T>  maxPoint;
            T maxDistance = -MathCommon::INF<T>;

            // Brute force search on all the vertices
            // TODO: Optimize this using sign of direction
            for (MathCommon::Vector3<T>& vertex : m_surface.m_verts) 
            {
                T distance = vertex.dot(direction);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxPoint = vertex;
                }
            }
            return maxPoint;
        }
        

    };

    template<typename T>
    using ColliderList = std::vector<Collider<T>* >; // returns a list of colliders whose AABBs collide 

    template<typename T>
    using ColliderPair = std::pair<Collider<T> *, Collider<T> *>;

    template<typename T>
    using ColliderPairList = std::list<ColliderPair<T>>;

    template<typename T>
    struct RayCastResult
    {
        bool hit;
        Collider<T> *collider;
        MathCommon::Plane3D<T> surface_hit;
    };

    template <typename T>
    struct ResultEntry
    {
        Collider<T> *collider;
        float t; // Eigen::Matrices perform type promotion
        MathCommon::Vector3<T> normal;

        bool operator<(const ResultEntry &rhs) const
        {
            // smaller t = closer
            return t > rhs.t;
        }
    };

    template <typename T>
    using ResultList = std::vector<ResultEntry<T>>;

    template<typename T>
    struct RigidBody
    {
        T mass;
        T mass_inverse;
        MathCommon::Matrix3<T> m_localInverseInertiaTensor;
        MathCommon::Matrix3<T> m_globalInverseInertiaTensor;
        MathCommon::Vector3<T> m_globalCentroid;
        MathCommon::Vector3<T> m_localCentroid;

        // World Frame
        /*Potential for improvement*/
        MathCommon::Vector3<T> m_position;  
        MathCommon::Matrix3<T> m_orientation;
        MathCommon::Matrix3<T> m_inverseOrientation;
        MathCommon::Vector3<T> m_linearVelocity;
        MathCommon::Vector3<T> m_angularVelocity;

        MathCommon::Vector3<T> m_forceAccumulator;
        MathCommon::Vector3<T> m_torqueAccumulator;

        ColliderList<T> m_collider_list;

        void UpdateGlobalCentroidFromPosition(void);
        void UpdatePositionFromGlobalCentroid(void);

        void UpdateOrientation(void);

        void AddCollider(Collider<T> &collider);

        const MathCommon::Vector3<T> LocalToGlobal(const MathCommon::Vector3<T> &local) const;
        const MathCommon::Vector3<T> GlobalToLocal(const MathCommon::Vector3<T> &global) const;
        const MathCommon::Vector3<T> LocalToGlobalVec(const MathCommon::Vector3<T> &v) const;
        const MathCommon::Vector3<T> GlobalToLocalVec(const MathCommon::Vector3<T> &v) const;

        void ApplyForce(const MathCommon::Vector3<T> &f, const MathCommon::Vector3<T> &at);
    };



    // convert below to .cpp file
    template<typename T>
    void RigidBody<T>::UpdateGlobalCentroidFromPosition(void)
    {
        m_globalCentroid = 
        m_orientation * m_localCentroid + m_position;
    }

    template<typename T>
    void RigidBody<T>::UpdatePositionFromGlobalCentroid(void)
    {
        m_position = 
        m_orientation * (-m_localCentroid) + m_globalCentroid;
    }

    template<typename T>
    void RigidBody<T>::AddCollider(Collider<T> &collider)
    {
        // add collider to collider list
        m_collider_list.push_back(collider);

        // reset local centroid & mass
        m_localCentroid.setZero();
        mass = T(0);

        // compute local centroid & mass
        for (int i=0;i<m_collider_list.size();i++)
        {
            // accumulate mass
            mass += m_collider_list[i].m_mass;

            // accumulate weighted contribution
            m_localCentroid += 
            m_collider_list[i].m_mass * m_collider_list[i].m_localCentroid;
        }

        // compute inverse mass
        mass_inverse = T(1) / mass;

        // compute final local centroid
        m_localCentroid *= mass_inverse;

        // compute local inertia tensor
        MathCommon::Matrix3<T> localInertiaTensor;
        localInertiaTensor.setzero();

        for (int i=0;i<m_collider_list.size();i++)
        {
            const MathCommon::Vector3<T> r = m_localCentroid - m_collider_list[i].m_localCentroid;
            const T rDotr = r.dot(r);
            const MathCommon::Matrix3<T> rOutr = r*r.transpose();

            // accumulate local inertia tensor contribution, 
            // using Parallel Axis Theorem
            localInertiaTensor += 
            collider.localInertiaTensor 
            + collider.m_mass * (rDotr * MathCommon::Identity3<T> - rOutr);
        }

    // compute inverse inertia tensor
        m_localInverseInertiaTensor = localInertiaTensor.inverse();
    }

    template<typename T>
    const MathCommon::Vector3<T> RigidBody<T>::LocalToGlobal(const MathCommon::Vector3<T> &local) const
    {
        return m_orientation * local + m_position;
    }

    template<typename T>
    const MathCommon::Vector3<T> RigidBody<T>::GlobalToLocal(const MathCommon::Vector3<T> &global) const
    {
        return m_inverseOrientation * (global - m_position);
    }

    template<typename T>
    const MathCommon::Vector3<T> RigidBody<T>::LocalToGlobalVec(const MathCommon::Vector3<T> &v) const
    {
        return m_orientation * v;
    }

    template<typename T>
    const MathCommon::Vector3<T> RigidBody<T>::GlobalToLocalVec(const MathCommon::Vector3<T> &v) const
    {
        return m_inverseOrientation * v;
    }

}
