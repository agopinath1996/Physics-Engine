#pragma once
#include<utility>
#include<list>
#include<vector>
#include "CommonMathTypes.h"

namespace Body
{
    template<typename T>
    struct AABB
    {
        MathCommon::Vector3<T> diagonal;

        AABB(){};
        AABB(const T& x_diff,const T& y_diff,const T& z_diff)
        {
        diagonal<<x_diff,y_diff,z_diff;
        }
        AABB(const T& x_min,const T& y_min,const T& z_min,const T& x_max,const T& y_max,const T& z_max)
        {
        diagonal<<(x_max-x_min),(y_max-y_min),(z_max-z_min);
        }
    };
    template<typename T>
    using AABBList = std::vector<AABB<T>*>;
    template<typename T>
    struct Collider
    {
        /* 
        A collider is a subset of a RigidBody that is capable of collision
        */

        // Mechanical properties
        T m_mass;
        MathCommon::Matrix3<T> m_localInertiaTensor;
        MathCommon::Vector3<T> m_localCentroid;

        // Geometrical properties
        AABB<T> m_aabb;

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
        MathCommon::Vector3<T> position;
        MathCommon::Vector3<T> normal;
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
