#pragma once
namespace RigidBody
{
template<typename T>
struct RigidBody
{
    T mass;
    T mass_inverse;
    Eigen::Matrix<T,3,3> m_localInverseInertiaTensor;
    Eigen::Matrix<T,3,3> m_globalInverseInertiaTensor;
    Eigen::Vector3d m_globalCentroid;
    Eigen::Vector3d m_localCentroid;

    Eigen::Vector3d m_position;
    Eigen::Matrix<T,3,3> m_orientation;
    Eigen::Vector3d m_linearVelocity;
    Eigen::Vector3d m_angularVelocity;

    Eigen::Vector3d m_forceAccumulator;
    Eigen::Vector3d m_torqueAccumulator;

    ColliderList m_colliders;

    void UpdateGlobalCentroidFromPosition(void);
    void UpdatePositionFromGlobalCentroid(void);

    void UpdateOrientation(void);

    void AddCollider(Collider &collider);

    const Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d &p) const;
    const Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d &p) const;
    const Eigen::Vector3d LocalToGlobalVec(const Eigen::Vector3d &v) const;
    const Eigen::Vector3d GlobalToLocalVec(const Eigen::Vector3d &v) const;

    void ApplyForce(const Eigen::Vector3d &f, const Eigen::Vector3d &at);
};
}