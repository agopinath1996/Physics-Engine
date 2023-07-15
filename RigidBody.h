#pragma once
namespace Bodies
{
template<typename T>
struct RigidBody
{
    T mass;
    T mass_inverse;
    Matrix3<T> m_localInverseInertiaTensor;
    Matrix3<T> m_globalInverseInertiaTensor;
    Vector3<T> m_globalCentroid;
    Vector3<T> m_localCentroid;

    //World frame
    Vector3<T> m_position;  
    Matrix3<T> m_orientation;
    Vector3<T> m_linearVelocity;
    Vector3<T> m_angularVelocity;

    Vector3<T> m_forceAccumulator;
    Vector3<T> m_torqueAccumulator;

    ColliderList m_colliders;

    void UpdateGlobalCentroidFromPosition(void);
    void UpdatePositionFromGlobalCentroid(void);

    void UpdateOrientation(void);

    void AddCollider(Collider &collider);

    const Vector3<T> LocalToGlobal(const Vector3<T> &p) const;
    const Vector3<T> GlobalToLocal(const Vector3<T> &p) const;
    const Vector3<T> LocalToGlobalVec(const Vector3<T> &v) const;
    const Vector3<T> GlobalToLocalVec(const Vector3<T> &v) const;

    void ApplyForce(const Vector3<T> &f, const Vector3<T> &at);
};

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




}