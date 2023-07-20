#pragma once
#include<vector>
#include "CommonTypes.h"

namespace Bodies
{

template<typename T>
struct Collider
{
  
  T m_mass;
  Matrix3<T> m_localInertiaTensor;
  Vector3<T> m_localCentroid;
 
  //TODO: Geometrical collider properties

};

template<typename T>
struct RigidBody
{
    T mass;
    T mass_inverse;
    Matrix3<T> m_localInverseInertiaTensor;
    Matrix3<T> m_globalInverseInertiaTensor;
    Vector3<T> m_globalCentroid;
    Vector3<T> m_localCentroid;

    // World Frame
    Vector3<T> m_position;  
    Matrix3<T> m_orientation;
    Matrix3<T> m_inverseOrientation;
    Vector3<T> m_linearVelocity;
    Vector3<T> m_angularVelocity;

    Vector3<T> m_forceAccumulator;
    Vector3<T> m_torqueAccumulator;

    std::vector<Collider<T>> m_collider_list;

    void UpdateGlobalCentroidFromPosition(void);
    void UpdatePositionFromGlobalCentroid(void);

    void UpdateOrientation(void);

    void AddCollider(Collider<T> &collider);

    const Vector3<T> LocalToGlobal(const Vector3<T> &local) const;
    const Vector3<T> GlobalToLocal(const Vector3<T> &global) const;
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
  Matrix3<T> localInertiaTensor;
  localInertiaTensor.setzero();

  for (int i=0;i<m_collider_list.size();i++)
  {
    const Vector3<T> r = m_localCentroid - m_collider_list[i].m_localCentroid;
    const T rDotr = r.dot(r);
    const Matrix3<T> rOutr = r*r.transpose();
 
    // accumulate local inertia tensor contribution, 
    // using Parallel Axis Theorem
    localInertiaTensor += 
      collider.localInertiaTensor 
      + collider.m_mass * (rDotr * Identity3<T> - rOutr);
  }
 
  // compute inverse inertia tensor
  m_localInverseInertiaTensor = localInertiaTensor.inverse();
}

template<typename T>
const Vector3<T> RigidBody<T>::LocalToGlobal(const Vector3<T> &local) const
{
  return m_orientation * local + m_position;
}

template<typename T>
const Vector3<T> RigidBody<T>::GlobalToLocal(const Vector3<T> &global) const
{
  return m_inverseOrientation * (global - m_position);
}

template<typename T>
const Vector3<T> RigidBody<T>::LocalToGlobalVec(const Vector3<T> &v) const
{
  return m_orientation * v;
}

template<typename T>
const Vector3<T> RigidBody<T>::GlobalToLocalVec(const Vector3<T> &v) const
{
  return m_inverseOrientation * v;
}



}