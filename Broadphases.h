#include<vector>
#include "RigidBody.h"
#include "CommonMathTypes.h"
#include "GeometryTypes.h"

namespace CollisionDetection
{

template<typename T>
class Broadphase
{
  
  public:
   
    // adds a new AABB to the broadphase
    virtual void Add(Geometry::AABB<T> *aabb) = 0;
 
    // updates broadphase to react to changes to AABB
    virtual void Update(void) = 0;
 
    // returns a list of possibly colliding colliders
    virtual const Body::ColliderPairList<T>& ComputePairs(void) = 0;
     
    // returns a collider that collides with a point
    // returns null if no such collider exists
    virtual Body::Collider<T>* Pick(const MathCommon::Vector3<T> &point) const = 0;
 
    
    // with a query AABB
    virtual void Query(const Geometry::AABB<T> &aabb, Body::ColliderList<T> &output) const = 0;
     
    // result contains the first collider the ray hits
    // result contains null if no collider is hit
    virtual Body::RayCastResult<T> RayCast(const MathCommon::Ray3<T> &ray) const = 0;
};

template<typename T>
class NSquared : public Broadphase<T>
{
    private:
 
    Geometry::AABBList<T> m_aabbs;
    Body::ColliderPairList<T> m_pairs;

  public:
     
    virtual void Add(Geometry::AABB<T> *aabb)
    {
      m_aabbs.push_back(aabb);
    }
     
    virtual void Update(void)
    {
      // TODO
    }
     
    virtual Body::ColliderPairList<T>& ComputePairs(void);
    virtual Body::Collider<T>* Pick(const MathCommon::Vector3<T> &point) const;
    virtual void Query(const Geometry::AABB<T> &aabb, Body::ColliderList<T> &out) const;
    virtual Body::RayCastResult<T> RayCast(const MathCommon::Ray3<T> &ray) const;

};

// convert below to .cpp file
template<typename T>
Body::ColliderPairList<T> &NSquared<T>::ComputePairs(void)
{
  m_pairs.clear();
 
  // outer loop
  typename Geometry::AABBList<T>::iterator i = m_aabbs.begin(); // These types are to complex
  for (;i != m_aabbs.end(); ++i)
  {
    
    // inner loop
    typename Geometry::AABBList<T>::iterator jStart = i;  // These types are to complex
    typename Geometry::AABBList<T>::iterator j = ++jStart; // These types are to complex
    for (; j != m_aabbs.end(); ++j)
    {
      Geometry::AABB<T> *aabbA = *i;
      Geometry::AABB<T> *aabbB = *j;
      Body::Collider<T> *colliderA = aabbA->Collider();
      Body::Collider<T> *colliderB = aabbB->Collider();
      Body::RigidBody<T> *bodyA = colliderA->Body();
      Body::RigidBody<T> *bodyB = colliderB->Body();
       
      // skip same-body collision
      if (bodyA == bodyB)
        continue;
       
      // add collider pair
      if (aabbA->Collides(aabbB))
        m_pairs.push_back(
          std::make_pair(aabbA->collider, aabbB->collider));
       
    } // end of inner loop
  } // end of outer loop
   
  return m_pairs;
}

template<typename T>
Body::Collider<T>* NSquared<T>::Pick(const MathCommon::Vector3<T>& point) const
{

  for (Geometry::AABB<T> &aabb : m_aabbs)
    if (aabb->Contains(point))
      return aabb->collider; //Does Collider() modify m_aabbs?
 
  // no collider found
  return nullptr;
}

template<typename T>
void NSquared<T>::Query(const Geometry::AABB<T> &aabb, Body::ColliderList<T> &out) const
{
  for (Geometry::AABB<T> &colliderAABB: m_aabbs)
    if (colliderAABB->Collides(aabb))
      out.push_back(colliderAABB->collider);
}

/*template<typename T>
Body::RayCastResult<T> NSquared<T>::RayCast(const MathCommon::Ray3<T> &ray) const
{
  // test AABBs for candidates
  Body::ColliderList<T> candidateList;
  candidateList.reserve(m_aabbs.size());

  for (Geometry::AABB<T> &aabb : m_aabbs)
    if (aabb->TestRay(ray))
      candidateList.push_back(aabb->collider);
   
  // test actual colliders
  Body::ResultList<T> resultList;
  resultList.reserve(candidateList.size());

  for (Body::Collider<T> *collider : candidateList)
  {
    // hit point = ray.pos + t * ray.dir
    if (TestRay(collider, ray))
    {
      Body::RayCastResult<T> entry{ collider, t, normal };
      resultList.push_back(entry);
    }
  }
 
  // sort the result list
  std::sort(resultList.begin(), resultList.end());
 
  Body::RayCastResult<T> result;
  if (!resultList.empty())
  {
    // the first result entry is the closest one
    Body::RayCastResult<T> &entry = resultList.front();
    result.hit = true;
    result.collider = entry.collider;
    result.t = entry.t;
    result.normal = entry.normal;
    result.intersection = ray.pos + entry.t * ray.dir;
  }
  else
    result.hit = false;
 
  return result;
}*/

}
