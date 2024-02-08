#pragma once
#include "RigidBody.h"
#include "CommonMathTypes.h"

namespace CollisionDetection{
    
    //Function to give a support point to GJK algorithm
    template <typename T>
    void CSOSupport(const Body::Collider<T> &colliderA,
                    const Body::Collider<T> &colliderB,
                    const MathCommon::Vector3<T> &dir, 
                    MathCommon::Vector3<T> &support, 
                    MathCommon::Vector3<T> &supportA, 
                    MathCommon::Vector3<T> &supportB)
    {
        const Body::RigidBody<T> *bodyA = colliderA.Surface();
        const Body::RigidBody<T> *bodyB = colliderB.Surface();

        // convert search direction to model space
        const MathCommon::Vector3<T> localDirA = bodyA->GlobalToLocalVec(dir);
        const MathCommon::Vector3<T> localDirB = bodyB->GlobalToLocalVec(-dir);

        // compute support points in model space
        supportA = colliderA.Support(localDirA);
        supportB = colliderB.Support(localDirB);

        // convert support points to world space
        supportA = bodyA->LocalToGlobal(supportA);
        supportB = bodyB->LocalToGlobal(supportB);

        // compute CSO support point
        support = supportA - supportB;
    }
}