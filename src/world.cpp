// world.cpp, created by Andrew Gossen.
// Utilises collision functions to actually apply impulses to Rigid Bodies after it has been discovered they're in collision.

#include "core/World.hpp"
#include "collision/Collision.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"
#include "core/Transform.hpp"
#include "collision/AABB.hpp"
#include <cmath>
#include <iostream>


void broadPhase(std::vector<RigidBody>& m_bodies){ 
    
    // -- 
    // This is the broad phase, where we want to know what objects are likely to be colliding
    // The concept is to first use AABB and partioning to discern objects likely to be in collision
    // We then only apply the SAT to objects likely to be colliding
    // This is to avoid running unnecessary code on two bodies in which it is apparent they're not in collision

    // param m_bodies - private member of the world, storing all RigidBodies in the world.
    // -- 

    // First do an AABB test
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        for (size_t j = i + 1; j < m_bodies.size(); ++j) {

            RigidBody& A=m_bodies[i];
            RigidBody& B=m_bodies[j];

            if (A.isStatic && B.isStatic) continue; // No point in evaluating two static bodies 

            // Transform each polygon's local space vertex's into world space 
            physEng::worldSpace(A);
            physEng::worldSpace(B);
            // Define each bodies' AABB bounding box 
            AABB A_AABB=getAABB(A);
            AABB B_AABB=getAABB(B);

            if (AABBintersection(A_AABB,B_AABB)){ // If the AABB boxes intersect move onto the next stage 
                // Move onto narrow phase, implement partioning latero n 
                narrowPhase(A,B); // This is the narrow phase 
            }
        }
    }
    
}

void World::step(float dt){ 

    // -- 
    // Step function for the world, called after each frame is rendered
    // param dt - The elapsed time between consecutive frames 
    // -- 

    for (auto& body : m_bodies){
        if (!body.isStatic){

            // Integrator using dt
            body.linearAcceleration = gravity;
            body.linearVelocity += body.linearAcceleration * dt;
            body.position += body.linearVelocity * dt;
            body.rotation+=body.angularVelocity*dt;
            body.force = Vec2(0, 0); // Going to implement forces later on 
 
        }

    }

    broadPhase(m_bodies); // Check the broad phase First 
    // Note, the narrowPahse is automatically called within the broadPhase function.

}

struct impulseManifold{ // Used to store impulses to apply all impulses only once all contact points are accounted for 
    Vec2 impulse;
    Vec2 rA;
    Vec2 rB;
};

void resolveCollision(Manifold& manifold){

    // -- 
    // Resolves a collision between two objects 
    // By 'resolving', this means applying appropriate forces such that each object seperates from one another 
    // param manifold - The collision data gathered from the SAT test, stores contact data and the normal ( pointing from A to B )
    // -- 

    RigidBody& A=manifold.A;
    RigidBody& B=manifold.B;
    Vec2 normal=manifold.normal;

    std::vector<Vec2> contacts;
    contacts.reserve(manifold.contactCount);
    if (manifold.contactCount >= 1) contacts.push_back(manifold.contact1);
    if (manifold.contactCount >= 2) contacts.push_back(manifold.contact2);

    std::vector<impulseManifold> impulses;

    impulses.reserve(contacts.size());

    for (auto& contact : contacts){ // Create impulse for each contact point 

        Vec2 radiusA=contact-A.position;
        Vec2 radiusB=contact-B.position;

        // Perpendicular radii
        Vec2 rA=Vec2(-radiusA.y,radiusA.x);
        Vec2 rB=Vec2(-radiusB.y,radiusB.x);

        Vec2 AtangentialVelocity=rA*A.angularVelocity;
        Vec2 BtangentialVelocity=rB*B.angularVelocity;

        Vec2 relativeVel= (
            (B.linearVelocity+BtangentialVelocity)-
            (A.linearVelocity+AtangentialVelocity)
        );
        
        float velAlongNormal = vecMath::dot(relativeVel, manifold.normal);
        if (velAlongNormal > 0.0f) continue;  // If they are already separating along the normal, so the collision is going to resolve on its own

        float rADot=vecMath::dot(rA,normal);
        float rBDot=vecMath::dot(rB,normal);

        // Same scalar impulse magnitude 
        float minRestitiution = std::min(A.restitution, B.restitution); // Variable e 

        float j = -(1.0f + minRestitiution) * velAlongNormal;
        float denominator= (A.inverseMass + B.inverseMass + (rADot*rADot)*A.inverseInertia + (rBDot*rBDot)*B.inverseInertia  );
        j /= denominator;
        j /= static_cast<float>(manifold.contactCount);
        Vec2 impulse=manifold.normal*j;

        // Construct impulse manifold
        impulseManifold rotManifold{impulse,radiusA,radiusB}; 
        impulses.push_back(rotManifold);

    }

    // Apply impulses after impulse for all contact points created 
    for (auto& impulseData : impulses){
        A.linearVelocity-=impulseData.impulse*A.inverseMass;
        B.linearVelocity+=impulseData.impulse*B.inverseMass;
        A.angularVelocity += -vecMath::cross(impulseData.rA, impulseData.impulse) * A.inverseInertia;
        B.angularVelocity += vecMath::cross(impulseData.rB, impulseData.impulse) * B.inverseInertia;
    }

};

void narrowPhase(RigidBody& A, RigidBody& B){ 
    
    // --
    // This is the narrow phase called when it is likely A and B are colliding
    // param A - A rigid body, that is likely to be colliding with RigidBody B
    // param B - A rigid body, that is likely to be colliding with RigidBody A 
    // -- 

    Manifold m = SATCollision(A, B); // Apply the SAT test to objectively discern if they are in collision
    if (!m.inCollision) return; // Two objects are not colliding. we can stop here

    resolveCollision(m); // At this point, the two objects are colliding, so we must resolve the collision

    // Apply position correction afterwards to seperate the two objects.

    const float percent = 0.4f;  // Error percentage 
    const float slop = 0.01f; // Precision, based on world distance unit 

    float invMassSum = A.inverseMass+B.inverseMass; // Zero implies two static bodies
    if (invMassSum > 0.f){ 
        // Positional correction if two objects are colliding and one is non-static based on penetration depth 
        float corrMag = std::max(m.penetration - slop, 0.f) / invMassSum * percent;
        Vec2 correction = m.normal * corrMag;
        if (!A.isStatic) A.position -= correction * A.inverseMass;
        if (!B.isStatic) B.position += correction * B.inverseMass;
    }

}

