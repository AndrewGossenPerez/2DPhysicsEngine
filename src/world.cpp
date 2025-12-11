
#include "core/World.hpp"
#include "collision/Collision.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"
#include "core/Transform.hpp"
#include <cmath>
#include <iostream>

void World::step(float dt){ // Called each frame

    for (auto& body : m_bodies){
        if (!body.isStatic){

            body.linearAcceleration = gravity;
            body.linearVelocity += body.linearAcceleration * dt;
            body.position += body.linearVelocity * dt;
            body.rotation+=body.angularVelocity*dt;

            body.force = Vec2(0, 0);


        }

    }

    for (size_t i = 0; i < m_bodies.size(); ++i) {
        for (size_t j = i + 1; j < m_bodies.size(); ++j) {
            resolvePair(m_bodies[i], m_bodies[j]);
        }
    }


}

struct impulseManifold{
    Vec2 impulse;
    Vec2 rA;
    Vec2 rB;
};


static inline Vec2 tangentialVelocity(float w, const Vec2& r) {
    // ω (about z) crossed with r = (-ω * r.y, ω * r.x)
    return Vec2(-w * r.y, w * r.x);
}

void resolveCollision(Manifold& manifold){

    RigidBody& A=manifold.A;
    RigidBody& B=manifold.B;
    Vec2 normal=manifold.normal;

    std::vector<Vec2> contacts;
    contacts.reserve(manifold.contactCount);
    if (manifold.contactCount >= 1) contacts.push_back(manifold.contact1);
    if (manifold.contactCount >= 2) contacts.push_back(manifold.contact2);

    float restitution = std::min(A.restitution, B.restitution);

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
        float minRestitiution = std::min(A.restitution, B.restitution);
        float j = -(1.0f + minRestitiution) * vecMath::dot(relativeVel,normal);
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

void resolvePair(RigidBody& A, RigidBody& B){ 
    // Check if Body A and B are colliding, if so resolve their

    physEng::worldSpace(A);
    physEng::worldSpace(B);

    Manifold m = SATCollision(A, B);
    if (!m.inCollision) return; // Two objects are not colliding

    // Positional correction
    const float percent=1.00;;
    const float slop=0.000005f;

    float invMassSum = A.inverseMass+B.inverseMass;

    if (invMassSum > 0.f){ 
        // Positional correction if two objects are colliding and one is non-static based on penetration depth 
        float corrMag = std::max(m.penetration - slop, 0.f) / invMassSum * percent;
        Vec2 correction = m.normal * corrMag;
        if (!A.isStatic) A.position -= correction * A.inverseMass;
        if (!B.isStatic) B.position += correction * B.inverseMass;
    }

    resolveCollision(m); // Collision solver 

}
