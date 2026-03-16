// world.cpp, created by Andrew Gossen.
// Utilises collision functions to detect and resolve collisions between rigid bodies

#include "core/World.hpp"
#include "collision/Collision.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"
#include "core/Transform.hpp"
#include "collision/AABB.hpp"
#include "collision/Partitioning.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

struct impulseManifold {
    Vec2 impulse;
    Vec2 rA;
    Vec2 rB;
};

Manifold narrowPhase(RigidBody& A, RigidBody& B) {
    // Narrow phase collision detection only
    // Returns a manifold, caller checks m.inCollision
    return SATCollision(A, B);
}

void broadPhase(std::vector<RigidBody>& bodies,std::vector<Manifold>& manifolds,WorldStats& m_stats) {

    // Broad-phase collision detection.
    // Builds candidate pairs from AABBs & Spatial grid partioning, runs narrow phase once per candidate,
    // and stores colliding manifolds for later solver iterations

    std::vector<AABB> aabbs;
    aabbs.reserve(bodies.size());

    for (auto& body : bodies) {
        physEng::worldSpace(body);
        aabbs.emplace_back(getAABB(body));
    }

    partioning::GridConfig gridConfig;
    auto pairs = partioning::buildPairsFromAABBs(aabbs, gridConfig); // Generate broad-phase pairs
    // likely to be in collision

    for (const auto& [i, j] : pairs) {

        m_stats.broadChecks++;

        RigidBody& A = bodies[i];
        RigidBody& B = bodies[j];

        if (A.isStatic && B.isStatic) continue; // Both bodies static, no collision detection/resolution needed
        if (!AABBintersection(aabbs[i], aabbs[j])) continue; // Cannot be colliding, AABB's dont intersect 

        m_stats.narrowChecks++;

        Manifold m = narrowPhase(A, B); // At this point, it's worth running SAT 

        if (m.inCollision) {
            manifolds.push_back(std::move(m)); // Add manifold to solver list 
        }

    }
}

void resolveCollision(Manifold& manifold) {

    // Resolves collision by applying impulses at each contact point.
    // Preconditions:
    // - manifold.inCollision == true
    // - manifold.normal is unit length and points from A -> B
    // - contactCount in [1,2]

    RigidBody& A = manifold.A;
    RigidBody& B = manifold.B;
    const Vec2& normal = manifold.normal;

    const Vec2* contacts[2];
    int contactCount = 0;
    if (manifold.contactCount >= 1) contacts[contactCount++] = &manifold.contact1;
    if (manifold.contactCount >= 2) contacts[contactCount++] = &manifold.contact2;

    impulseManifold impulses[4];
    int impulseCount = 0;

    float staticFriction = std::min(A.staticFriction, B.staticFriction);
    float dynamicFriction = std::min(A.dynamicFriction, B.dynamicFriction);

    for (int idx = 0; idx < contactCount; ++idx) {
        
        // Goes through each vertex contact point in a manifold,
        // and applies an impulse resolution

        const Vec2& contact = *contacts[idx];

        Vec2 radiusA = contact - A.position;
        Vec2 radiusB = contact - B.position;

        // Perpendicular radii
        Vec2 rA(-radiusA.y, radiusA.x);
        Vec2 rB(-radiusB.y, radiusB.x);

        Vec2 AtangentialVelocity = rA * A.angularVelocity;
        Vec2 BtangentialVelocity = rB * B.angularVelocity;

        Vec2 relativeVel =
            (B.linearVelocity + BtangentialVelocity) -
            (A.linearVelocity + AtangentialVelocity);

        float velAlongNormal = vecMath::dot(relativeVel, normal);
        if (velAlongNormal > 0.0f) continue;

        Vec2 tangent = relativeVel - normal * vecMath::dot(relativeVel, normal);

        bool applyFriction = true;
        if (vecMath::floatCloselyEqual(tangent.length(), 0.0f)) {
            applyFriction = false;
        } else {
            tangent = tangent.normalise();
        }

        float rADot = vecMath::dot(rA, normal);
        float rBDot = vecMath::dot(rB, normal);
        float minRestitution = std::min(A.restitution, B.restitution);

        float denominator =
            A.inverseMass + B.inverseMass +
            (rADot * rADot) * A.inverseInertia +
            (rBDot * rBDot) * B.inverseInertia;

        if (vecMath::floatCloselyEqual(denominator, 0.0f)) continue;

        float j = -(1.0f + minRestitution) * velAlongNormal;
        j /= denominator;
        j /= static_cast<float>(manifold.contactCount);

        Vec2 impulse = normal * j;
        impulses[impulseCount++] = { impulse, radiusA, radiusB };

        if (applyFriction) {

            float rADotTangential = vecMath::dot(rA, tangent);
            float rBDotTangential = vecMath::dot(rB, tangent);

            float denominatorTangential =
                A.inverseMass + B.inverseMass +
                (rADotTangential * rADotTangential) * A.inverseInertia +
                (rBDotTangential * rBDotTangential) * B.inverseInertia;

            if (!vecMath::floatCloselyEqual(denominatorTangential, 0.0f)) {
                float jTangent = -vecMath::dot(relativeVel, tangent);
                jTangent /= denominatorTangential;
                jTangent /= static_cast<float>(manifold.contactCount);

                Vec2 frictionImpulse;
                if (std::abs(jTangent) <= j * staticFriction) {
                    frictionImpulse = tangent * jTangent;
                } else {
                    frictionImpulse = tangent * (-j * dynamicFriction);
                }

                impulses[impulseCount++] = { frictionImpulse, radiusA, radiusB };
            }

        }

    }

    for (int i = 0; i < impulseCount; ++i) {

        // Impulse calculations

        const impulseManifold& impulseData = impulses[i];

        A.linearVelocity -= impulseData.impulse * A.inverseMass;
        B.linearVelocity += impulseData.impulse * B.inverseMass;
        A.angularVelocity += -vecMath::cross(impulseData.rA, impulseData.impulse) * A.inverseInertia;
        B.angularVelocity += vecMath::cross(impulseData.rB, impulseData.impulse) * B.inverseInertia;

    }

}

void positionalCorrection(Manifold& m) {

    // Corrects penetration after impulse solving

    RigidBody& A = m.A;
    RigidBody& B = m.B;

    const float percent = 0.8f;
    const float slop = 0.01f; // Intentionally allow for some overlapping to reduce jitter 

    float invMassSum = A.inverseMass + B.inverseMass;
    if (invMassSum <= 0.0f) return;

    float corrMag = std::max(m.penetration - slop, 0.0f) / invMassSum * percent;
    Vec2 correction = m.normal * corrMag;

    if (!A.isStatic) {
        A.position -= correction * A.inverseMass;
        A.update = true;
    }

    if (!B.isStatic) {
        B.position += correction * B.inverseMass;
        B.update = true;
    }

}

void World::step(float dt) {

    // Advances the simulation by dt seconds
    // The current order of things:
    // - Integrate
    // - Cull dead/out-of-bounds bodies
    // - Detect collisions once
    // - Solve impulses solverIterations times
    // - Apply positional correction once

    for (auto& body : m_bodies) {
        if (!body.isStatic) {
            body.linearAcceleration = gravity;
            body.linearVelocity += body.linearAcceleration * dt;
            body.position += body.linearVelocity * dt;
            body.rotation += body.angularVelocity * dt;
            body.force = Vec2(0, 0);
            body.update = true;
            m_stats.bodyUpdates++;
        }
    }

    m_bodies.erase(
        std::remove_if(
            m_bodies.begin(),
            m_bodies.end(),
            [&](const RigidBody& body) {
                return body.position.y < -m_yBounds;
            }),
        m_bodies.end()
    );

    std::vector<Manifold> manifolds;
    manifolds.reserve(m_bodies.size());

    broadPhase(m_bodies, manifolds, m_stats); // The broadphase will run the narrowphase on 
    // good candidates, which will add to the manifolds list if in collision

    for (int i = 0; i < solverIterations; ++i) {
        for (auto& manifold : manifolds) { 
            // For each manifold collected (Which is in collision proven by SAT), 
            // calculate impulses 
            resolveCollision(manifold);
        }
    }

    for (auto& manifold : manifolds) {
        positionalCorrection(manifold); // Apply calculated impulses from before 
        m_stats.contactsResolved++;
    }

    m_stats.steps++;

}