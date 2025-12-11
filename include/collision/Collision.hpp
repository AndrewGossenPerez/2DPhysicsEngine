
#pragma once 
#include "core/RigidBody.hpp"
#include "core/Vector2.hpp"
#include <vector>

struct Manifold{ 
    // Two bodies in collision
    RigidBody& A;
    RigidBody& B;
    Vec2 normal;
    Vec2 contact1;
    Vec2 contact2;
    int contactCount;
    float penetration;
    bool inCollision;
};

Manifold SATCollision(RigidBody& RigidBodyA,RigidBody& RigidBodyB);  // Returns whether there is a collision, works on transformed vertices ( world space ) 

