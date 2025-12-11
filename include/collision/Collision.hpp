
// Collision.hpp, created by Andrew Gossen.
// Stores the Manifold struct, used to store data between two rigid bodies in collision

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

// Defined in collision.cpp 
Manifold SATCollision(RigidBody& RigidBodyA,RigidBody& RigidBodyB);  

