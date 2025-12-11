// World.hpp, created by Andrew Gossen.
// Establishes the World class, which encompasses the entire physics world holding all bodies.

#pragma once
#include "core/RigidBody.hpp"
#include <vector>

class World{ 

    public:
    Vec2 getGravity() const{ return gravity; }
    std::vector<RigidBody>& getBodies() { return m_bodies; } // Return rigid bodies in the world 
    void step(float dt);

    private:
    std::vector<RigidBody> m_bodies; // All rigid bodies, static and non-static, in the world
    Vec2 gravity{0.0f,-9.81f};

};

void narrowPhase(RigidBody& A,RigidBody& B); // The narrow phase for collision checking, defined in world.cpp
