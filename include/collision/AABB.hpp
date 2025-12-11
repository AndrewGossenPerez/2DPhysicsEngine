// AABB ( Axis Aligned Bounding Box ), created by Andrew Gossen.
#include "core/Vector2.hpp"
#include "core/RigidBody.hpp"

struct AABB{ // AAB object, with a max and min for easy collision detection
    Vec2 min;
    Vec2 max;
};

AABB getAABB(const RigidBody& body); 