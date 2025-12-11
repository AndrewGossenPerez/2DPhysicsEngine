// AABB ( Axis Aligned Bounding Box ), created by Andrew Gossen.
// Creates an AABB bounding box for polygons, which is just a cuboid surrounding the entirety of the polygon.
// THis is used as a cheap and fast way to discern if two polygons are likely to be in collision.

#pragma once
#include "core/Vector2.hpp"
#include "core/RigidBody.hpp"

struct AABB{ // AAB object, with a max and min for easy collision detection
    Vec2 min;
    Vec2 max;
};

AABB getAABB(const RigidBody& Body){

    // -- 
    // Returns an AABB bounding box for a polygon
    // param Body - The polygon to create a bounding box for 
    // This is assuming that the polygon has had it's transformed vertices calculated already.
    // -- 

    const Vec2& first = Body.transformedVertices[0];
    Vec2 min = first;
    Vec2 max = first;

    for (auto& v : Body.transformedVertices){
        if (v.x < min.x) min.x = v.x;
        if (v.y < min.y) min.y = v.y;
        if (v.x > max.x) max.x = v.x;
        if (v.y > max.y) max.y = v.y;
    }

    AABB result{ // The actual AABB struct representing the bounding box 
        Vec2(min.x,min.y),
        Vec2(max.x,max.y)
    };

    return result;

}

bool AABBintersection(const AABB& a, const AABB& b) {

    // -- 
    // Checks if two AABB bounding boxes are intersecting 
    // param a - The AABB bounding box for rigid body a
    // param b - The AABB bounding box for rigid body b
    // -- 

    // If one box is completely to the left of the other
    if (a.max.x < b.min.x || b.max.x < a.min.x) return false;
    // If one box is completely above the other
    if (a.max.y < b.min.y || b.max.y < a.min.y) return false;
    // Otherwise, they overlap (including touching edges)
    return true;
    
} 