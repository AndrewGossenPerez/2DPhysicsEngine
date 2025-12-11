#include "collision/AABB.hpp"
#include "core/RigidBody.hpp"
#include "math/Math.hpp"

AABB getABB(const RigidBody& Body){

    const Vec2& first = Body.transformedVertices[0];
    Vec2 min = first;
    Vec2 max = first;

    for (size_t i = 1; i < Body.transformedVertices.size(); ++i){
        
        const Vec2& v = Body.transformedVertices[i];

        if (v.x < min.x) min.x = v.x;
        if (v.y < min.y) min.y = v.y;
        if (v.x > max.x) max.x = v.x;
        if (v.y > max.y) max.y = v.y;

    }

    AABB result{
        Vec2(min.x,min.y),
        Vec2(max.x,max.y)
    };

    return result;

}