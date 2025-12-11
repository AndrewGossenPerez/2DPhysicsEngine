// Transform.hpp, created by Andrew Gossen.
// Used for transformation calculations, used to convert the local vertices to world-space.

#pragma once 
#include "Vector2.hpp"
#include "RigidBody.hpp"

struct Transform{ 

    Vec2 position{0.0f,0.0f};
    float rotation{0.0f}; // measured in radians  

    Transform()=default;
    explicit Transform(const Vec2& position,float rotation) : position(position), rotation(rotation) {}

    void Translate(const Vec2& translation){ // Changes this transform's position 
        position.x+=translation.x;
        position.y+=translation.y;
    }

    void rotate(float translation){ // Changes this Transform's rotation 
        rotation+=translation; 
    }

    Vec2 applyTransform(const Vec2& p) const { 
        float c=std::cos(rotation);
        float s=std::sin(rotation);
        Vec2 rotated(
            p.x * c - p.y * s,
            p.x * s + p.y * c 
        );
        return rotated + position; // Return new transformed vector 
    }

};

namespace physEng{

    inline void worldSpace(RigidBody& body) { // Updates vertices from local space to world spaces 

        if (!body.update && !body.transformedVertices.empty())
            return;

        Transform t(body.position, body.rotation);

        body.transformedVertices.clear();
        body.transformedVertices.reserve(body.vertices.size());

        for (const Vec2& local : body.vertices) {
            body.transformedVertices.push_back(t.applyTransform(local));
        }
        
    }


};

