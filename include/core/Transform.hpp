
// Transform.hpp, created by Andrew Gossen.
// Used for transformation calculations, ultimately to convert the local vertices to world-space.

#pragma once 
#include "Vector2.hpp"
#include "RigidBody.hpp"

struct Transform{ 

    Vec2 position{0.0f,0.0f}; // Transform position 
    float rotation{0.0f}; // Transform measured in radians  

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
        // -- 
        // Applies a transformation to position p
        // param p - Vector 2 position
        // -- 
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

    inline void worldSpace(RigidBody& body) { 

        // -- 
        // This is used to update a RigidBody's vertices from local space ( relative to it's com ) to world space ( Using the x-y world co-ordinate system)
        // param body - RigidBody to update vertices from local space to world space for 
        // -- 

        if (!body.update && !body.transformedVertices.empty()) return;

        Transform t(body.position, body.rotation);

        body.transformedVertices.clear();
        body.transformedVertices.reserve(body.vertices.size());

        for (const Vec2& local : body.vertices) {
            body.transformedVertices.push_back(t.applyTransform(local));
        }
        
    }

};

