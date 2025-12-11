// RigidBody.cpp, created by Andrew Gossen.
// Handles constructor for the RigidBody.hpp, and defines useful functions to generate RigidBodies. 

#include "core/RigidBody.hpp"
#include "core/Vector2.hpp"

std::vector<Vec2> generateRegularPolygon(int n, float r){

    // ---
    // Creates an n-sided Polygon, of radius r which winds clockwise.
    // param n - Sides
    // param r - Radius
    // -- 
    
    std::vector<Vec2> verts;
    if (n < 3) return verts;
    verts.reserve(n);

    // Angle between consecutive vertices
    const float dTheta = 2.0f * M_PI / static_cast<float>(n);
    // Rotate so one vertex points up
    const float startAngle = -M_PI / 2.0f;

    for (int i = 0; i < n; ++i){ // Iterate through each side, and generate a vertex 
        float theta = startAngle + i * dTheta;
        float x = r * std::cos(theta);
        float y = r * std::sin(theta);
        verts.emplace_back(x, y); // Construct the Vector2 point of this vertex within the verts vector 
    }

    return verts;
}

float computeRegularPolygonInertia(int n, float m, float r){ 

    // ---
    // Computes the inertia for an n sided Polygon of radius r and mass m
    // param n - Sides
    // param m - mass
    // param r - radius 
    // -- 

    if (n < 3 || m <= 0.0f) return 0.0f; // Either an invalid polygon or a static object 
    float n = static_cast<float>(n);
    float angle = 2.0f * M_PI / n;
    float I = (m * r * r / 12.0f) * (3.0f + std::cos(angle));
    return I;

}

float computeInverseMass(float mass, bool isStatic){

    // ---
    // Computes the inverse mass, used to avoid division by zero 
    // param mass - Mass to inverse 
    // param isStatic - Whether the mass of the object we're calculating the inverse mass for is static or not
    // Note : static implies 0 inverse mass 
    // -- 

    if (isStatic || mass <= 0.0f) return 0.0f;
    return 1.0f / mass;

}

RigidBody::RigidBody(int n,float r,float m) : sides(n), radius(r), mass(mass) {

    // ---
    // Constructs an n sided Polygon RigidBody of radius r and mass m
    // param n - Sides
    // param r- Radius
    // param m - Mass
    // -- 

    vertices=generateRegularPolygon(n,r);
    inertia=computeRegularPolygonInertia(n,m,r);
    inverseInertia=1/inertia;
    inverseMass=computeInverseMass(m,isStatic);

}