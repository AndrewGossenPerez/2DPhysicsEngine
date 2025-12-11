// main.cpp
#include <iostream>
#include <vector>
#include "core/World.hpp"
#include "core/RigidBody.hpp"
#include "core/Transform.hpp"
#include "visuals/Visuals.hpp"

void createDefaultBox(World& world,Colour c,Vec2 p){
    
    RigidBody box;
    setBoxVertices(box, 1.0f, 1.0f);     
    box.snapTo(p);        
    box.rotate(1.5708f);                 
    box.colour =c;
    box.restitution=0.9f;
    box.mass=5.0f;
    world.getBodies().push_back(box);

}

int main(){

    Visuals gfx;
    World world;

    RigidBody floor;
    setBoxVertices(floor, 30.0f, 30.0f);    
    floor.snapTo(Vec2(0.0f, -27.0f));         
    floor.rotate(1.5708f);                  
    floor.colour = Colour{255.0f, 255.0f, 255.0f};
    floor.isStatic=true;
    floor.restitution=1.0f;
    world.getBodies().push_back(floor);

    RigidBody triangle(3,1.0,2.0);
    triangle.snapTo(Vec2(5.0f, 3.0f));   
    triangle.rotate(1.5708/1.20f);      
    triangle.restitution=0.6f;
    world.getBodies().push_back(triangle);

    for (int i=1;i<10;i++){
        RigidBody triangle2(4,1.0,2.0);
        triangle2.snapTo(Vec2(5.0f, 8.0f+i));   
        triangle2.rotate(1.5708/1.20f);    
        triangle2.colour=Colour{250.0f,250.0f,220.0f};
        triangle2.restitution=0.6f;
        world.getBodies().push_back(triangle2);
    };
   
    // Main loop
    gfx.renderLoop(world);

    return 0;

}
