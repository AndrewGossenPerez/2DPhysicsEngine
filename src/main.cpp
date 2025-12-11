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
    floor.colour = Colour{150.0f, 255.0f, 255.0f};
    floor.isStatic=true;
    floor.restitution=1.0f;
    world.getBodies().push_back(floor);


    RigidBody floor2;
    setBoxVertices(floor2, 15.0f, 0.6f);    
    floor2.snapTo(Vec2(10.0f, 0.9f));         
    floor2.rotate(1.5708*1.1f);                  
    floor2.colour = Colour{150.0f, 255.0f, 255.0f};
    floor2.isStatic=true;
    floor2.restitution=1.0f;
    world.getBodies().push_back(floor2);

    // Restitiution example 
    RigidBody t3(4,1.0,2.0);
    t3.snapTo(Vec2(0.0f, -2.0f));   
    t3.rotate(1.5708*1.5);      
    t3.restitution=0.6f;
    t3.linearVelocity=Vec2(0.0f,0.0f);
    world.getBodies().push_back(t3);

    RigidBody t4(4,1.0,2.0);
    t4.snapTo(Vec2(-3.0f, -2.0f));   
    t4.rotate(1.5708*1.5);      
    t4.restitution=0.6f;
    t4.linearVelocity=Vec2(0.0f,0.0f);
    world.getBodies().push_back(t4);


    RigidBody t5(6,1.0,2.0);
    t5.snapTo(Vec2(-9.0f, -11.0f));   
    t5.rotate(1.5708*1.5);      
    t5.restitution=0.6f;
    t5.linearVelocity=Vec2(5.0f,0.0f);
    world.getBodies().push_back(t5);

    RigidBody triangle(4,1.0,2.0);
    triangle.snapTo(Vec2(15.0f, 15.0f));   
    triangle.rotate(1.5708*1.5f);      
    triangle.restitution=0.3f;
    world.getBodies().push_back(triangle);

    RigidBody box2(10,1.0,2.0);
    box2.snapTo(Vec2(-20.0f, 6.0f));   
    box2.rotate(1.5708f);      
    box2.restitution=0.3f;
    box2.linearVelocity=Vec2(0.0f,0.0f);
    world.getBodies().push_back(box2);

    RigidBody triangle2(3,1.0,2.0);
    triangle2.snapTo(Vec2(6.0f, -2.0f));   
    triangle2.rotate(1.5708);      
    triangle2.restitution=0.2f;
    triangle2.linearVelocity=Vec2(1.0f,0.0f);
    world.getBodies().push_back(triangle2);

    for (int i=1;i<5;i++){
        RigidBody triangle2(6 ,1.0,2.0);
        triangle2.snapTo(Vec2(5.0f+(i*2), 40.0f+(i+50)));   
        triangle2.rotate(1.5708/1.20f);    
        triangle2.colour=Colour{250.0f,250.0f,220.0f};
        triangle2.restitution=0.6f;
        world.getBodies().push_back(triangle2);
    };
   
    for (int i=1;i<10;i++){
        RigidBody triangle2(4 ,1.0,2.0);
        triangle2.snapTo(Vec2(-10.0f+(i*5), 60.0f+(i+50)));   
        triangle2.rotate(1.5708/1.20f);    
        triangle2.colour=Colour{250.0f,250.0f,220.0f};
        triangle2.restitution=0.6f;
        world.getBodies().push_back(triangle2);
    };
    // Main loop
    gfx.renderLoop(world);

    return 0;

}
