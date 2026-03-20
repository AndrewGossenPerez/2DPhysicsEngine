#include <iostream>
#include <vector>
#include "core/World.hpp"
#include "core/RigidBody.hpp"
#include "core/Transform.hpp"
#include "visuals/Visuals.hpp"

int main() {

    World world;
    Visuals gfx(world);

    RigidBody floor;
    setBoxVertices(floor, 30.0f, 30.0f);
    floor.setStatic(true);
    floor.snapTo(Vec2(0.0f, -35.0f));
    floor.rotate(1.5708f);
    floor.colour = Colour{150.0f, 255.0f, 255.0f};
    floor.restitution = 1.0f;
    world.getBodies().push_back(floor);

    RigidBody incline;
    setBoxVertices(incline, 10.0f, 0.8f);
    floor.setStatic(true);
    incline.snapTo(Vec2(-11.0f, 3.0f));
    incline.rotate(1.5708f * -0.05f);
    incline.colour = Colour{150.0f, 255.0f, 255.0f};
    incline.restitution = 1.0f;
    //world.getBodies().push_back(incline);

    // Static box pegs in plinko arrangement
    const int rows = 10;
    const int pegsPerRow = 8;
    const float startY = 13.0f;
    const float rowSpacing = 2.2f;
    const float colSpacing = 2.4f;
    const float pegSize = 0.5f;

    for (int row = 0; row < rows; ++row) {

        float y = startY - row * rowSpacing;
        float xOffset = (row % 2 == 0) ? 0.0f : colSpacing * 0.5f;

        for (int col = 0; col < pegsPerRow; ++col) {

            float x = -10.0f + col * colSpacing + xOffset;

            RigidBody peg(25,0.4f,1.0f);
            peg.setStatic(true);
            peg.snapTo(Vec2(x, y));
            peg.colour = Colour{255.0f, 255.0f, 255.0f};
            peg.restitution = 0.2f;
            peg.staticFriction = 0.0f;
            peg.dynamicFriction = 0.0f;

            world.getBodies().push_back(peg);

        }
    }

    gfx.renderLoop();
    return 0;
    
}