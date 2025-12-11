// Visuals.hpp, created by Andrew Gossen.
// Handles all OpenGL visuals to keep main.cpp tidier.

#pragma once
#include "core/RigidBody.hpp"
#include "core/World.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class Visuals {

public:

    Visuals();  
    ~Visuals();  

    bool isValid() const { return ok; }

    // Draw a single rigid body using internal VAO/VBO + shader
    void drawRigidBody(RigidBody& body);

    // The actual render loop running each frame. 
    void renderLoop(World& world);

    // Getters
    GLFWwindow* getWindow() const { return window; }

    // Setters
    void setZoom(float z) { zoom = z; }
    float& getZoom() { return zoom; }

private:

    // Window / GL context
    GLFWwindow* window = nullptr;

    int winWidth  = 800;
    int winHeight = 600;
    int fbWidth, fbHeight;

    // Shader program and uniforms
    GLuint shaderProgram = 0;
    GLint  colourLoc = -1;
    GLint  aspectLoc = -1;
    GLint  zoomLoc   = -1;

    // Geometry buffers
    GLuint vao = 0;
    GLuint vbo = 0;

    float zoom = 0.07f;  
    bool ok = false;    

};


#pragma once
#include "core/Vector2.hpp"

namespace DebugDraw {

    // Draws a small cross at a world-space point
    inline void drawContactPoint(const Vec2& p, float size = 0.05f) {
        // Assuming you already have an orthographic projection + view set up.
        // Using old-school immediate mode for simplicity.

        glBegin(GL_LINES);

        // Horizontal line (red)
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex2f(p.x - size, p.y);
        glVertex2f(p.x + size, p.y);

        // Vertical line (red)
        glVertex2f(p.x, p.y - size);
        glVertex2f(p.x, p.y + size);

        glEnd();
    }

    // Optional: draw a line between two contact points to visualise the contact "edge"
    inline void drawContactSegment(const Vec2& a, const Vec2& b) {
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f); // green
        glVertex2f(a.x, a.y);
        glVertex2f(b.x, b.y);
        glEnd();
    }

}

