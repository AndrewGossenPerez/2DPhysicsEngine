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
