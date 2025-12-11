
// Vector2.h, created by Andrew Gossen.
// Establishes the Vector2 object, used to store Vector values with two components such as speed, acceleration, position.

#pragma once
#include <cmath>

struct Vec2{ // Defines a vector2 object 

    float x{0.0f};
    float y{0.0f};
    Vec2()=default;
    Vec2(float x,float y) : x(x), y(y) {}

    // Create operators for easy manipulation later on 
    
    Vec2 operator +(const Vec2& otherVector) const { // Add this vector with another 
        return Vec2(x+otherVector.x,y+otherVector.y);
    } 
    Vec2 operator -(const Vec2& otherVector) const { // Subtract this vector with another 
        return Vec2(x-otherVector.x,y-otherVector.y);
    } 
    Vec2 operator *(float scalar) const { // Scalar multiplier 
        return Vec2(x*scalar,y*scalar);
    }
    Vec2 operator /(float scalar) const { // Scalar division 
        return Vec2(x/scalar,y/scalar);
    }
    Vec2& operator +=(const Vec2& otherVector) { 
        x+=otherVector.x; y+=otherVector.y;
        return *this;
    };
    Vec2& operator -=(const Vec2& otherVector) { 
        x-=otherVector.x; y-=otherVector.y;
        return *this;
    };
    Vec2& operator *=(const Vec2& otherVector) { 
        x*=otherVector.x; y*=otherVector.y;
        return *this;
    };

    bool operator ==(const Vec2& otherVector) const { // Check whether this vector and otherVector are equivelant
        return ( (x==otherVector.x) && (y==otherVector.y )); 
    }

    float getLength() const{
        return std::sqrt(x*x + y*y);
    }

    Vec2 normalise() const{ // Get unit vector and avoid division by zero
        float len=getLength();
        if (len > 1e-6f) {
            return Vec2{x / len, y / len};
        }
        return Vec2{0.0f, 0.0f};
    }

};


