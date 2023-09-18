#pragma once

#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace RRLib {
using namespace okapi::literals;

// A class for a 2 dimensional vector, using okapi units, designed for position
class Vector2 {
    okapi::QLength x;
    okapi::QLength y;

public:
    Vector2();
    Vector2(okapi::QLength x, okapi::QLength y);
    Vector2(okapi::QAngle theta);

    static Vector2 add(Vector2 a, Vector2 b);
    static Vector2 subtract(Vector2 a, Vector2 b);
    static Vector2 multiply(Vector2 a, double n);
    static Vector2 divide(Vector2 a, double n);
    static Vector2 limit(Vector2 a, okapi::QLength limit);
    static Vector2 rotate(Vector2 a, okapi::QAngle theta);
    static Vector2 normalize(Vector2 a, okapi::QLength unit = okapi::meter);
    static okapi::QLength distanceBetween(Vector2 a, Vector2 b);
    static okapi::QAngle angleBetween(Vector2 a, Vector2 b);
    static okapi::QLength dot(Vector2 a, Vector2 b);

    void setX(okapi::QLength x);
    void setY(okapi::QLength y);
    void setSelf(Vector2 other);
    void addSelf(Vector2 other);
    void subtractSelf(Vector2 other);
    void multiplySelf(double n);
    void divideSelf(double n);
    void limitSelf(okapi::QLength n);
    void rotateSelf(okapi::QAngle theta);
    void normalizeSelf(okapi::QLength unit = okapi::meter);

    // TODO make a bunch of statics, then layer these ontop
    // ^ mostly done
    okapi::QLength distanceTo(Vector2 other);
    okapi::QAngle angleTo(Vector2 other);
    okapi::QLength dot(Vector2 other);

    okapi::QLength getX();
    okapi::QLength getY();
    okapi::QAngle getTheta();
    okapi::QLength getMag();
    okapi::QLength getMagSquared();

    Vector2 operator+(const Vector2& rhs);
    Vector2 operator-(const Vector2& rhs);
    Vector2 operator*(double scalar);
    Vector2 operator/(double scalar);
};
}