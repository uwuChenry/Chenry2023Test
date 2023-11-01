#include "rrlib/util/Vector2.hpp"

namespace RRLib {
using namespace okapi;

Vector2::Vector2(QLength x, QLength y)
    : x(x)
    , y(y) {}
Vector2::Vector2()
    : Vector2(0_m, 0_m) {}
Vector2::Vector2(QAngle theta) {}

void Vector2::setX(QLength x) {
    this->x = x;
}

void Vector2::setY(QLength y) {
    this->y = y;
}

Vector2 Vector2::add(Vector2 a, Vector2 b) {
    return Vector2(a.x + b.x, a.y + b.y);
}

Vector2 Vector2::subtract(Vector2 a, Vector2 b) {
    return Vector2(a.x - b.x, a.y - b.y);
}

Vector2 Vector2::multiply(Vector2 a, double n) {
    return Vector2(a.x * n, a.y * n);
}

Vector2 Vector2::divide(Vector2 a, double n) {
    return multiply(a, 1.0 / n);
}

Vector2 Vector2::limit(Vector2 a, QLength n) {
    if (a.getMagSquared().convert(meter) > pow(n.convert(meter), 2)) {
        return multiply(
            normalize(a, meter),
            n.convert(meter));
    }
    return a;
}

Vector2 Vector2::rotate(Vector2 a, QAngle theta) {
    return Vector2(
        a.x * std::cos(theta.convert(radian)) - a.y * std::sin(theta.convert(radian)),
        a.x * std::sin(theta.convert(radian)) + a.y * std::cos(theta.convert(radian)));
}

Vector2 Vector2::normalize(Vector2 a, QLength unit) {
    return divide(
        a,
        a.getMag().convert(unit));
}

QLength Vector2::distanceBetween(Vector2 a, Vector2 b) {
    auto dX = (a.x - b.x).convert(meter);
    auto dY = (a.y - b.y).convert(meter);

    return std::sqrt(dX * dX + dY * dY) * meter;
}

QAngle Vector2::angleBetween(Vector2 a, Vector2 b) {
    return subtract(b, a).getTheta();
}

QLength Vector2::dot(Vector2 a, Vector2 b) {
    return meter * (a.x.convert(meter) * b.x.convert(meter) + a.y.convert(meter) * b.y.convert(meter));
}

void Vector2::setSelf(Vector2 other) {
    x = other.x;
    y = other.y;
}

void Vector2::addSelf(Vector2 other) {
    setSelf(add(*this, other));
}

void Vector2::subtractSelf(Vector2 other) {
    setSelf(subtract(*this, other));
}

void Vector2::multiplySelf(double n) {
    setSelf(multiply(*this, n));
}

void Vector2::divideSelf(double n) {
    multiplySelf(1.0 / n);
}

void Vector2::limitSelf(QLength n) {
    setSelf(limit(*this, n));
}

void Vector2::rotateSelf(QAngle theta) {
    setSelf(rotate(*this, theta));
}

void Vector2::normalizeSelf(QLength unit) {
    setSelf(normalize(*this, unit));
}

QLength Vector2::distanceTo(Vector2 other) {
    return distanceBetween(*this, other);
}

QAngle Vector2::angleTo(Vector2 other) {
    return angleBetween(*this, other);
}

QLength Vector2::dot(Vector2 other) {
    return dot(*this, other);
}

QLength Vector2::getX() {
    return x;
}

QLength Vector2::getY() {
    return y;
}

QAngle Vector2::getTheta() {
    return radian * std::atan2(y.convert(meter), x.convert(meter));
}

QLength Vector2::getMag() {
    return meter * std::sqrt(pow(x.convert(meter), 2) + pow(y.convert(meter), 2));
}

QLength Vector2::getMagSquared() {
    return meter * (pow(x.convert(meter), 2) + pow(y.convert(meter), 2));
}

Vector2 Vector2::operator+(const Vector2& rhs) {
    return Vector2::add(*this, rhs);
}

Vector2 Vector2::operator-(const Vector2& rhs) {
    return Vector2::subtract(*this, rhs);
}

Vector2 Vector2::operator*(double scalar) {
    return Vector2::multiply(*this, scalar);
}

Vector2 Vector2::operator/(double scalar) {
    return Vector2::divide(*this, scalar);
}

bool Vector2::operator==(const Vector2& other){
    return abs(x - other.x) < 1E-9 * meter && abs(y - other.y) < 1E-9 * meter;
}

bool Vector2::operator!=(const Vector2& other){
    return !(*this==other);
}

}