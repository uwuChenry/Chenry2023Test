#include "rrlib/util/vector2D.hpp"


Point2D Vector2D::getSecondPoint(){
    return Point2D(x + mag * cos(angle * 3.14159265358979323846 / 180), y + mag * sin (angle * 3.14159265358979323846 / 180));
}

Point2D Vector2D::getPoint(){
    return Point2D(x, y);
}

void Vector2D::addAngle(double relativeAngle){
    angle += relativeAngle;
}
