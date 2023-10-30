#include "rrlib/util/math.hpp"
#include "rrlib/util/Vector2.hpp"

namespace RRLib {
using namespace okapi;

namespace math {
double constrainAngleDouble(double angle){
    return atan2(sin(angle / 180.0 * 3.14159265358979323846), cos(angle / 180.0 * 3.14159265358979323846))*180/3.14159265358979323846;
}

QAngle constrainAngle(QAngle in, QAngle min, QAngle max) {
    double out = in.convert(radian);

    while (out > max.convert(radian)) {
        out -= 2_pi;
    }

    while (out < min.convert(radian)) {
        out += 2_pi;
    }

    return out * radian;
}


QLength getCircumRadius(Vector2 a, Vector2 b, Vector2 c){
    QLength a1 = b.distanceTo(c);
    QLength b1 = c.distanceTo(a);
    QLength c1 = a.distanceTo(b);
    

    QLength semiPerimeter = (a1 + b1 + c1) / 2.0;
    double area = 
    sqrt(semiPerimeter.convert(meter) * (semiPerimeter.convert(meter) - a1.convert(meter)) * (semiPerimeter.convert(meter) - b1.convert(meter)) * (semiPerimeter.convert(meter) - c1.convert(meter)));
    QLength radius = a1.convert(meter) * b1.convert(meter) * c1.convert(meter) / area / 4.0 * meter;
    return radius;
}

std::optional<double> circleLineIntersection(Vector2& start, Vector2& end, Vector2& point, QLength radius){
    Vector2 d = end - start;
    Vector2 f = start - point;

    auto a = d.dot(d);
    auto b = 2 * (f.dot(d));
    auto c = (f.dot(f).convert(meter) - (radius.convert(meter) * radius.convert(meter))) * meter;
    auto discriminant = b * b - 4 * a * c; 

    if(discriminant.getValue() >= 0){
        const auto dis = sqrt(discriminant);
        const double t1 = ((-1 * b - dis) / (2 * a)).convert(number);
        const double t2 = ((-1 * b + dis) / (2 * a)).convert(number);

        if(t2 >= 0 && t2 <= 1){
            return t2;
        }
        else if(t1 >= 0 && t1 <= 1){
            return t1;
        }   
    }

    return std::nullopt;
}


}
}