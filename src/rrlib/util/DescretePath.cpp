#include "rrlib/util/DescretePath.hpp"
#include "rrlib/util/math.hpp"

namespace RRLib{
DiscretePath::DiscretePath(const std::initializer_list<Vector2>& waypoint) : path(waypoint){}

DiscretePath::DiscretePath(const std::vector<Vector2>& waypoint) : path(waypoint){}

DiscretePath DiscretePath::operator+(const DiscretePath& rhs) const{
    DiscretePath result(path);
    result.path.insert(result.path.end(), rhs.path.begin(), rhs.path.end());
    return result;
}

DiscretePath DiscretePath::operator+(const Vector2& rhs) const{
    DiscretePath result(path);
    result.path.emplace_back(rhs);
    return result;
}

DiscretePath& DiscretePath::operator+=(const DiscretePath& rhs){
    path.insert(path.end(), rhs.path.begin(), rhs.path.end());
    return *this;
}

DiscretePath& DiscretePath::operator+=(const Vector2& rhs){
    path.emplace_back(rhs);
    return *this;
}

std::vector<Vector2>::iterator DiscretePath::begin(){
    return path.begin();
}

std::vector<Vector2>::iterator DiscretePath::end(){
    return path.end();
}

std::vector<Vector2>::reverse_iterator DiscretePath::rbegin(){
    return path.rbegin();
}

std::vector<Vector2>::reverse_iterator DiscretePath::rend(){
    return path.rend();
}

std::vector<Vector2>::const_iterator DiscretePath::begin() const{
    return path.begin();
}

std::vector<Vector2>::const_iterator DiscretePath::end() const{
    return path.end();
}

std::vector<Vector2>::const_reverse_iterator DiscretePath::rbegin() const{
    return path.rbegin();
}

std::vector<Vector2>::const_reverse_iterator DiscretePath::rend() const{
    return path.rend();
}

Vector2& DiscretePath::operator[](int index){
    return path[index];
}

const Vector2& DiscretePath::operator[](int index) const{
    return path[index];
}

Vector2& DiscretePath::front(){
    return path.front();
}

const Vector2& DiscretePath::front() const{
    return path.front();
}

Vector2& DiscretePath::back(){
    return path.back();
}

const Vector2& DiscretePath::back() const{
    return path.back();
}

int DiscretePath::size() const{
    return (int)path.size();
}

double DiscretePath::getCurvature(int index) const{
    if(index <= 0 || index >= (int)path.size()-1){
        return 0 ;
    }

    QLength radius = math::getCircumRadius(path[index-1], path[index], path[index+1]);

    if(std::isnan(radius.getValue())){
        return 0;
    }

    return 1 / radius.convert(meter);
}

std::vector<Vector2>::iterator closestPoint(std::vector<Vector2>::iterator begin, std::vector<Vector2>::iterator end, Vector2 point){
    const auto comparison = [point](Vector2& a, Vector2& b){
        return a.distanceTo(point) < b.distanceTo(point);
    };

    return std::min_element(begin, end, comparison);
}



} 