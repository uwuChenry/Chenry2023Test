#pragma once
#include "rrlib/util/Vector2.hpp"
#include "okapi/api/units/QArea.hpp"
#include "rrlib/util/math.hpp"
#include<vector>   
#include<exception>
#include<algorithm>

namespace RRLib{
using namespace okapi;

class DiscretePath{
    public:
    DiscretePath() = default;
    DiscretePath(const std::initializer_list<Vector2>& waypoint);
    DiscretePath(const std::vector<Vector2>& waypoint);
    ~DiscretePath() = default;

    DiscretePath operator+(const DiscretePath& rhs) const;
    DiscretePath operator+(const Vector2& rhs) const;
    DiscretePath& operator+=(const DiscretePath& rhs);
    DiscretePath& operator+=(const Vector2& rhs);

    std::vector<Vector2>::iterator begin();
    std::vector<Vector2>::const_iterator begin() const;
    std::vector<Vector2>::iterator end();
    std::vector<Vector2>::const_iterator end() const;
    
    std::vector<Vector2>::reverse_iterator rbegin();
    std::vector<Vector2>::const_reverse_iterator rbegin() const;
    std::vector<Vector2>::reverse_iterator rend();
    std::vector<Vector2>::const_reverse_iterator rend() const;


    Vector2& operator[](int index);
    const Vector2& operator[](int index) const;
    Vector2& front();
    const Vector2& front() const;
    Vector2& back();
    const Vector2& back() const;
    double getCurvature(int index) const;
    int size() const;

    private:
    std::vector<Vector2> path;
};

std::vector<Vector2>::iterator closestPoint(std::vector<Vector2>::iterator begin, std::vector<Vector2>::iterator end, Vector2 point);
}