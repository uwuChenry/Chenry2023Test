#include "rrlib/motion/PurePursuit.hpp"

namespace RRLib{

int AdaptivePurePursuitController::getClosestPoint(Pose currentPos, DiscretePath& path){
    QLength minDist {std::numeric_limits<double>::max()};
    int closest = prevClosest.value_or(0);

    for(int i = closest; i < path.size(); i++){
        QLength dist = currentPos.position.distanceTo(path[i]);
        if(dist < minDist){
            minDist = dist;
            closest = i;
        }
    }

    prevClosest = closest;
    return closest;
}


void AdaptivePurePursuitController::followPath(DiscretePath& path, QTime timeout, bool isReversed){
    int closestPointIndex = 0;
    const ChassisScales scales = chassisController->getChassisScales();
    Vector2 lookAheadPoint = path[0];
    timeUtil.getTimer()->placeMark();

    do{
        Pose pos = {chassisController->getState().x, chassisController->getState().y, chassisController->getState().theta};
        closestPointIndex = getClosestPoint(pos, path);
        
        lookAheadPoint = getLookaheadPoint(path, closestPointIndex, pos.position, lookAhead).value_or(lookAheadPoint);


        //auto thing = *closestPointIter;
        //printf("%f cloestpointx, %f pos.possition \n", thing.getX().convert(centimeter), pos.position.getX().convert(centimeter));
        //printf("%f path back", path.back().getX().convert(centimeter));
        pros::delay(10);
    } while(timeUtil.getTimer()->getDtFromMark() < timeout);
    chassisController->stop();
}

// void AdaptivePurePursuitController::stop(){
//     task.notify();
//     pros::delay(10);
//     chassis->stop();
// }

void waitUntilSettled(){
    while(true){
        pros::delay(10);
    }
}

std::optional<Vector2> AdaptivePurePursuitController::getLookaheadPoint(DiscretePath& path, int minIndex, Vector2 point, QLength radius){
    for(int i = minIndex; i < path.size(); i++){
        Vector2& start = path[i];
        Vector2& end = path[i+1];
        const auto t = math::circleLineIntersection(start, end, point, radius);
        if(t){
            return start + (end - start) * t.value();
        }
    }

    return std::nullopt;
}

}