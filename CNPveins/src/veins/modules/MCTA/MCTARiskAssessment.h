#include <vector>
#include <map>
//#include "MCTAposition.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <veins/modules/MCTA/ConvexHullAlgorithms.h>

#define pi 3.141592653589793

struct vehicle
{
    std::string vehicleId;
    Coord vehiclePosition;
    double vehicleDirection;//Phi
    double vehicleSpeed;
    double acceleration;
    double angleToObstacle;
    int laneID; //Subject to change the type (Used to Maneuver Vehicles)
};

 class RiskAssessment: public convexhull
{
public:
    void testingScenario();
    std::vector<vehicle> returnVehiclesInMyContour(std::vector<std::vector<Coord>> vehContour, std::vector<vehicle> neighbourVehicles);
    double computeTimeToCollision(Coord pivotVehPos, Coord neighbVehPos, double pivVehSpeed, double neighbVehSpeed);
    double computeIntervehicleTime(Coord pivotVehPos, Coord neighbVehPos, double neighbVehSpeed);
    double computeEquivalentEnergeticSpeed(double CurVehMass, double neighbVehMass, double CurVehSpeed, double neighbVehSpeed);
    bool isObstacleInMyContour(std::vector<std::vector<Coord>> contCoord, vehicle ObstacleInfo);
private:
    std::vector<Coord> returnVehiclesInMyContour_other(std::vector<std::vector<Coord>> vehContour, std::vector<Coord> neighbourVehiclesPositions);
};
