#include "MCTARiskAssessment.h"
#include <iostream>
#include <fstream>


struct ManeuverInfo{
    int manLaneDir = 0; //this manLaneDir variable takes the value -1 when turning right, 0 when keep Lane and 1 when turning left (increase in Lane)
    float phi = 0; // the Turning Angle (Steering Angle)
    float a = 0;
    float curThetha = 0;  // The direction Angle
    Coord targetPosition;
    std::vector<Coord> manPos;
};
class motionPlanning : public RiskAssessment
{
public:
	motionPlanning();
	~motionPlanning();

    double returnManeuverAngle(vehicle actualKinnematics, vehicle ObstacleInfo, int ManeuvLane, double T2C);
    ManeuverInfo computeManeuvers(std::map<double, std::vector<Coord>> contInfo, Coord targPos, Coord curPos);
    ManeuverInfo returnManeuverInfo(vehicle actualKinnematics, vehicle ObstacleInfo, int ManeuvLane, double T2C);
    ManeuverInfo minimumErrorManeuver (vehicle ActualKinnematics,std::vector<Coord> Ref_Path, int ManLane, double ManTime, double DirectionAngle, int updnum);
    ManeuverInfo returnManeuverUpdate (vehicle ActualKinnematics, vehicle obstaIn, std::vector<Coord> Ref_Path, double Ref_phi, Coord TargPos, int ManLa, double manTime, double DirectionAngle);
    std::vector<Coord> returnPlanManeuverPath();
    double returnUpdatedManeuverAngle (vehicle actualKinnematics,double maneuvT, double elapsedTime, Coord ostPosition, int ManLane);
	double returnVehicleTurningAngle (std::map<double, std::vector<Coord>> contInfo, Coord obstaclePosition, Coord curPos);

	void emergencyVehManeuver(std::vector<vehicle> NeighborVehicles, vehicle accVeh);

    double computeTimetoCollision(double NeighbourSpeed, Coord NeighborPosition, double pivVehSpeed, Coord pivVehPosition);
    double computeIntervehicleTime(double NeighbourSpeed, Coord NeighborPosition, double pivVehSpeed, Coord pivVehPosition);
    double computeContourVehiclesDensity(int contourNumberofVehicles, double pivVehSpeed);

    bool obstacleIsOnMyWay(vehicle actualKinnematics, vehicle ObstacleInfo);
    bool checkAmFrontVehicle(vehicle actualKinnematics, std::vector<vehicle> neighborList, vehicle obstacleInf);
    bool verifyCanSafelyStop (vehicle ActualKinnematics, std::vector<vehicle> neighborList, double minAcc);

    std::map<double, std::vector<Coord>> compute_contour_polygon(vehicle CurVeh, double reactTime/*, int ManeuverLane*/); //return contour coordinates
    std::map<double, std::vector<Coord>> compute_contour_polygon_BackUp(vehicle CurVeh, double reactTime, int ManeuverLane); //return contour coordinates
    std::map<double, std::vector<Coord>> compute_contour_polygonUpdate(vehicle CurVeh, double reactTime, int ManeuverLane, double curDirection); //return contour coordinates

     enum MovementType{
       STRAIGHT,
       TURN_LEFT,
       TURN_RIGHT
     };

};
