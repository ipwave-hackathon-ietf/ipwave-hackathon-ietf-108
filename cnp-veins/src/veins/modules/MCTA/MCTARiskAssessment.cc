#include "MCTARiskAssessment.h"


//This function returns the vehicles in my contour
std::vector<vehicle> RiskAssessment::returnVehiclesInMyContour(std::vector<std::vector<Coord>> vehContour, std::vector<vehicle> neighborVehicles){

    std::vector<vehicle> vehiclesInContour;
    std::cout <<" Neighbor Vehicles : "<<std::endl;
    for (unsigned int i = 0; i < neighborVehicles.size(); i++){
        std::cout <<neighborVehicles[i].vehicleId <<", Position: ("<<neighborVehicles[i].vehiclePosition.x<<"; "<<neighborVehicles[i].vehiclePosition.y<<"; "<<neighborVehicles[i].vehiclePosition.z<<")"<<std::endl;
    }
    std::cout <<std::endl;

    for (unsigned int n = 0; n < neighborVehicles.size(); n++){
        for (unsigned int i = 0; i < vehContour.size(); i++){
            bool flag = false;
            for (unsigned int j= 0; j < vehContour[i].size(); j++){
                if (abs(neighborVehicles[n].vehiclePosition.x) <= abs(vehContour[i][j].x) && abs(neighborVehicles[n].vehiclePosition.y) <= abs(vehContour[i][j].y)){
                    vehiclesInContour.push_back(neighborVehicles[n]);
                    flag = true;
                    break;
                }
            }
            if (flag == true){
                break;
            }
        }
    }
    return vehiclesInContour;
}



/* If a vehicle is approaching faster to the pivot vehicle, this test will be the most likely used to decide about the risk assessment
*/
double RiskAssessment::computeTimeToCollision(Coord pivotVehPos, Coord neighbVehPos, double pivVehSpeed, double neighbVehSpeed){
    double interVehDist = 0; //intervehicle sistance
    double TTC = 0; //Time to Collision
    double X1 = pivotVehPos.x - neighbVehPos.x;
    double Y1 = pivotVehPos.y - neighbVehPos.y;
    double Z1 = pivotVehPos.z - neighbVehPos.z;
//    interVehDist = sqrt(pow(X1, 2) + pow(Y1, 2) + pow(Z1,2));
    interVehDist = sqrt(pow(X1, 2) + pow(Y1, 2));
    std::cout <<" IDst = " <<interVehDist <<", neighbVehSpeed = " << neighbVehSpeed << " ; OstSpeed = " << pivVehSpeed<<" ; and Speed Diff = " <<fabs(neighbVehSpeed-pivVehSpeed)<<std::endl;
    TTC = interVehDist/(fabs(neighbVehSpeed-pivVehSpeed));

    return TTC;
}
/* When there is a small intervehicle distance and closely similar speeds, the Intervehicle time is more lickely to be used to decide the risk assessment
 */
double RiskAssessment::computeIntervehicleTime(Coord pivotVehPos, Coord neighbVehPos, double neighbVehSpeed){

    double interVehDist = 0; //intervehicle sistance
    double IVT = 0; //Inter-vehicle time
    double X1 = pivotVehPos.x - neighbVehPos.x;
    double Y1 = pivotVehPos.y - neighbVehPos.y;
    double Z1 = pivotVehPos.z - neighbVehPos.z;
    interVehDist = sqrt(pow(X1, 2) + pow(Y1, 2) + pow(Z1,2));
    IVT = interVehDist/fabs(neighbVehSpeed);

    return IVT;
}
/*Those condition will be used to measure the collision impact.*/
double RiskAssessment::computeEquivalentEnergeticSpeed(double CurVehMass, double neighbVehMass, double CurVehSpeed, double neighbVehSpeed){
   
    double EES = 0;
    EES = ((2 * neighbVehMass)/neighbVehMass + CurVehMass) * (neighbVehSpeed - CurVehSpeed);
    return EES;
}
