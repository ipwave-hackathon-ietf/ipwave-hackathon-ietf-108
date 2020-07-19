#include <veins/modules/MCTA/MCTAmotionPlanning.h>

double MinManeuverIVT = 1.0;
double MaxManeuverIVT = 3.0;
//coord manTagetPos;
//static bool onceTargSet = false;
motionPlanning::motionPlanning(){

}
motionPlanning::~motionPlanning(){

}
bool motionPlanning::obstacleIsOnMyWay(vehicle actualKinnematics, vehicle ObstacleInfo){
    bool onmyway=false;
    if (ObstacleInfo.vehiclePosition.y < actualKinnematics.vehiclePosition.y){ // Obstacle is ahead of me
        onmyway=true;
    }
    return onmyway;
}
bool motionPlanning::checkAmFrontVehicle(vehicle actualKinnematics, std::vector<vehicle> neighborList, vehicle obstacleInf){
    bool flagreturn = false;
    bool flag1 = false; // Check if no other vehicles in my Lane apart from me
    std::cout <<" -----------> neighborList size : " <<neighborList.size()<<std::endl;
    for (unsigned int i = 0; i < neighborList.size(); i++){
        if ((neighborList[i].vehicleId).compare(obstacleInf.vehicleId) != 0){
            if (actualKinnematics.laneID == neighborList[i].laneID){
                flag1 = true;
                if ((actualKinnematics.vehiclePosition.y < neighborList[i].vehiclePosition.y) &&
                        (actualKinnematics.vehiclePosition.y > obstacleInf.vehiclePosition.y)){
                    flagreturn = true;
                    std::cout << " ===========> I am front!!" <<std::endl;
                }
            }
        }
    }
    if (flag1 == false){ //no other vehicle in my lane
        flagreturn = true;
        std::cout << " ===========> I am the only one!!" <<std::endl;
    }
    else{
        std::cout << " ===========> There others in my Lane!!" <<std::endl;
    }
    return flagreturn;
}
bool motionPlanning::verifyCanSafelyStop (vehicle ActualKinnematics, std::vector<vehicle> neighborList, double minAcc){
    bool flagreturn = false;
    double IVD = 0; //Inter-vehicle distance
    double T2S = 0; //Time To Stop
    double Ds = 0; //The distance that Actual vehicle will travel to Stop
    double Df = 0; //The distance that front vehicle will traverse relatively to rear Vehicle to Stop
    T2S = fabs(ActualKinnematics.vehicleSpeed/minAcc);
    Ds = (minAcc * pow(T2S,2))/2 + ActualKinnematics.vehicleSpeed * T2S;
    for (unsigned int i = 0; i < neighborList.size(); i++){
        if ((neighborList[i].laneID == ActualKinnematics.laneID) && neighborList[i].vehiclePosition.y < ActualKinnematics.vehiclePosition.y){
            double tmpivd = sqrt(pow(ActualKinnematics.vehiclePosition.x - neighborList[i].vehiclePosition.x, 2)
                    + pow(ActualKinnematics.vehiclePosition.y - neighborList[i].vehiclePosition.y, 2)
                    +pow(ActualKinnematics.vehiclePosition.z - neighborList[i].vehiclePosition.z, 2));
            if (IVD == 0 || (IVD < tmpivd)){
                IVD = tmpivd;
                Df = tmpivd + neighborList[i].vehicleSpeed * T2S;
            }
        }
    }

    if (Ds < Df){
        flagreturn = true;
    }
    return flagreturn;
}

double motionPlanning::returnManeuverAngle(vehicle actualKinnematics, vehicle ObstacleInfo, int ManeuvLane, double T2C){
    double reshTime = 0.5;
    double phi = 0; //vehicle direction
    double Turn_Angle = 0;
    double LWidth = 3.0;
//    double reactionTime = 1.5;
    double reactionTime = T2C;
    std::vector<std::vector<Coord>> contCoord;
    std::vector<vehicle> NeighborInCont ;
    std::vector<double> TurningAngles;
    std::map<double, std::vector<Coord>>  maneuvCont;
    Coord dest = ObstacleInfo.vehiclePosition;
    dest.x = dest.x + LWidth;

    std::cout <<"----> Obstacle Pos :(" <<ObstacleInfo.vehiclePosition.x <<"; "<<ObstacleInfo.vehiclePosition.y <<"; "<<ObstacleInfo.vehiclePosition.z <<")"<<std::endl;
    maneuvCont = compute_contour_polygon(actualKinnematics, reactionTime/*, ManeuvLane*/);
    for (std::map<double, std::vector<Coord>>::iterator myit = maneuvCont.begin(); myit != maneuvCont.end(); myit++){
        std::cout <<" Angle = " << myit->first<< "  ==>> ";
        for (unsigned int i = 0; i < myit->second.size(); i++){
            std::cout <<" ("<<myit->second[i].x<<" ; " << myit->second[i].y <<" ; "<<myit->second[i].z <<") ";
        }
        std::cout <<std::endl <<std::endl;
    }
    Turn_Angle = returnVehicleTurningAngle(maneuvCont, dest, actualKinnematics.vehiclePosition);
    std::cout <<"Turn_Angle <<==>> : " <<Turn_Angle <<std::endl;

    return Turn_Angle;
}
ManeuverInfo motionPlanning::returnManeuverInfo(vehicle actualKinnematics, vehicle ObstacleInfo, int ManeuvLane, double T2C){
    ManeuverInfo ManI;
    double reshTime = 0.5;
    double phi = 0; //vehicle direction
    double LWidth = 3.0;
//    double reactionTime = 1.5;
    double reactionTime = T2C;
    std::vector<std::vector<Coord>> contCoord;
    std::vector<vehicle> NeighborInCont ;
    std::vector<double> TurningAngles;
    std::map<double, std::vector<Coord>>  maneuvCont;
    Coord dest = ObstacleInfo.vehiclePosition;

  /*  if (ManeuvLane < actualKinnematics.laneID){ //BAM comment due to Lane issue
        dest.x = dest.x - LWidth;
    }
    else{
        dest.x = dest.x + LWidth;
    }*/

    std::cout <<"----> Obstacle Pos :(" <<ObstacleInfo.vehiclePosition.x <<"; "<<ObstacleInfo.vehiclePosition.y <<"; "<<ObstacleInfo.vehiclePosition.z <<")"<<
            "; Chosen Destination :(" << dest.x <<"; " <<dest.y <<"; " << dest.z <<")" << ": Chosen Lane = " << ManeuvLane <<std::endl;

    maneuvCont = compute_contour_polygon(actualKinnematics, reactionTime/*, ManeuvLane*/);

//    std::fstream cont;
//    cont.open("contourData.vsa", std::ofstream::out | std::ofstream::app);
    /*if (cont.is_open()){
        std::cout <<"-------------------------------------------------------------------------------------" <<std::endl;
    }
    else{
        std::cout <<" ----Check what is wrong! ----" <<std::endl;
    }*/
//    cont <<"Turrning Angle " <<", " <<" X " <<", " <<" Y" <<", "<<" Z" <<std::endl;
    for (std::map<double, std::vector<Coord>>::iterator myit = maneuvCont.begin(); myit != maneuvCont.end(); myit++){
        std::cout <<" Angle = " << myit->first<< "  ==>> ";
        for (unsigned int i = 0; i < myit->second.size(); i++){
            std::cout <<" ("<<myit->second[i].x<<" ; " << myit->second[i].y <<" ; "<<myit->second[i].z <<") ";
//            cont<<myit->first <<", " <<myit->second[i].x <<", " << myit->second[i].y <<", " <<myit->second[i].z<<std::endl;
        }
        std::cout <<std::endl <<std::endl;
//        cont <<std::endl <<std::endl;
    }

    ManI = computeManeuvers(maneuvCont, dest, actualKinnematics.vehiclePosition);
    ManI.curThetha = pi/2 + (actualKinnematics.vehicleSpeed/ 2.62970 * tan(ManI.phi) * 0.05);
//    cont.close();

    return ManI;
}
ManeuverInfo motionPlanning::computeManeuvers(std::map<double, std::vector<Coord>> contInfo, Coord targPos, Coord curPos){
    ManeuverInfo ManInfo;
    std::map<double, std::vector<Coord>>::iterator mit;
    std::vector<Coord> turnroute;
    Coord dstPos; //Destination Position
    dstPos.x = 0;
    dstPos.y = 0;
    dstPos.z = 0;
    double turnAngle = 0;
    double PrevDst = 0;

    for (mit = contInfo.begin(); mit != contInfo.end(); mit++){
        int vSize = mit->second.size();
        Coord endPos = mit->second[vSize-1];

        double tmpdst = sqrt(pow(endPos.x - targPos.x, 2) + pow(endPos.y - targPos.y, 2));
        if (turnroute.empty()){
            turnAngle = mit->first;
            turnroute = mit->second;
            PrevDst = tmpdst;
        }
        else if (tmpdst < PrevDst){
            turnAngle = mit->first;
            turnroute = mit->second;
            PrevDst = tmpdst;
        }
    }

    ManInfo.targetPosition = targPos;
    ManInfo.phi = turnAngle;
    ManInfo.manPos = turnroute;

    std::cout << "!!!! Chosen steering Angle is :  " <<turnAngle <<std::endl;

    return ManInfo;
}
double motionPlanning::returnUpdatedManeuverAngle (vehicle actualKinnematics,double tman, double elapsedTime, Coord obsPosition, int ManLane){
    std::map<double, std::vector<Coord>>  maneuvCont;
    double Turn_Angle = 0;
    double reactionTime = tman - elapsedTime;
    double LaneconsidTime =  2*tman/3;
    maneuvCont =  compute_contour_polygon(actualKinnematics, reactionTime/*, ManLane*/);

  /* if (elapsedTime < LaneconsidTime){
        maneuvCont =  compute_contour_polygon(actualKinnematics, reactionTime, ManLane);
    }
    else{
        maneuvCont =  compute_contour_polygonUpdate(actualKinnematics, reactionTime);
    }*/
    std::cout <<"----> Obstacle Position Pos:(" <<obsPosition.x <<"; "<<obsPosition.y <<"; "<<obsPosition.z <<")"<<std::endl;
    for (std::map<double, std::vector<Coord>>::iterator myit = maneuvCont.begin(); myit != maneuvCont.end(); myit++){
        std::cout <<"Next Angle = " << myit->first<< "  ==>> ";
        for (unsigned int i = 0; i < myit->second.size(); i++){
            std::cout <<" ("<<myit->second[i].x<<" ; " << myit->second[i].y <<" ; "<<myit->second[i].z <<") ";
        }
        std::cout <<std::endl <<std::endl;
    }
    Turn_Angle = returnVehicleTurningAngle(maneuvCont, obsPosition, actualKinnematics.vehiclePosition);
    std::cout << "!!! Turn_Angle in returnUpdatedManeuverAngle  =  " << Turn_Angle <<std::endl;

    return Turn_Angle;
}

ManeuverInfo motionPlanning::minimumErrorManeuver (vehicle ActualKinnematics,std::vector<Coord> Ref_Path,  int ManLane, double ManTime, double DirectionAngle, int updnum){
    std::map<double, std::vector<Coord>>  maneuvCont;
    std::map<double, std::vector<Coord>>::iterator mit;
    std::vector<Coord>  stepRefpath = Ref_Path;
    stepRefpath.erase(stepRefpath.begin()+updnum);
    ManeuverInfo ManUp;
    double Turn_Angle = 0;
    std::vector<Coord> ManPath;

    maneuvCont =  compute_contour_polygonUpdate(ActualKinnematics, ManTime,ManLane, DirectionAngle);
    double minCumError = 0;
    bool flag = false;
    std::cout <<" Number of Updates = " << updnum <<" ; and Reference path Size " << Ref_Path.size() <<"Maneuver Time = " <<ManTime<< std::endl;
    for (mit = maneuvCont.begin(); mit != maneuvCont.end(); mit++){
        double Cumulative_Path_Error = 0;
//        for (unsigned int i = 0; i < mit->second.size(); i++){
        for (unsigned int i = 0; i < stepRefpath.size(); i++){
//            std::cout <<"[(" <<mit->second[i].x<<"; "<<mit->second[i].y <<"; "<<mit->second[i].z <<") : ("<<stepRefpath[i].x <<"; "<<stepRefpath[i].y <<"; "<<stepRefpath[i].z<<")]   " ;
            Cumulative_Path_Error = Cumulative_Path_Error + sqrt(pow(mit->second[i].x - stepRefpath[i].x, 2) + pow(mit->second[i].y - stepRefpath[i].y, 2));
        }
//        std::cout <<std::endl<<mit->first <<"  makes an error of = " << Cumulative_Path_Error << " Path Size = " << mit->second.size() <<std::endl;
        if (!flag){
            flag = true;
            minCumError = Cumulative_Path_Error;
            Turn_Angle = mit->first;
            ManPath = mit->second;
        }
        else if (Cumulative_Path_Error < minCumError){
            minCumError = Cumulative_Path_Error;
            Turn_Angle = mit->first;
            ManPath = mit->second;
        }
    }
    ManUp.phi = Turn_Angle;
    ManUp.curThetha = DirectionAngle + (fabs(ActualKinnematics.vehicleSpeed)/ 2.62970 * tan(Turn_Angle) * 0.05);

    return ManUp;
}
ManeuverInfo motionPlanning::returnManeuverUpdate (vehicle ActualKinnematics,vehicle obstIn, std::vector<Coord> Ref_Path, double Ref_phi, Coord TargPos, int ManLa, double manTime,double DirectionAngle){

    std::map<double, std::vector<Coord>>  maneuvCont;
    std::map<double, std::vector<Coord>>::iterator mit;
    double Turn_Angle = 0;
    std::vector<Coord> ManPath;
    ManeuverInfo ManUp;
    double minxDif = 0;

    maneuvCont =  compute_contour_polygonUpdate(ActualKinnematics, manTime,ManLa, DirectionAngle);

    for (mit = maneuvCont.begin(); mit != maneuvCont.end(); mit++){

        int vSize = mit->second.size();
        Coord endPos = mit->second[vSize-1];
        double xdif = fabs(endPos.x - TargPos.x);
       if((xdif <= 1)){

           if (Turn_Angle == 0){
               ManPath = mit->second;
               Turn_Angle = mit->first;
           }
           else if (fabs(Turn_Angle) < fabs(mit->first)){
               ManPath = mit->second;
               Turn_Angle = mit->first;
           }
       }
    }

   /* for (mit = maneuvCont.begin(); mit != maneuvCont.end(); mit++){
        std::cout <<" Angle = " << mit->first<< "  ==>> ";
        for (unsigned int i = 0; i < mit->second.size(); i++){
            std::cout <<" ("<<mit->second[i].x<<" ; " << mit->second[i].y <<" ; "<<mit->second[i].z <<") ";
        }
        std::cout <<std::endl <<std::endl;
    }
    std::cout <<"Target Position : (" <<TargPos.x <<"; " <<TargPos.y<<"; "<<TargPos.z <<")" <<std::endl;

    std::cout <<"Current Position : (" <<ActualKinnematics.vehiclePosition.x <<"; " <<ActualKinnematics.vehiclePosition.y
            <<"; "<<ActualKinnematics.vehiclePosition.z <<")" <<std::endl;
    std::cout <<"Obstacle Pos: (" <<obstIn.vehiclePosition.x <<"; " <<obstIn.vehiclePosition.y<<"; "<<obstIn.vehiclePosition.z <<")" <<std::endl;
    if (!ManPath.empty()){
        std::cout <<"Chosen Heading Position : (" <<ManPath[0].x <<"; " <<ManPath[1].y<<"; "<<ManPath[2].z <<")" <<std::endl;
    }
    std::cout << " Chosen Angle = " << Turn_Angle << std::endl;*/
    ManUp.phi = Turn_Angle;
    ManUp.curThetha = DirectionAngle + (fabs(ActualKinnematics.vehicleSpeed)/ 2.62970 * tan(Turn_Angle) * 0.05);

    return ManUp;
}

double motionPlanning::returnVehicleTurningAngle (std::map<double, std::vector<Coord>> contInfo, Coord obsPosition, Coord curPos){
    std::map<double, std::vector<Coord>>::iterator mit;
    std::vector<Coord> turnroute;
    Coord dstPos; //Destination Position
    dstPos.x = 0;
    dstPos.y = 0;
    dstPos.z = 0;
    double turnAngle = 0;
    bool setMetCondition = false;
    bool flag1 = false;
    for (mit = contInfo.begin(); mit != contInfo.end(); mit++){
        int numPos = mit->second.size();
        Coord Rdst; //Route Destination
        Rdst = mit->second[numPos-1];

        if (Rdst.y < obsPosition.y){
            if (!setMetCondition){
                dstPos = Rdst;
                turnAngle = mit->first;
                turnroute = mit->second;
                setMetCondition = true;
            }
            else{
                bool flagPrev = false;
                if (flagPrev){
                    double dstPosClose2Obstacle = 0; //Turn the vehicle toward the closest to Next Lane Position
                    double RdstClose2Obstacle = 0;
                    if (dstPos.x < obsPosition.x){
                        dstPosClose2Obstacle = fabs(fabs(dstPos.x - obsPosition.x) - 3);
                        RdstClose2Obstacle = fabs(fabs(Rdst.x - obsPosition.x) - 3);
                    }
                    else{

                        dstPosClose2Obstacle = fabs((dstPos.x - obsPosition.x) - 3);
                        RdstClose2Obstacle = fabs((Rdst.x - obsPosition.x) - 3);
                    }

                    if (RdstClose2Obstacle < dstPosClose2Obstacle){
                        dstPos = Rdst;
                        turnAngle = mit->first;
                        turnroute = mit->second;
                    }
                }
                else{
                    if (fabs(curPos.x - obsPosition.x) > 2){ //Follow the minimal Angle of the Contour Turning Angles
                        if (fabs(mit->first) < fabs(turnAngle)){
                            turnAngle = mit->first;
                        }
                    }
                    else{
                        if (fabs(Rdst.x - obsPosition.x) > 2.5 && fabs(Rdst.x - obsPosition.x) < 3.5){
                            flag1 = true;
                            if (fabs(mit->first) < fabs(turnAngle)){
                                dstPos = Rdst;
                                turnAngle = mit->first;
                                turnroute = mit->second;
                            }
                        }
                        else if ((flag1 == false) && (fabs(mit->first) < fabs(turnAngle))){
                            dstPos = Rdst;
                            turnAngle = mit->first;
                            turnroute = mit->second;
                        }
                    }
                }
            }
        }
    }
    std::cout << "!!!! Chosen steering Angle is :  " <<turnAngle <<std::endl;
    return turnAngle;
}

double motionPlanning::computeTimetoCollision(double NeighbourSpeed, Coord NeighborPosition, double pivVehSpeed, Coord pivVehPosition){
    double InterDist = 0;
    double T2C = 0;
    InterDist = sqrt(pow(NeighborPosition.x - pivVehPosition.x, 2) + pow(NeighborPosition.y - pivVehPosition.y, 2)
            + pow(NeighborPosition.z - pivVehPosition.z, 2));
    T2C = fabs(InterDist/(fabs(pivVehSpeed - NeighbourSpeed)));

    return T2C;
}
double motionPlanning::computeIntervehicleTime(double NeighbourSpeed, Coord NeighborPosition, double pivVehSpeed, Coord pivVehPosition){
    double InterDist = 0;
    double IVT = 0;
    InterDist = sqrt(pow(NeighborPosition.x - pivVehPosition.x, 2) + pow(NeighborPosition.y - pivVehPosition.y, 2)
            + pow(NeighborPosition.z - pivVehPosition.z, 2));
    IVT = fabs(InterDist/fabs(pivVehSpeed));

    return IVT;
}
double motionPlanning::computeContourVehiclesDensity(int contourNumberofVehicles, double pivVehSpeed){
    return contourNumberofVehicles/pivVehSpeed;
}
std::map<double,std::vector<Coord>> motionPlanning::compute_contour_polygon(vehicle CurVeh, double reactTime/*, int ManeuverLane*/){//key: phi, values: path Coordinates
    double L = 2.62970; // was 2.8 ==>  //Vehicle wheelbase
    double max_turing_angle = pi/18 * 2.5; //25 degrees
    Coord in_Pos;
    in_Pos.x = CurVeh.vehiclePosition.x;
    in_Pos.y = CurVeh.vehiclePosition.y;
    in_Pos.z = CurVeh.vehiclePosition.z;
    double delta_t = 0.05; //Update interval
    double v = CurVeh.vehicleSpeed;
    double t = reactTime;
//    std::cout <<" ---> Reflesh time : " << t <<std::endl;
    Coord Pos;
    std::vector<std::vector<Coord>> veh_Trajectory; //M
    std::map<double,std::vector<Coord>> cont_cood;//Turning Angle (TA) and corresponding step by step vehicle Positions

    Pos.x = in_Pos.x;
    Pos.y = in_Pos.y;
    Pos.z = in_Pos.z;

    int turn_angle_number = int (max_turing_angle/pi * 180);
   /* std::cout <<" ---> turn_angle_number : " <<turn_angle_number << "; and reaction time = "<< reactTime <<std::endl;
    std::cout <<" CurPos : (" << CurVeh.vehiclePosition.x <<"; " << CurVeh.vehiclePosition.y <<"; " << CurVeh.vehiclePosition.z <<")"
            <<"; Speed : " <<CurVeh.vehicleSpeed << "; LANE ID = " <<  CurVeh.laneID << std::endl;*/
    std::vector<double> TA;// Turning angles

    for (int i = 1; i <= turn_angle_number; i++){
        TA.push_back(pi * i/180); //TA in rad
    }

//    Straight movement
    int n = int (floor(t/delta_t));
    double delta_d = t- n*delta_t;
    if (delta_d > 0.01){
        n = n+1;
    }

    std::vector<Coord> M; //Vehicle trajectory
    double T = pi/2;
    double acc = 0;
    double phi = 0;
//    std::cout <<"Phi = " << phi <<" ==>";
    for (int j = 1; j < n+1; j++){
        if (j == n && delta_d > 0.01){
            v = v + acc * delta_d;
        }
        else{
            v = v + acc * delta_t;
        }

        T = T + (fabs(v)/ L* tan(phi) * delta_t);
        Pos.x = Pos.x + (v * cos(T))* delta_t;
        Pos.y = Pos.y+ (v * sin(T)) * delta_t;
        Pos.z = 0;
        M.push_back(Pos);
//        std::cout <<T <<" -->  ";
    }
//    std::cout<<std::endl;
    cont_cood.insert(std::make_pair(phi, M));
    M.clear();
//    Left turning
    for (int i = 0; i < turn_angle_number; i++){
        n = int (floor(t/delta_t));
        delta_d = t- n*delta_t;
        if (delta_d > 0.01){
            n = n+1;
        }
        phi = TA[i]; //acc and T are like above
        T = pi/2;
        Pos.x = in_Pos.x;
        Pos.y = in_Pos.y;
        Pos.z = in_Pos.z;
//        std::cout <<"Phi = " << phi <<" ==>";
        for (int j = 1; j < n+1; j++){
            if (j == n && delta_d > 0.01){
                v = v + acc * delta_d;
            }
            else{
                v = v + acc * delta_t;
            }
            T = T + (fabs(v)/ L* tan(phi) * delta_t);
            Pos.x = Pos.x + (v * cos(T))* delta_t;
            Pos.y = Pos.y + (v * sin(T)) * delta_t;
            Pos.z = 0;
            M.push_back(Pos);
//            std::cout <<T <<" -->  ";
        }
//        std::cout<<std::endl;
        cont_cood.insert(std::make_pair(phi, M));
        M.clear();
    }
//    Right turning
    for (int i = turn_angle_number; i > 0; i--){
        n = int (floor(t/delta_t));
        delta_d = t- n*delta_t;
        if (delta_d > 0.01){
            n = n+1;
        }
        phi = -TA[i-1]; //acc and T are like above
        T = pi/2;
        Pos.x = in_Pos.x;
        Pos.y = in_Pos.y;
        Pos.z = in_Pos.z;
//        std::cout <<"Phi = " << phi <<" ==>";
        for (int j = 1; j < n+1; j++){
            if (j == n && delta_d > 0.01){
                v = v + acc * delta_d;
            }
            else{
                v = v + acc * delta_t;
            }
            T = T + (fabs(v)/ L* tan(phi) * delta_t);
            Pos.x = Pos.x + (v * cos(T))* delta_t;
            Pos.y = Pos.y + (v * sin(T)) * delta_t;
            Pos.z = 0;
            M.push_back(Pos);//in matlab code they used N
//            std::cout <<T <<" -->  ";
        }
//        std::cout<<std::endl;

        cont_cood.insert(std::make_pair(phi, M));
        M.clear();
    }
    return cont_cood;
}
std::map<double, std::vector<Coord>> motionPlanning::compute_contour_polygonUpdate(vehicle CurVeh, double reactTime, int ManeuverLane, double curDirection){
    double L = 2.62970; // was 2.8 ==>  //Vehicle wheelbase
    double max_turing_angle = pi/18 * 2.5; //25 degrees
    Coord in_Pos;
    in_Pos.x = CurVeh.vehiclePosition.x;
    in_Pos.y = CurVeh.vehiclePosition.y;
    in_Pos.z = CurVeh.vehiclePosition.z;
    double delta_t = 0.05; //Update interval
    double v = CurVeh.vehicleSpeed;
    double t = reactTime;
    std::cout <<" ---> Reflesh time : " << t <<std::endl;
    Coord Pos;
    std::vector<std::vector<Coord>> veh_Trajectory; //M
    std::map<double,std::vector<Coord>> cont_cood;//Turning Angle (TA) and corresponding step by step vehicle Positions

    Pos.x = in_Pos.x;
    Pos.y = in_Pos.y;
    Pos.z = in_Pos.z;

    int turn_angle_number = int (max_turing_angle/pi * 180);
    std::cout <<" ---> turn_angle_number : " <<turn_angle_number <<std::endl;
    std::cout <<" CurPos : (" << CurVeh.vehiclePosition.x <<"; " << CurVeh.vehiclePosition.y <<"; " << CurVeh.vehiclePosition.z <<")"
            <<" Speed : " <<CurVeh.vehicleSpeed <<std::endl;
    std::vector<double> TA;// Turning angles

    for (int i = 0; i < turn_angle_number; i++){
        TA.push_back(pi * i/180); //TA in rad
    }

//    Straight movement
    int n = int (floor(t/delta_t));
    double delta_d = t- n*delta_t;
    if (delta_d > 0.01){
        n = n+1;
    }

    std::vector<Coord> M; //Vehicle trajectory
    double T = curDirection;
    double acc = 0;
    double phi = 0;

    for (int j = 1; j < n; j++){
        if (j == n && delta_d > 0.01){
            v = v + acc * delta_d;
        }
        else{
            v = v + acc * delta_t;
        }
        T = T + (fabs(v)/ L* tan(phi) * delta_t);
        Pos.x = Pos.x + (v * cos(T))* delta_t;
        Pos.y = Pos.y + (v * sin(T)) * delta_t;
        Pos.z = 0;
        M.push_back(Pos);
    }

    cont_cood.insert(std::make_pair(phi, M));
    M.clear();
    //    Left turning
    for (int i = 0; i < turn_angle_number; i++){
        n = int (floor(t/delta_t));
        delta_d = t- n*delta_t;
        if (delta_d > 0.01){
            n = n+1;
        }
         phi = TA[i]; //acc and T are like above
         T = curDirection;
         Pos.x = in_Pos.x;
         Pos.y = in_Pos.y;
         Pos.z = in_Pos.z;
         for (int j = 1; j < n+1; j++){
             if (j == n && delta_d > 0.01){
                 v = v + acc * delta_d;
             }
             else{
                 v = v + acc * delta_t;
             }
             T = T + (fabs(v)/ L* tan(phi) * delta_t);
             Pos.x = Pos.x + (v * cos(T))* delta_t;
             Pos.y = Pos.y + (v * sin(T)) * delta_t;
             Pos.z = 0;
             M.push_back(Pos);
         }
         cont_cood.insert(std::make_pair(phi, M));
         M.clear();
    }
//        Right turning
    for (int i = turn_angle_number; i > 0; i--){
        n = int (floor(t/delta_t));
        delta_d = t- n*delta_t;
        if (delta_d > 0.01){
            n = n+1;
        }
         phi = -TA[i-1]; //acc and T are like above
         T = curDirection;
         Pos.x = in_Pos.x;
         Pos.y = in_Pos.y;
         Pos.z = in_Pos.z;
         for (int j = 1; j < n+1; j++){
             if (j == n && delta_d > 0.01){
                 v = v + acc * delta_d;
             }
             else{
                 v = v + acc * delta_t;
             }
             T = T + (fabs(v)/ L* tan(phi) * delta_t);
             Pos.x = Pos.x + (v * cos(T))* delta_t;
             Pos.y = Pos.y + (v * sin(T)) * delta_t;
             Pos.z = 0; // To be re-thought to model 3D vehicle position tracking!!!
             M.push_back(Pos);//in matlab code they used N
         }
         cont_cood.insert(std::make_pair(phi, M));
         M.clear();
    }
    return cont_cood;
}
