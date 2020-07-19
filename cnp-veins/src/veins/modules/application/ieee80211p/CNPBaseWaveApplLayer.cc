//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/ieee80211p/CNPBaseWaveApplLayer.h"

const simsignalwrap_t CNPBaseWaveApplLayer::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t CNPBaseWaveApplLayer::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

vehicle ObstacleInfo;
TraCIMobility* obMob;
std::vector<Clusterstruct> AllClustInfo;
static int clusterValue = 1; //This value will keep increasing the number of cluster increases
double nextClusterFormationStartTime = 0;
bool isFirstClustVeh = false;
double clusterFormationStart = 0;
bool AheadCHinEmMan = false;
double AheadManeuverStartTime = 0;

double AheadManDuration = 0.0;
int glManLane = 0;
bool sortLaneVehs(const TraCIMobility* Mob1, const TraCIMobility* Mob2){
    return (Mob1->getCurrentPosition().y <= Mob2->getCurrentPosition().y);
}
//#define DEBUG_ACCIDENTEVT
//#define DEBUG_ACCIDENTPACKET
//#define DEBUG_EMERGENTMANEUVER
//#define DEBUG_CLUSTMEMBEREXIT
//Recording CAN data
std::vector<CANDataCollection> CANStatisticsCollection;
std::vector<collisionRecords> recordedCollisions ;
int numVehsinroad = 0;
int overheadRecord = 0;

void CNPBaseWaveApplLayer::initialize(int stage) {
    BaseApplLayer::initialize(stage);

    if (stage==0) {

        //initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = NULL;
            mobility = NULL;
            traciVehicle = NULL;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();

        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconUserPriority = par("beaconUserPriority").longValue();
        beaconInterval =  par("beaconInterval");

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;

        isParked = false;

//        BAM
        maxIvTThresh = par("maxivtThresh").doubleValue();
        minIvTThresh = par("minivtThresh").doubleValue();
        maxT2CThresh = par("maxt2cThresh").doubleValue();
        minT2CThresh = par("mint2cThresh").doubleValue();
        clustFormPeriod = par("singleClusterFormationPeriod").doubleValue();

//       if ( strcmp(findHost()->getFullName(), "node[0]") == 0){
//           Coord JPos = mobility->getCommandInterface()->junction("1/1").getPosition();
//           std::cout <<" JPos = (" << JPos.x <<"; "<<JPos.y <<"; " <<JPos.z <<")" <<std::endl;
//       }
        ckeckLaneChange = par("tesVeh").boolValue();
////        end BAM

        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt           = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt              = new cMessage("wsa evt", SEND_WSA_EVT);
        triggerAccEvt           = new cMessage("Accident Trigger");
        triggerManeuverEvt      = new cMessage("Emergency maneuver");
        rearVehManEvt           = new cMessage("Rear vehicles Maneuver");
        updateManeuverPath      = new cMessage("Maneuver Path Update");
        checkProximitySensorEvt = new cMessage("Proximity Sensing");
        stopvehicle             = new cMessage ("artificial accident");

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;
        if (strcmp(findHost()->getFullName(), "node[2]") == 0){
            scheduleAt(simTime() + 20, stopvehicle);
        }
        if (strcmp(findHost()->getName(), "node") == 0){//Assign Newly injected Vehicles to a Corresponding Cluster
            InjTime = simTime().dbl();
            if  (!isFirstClustVeh){
                isFirstClustVeh = true;
                Clusterstruct tmpClust;
                tmpClust.cluster_id = clusterValue;
                clusterFormationStart = simTime().dbl();
                tmpClust.cluster_Hid = mobility->getExternalId();
                tmpClust.cluster_id = clusterValue;
                tmpClust.clusterMembMob.insert(std::make_pair(mobility->getExternalId(),mobility));
                AllClustInfo.push_back(tmpClust);
//                std::cout <<" Cluster = " << clusterValue << "  started by " << mobility->getExternalId() <<std::endl;
            }
            else if (simTime().dbl() - clusterFormationStart < clustFormPeriod){
//                std::cout <<"{********** } Cluster Before: " <<AllClustInfo.back().cluster_id <<" ; VID = " <<mobility->getExternalId() << " ;  Its size = " <<AllClustInfo.back().clusterMembMob.size() << std::endl;
                AllClustInfo.back().clusterMembMob.insert(std::make_pair(mobility->getExternalId(),mobility));
                std::string CHID = getClusterHeadID(AllClustInfo.back().clusterMembMob);
//                std::cout <<"{**********} Cluster After: " <<AllClustInfo.back().cluster_id <<" ; VID = " <<mobility->getExternalId() << " ;  Its size = " <<AllClustInfo.back().clusterMembMob.size()<< std::endl<< std::endl << std::endl;
                AllClustInfo.back().cluster_Hid = CHID;
            }
            else{
//                std::cout <<"{########## Prev } Cluster ID : " <<AllClustInfo.back().cluster_id <<" ; CSize = "<<AllClustInfo.back().clusterMembMob.size()<<" ; CH = " <<AllClustInfo.back().cluster_Hid <<std::endl;
                Clusterstruct tmpClust;
                clusterValue = clusterValue + 1;
                tmpClust.cluster_id = clusterValue;
                tmpClust.cluster_Hid = mobility->getExternalId();
                tmpClust.cluster_id = clusterValue;
                tmpClust.clusterMembMob.insert(std::make_pair(mobility->getExternalId(),mobility));
                AllClustInfo.push_back(tmpClust);
                clusterFormationStart = simTime().dbl();
//                std::cout <<"{########## Next }  Cluster ID = " << clusterValue <<" ; CSize = "<<AllClustInfo.back().clusterMembMob.size()<< " ;  started by " << mobility->getExternalId() <<std::endl;
            }
            mycluster = AllClustInfo.back();
            numVehsinroad ++;
//            std::cout <<std::endl<<std::endl;
//            std::cout << " 111111111  Vehicle  "<<mobility->getExternalId() <<"  Cluster ID = " << AllClustInfo.back().cluster_id<<std::endl;
        }
    }
    else if (stage == 1) {
        //simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            std::cerr << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if ( beaconInterval.raw() % (mac->getSwitchingInterval().raw()*2)) {
                    std::cerr << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2*mac->getSwitchingInterval() << "). "
                            << "This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, type_CCH);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
        if (par("tesVeh").boolValue() == true){
            int curlane = mobility->getVehicleCommandInterface()->getLaneIndex();
            std::cout <<"------>  Change Lane Invoked by " << findHost()->getFullName() << ", from Lane " << curlane <<std::endl;
            if (curlane > 0){
                mobility->getVehicleCommandInterface()->changeLane(curlane-1, 1000);
            }
            else{
                mobility->getVehicleCommandInterface()->changeLane(curlane+1, 1000);
            }
        }
    }
}

simtime_t CNPBaseWaveApplLayer::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); //usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval*2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() %switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    //check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw()  % (2*switchingInterval.raw()) > switchingInterval.raw()) {
        //firstEvent is within a sch interval
        if (chan == type_CCH) firstEvent -= switchingInterval;
    }
    else {
        //firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == type_SCH) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void CNPBaseWaveApplLayer::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {

    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);


    if (CCM* bsm = dynamic_cast<CCM*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
        bsm->setSenderID(findHost()->getFullName());
        if (strcmp(findHost()->getName(), "node") == 0){
            bsm->setSendingSpeed(mobility->getSpeed());
        }
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (ECM* acc = dynamic_cast<ECM*>(wsm)){
        acc->setSenderPos(curPosition);
        acc->setAccVehSpeed(0);
        acc->setChannelNumber(Channels::SCH1);
        acc->setPsid(currentOfferedServiceId);
        acc->setSendingTime(simTime());
        acc->setAccidentLane(mobility->getVehicleCommandInterface()->getLaneId().c_str());
        acc->setSenderid(mobility->getExternalId().c_str());
        acc->setAccLInd(mobility->getVehicleCommandInterface()->getLaneIndex());
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(Channels::SCH1); //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else wsm->setChannelNumber(Channels::CCH);
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void CNPBaseWaveApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void CNPBaseWaveApplLayer::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
    if (strcmp(findHost()->getFullName() , "node[1]") == 0){
//        std::cout <<"Time: "<<simTime()<<"Pos : (" <<curPosition.x <<"; "<<curPosition.y <<"; "<<curPosition.z <<")" <<std::endl;
    }
}

void CNPBaseWaveApplLayer::handleParkingUpdate(cObject* obj) {
    //this code should only run when used with TraCI
    isParked = mobility->getParkingState();
    if (communicateWhileParked == false) {
        if (isParked == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

void CNPBaseWaveApplLayer::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (CCM* bsm = dynamic_cast<CCM*>(wsm)) {
        std::cout <<"CCM Received by "<< findHost()->getFullName()  << std::endl;
//        receivedBSMs++;
//        onBSM(bsm);
        NeighborInfo tmpinfo;

        tmpinfo.vehicle_t.vehicleId = bsm->getSenderID();
        tmpinfo.ReceptionTime = simTime();
        tmpinfo.vehicle_t.vehiclePosition.x = bsm->getSenderPos().x;
        tmpinfo.vehicle_t.vehiclePosition.y = bsm->getSenderPos().y;
        tmpinfo.vehicle_t.vehiclePosition.z = bsm->getSenderPos().z;
        tmpinfo.vehicle_t.vehicleSpeed = bsm->getSendingSpeed();


        if (strcmp(findHost()->getName(), "node") == 0){
            if (checkIsCH(mobility->getExternalId())){
                NeigborsRecording(tmpinfo);
            }
        }
        else{
            NeigborsRecording(tmpinfo);
        }
    }
    else if (ECM* pkt = dynamic_cast<ECM*>(wsm)){
        if (strcmp(findHost()->getName(), "node") == 0){
            std::cout <<" ECM Pkt RCVD by " << mobility->getExternalId() <<std::endl;
            if (ManeuverProcessEnabled == false){
                if (checkIsCH(mobility->getExternalId())){
                    ManeuverProcessEnabled = true;
                    std::cout <<"<------ > First triggerManeuverEvt by " << findHost()->getFullName() << std::endl;
                    // scheduleAt(simTime() + 0.01, triggerManeuverEvt);
                    clusterManeuverHandler();
                    if (triggerAccEvt->isScheduled() == true) {
                        cancelEvent(triggerAccEvt);
                    }
                    scheduleAt(simTime() + 0.3, triggerAccEvt);
                    CollDetTime  = simTime().dbl();
                }
            }
        }
#ifdef DEBUG_ACCIDENTPACKET
        if (strcmp(findHost()->getName(), "node") == 0){
            std::cout <<" ACC Pkt RCVD by " << mobility->getExternalId() <<std::endl;
            if (ManeuverProcessEnabled == false){
                if (checkIsCH(mobility->getExternalId())){
                    std::cout <<"YYYYYYYYY> Accpkt received by ch --- >" <<findHost()->getFullName() <<std::endl;
                    std::cout<<"_____________________"<<findHost()->getFullName()<<"________________________"<<std::endl;
                    std::cout <<"<------ > First triggerManeuverEvt by " << findHost()->getFullName() << std::endl;
                }
            }
            else{
                std::cout <<"xxxxxxxxxxxxxxxxxxxx> Accpkt was already received by " <<findHost()->getFullName() <<std::endl;
            }
        }
#endif
    }
    else {
//        receivedWSMs++;
//        onWSM(wsm);
    }

    delete(msg);
}

void CNPBaseWaveApplLayer::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        CCM* bsm = new CCM();
        populateWSM(bsm);
        sendDown(bsm);
        if (!sendBeaconEvt->isScheduled()){
            scheduleAt(simTime() + beaconInterval * 0.3, sendBeaconEvt);
        }
        break;
    }
    case SEND_WSA_EVT:   {
        WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    default: {
        if (msg)
            DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
    if (msg == triggerAccEvt){
        if (strcmp(findHost()->getFullName(), "node[2]") == 0) {
            Coord pos;
            pos.x = traciVehicle->ReturnSumoCurrentposition(curPosition).x;
            pos.y = traciVehicle->ReturnSumoCurrentposition(curPosition).y;
            pos.z = curPosition.z;
            ObstacleInfo.vehicleId = mobility->getExternalId();
            ObstacleInfo.vehiclePosition = pos;
            int li = mobility->getVehicleCommandInterface()->getLaneIndex();
            ObstacleInfo.laneID = li;
            obMob = mobility;

            std::string lo = mobility->getVehicleCommandInterface()->getLaneId();
        }

//        std::cout <<" Obstacle Lane = " << lo << " Lane Index = " << li << std::endl;
        overheadRecord ++;
        ECM* acc = new ECM();
        populateWSM(acc);
        sendDown(acc);

#ifdef DEBUG_ACCIDENTPACKET
        Coord CurPos = mobility->getCurrentPosition();
        std::cout << "CUR POS  = (" << CurPos.x <<"; " << CurPos.y <<"; " << CurPos.z <<")" << std::endl;
        std::cout <<"---> Acc MSG SENT !" <<std::endl;
#endif
        scheduleAt(simTime() + 0.3, triggerAccEvt);
    }
    else if (msg == stopvehicle){
        mobility->getVehicleCommandInterface()->setSpeed(0);
        if (sendBeacons){ //Send only Accident Packets when using Network sensing
            reorganizethisCluster(mobility->getExternalId());
            scheduleAt(simTime() + 0.1, triggerAccEvt);
        }
#ifdef DEBUG_ACCIDENTEVT
         std::cout <<"Accident Position: ("<<mobility->getCurrentPosition().x <<"; "<<mobility->getCurrentPosition().y <<mobility->getCurrentPosition().z<<")"<<std::endl;
#endif
    }
    else if (msg == triggerManeuverEvt){
        #ifdef DEBUG_ACCIDENTPACKET
                std::cout <<"---------> triggerManeuverEvt Executed by  " <<findHost()->getFullName() <<std::endl;
                std::cout <<" Obstacle Pos:("<<ObstacleInfo.vehiclePosition.x <<"; "
                        <<ObstacleInfo.vehiclePosition.y <<")"
                        <<" Curr Pos :(" <<traciVehicle->ReturnSumoCurrentposition(mobility->getCurrentPosition()).x <<"; "
                        <<traciVehicle->ReturnSumoCurrentposition(mobility->getCurrentPosition()).y <<")"
                        <<"  My Speed = " << mobility->getSpeed()<<std::endl;
        #endif
        clusterManeuverHandler();
    }
}

void CNPBaseWaveApplLayer::finish() {
    // recordScalar("generatedWSMs",generatedWSMs);
    // recordScalar("receivedWSMs",receivedWSMs);

    // recordScalar("generatedBSMs",generatedBSMs);
    // recordScalar("receivedBSMs",receivedBSMs);

    // recordScalar("generatedWSAs",generatedWSAs);
    // recordScalar("receivedWSAs",receivedWSAs);
    if (strcmp(findHost()->getName(), "node") == 0){
        if (strcmp(mobility->getExternalId().c_str(), ObstacleInfo.vehicleId.c_str()) == 0){

            ofstream res;
            res.open("/home/iotlab_aime/Documents/CAN/caneva/June/LambdaCAN_SUMO_20200215/examples/veins/results/collprobres.csv");
            res << "Cluster_ID: ,"
                       <<"Number_Members: ,"
                       <<"Em_CP: ,"
                       <<"Time_Stamp: ,"
                       <<"Detection_Delay: ,"
                       <<"RighL_CP: ,"
                       <<"LeftL_CP:" <<std::endl;
            std::cout <<" CANStatisticsCollection Size = " <<CANStatisticsCollection.size() <<std::endl;
            for(unsigned int i = 0; i < CANStatisticsCollection.size(); i++){
                res << CANStatisticsCollection[i].cid <<", "
                        << CANStatisticsCollection[i].ncm <<", "
                        << CANStatisticsCollection[i].emcp <<", "
                        <<CANStatisticsCollection[i].recordTime <<", "
                        <<CANStatisticsCollection[i].collDetDelay << ", "
                        << CANStatisticsCollection[i].rightmanlcp << ", "
                        << CANStatisticsCollection[i].leftmanlcp << std::endl;
            }
            res.close();

            ofstream acc;
            acc.open("/home/iotlab_aime/Documents/CAN/caneva/June/LambdaCAN_SUMO_20200215/examples/veins/results/accidentres.csv");
            acc << "Collision_Numbers: ,"
                        <<"Collision_Strength: ," 
                        << "Totat_Vehicles: ,"
                        << numVehsinroad << ", "
                        <<"Overhead: ,"
                        <<overheadRecord<< std::endl;

            for (unsigned int j = 0; j <recordedCollisions.size(); j++){
                   acc << recordedCollisions[j].collNum <<", "
                        << recordedCollisions[j].CollStrength <<std::endl;
            }
            acc.close();
            CANStatisticsCollection.clear();
            recordedCollisions.clear();
            AllClustInfo.clear();
        }
    }
}

CNPBaseWaveApplLayer::~CNPBaseWaveApplLayer() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    unregisteringFromClusters();//BAM Add to remove from Clusters
    // BAM Events ending
    cancelAndDelete(triggerAccEvt);
    cancelAndDelete(triggerManeuverEvt);
    cancelAndDelete(rearVehManEvt);
    cancelAndDelete(updateManeuverPath);
    cancelAndDelete(checkProximitySensorEvt);
    cancelAndDelete(stopvehicle);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void CNPBaseWaveApplLayer::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

//    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, type_CCH);
//    scheduleAt(wsaTime, sendWSAEvt);

}

void CNPBaseWaveApplLayer::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void CNPBaseWaveApplLayer::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void CNPBaseWaveApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void CNPBaseWaveApplLayer::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (dynamic_cast<CCM*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
        DBG_APP << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}
void CNPBaseWaveApplLayer::clusterManeuverHandler(){
    UpdateMyClusterMembers();
    if (isMyClusterInManeuver == true){
        bool continueClusterManeuvers = false;
        for (std::map<std::string, Veins::TraCIMobility*>::iterator it = mycluster.clusterMembMob.begin(); it != mycluster.clusterMembMob.end(); it++){
            if (it->second->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second->getCurrentPosition()).y >  ObstacleInfo.vehiclePosition.y){
                continueClusterManeuvers = true;
                if (it->second->getSpeed() > 0){
                    continueClusterManeuvers = true;
                }
            }
        }
        if (continueClusterManeuvers){
            scheduleAt(simTime() + 0.1, triggerManeuverEvt);
        }
        else{
            if (triggerAccEvt->isScheduled() == true) {
                cancelEvent(triggerAccEvt);
            }
            std::cout <<"VEHICLE <------------> " <<mobility->getExternalId() <<" ENDS MANEUVER PROCESS " <<std::endl;
            #ifdef DEBUG_ACCIDENTPACKET
                std::cout <<"^^^^^^> Stop processing Maneuver! CMs are: " <<findHost()->getFullName() <<std::endl;
            for (std::map<std::string, Veins::TraCIMobility*>::iterator it = mycluster.clusterMembMob.begin(); it != mycluster.clusterMembMob.end(); it++){
                std::cout <<it->first<<" <--> " <<it->second->getExternalId() <<" :  ";
            }
            std::cout <<"CH -->" <<findHost()->getFullName() <<std::endl<<std::endl;
            #endif
        }
    }
    else{
        bool flag1 = false; //In risky status
        bool flag2 = false; //Not yet in risks
        bool flag3 = false; //Member Collided
        double maxManeuverableCP = 0;
        double manTime = 0;
        Veins::TraCIMobility* nextManVMob;
        Veins::TraCIMobility* CollMob;
        //            std::cout <<"====>> VEHICLE " <<mobility->getExternalId() <<"  ABOUT TO START MANEUVER " <<std::endl;
        for (std::map<std::string, Veins::TraCIMobility*>::iterator it = mycluster.clusterMembMob.begin(); it != mycluster.clusterMembMob.end(); it++){
            Coord pos = it->second->getCurrentPosition();
            TraCICoord vpos = it->second->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second->getCurrentPosition());
            if (it->second->getVehicleCommandInterface()->getLaneIndex() == ObstacleInfo.laneID){
                if (vpos.y >= ObstacleInfo.vehiclePosition.y){//Check headind to obstacle
                    cp2obstacle vcp = CP2Obstacle(it->second);
                    std::cout <<"   CH %%%%% " << mobility->getExternalId()<<" --- > CP to " <<it->second->getExternalId()<<", With Speed = " << it->second->getSpeed() <<" ;  =  " 
                    << vcp.cp <<" Obstacle Pos : (" <<ObstacleInfo.vehiclePosition.x <<" ; " <<ObstacleInfo.vehiclePosition.y <<"); My Pos : (" <<vpos.x <<" ;" <<vpos.y <<")"<<std::endl;
                    if (vcp.cp == 1){//Certainly this vehicle collides with an obstacle
                        // it->second->getVehicleCommandInterface()->setSpeed(0); //Leave it till hit the obstacle
                        // updateObstacleInformation();
                        maxManeuverableCP = vcp.cp;
                        collisionRecords tmpc;
                        tmpc.collNum = 1;
                        double vm1 = 2000, vm2 = 2000;//considering vehicle of same mass
                        double Dy = vpos.y - ObstacleInfo.vehiclePosition.y;
                        double vo = it->second->getSpeed();
                        // double t = requiredTimeToCollision(Dy, vo);
                        double t = vcp.ManT;
                        double vc = (-5) * t + vo;
                        tmpc.CollStrength = ComputeEquivalentEnergySpeed(vm1,vm2,ObstacleInfo.vehicleSpeed, vc);
                        myMembersInAccident.insert(std::make_pair(it->first,it->second));
                        recordedCollisions.push_back(tmpc);
                        std::cout <<"________>>> " <<"     Colliding Vehicle  =   " <<it->second->getExternalId() << std::endl;
                        CollMob = it->second;
                        ObstacleInfo.vehiclePosition.y = ObstacleInfo.vehiclePosition.y + 5.0;//5.0 is Vehicle Length
                        // error("Collision");
                        flag1 = true;
                    }
                    else if (maxManeuverableCP == 0){
                        maxManeuverableCP = vcp.cp;
                        nextManVMob = it->second;
                        manTime = vcp.ManT;
                        flag2 = true;
                    }
                    else if(vcp.cp > maxManeuverableCP){
                        std::cout <<"<-----> Current CP  =  " <<vcp.cp <<std::endl;   
                        maxManeuverableCP = vcp.cp;
                        nextManVMob = it->second;
                        manTime = vcp.ManT;
                        flag2 = true;
                    }
                }
            }
            else{
                if (vpos.y >= ObstacleInfo.vehiclePosition.y){//Check headind to obstacle
                    flag3=true;
                }
            }
        }
        if (flag1){
            double closey = 0;
            for (std::map<std::string, Veins::TraCIMobility*>::iterator it = mycluster.clusterMembMob.begin(); it != mycluster.clusterMembMob.end(); it++){
                std::map<std::string, Veins::TraCIMobility*>::iterator it2 = myMembersInAccident.find(it->first);
                if (it2 == myMembersInAccident.end()){
                    TraCICoord vpos = it->second->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second->getCurrentPosition());
                    if (closey == 0){
                        closey = vpos.y;
                        nextManVMob = it->second;
                    }
                    else if(closey > vpos.y){
                        closey = vpos.y;
                        nextManVMob = it->second;
                    }
                }
            }

            CMManeuverPlanningBasedLaneProbability(mycluster, 1, nextManVMob, minIvTThresh);
            AheadCHinEmMan = true;
            isMyClusterInManeuver = true;
            std::cout <<"<----------------- Maneuver Called by   ----------------->"<<findHost()->getFullName() <<std::endl<<std::endl;
        
            scheduleAt(simTime() + 0.1, triggerManeuverEvt);
        }
        else if (flag2){
            std::cout <<"^^^^^^  The most risky vehicle Probability = " <<maxManeuverableCP <<std::endl;
            if (maxManeuverableCP > 0){
                CMManeuverPlanningBasedLaneProbability(mycluster, maxManeuverableCP, nextManVMob, manTime);
                AheadCHinEmMan = true;
                isMyClusterInManeuver = true;
                std::cout <<"<----------------- Maneuver Called by   ----------------->"<<findHost()->getFullName() <<std::endl<<std::endl;
            }
            scheduleAt(simTime() + 0.1, triggerManeuverEvt);
        }
        else if (flag3){
            scheduleAt(simTime() + 0.1, triggerManeuverEvt);
        }
    }
}
std::vector<vehicle> CNPBaseWaveApplLayer::returnNeigbourVehicles(std::vector<NeighborInfo> NeigbourInf){
    std::vector<vehicle> neighbVeh;
 /*   std::cout <<" Neighbor contents:  "<<std::endl;
    for (unsigned int i = 0; i < NeigbourInf.size(); i++){
        std::cout << NeigbourInf[i].vehicle_t.vehicleId << "   ;   ";
    }
    std::cout <<std::endl;*/
    for (unsigned i = 0; i < NeigbourInf.size(); i++){
        if ((NeigbourInf[i].vehicle_t.vehicleId.find("rsu") ==  std::string::npos) &&
                NeigbourInf[i].vehicle_t.vehicleId.find("edg") ==  std::string::npos){ //Only consider vehicles. Remove rsus.
            neighbVeh.push_back(NeigbourInf[i].vehicle_t);
        }
    }

/*    for (unsigned int i = 0; i < neighbVeh.size(); i++){
        std::cout << neighbVeh[i].vehicleId << "   ;   ";
    }
    std::cout <<std::endl;*/

//    error("Stop");

    return neighbVeh;
}
double CNPBaseWaveApplLayer::returnDst(Coord Pos1, Coord Pos2){
    double InterDist = sqrt(pow(Pos2.x - Pos1.x, 2) + pow(Pos2.y - Pos1.y, 2) + pow(Pos2.z - Pos1.z, 2));
    return InterDist;
}
void CNPBaseWaveApplLayer::callForManeuver(double Time2Maneuver){
    std::vector<vehicle> Neighbors = returnNeigbourVehicles(NeighborInformation);
    int ManeuverLaneIndex = returnTheManeuverLane (Neighbors, actualVeh); //(We Determine the Maneuver Lane);
    // int ManeuvTime = int (Time2Maneuver);
    mobility->getVehicleCommandInterface()->changeLane(ManeuverLaneIndex, 1000);
    error("stop");
}
int CNPBaseWaveApplLayer::returnTheManeuverLane(std::vector<vehicle> NeighborVehs, vehicle curVeh){

//    int actLane = curVeh.laneID;
    int ManLane = 0;
    int endLaneRight = 0; //left index
    int endLaneLeft = 3; //Right Index
    int LaneIndex = curVeh.laneID;
    std::cout <<"Current Lane Index = " <<LaneIndex << std::endl;
    if (LaneIndex == endLaneRight){
        ManLane = endLaneRight + 1;
    }
    else if (LaneIndex == endLaneLeft){
        ManLane = endLaneLeft - 1;
    }
    else{
        ManLane = chooseLanebyLaneDrivingKinnematics(NeighborVehs, curVeh);
        if (ManLane == 0){
            ManLane = chooseLanebyDrivingDensity(NeighborVehs,LaneIndex);
        }
    }
    std::cout <<findHost()->getFullName() <<"  ------->  Chosen Maneuver Lane = " << ManLane <<std::endl;
    return ManLane;
}
int CNPBaseWaveApplLayer::chooseLanebyLaneDrivingKinnematics(std::vector<vehicle> NeighborVehs,  vehicle curVehicle){
    int retLane = 0;

    vehicle neighborLeft; int LeftLaneIndex = 0;
    vehicle neighborRight; int RightLaneIndex = 0;
    int LaneIndex = curVehicle.laneID;
    for (unsigned int i = 0; i < NeighborVehs.size(); i++){

        int NeigLaneIndex = NeighborVehs[i].laneID;

        if (NeigLaneIndex == LaneIndex-1){
            if (RightLaneIndex == 0){
                RightLaneIndex = NeigLaneIndex;
            }
            else if (NeighborVehs[i].vehiclePosition.y < neighborRight.vehiclePosition.y){
                neighborRight = NeighborVehs[i];
            }
        }
        else if (NeigLaneIndex == LaneIndex + 1){
            if (LeftLaneIndex == 0){
                LeftLaneIndex =NeigLaneIndex;

            }
            else if  (NeighborVehs[i].vehiclePosition.y < neighborLeft.vehiclePosition.y){
                neighborLeft = NeighborVehs[i];
            }
        }
    }
    if (RightLaneIndex == 0){
        retLane = LaneIndex + 1;
    }
    else if (LeftLaneIndex == 0){
        retLane = LaneIndex - 1;
    }
    else{
        double t2cRight = mp.computeTimeToCollision(curVehicle.vehiclePosition, neighborRight.vehiclePosition, curVehicle.vehicleSpeed, neighborRight.vehicleSpeed);
        double t2cLeft = mp.computeTimeToCollision(curVehicle.vehiclePosition, neighborLeft.vehiclePosition, curVehicle.vehicleSpeed, neighborLeft.vehicleSpeed);
        if (t2cLeft > 1.5){
            retLane = LaneIndex - 1;
        }
        else if (t2cRight > 1.5){
            retLane = LaneIndex + 1;
        }
    }
    return retLane;
}
int CNPBaseWaveApplLayer::chooseLanebyDrivingDensity(std::vector<vehicle> NeighborVehs, int curLane){
    int retLane = 0;
    int numLeft = 0, numRight = 0;

    for (unsigned int i = 0; i < NeighborVehs.size(); i++){
        int LaneIndex = NeighborVehs[i].laneID;

        if (LaneIndex == curLane- 1){
            numLeft ++;
        }
        else if (LaneIndex == curLane + 1){
            numRight ++;
        }
    }
    if (numLeft > numRight){ //Turn towards a lane with little vehicle density
        retLane = curLane + 1;
    }
    else{
        retLane = curLane - 1;
    }

    return retLane;
}
std::string CNPBaseWaveApplLayer::getClusterHeadID (std::map<std::string,Veins::TraCIMobility*> Allvehicles){
    std::string CHID;
    std::map<std::string,Veins::TraCIMobility*>::iterator mit;
    if(Allvehicles.size() == 0){
        error("Empty Cluster!@");
    }
    if (Allvehicles.size() == 1){
        mit = Allvehicles.begin();
        CHID = mit->first;
//        std::cout <<"*** 111 CHID = "  <<CHID<<std::endl;
    }
    else if (Allvehicles.size() == 2){
        double prevY = 0;
        for (mit = Allvehicles.begin(); mit != Allvehicles.end(); mit++){
            Coord vpos = mit->second->getCurrentPosition();
            if (prevY == 0){
                prevY = vpos.y;
                CHID = mit->first;
            }
            else if (prevY > vpos.y){
                CHID = mit->first;
            }
//            std::cout <<"*** 222 CHID = "  <<CHID<<std::endl;
        }
    }
    else{
        CHID = getInterVehiclesAveragedistances(Allvehicles);
//        std::cout <<"*** 333 CHID = "  <<CHID<<std::endl;
    }
    return CHID;
}
std::string CNPBaseWaveApplLayer::getInterVehiclesAveragedistances(std::map<std::string,Veins::TraCIMobility*> Allvehicles){
    std::map <std::string,double> AverageDistances;
//    std::cout<<"Size = " <<Allvehicles.size()<<" and Cluster Members are: " <<std::endl;
    for (std::map<std::string,Veins::TraCIMobility*>::iterator mpit1 = Allvehicles.begin(); mpit1 != Allvehicles.end(); mpit1++){
        Coord curPos = mpit1->second->getCurrentPosition();
//        std::cout <<mpit1->first <<"  , Pos  ("<<curPos.x <<"; "<<curPos.y <<"; " <<curPos.z <<") || ";
        std::vector <double> distToOtherVehs;
        for (std::map<std::string,Veins::TraCIMobility*>::iterator mpit2 = Allvehicles.begin(); mpit2 != Allvehicles.end(); mpit2++){
            if (mpit2->first != mpit1->first){
                Coord NextVehPos = mpit2->second->getCurrentPosition();
                double distance = sqrt(pow(NextVehPos.x - curPos.x, 2) + pow(NextVehPos.y - curPos.y, 2) + pow(NextVehPos.z - curPos.z, 2));
                distToOtherVehs.push_back(distance);
            }
        }
        double Tot = 0;
        double AvrDist = 0;
        for (unsigned int i = 0; i < distToOtherVehs.size(); i++){
            Tot = Tot + distToOtherVehs[i];
        }
        AvrDist = Tot/distToOtherVehs.size();
        AverageDistances.insert(std::make_pair(mpit1->first,AvrDist));
    }
//    std::cout<<std::endl<<std::endl;
    std::string ClustHead = "";
    double smallAvDst = 0;
    for (std::map <std::string,double>::iterator it = AverageDistances.begin(); it != AverageDistances.end(); it++){
        if (smallAvDst == 0){
            smallAvDst = it->second;
            ClustHead = it->first;
        }
        else if (it->second < smallAvDst){
            smallAvDst = it->second;
            ClustHead = it->first;
        }
    }
    return ClustHead;
}
bool CNPBaseWaveApplLayer::checkIsCH(std::string vehID){
    bool flag = false;
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
//        std::cout <<"CH  " <<i <<" is  " <<AllClustInfo[i].cluster_Hid <<"   ||  ";
        if (strcmp(vehID.c_str(), AllClustInfo[i].cluster_Hid.c_str()) == 0){
            flag = true;
            mycluster = AllClustInfo[i];
            break;
        }
    }
//    std::cout <<std::endl;
   return flag;
}
void CNPBaseWaveApplLayer::NeigborsRecording(NeighborInfo recInf){
    bool senderwasInMyNeighbor = false;
    for (unsigned i = 0; i < NeighborInformation.size(); i++){
        if (strcmp(NeighborInformation[i].vehicle_t.vehicleId.c_str(), recInf.vehicle_t.vehicleId.c_str()) == 0){
            NeighborInformation[i] = recInf;
            senderwasInMyNeighbor = true;
        }
    }
    if (senderwasInMyNeighbor == false){
        NeighborInformation.push_back(recInf);
    }
//        Remove from my Neighbors Neighbors which did not update for a time longer than refresh time
    for (unsigned i = 0; i < NeighborInformation.size(); i++){
        if (NeighborInformation[i].ReceptionTime.dbl() + 1.5 < simTime().dbl() ){ //We consider 2 seconds to be the refresh time
            NeighborInformation.erase(NeighborInformation.begin() + i);
        }
    }
}
/*First: Remove Obstacle from the Cluster
 * Second: Choose and set the other CH for the Remaining CMs if Obstacle was CH*/
void CNPBaseWaveApplLayer::reorganizethisCluster(std::string MID){
    Clusterstruct clustVeh;
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        if (strcmp(MID.c_str(), AllClustInfo[i].cluster_Hid.c_str()) == 0){
            for (std::map<std::string, Veins::TraCIMobility*>::iterator iter = AllClustInfo[i].clusterMembMob.begin(); iter != AllClustInfo[i].clusterMembMob.end(); iter++){
                if (strcmp(iter->first.c_str(), MID.c_str()) == 0){
                    AllClustInfo[i].clusterMembMob.erase(iter->first);
                    std::string CH = getClusterHeadID(AllClustInfo[i].clusterMembMob);
                    AllClustInfo[i].cluster_Hid = CH;
                    std::cout <<"reorganizethisCluster : "<<AllClustInfo[i].cluster_id  <<"  ; CHID 1 = " << CH <<std::endl;
//                    error("Newly Chosen CH");
                    break;
                    break;
                }
            }
        }
        else{
            for (std::map<std::string, Veins::TraCIMobility*>::iterator iter = AllClustInfo[i].clusterMembMob.begin(); iter != AllClustInfo[i].clusterMembMob.end(); iter++){
                if (strcmp(iter->first.c_str(), MID.c_str()) == 0){
                    AllClustInfo[i].clusterMembMob.erase(iter);
                    break;
                    break;
                }
            }
        }
    }
}
void CNPBaseWaveApplLayer::CMManeuverPlanning(int lnum, double mt){
    //Categorize by Lanes
    int eml = ObstacleInfo.laneID;
    classifyClusterMembersbyLane(mycluster, lnum);
    std::map<int,  std::vector<Veins::TraCIMobility*>>::iterator it = ClusterMembersbyLane.find(eml);
    if (it != ClusterMembersbyLane.end()){ //Invoke Maneuver process
        if (it->second.size() > 0){
            if (ObstacleInfo.laneID == 0){
                for (unsigned int i = 0; i < it->second.size(); i++){
                    std::cout << " My  Y = " << it->second[i]->getCurrentPosition().y  << "  OBST Y = " <<ObstacleInfo.vehiclePosition.y <<std::endl;
                    TraCICoord mpos =  it->second[i]->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second[i]->getCurrentPosition());

                    if (mpos.y > ObstacleInfo.vehiclePosition.y){
                        it->second[i]->getVehicleCommandInterface()->changeLane(eml+1, 1000);
                    }
                }
            }
            else if(ObstacleInfo.laneID == lnum-1){
//                std::cout <<" <<<<>>>>  ACCIDENT IN :  " << ObstacleInfo.laneID <<"   LANE!" <<std::endl;
                for (unsigned int i = 0; i < it->second.size(); i++){
                    TraCICoord mpos =  it->second[i]->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second[i]->getCurrentPosition());
                    if (mpos.y > ObstacleInfo.vehiclePosition.y){
//                        std::cout <<"2222222222222 Maneuvered Vehicle " <<it->second[i]->getExternalId()<< "  Towards Lane  :  " << eml-1 <<std::endl;
                        it->second[i]->getVehicleCommandInterface()->changeLane(eml-1, 1000);
                    }
                }
            }
            else{
                int manL = eml + 1;
//                std::cout <<" <<<<>>>>  ACCIDENT IN :  " << ObstacleInfo.laneID <<"   LANE!" <<std::endl;
                for (unsigned int i = 0; i < it->second.size(); i++){
                    TraCICoord mpos =  it->second[i]->getVehicleCommandInterface()->ReturnSumoCurrentposition(it->second[i]->getCurrentPosition());
                    if (mpos.y > ObstacleInfo.vehiclePosition.y){
                        it->second[i]->getVehicleCommandInterface()->changeLane(manL, 1000);
//                        std::cout <<"3333333333 Maneuvered Vehicle " <<it->second[i]->getExternalId()<< "  Towards Lane  :  " << manL <<std::endl;
                        if (manL == eml+1)
                            manL = eml-1;
                        else
                            manL = eml+1;
                    }
                }
            }
        }
        else{
            error("No vehicles in Accident Lane!");
        }
    }
    else{//Verify no member heading towards the Obstacle

    }
}
void CNPBaseWaveApplLayer::CMManeuverPlanningBasedLaneProbability (Clusterstruct CMembers, double CP2Obst, TraCIMobility*  emvmob, double ManTime){
    classifyClusterMembersbyLane(CMembers, 3);
    std::map<int,  std::vector<Veins::TraCIMobility*>>::iterator it = ClusterMembersbyLane.find(ObstacleInfo.laneID);
    std::map<int,double> NeighbLaneCP = CPManeuverLanes(CMembers, CP2Obst, emvmob, ManTime);
    CANDataCollection tmpdata;
    tmpdata.cid = CMembers.cluster_id;
    tmpdata.emcp = CP2Obst;
    tmpdata.recordTime = simTime().dbl();
    tmpdata.ncm = CMembers.clusterMembMob.size();
    if (it != ClusterMembersbyLane.end()){

        std::cout <<"!!!!! Number of possible maneuver Lane = " <<NeighbLaneCP.size() <<std::endl;
        int nem = it->second.size();
        if (NeighbLaneCP.size() == 1){
            int ManLane;
            double tmpcp = 0;
            for (auto j:NeighbLaneCP){
                if (tmpcp == 0){
                    ManLane = j.first;
                    tmpcp = j.second;
                }
                else if(tmpcp > j.second){
                    ManLane = j.first;
                    tmpcp = j.second;
                }
            }
            for (auto i:it->second){
//                i->getVehicleCommandInterface()->changeLane(ManLane, ManTime);
                i->getVehicleCommandInterface()->changeLane(ManLane, 2000);
            }
            if (ManLane > ObstacleInfo.laneID){
                tmpdata.leftmanlcp = tmpcp;
                tmpdata.rightmanlcp = 0;
            }
            else{
                tmpdata.leftmanlcp = 0;
                tmpdata.rightmanlcp = tmpcp;
            }

        }
        else{
            bool NonManProbFlag = false;
            if (NonManProbFlag){
                for (auto i:it->second){
                    int curlane = i->getVehicleCommandInterface()->getLaneIndex();
                    if (glManLane < curlane){
                        i->getVehicleCommandInterface()->changeLane(glManLane, 2000);
                        std::cout <<" .........> Change Lane (VID): " << i->getExternalId() << "LID = " <<
                                glManLane<< std::endl;
                        glManLane = curlane+1;
                    }
                    else{
                        i->getVehicleCommandInterface()->changeLane(glManLane, 2000);
                        std::cout <<" ----------> Change Lane (VID): " << i->getExternalId() << "LID = " <<
                                glManLane << std::endl;
                        glManLane = curlane - 1;     
                    }
                }
            }
            else{
                double cpleft = 0 ;
                double cpright = 0;
                int l = ObstacleInfo.laneID + 1;
                int r = ObstacleInfo.laneID - 1;
                std::map<int,double>::iterator t1 = NeighbLaneCP.find(l);
                if (t1 != NeighbLaneCP.end()){
                    cpleft = t1->second;
                }
                std::map<int,double>::iterator t2 = NeighbLaneCP.find(r);
                if (t2 != NeighbLaneCP.end()){
                    cpright = t2->second;
                }
                tmpdata.leftmanlcp = cpleft;
                tmpdata.rightmanlcp = cpright;
                int nl = round(nem * cpright)/(cpright + cpleft);
                int nr = round(nem * cpleft)/(cpright + cpleft);
                int nexl = l;
                int lc = 0, rc = 0 ;
                for (auto i:it->second){
                    std::map<std::string, Veins::TraCIMobility*>::iterator it2;
                    it2 = myMembersInAccident.find(i->getExternalId());
                    if (it2 == myMembersInAccident.end()){
                        if (ObstacleInfo.laneID == 0){
                            i->getVehicleCommandInterface()->changeLane(ObstacleInfo.laneID+1, 2000);
                        }
                        else if (ObstacleInfo.laneID == 2){
                            i->getVehicleCommandInterface()->changeLane(ObstacleInfo.laneID-1, 2000);
                        }
                        else{
                            if (nexl == l && lc < nl){
            //                    i->getVehicleCommandInterface()->changeLane(nexl, ManTime);
                                i->getVehicleCommandInterface()->changeLane(nexl, 2000);
                                std::cout <<"<<+++++++++>> vehicle : " << i->getExternalId() << "changes Lane towards : " << nexl << std::endl;
                                nexl = r;
                                rc++;
                            }
                            else if(rc < nr){
            //                    i->getVehicleCommandInterface()->changeLane(nexl, ManTime);
                                i->getVehicleCommandInterface()->changeLane(nexl, 2000);
                                std::cout <<"<<---------->> vehicle : " << i->getExternalId() << "changes Lane towards : " << nexl << std::endl;
                                nexl = l;
                                lc++;
                            }
                            else{
            //                    i->getVehicleCommandInterface()->changeLane(nexl, ManTime);
                                i->getVehicleCommandInterface()->changeLane(nexl, 2000);
                                std::cout <<"<<xxxxxxxxxxx>> vehicle : " <<i->getExternalId() << "changes Lane towards : " << nexl << std::endl;
                                if (nexl == l){
                                    nexl = r;
                                }
                                else{
                                    nexl = l;
                                }
                            }
                        }






                    }
                }
            }
        }
    }
    if (InjTime < 60){
        tmpdata.collDetDelay = CollDetTime - 60;
    }
    else{
        tmpdata.collDetDelay = CollDetTime - InjTime;
    }

    CANStatisticsCollection.push_back(tmpdata);
    std::cout << "+++++++ CANStatisticsCollection Current Size = " <<CANStatisticsCollection.size() <<std::endl;
}
void CNPBaseWaveApplLayer::updateObstacleInformation(){
    ObstacleInfo.vehiclePosition.y = ObstacleInfo.vehiclePosition.y + 3.8; //add vehicle length (3.8)
}
int CNPBaseWaveApplLayer::returnManLane(std::map<int,double> NeighCP){
    int ManLane;
    double tmpcp = 0;
    for (auto j:NeighCP){
        if (tmpcp == 0){
            ManLane = j.first;
            tmpcp = j.second;
        }
        else if(tmpcp > j.second){
            ManLane = j.first;
            tmpcp = j.second;
        }
    }
    std::cout <<" Only One Neigbor Lane = " <<ManLane <<std::endl;
    return ManLane;
}
int CNPBaseWaveApplLayer::returnCID(std::string vehID){
    int CID = 0;
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        for (std::map<std::string, Veins::TraCIMobility*>::iterator iter = AllClustInfo[i].clusterMembMob.begin(); iter != AllClustInfo[i].clusterMembMob.end(); iter++){
            if (strcmp(iter->first.c_str(), vehID.c_str()) == 0){
                CID = AllClustInfo[i].cluster_id;
                break;
                break;
            }
        }
    }
    return CID;
}
void CNPBaseWaveApplLayer::classifyClusterMembersbyLane(Clusterstruct clust, int nl){
//    std::cout <<"--------- my Cluster ID = " <<clust.cluster_id << "  And Members are: "<<std::endl;
    if (ClusterMembersbyLane.empty() == false){
        ClusterMembersbyLane.clear();
    }
    for (int i =0; i <nl; i++){
       std::vector<Veins::TraCIMobility*> tmpv;
       ClusterMembersbyLane.insert(std::make_pair(i, tmpv));
    }
    for (std::map<std::string, Veins::TraCIMobility*>::iterator j = clust.clusterMembMob.begin(); j != clust.clusterMembMob.end(); j++){
        int tmpLane = j->second->getVehicleCommandInterface()->getLaneIndex();
        std::map<int,  std::vector<Veins::TraCIMobility*>>::iterator mit;
        mit = ClusterMembersbyLane.find(tmpLane);
        if (mit != ClusterMembersbyLane.end()){
            mit->second.push_back(j->second);
        }
        else{
//            std::vector<Veins::TraCIMobility*> tmpv;
//            tmpv.push_back(j->second);
//            ClusterMembersbyLane.insert(std::make_pair(tmpLane, tmpv));
            error ("Unkown Lane Assigned");
        }
    }
}
void CNPBaseWaveApplLayer::UpdateMyClusterMembers(){//Remove from Clusters vehicles that exited from Road
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        if(AllClustInfo[i].cluster_id == mycluster.cluster_id){
            mycluster = AllClustInfo[i];
        }
    }
}
void CNPBaseWaveApplLayer::unregisteringFromClusters(){
#ifdef DEBUG_CLUSTMEMBEREXIT
    std::cout <<"/////////// unregisteringFromClusters() called by Vehicle : " <<mobility->getExternalId() <<std::endl;
    for(auto j:AllClustInfo){
        std::cout<<std::endl<<" <----> Cluster : " <<j.cluster_id <<" Members: " ;
        for (auto k:j.clusterMembMob){
            std::cout <<k.first << "  | ";
        }
        std::cout<<endl;
    }
#endif
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        if (AllClustInfo[i].cluster_id == mycluster.cluster_id){
//            std::cout<<std::endl <<"VVVVVVVVV condition Satisfied" <<std::endl;
            std::map<std::string, Veins::TraCIMobility*>::iterator it;

            for (it = AllClustInfo[i].clusterMembMob.begin(); it != AllClustInfo[i].clusterMembMob.end(); it++){
                if (strcmp(it->first.c_str(), mobility->getExternalId().c_str()) == 0){
                    std::cout <<" Remove Vehicle " <<mobility->getExternalId().c_str() <<std::endl;
                    AllClustInfo[i].clusterMembMob.erase(mobility->getExternalId().c_str());
                }
            }
        }
    }

#ifdef DEBUG_CLUSTMEMBEREXIT
    std::cout <<" After " <<mobility->getExternalId() <<"  removal" <<std::endl;
    for(auto j:AllClustInfo){
        std::cout<<std::endl<<" <----> Cluster : " <<j.cluster_id <<" Members: " ;
        for (auto k:j.clusterMembMob){
            std::cout <<k.first << "  | ";
        }
        std::cout<<endl;
    }
#endif
}

cp2obstacle CNPBaseWaveApplLayer::CP2Obstacle(TraCIMobility* vehMob){
    vehicle membveh;
    cp2obstacle vcp;
    membveh.vehiclePosition.x = vehMob->getVehicleCommandInterface()->ReturnSumoCurrentposition(vehMob->getCurrentPosition()).x;
    membveh.vehiclePosition.y = vehMob->getVehicleCommandInterface()->ReturnSumoCurrentposition(vehMob->getCurrentPosition()).y;
    membveh.vehiclePosition.z = vehMob->getCurrentPosition().z;
    membveh.vehicleSpeed = vehMob->getSpeed();
    double t2c = mp.computeTimeToCollision(membveh.vehiclePosition, ObstacleInfo.vehiclePosition, membveh.vehicleSpeed, ObstacleInfo.vehicleSpeed);
    
    std::cout << "-->  T2C = " <<t2c <<"  ";
    if (t2c >= maxT2CThresh){
        // std::cout <<" !!!!! Higher than Max T2C  =  " << t2c << std::endl;
        vcp.ManT = INFINITY;
        vcp.cp = 0;
    }
    else if (t2c <= minT2CThresh){
        // std::cout <<" ?????????? under minimal T2C  =  " << t2c << std::endl;
        vcp.ManT = t2c; //use it to compute the speed during collision
        vcp.cp = 1;
    }
    else{
        vcp.ManT = t2c;
        vcp.cp = (maxT2CThresh - t2c)/(maxT2CThresh-minT2CThresh);
    }
    std::cout<<std::endl;
    return vcp;
}
std::map<int,double>  CNPBaseWaveApplLayer::CPManeuverLanes(Clusterstruct CMs, double CP2Obst, TraCIMobility*  emvmob, double MT){
    std::map <int,double> LanesCP;
    std::vector<TraCIMobility*> RightLaneVehs;
    std::vector<TraCIMobility*> LeftLaneVehs;
    bool accInMostLeftLane = false;
    bool accInMostRightLane = false;
    vehicle emv;
    TraCICoord tpos = emvmob->getVehicleCommandInterface()->ReturnSumoCurrentposition(emvmob->getCurrentPosition());
    emv.vehiclePosition.x = tpos.x;
    emv.vehiclePosition.y = tpos.y;
    emv.vehiclePosition.z = 0;
    emv.vehicleSpeed = emvmob->getSpeed();
    emv.laneID = emvmob->getVehicleCommandInterface()->getLaneIndex();
//    std::cout <<" <<<<+++++++++++>>>>>> mycluster number of Members: " <<mycluster.clusterMembMob.size() <<std::endl;
    for (auto i:mycluster.clusterMembMob){
        if(strcmp(emvmob->getExternalId().c_str(), i.second->getExternalId().c_str()) != 0){
            if (ObstacleInfo.laneID == 0){
                if (i.second->getVehicleCommandInterface()->getLaneIndex() == ObstacleInfo.laneID + 1){
                    LeftLaneVehs.push_back(i.second);
                    accInMostLeftLane = true;
                }
            }
            else if (ObstacleInfo.laneID == 2){
                if (i.second->getVehicleCommandInterface()->getLaneIndex() == ObstacleInfo.laneID - 1){
                    RightLaneVehs.push_back(i.second);
                    accInMostRightLane = true;
                }
            }
            else{
                if (i.second->getVehicleCommandInterface()->getLaneIndex() == ObstacleInfo.laneID + 1){
                    LeftLaneVehs.push_back(i.second);
                }
                else if (i.second->getVehicleCommandInterface()->getLaneIndex() == ObstacleInfo.laneID - 1){
                    RightLaneVehs.push_back(i.second);
                }
            }
        }
    }

//    Initialize the Contour info to the Obstacle's
//    std::cout <<" >>> Emergency Vehicle  :   " <<emvmob->getExternalId() <<" About to Compute Its Convex Hull !! " << std::endl;
    std::map<double,std::vector<Coord>> PrevCont = mp.compute_contour_polygon(emv, MT);
    std::vector<Coord> PrevContPolygon;
    for (auto j:PrevCont){
        int n = j.second.size()-1;
        Coord pos = j.second[n];
        PrevContPolygon.push_back(pos);
    }
    vector<Coord> emvcvx = mp.getContourConvex(PrevContPolygon); //Emergence vehicle convex Hull

//    Compute Contours info Lane by Lane
    std::cout <<" >>>  Left Lane vehicles = " <<LeftLaneVehs.size()<<std::endl;
    std::cout <<">>> Right Lane vehicles = "  <<RightLaneVehs.size() <<std::endl;
    if (RightLaneVehs.size() > 0){
        std::sort(RightLaneVehs.begin(), RightLaneVehs.end(),sortLaneVehs);
        double LaneCP = 0;
        for (auto j:RightLaneVehs){
            vehicle tmpveh;
            tmpveh.laneID=j->getVehicleCommandInterface()->getLaneIndex();
            tmpveh.vehicleSpeed = j->getSpeed();
            TraCICoord tmp1 = j->getVehicleCommandInterface()->ReturnSumoCurrentposition(j->getCurrentPosition());
            tmpveh.vehiclePosition.x = tmp1.x;
            tmpveh.vehiclePosition.y = tmp1.y;
            tmpveh.vehiclePosition.z = 0;
            std::cout <<" >>> Vehicle  :   " <<j->getExternalId() <<" About to Compute Its Convex Hull !! " << std::endl;
            std::map<double,std::vector<Coord>> Cont = mp.compute_contour_polygon(tmpveh, MT);
            std::vector<Coord> ContPolygon1;
            for (std::map<double,std::vector<Coord>>::iterator it = Cont.begin(); it!= Cont.end(); it++){
                int ind = it->second.size()-1;
                Coord pos1 = it->second[ind];
                ContPolygon1.push_back(pos1);
            }
            vector<Coord> cvx1 = mp.getContourConvex(ContPolygon1);
            std::cout <<"------>cvx1 ----------> = " <<cvx1.size() <<std::endl;

//            Intersection and Probabilities Calculations
            vector<Coord> inter1 = mp.returnConvexesIntersection(emvcvx, cvx1);
            std::cout <<"------> 11111 : inter1 size = " <<inter1.size() <<std::endl;
            vector<Coord> inter2 = mp.returnConvexesIntersection(emvcvx, cvx1);
            std::cout <<"------> 22222 : inter2 size = " <<inter2.size() <<std::endl;
            vector<Coord> convexIntes = inter1;
            convexIntes.insert(convexIntes.end(), inter2.begin(), inter2.end());
            if (convexIntes.size() > 2){
                double frontArea = mp.returnConvexArea(emvcvx);
                double intersArea = mp.returnConvexArea(convexIntes);
                LaneCP = LaneCP * (intersArea/frontArea);
            }
            else{
                std::cout <<">>>  No Convex Intersection Available!" <<std::endl;
                break;
            }
        }
        LanesCP.insert(std::make_pair(ObstacleInfo.laneID - 1, LaneCP));
    }
    if (LeftLaneVehs.size() > 0){
        std::sort(LeftLaneVehs.begin(), LeftLaneVehs.end(),sortLaneVehs);
        double LaneCP = 0;
        for (auto j:LeftLaneVehs){
            vehicle tmpveh;
            tmpveh.laneID=j->getVehicleCommandInterface()->getLaneIndex();
            tmpveh.vehicleSpeed = j->getSpeed();
            TraCICoord tmp1 = j->getVehicleCommandInterface()->ReturnSumoCurrentposition(j->getCurrentPosition());
            tmpveh.vehiclePosition.x = tmp1.x;
            tmpveh.vehiclePosition.y = tmp1.y;
            tmpveh.vehiclePosition.z = 0;
            std::cout <<" >>> Vehicle  :   " <<j->getExternalId() <<" About to Compute Its Convex Hull !! " << std::endl;
            std::map<double,std::vector<Coord>> Cont = mp.compute_contour_polygon(tmpveh, MT);
            std::vector<Coord> ContPolygon2;
            for (auto j:Cont){
                int n = j.second.size()-1;
                Coord pos2 = j.second[n];
                ContPolygon2.push_back(pos2);
            }
            vector<Coord> cvx2= mp.getContourConvex(ContPolygon2);
            std::cout <<"------>cvx2 ----------> = " <<cvx2.size() <<std::endl;


//            Intersection and Probabilities Calculations
            vector<Coord> inter1 = mp.returnConvexesIntersection(emvcvx, cvx2);
            std::cout <<"------> 33333 : inter1 size = " <<inter1.size() <<std::endl;
            vector<Coord> inter2 = mp.returnConvexesIntersection(emvcvx, cvx2);
            std::cout <<"------> 44444 : inter2 size = " <<inter2.size() <<std::endl;
            vector<Coord> convexIntes = inter1;
            convexIntes.insert(convexIntes.end(), inter2.begin(), inter2.end());
            if (convexIntes.size() > 2){
                double frontArea = mp.returnConvexArea(emvcvx);
                double intersArea = mp.returnConvexArea(convexIntes);
                LaneCP = LaneCP * (intersArea/frontArea);
            }
            else{
                std::cout <<">>>  No Convex Intersection Available!" <<std::endl;
                break;
            }
        }
        LanesCP.insert(std::make_pair(ObstacleInfo.laneID + 1, LaneCP));
    }
    return LanesCP;
}
double CNPBaseWaveApplLayer::ComputeEquivalentEnergySpeed(double m1, double m2, double v1, double v2){
    double Eqv = (v1+v2)/2;
    double EES = v2-Eqv;
    // double EES = (2 * m2)(v2-Eqv)/(m1+m2);
    return EES;
}
double CNPBaseWaveApplLayer::requiredTimeToCollision(double dx, double vo){
    double dec = -5;
    double di = (vo*vo) -4*(dec * (-1)*dx);
    double t1 = ((-vo)-sqrt(di))/(2*dec); 
    double t2 = ((-vo)-sqrt(di))/(2*dec);
    if (t2 > 0){
        return t2;
    }
    else{
        return t1;
    }
}
