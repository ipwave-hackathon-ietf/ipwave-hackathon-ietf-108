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
        std::cout <<" <^^^^^^^^^^^^^^^^^^^>  BasewaveAppLayer.cc is running" <<std::endl;
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
        sendBeacons = par("sendCCM").boolValue();
        beaconLengthBits = par("CCMLengthBits").longValue();
        beaconUserPriority = par("CCMUserPriority").longValue();
        beaconInterval =  par("CCMInterval");

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;

        isParked = false;

        clustFormPeriod = par("singleClusterFormationPeriod").doubleValue();


        ckeckLaneChange = par("tesVeh").boolValue();

        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendCCMEvt              = new cMessage("beacon evt", CCM_EVT);
        sendWSAEvt              = new cMessage("wsa evt", SEND_WSA_EVT);
        triggerAccEvt           = new cMessage("Accident Trigger",ECM_EVT);
        rearVehManEvt           = new cMessage("Rear vehicles Maneuver");
        updateManeuverPath      = new cMessage("Maneuver Path Update");
        checkProximitySensorEvt = new cMessage("Proximity Sensing");

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

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
                scheduleAt(firstBeacon, sendCCMEvt);
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
    TraCICoord s;
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


    if (CooperativeContMsg* bsm = dynamic_cast<CooperativeContMsg*>(wsm) ) {
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
    else if (EmergentContextMsg* acc = dynamic_cast<EmergentContextMsg*>(wsm)){
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

    delete(msg);
}

void CNPBaseWaveApplLayer::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case CCM_EVT: {
        CooperativeContMsg* bsm = new CooperativeContMsg();
        populateWSM(bsm);
        sendDown(bsm);
        std::cout <<"CCM sent by " <<findHost()->getFullName() <<std::endl;
        if (!sendCCMEvt->isScheduled()){
            scheduleAt(simTime() + beaconInterval * 0.3, sendCCMEvt);
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
    case ECM_EVT:{

//        std::cout <<" Obstacle Lane = " << lo << " Lane Index = " << li << std::endl;
        overheadRecord ++;
        EmergentContextMsg* ecm = new EmergentContextMsg();
        populateWSM(ecm);
        sendDown(ecm);

        std::cout <<" ECM msg sent by "<< findHost()->getFullName() <<std::endl;

#ifdef DEBUG_ACCIDENTPACKET
        Coord CurPos = mobility->getCurrentPosition();
        std::cout << "CUR POS  = (" << CurPos.x <<"; " << CurPos.y <<"; " << CurPos.z <<")" << std::endl;
        std::cout <<"---> Acc MSG SENT !" <<std::endl;
#endif
        scheduleAt(simTime() + 0.3, triggerAccEvt);
        break;
    }
    default: {
        if (msg)
            DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void CNPBaseWaveApplLayer::finish() {

}

CNPBaseWaveApplLayer::~CNPBaseWaveApplLayer() {
    cancelAndDelete(sendCCMEvt);
    cancelAndDelete(sendWSAEvt);
    // BAM Events ending
    cancelAndDelete(triggerAccEvt);
    cancelAndDelete(rearVehManEvt);
    cancelAndDelete(updateManeuverPath);
    cancelAndDelete(checkProximitySensorEvt);
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

    if (dynamic_cast<CooperativeContMsg*>(msg)) {
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

double CNPBaseWaveApplLayer::returnDst(Coord Pos1, Coord Pos2){
    double InterDist = sqrt(pow(Pos2.x - Pos1.x, 2) + pow(Pos2.y - Pos1.y, 2) + pow(Pos2.z - Pos1.z, 2));
    return InterDist;
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
