//
// Copyright (C) 2016 David Eckhoff <eckhoff@cs.fau.de>
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

#ifndef BASEWAVEAPPLLAYER_H_
#define BASEWAVEAPPLLAYER_H_

#include <map>
#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/messages/WaveServiceAdvertisement_m.h"
#include "veins/modules/messages/CCM_m.h"
#include "veins/modules/messages/ECM_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/MCTA/MCTAmotionPlanning.h"
#include "veins/modules/mobility/traci/TraCICoord.h" //BAM
#include <algorithm>

//BAM
#include "inet/common/INETDefs.h"
#include "inet/applications/base/ApplicationBase.h"
//#include "inet/transportlayer/contract/udp/UdpSocket.h"

//BAM

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;
using Veins::TraCICoord;//BAM

//#define DBG_APP std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

#ifndef DBG_APP
#define DBG_APP EV
#endif

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */


struct NeighborInfo{
    vehicle vehicle_t;
    simtime_t ReceptionTime;
};
struct RemoteSensedVehicles{
    std::string vehID;
    simtime_t sensTime;
    double Dst;
    std::vector <double> vehPos;
};
struct Clusterstruct{
    int cluster_id;
    std::string cluster_Hid;
    int cluster_HMac;
    std::map<std::string, Veins::TraCIMobility*> clusterMembMob; //Member Vehicle Id and its corresponding mobility
};
struct cp2obstacle{
    double ManT;
    double cp;
};
struct CANDataCollection{
    int cid;
    int ncm;
    double emcp;
    double recordTime;
    double rightmanlcp;
    double leftmanlcp;
    double collDetDelay;
};
struct collisionRecords{
    int collNum = 0;
    double CollStrength = 0;
};


class CNPBaseWaveApplLayer : public BaseApplLayer{

    public:
        ~CNPBaseWaveApplLayer();
        virtual void initialize(int stage);
        virtual void finish();

        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
//
//        enum WaveApplMessageKinds {
//          SEND_BEACON_EVT,
//          SEND_WSA_EVT,
//        };
        enum WaveApplMessageKinds {
            SEND_WSA_EVT,
            ECM_EVT,
            CCM_EVT
        };

    protected:

        static const simsignalwrap_t mobilityStateChangedSignal;
        static const simsignalwrap_t parkingStateChangedSignal;

        /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
        virtual void handleLowerMsg(cMessage* msg);

        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);

        /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
        virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

        /** @brief this function is called upon receiving a WaveShortMessage */
        virtual void onWSM(WaveShortMessage* wsm) { };

        /** @brief this function is called upon receiving a BasicSafetyMessage, also referred to as a beacon  */
        virtual void onBSM(CooperativeContMsg* bsm) { };

        /** @brief this function is called upon receiving a WaveServiceAdvertisement */
        virtual void onWSA(WaveServiceAdvertisment* wsa) { };

        /** @brief this function is called every time the vehicle receives a position update signal */
        virtual void handlePositionUpdate(cObject* obj);

        /** @brief this function is called every time the vehicle parks or starts moving again */
        virtual void handleParkingUpdate(cObject* obj);

        /** @brief This will start the periodic advertising of the new service on the CCH
         *
         *  @param channel the channel on which the service is provided
         *  @param serviceId a service ID to be used with the service
         *  @param serviceDescription a literal description of the service
         */
        virtual void startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription);

        /** @brief stopping the service and advertising for it */
        virtual void stopService();

        /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
         *
         * @param interval the interval length of the periodic message
         * @param chantype the type of channel, either type_CCH or type_SCH
         */
        virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, t_channel chantype);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         */
        virtual void sendDown(cMessage* msg);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         * @param delay the delay for the message
         */
        virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

        /**
         * @brief helper function for error handling and stats recording purposes
         *
         * @param msg the message to be checked and tracked
         */
        virtual void checkAndTrackPacket(cMessage* msg);

    protected:

        /* pointers ill be set when used with TraCIMobility */
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        AnnotationManager* annotations;
        WaveAppToMac1609_4Interface* mac;

        /* support for parking currently only works with TraCI */
        bool isParked;
        bool communicateWhileParked;

        /* BSM (beacon) settings */
        uint32_t beaconLengthBits;
        uint32_t  beaconUserPriority;
        simtime_t beaconInterval;
        bool sendBeacons;

        /* WSM (data) settings */
        uint32_t  dataLengthBits;
        uint32_t  dataUserPriority;
        bool dataOnSch;

        /* WSA settings */
        int currentOfferedServiceId;
        std::string currentServiceDescription;
        Channels::ChannelNumber currentServiceChannel;
        simtime_t wsaInterval;

        /* state of the vehicle */
        Coord curPosition;
        Coord curSpeed;
        int myId;
        int mySCH;

//        BAM
        double maxVehAccel;

        double maxIvTThresh = 0;
        double minIvTThresh = 0;
        double maxT2CThresh = 0;
        double minT2CThresh = 0;
        double accidentTime = 0;
        double clustFormPeriod = 0;
        double vSpeed;


        std::vector<RemoteSensedVehicles> SensedVehicles;
        std::vector<NeighborInfo> NeighborInformation;
        std::string LastRecvAccPktSenderID = "";
        motionPlanning mp;
        vehicle actualVeh;
        void callForManeuver(double Time2Maneuver);
        std::vector<vehicle> returnNeigbourVehicles(std::vector<NeighborInfo> NeigbourInf);
        double returnDst(Coord Pos1, Coord Pos2);
        double AccidentDetectionDst = 0;
        Coord obstPosition;
        double tmaneuver = 0; //Time To maneuver The Current Vehicle
        int returnTheManeuverLane(std::vector<vehicle> NeighborVehs, vehicle curVehicle);
        int chooseLanebyLaneDrivingKinnematics(std::vector<vehicle> NeighborVehs,  vehicle curVehicle);
        int chooseLanebyDrivingDensity(std::vector<vehicle> NeighborVehs, int actualLane);
        void NeigborsRecording(NeighborInfo recInf);
        bool ckeckLaneChange = false;
        void clusterManeuverHandler();
//        BAM

        /* stats */
        uint32_t generatedWSMs;
        uint32_t generatedWSAs;
        uint32_t generatedBSMs;
        uint32_t receivedWSMs;
        uint32_t receivedWSAs;
        uint32_t receivedBSMs;

        /* messages for periodic events such as beacon and WSA transmissions */
        cMessage* sendCCMEvt;
        cMessage* sendWSAEvt;

        cMessage* stopvehicle;
        cMessage* triggerAccEvt;
        cMessage* triggerManeuverEvt;
        cMessage* updateManeuverPath;
        cMessage* rearVehManEvt;
        cMessage* checkProximitySensorEvt;
        std::string getClusterHeadID (std::map<std::string,Veins::TraCIMobility*> Allvehicles);
        std::string getInterVehiclesAveragedistances(std::map<std::string,Veins::TraCIMobility*> Allvehicles);
        void reorganizethisCluster(std::string MemberID);
        bool checkIsCH(std::string vehID);
        bool ManeuverProcessEnabled = false;
//        bool checkIsthereAClusterMemberHeadingToOstaclePosition(vehicle Obstacle, int ClustID);
        int returnCID(std::string vehID);
        Clusterstruct mycluster;
        std::map<std::string, Veins::TraCIMobility*> myMembersInAccident; 
        bool isMyClusterInManeuver = false;
        double ExpTimeToCollision = 0;
        double InjTime = 0;
        double CollDetTime = 0;
        std::map<int,  std::vector<Veins::TraCIMobility*>> ClusterMembersbyLane;
        void classifyClusterMembersbyLane(Clusterstruct clust, int numLanes);
        void UpdateMyClusterMembers();
        void unregisteringFromClusters();
        cp2obstacle CP2Obstacle(TraCIMobility* tmob);
        void CMManeuverPlanning(int ManLanes, double ManT);
        void CMManeuverPlanningBasedLaneProbability (Clusterstruct CMembers, double CP2Obst, TraCIMobility*  emvmob, double ManTime);
        void updateObstacleInformation();
        double ComputeEquivalentEnergySpeed(double m1, double m2, double v1, double v2);
        int returnManLane(std::map<int,double> NeighCP);
        std::map<int,double> CPManeuverLanes(Clusterstruct CMembers, double CP2Obst, TraCIMobility*  emvmob, double ManTime);
        double requiredTimeToCollision(double dx, double vo);
};
#endif /* BASEWAVEAPPLLAYER_H_ */
