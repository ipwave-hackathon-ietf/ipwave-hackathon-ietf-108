//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
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

#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include <iterator>

#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/messages/PhyControlMessage_m.h"
#include "veins/modules/messages/AckTimeOutMessage_m.h"
#include "veins/modules/messages/WaveShortMessage_m.h"

//XIANG ZHONG
//Interface Table
#include <stdio.h>
#include <string.h>
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/queue/IPassiveQueue.h"
#include "inet/common/Simsignals.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/common/UserPriorityTag_m.h"
#include "inet/networklayer/icmpv6/Icmpv6Header_m.h"
#include "inet/networklayer/ipv6/Ipv6Header_m.h"
#include "inet/networklayer/common/InterfaceEntry.h"
#include "inet/common/IInterfaceRegistrationListener.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include "inet/common/packet/tag/TagSet.h"
#include "veins/modules/messages/WSM_ipwave_m.h"
/*---------------END-------------------*/



using std::unique_ptr;
using omnetpp::simtime_t;
using omnetpp::simTime;

#if OMNETPP_VERSION >= 0x500
#define OWNER owner->
#else
#define OWNER
#endif

#define DBG_MAC EV
//#define DBG_MAC std::cerr << "[" << simTime().raw() << "] " << myId << " "
//BAM
vehicle ObstacleInfo;
TraCIMobility* obMob;
std::vector<Clusterstruct> AllClustInfo;
static int clusterValue = 1; //This value will keep increasing the number of cluster increases
double nextClusterFormationStartTime = 0;
bool isFirstClustVeh = false;
double clusterFormationStart = 0;
bool AheadCHinEmMan = false;
double AheadManeuverStartTime = 0;
int glManLane = 0;
bool sortLaneVehs(const TraCIMobility* Mob1, const TraCIMobility* Mob2){
    return (Mob1->getCurrentPosition().y <= Mob2->getCurrentPosition().y);
}
//BAM


Define_Module(Mac1609_4);

void Mac1609_4::initialize(int stage) {

	BaseMacLayer::initialize(stage);

	if (stage == 0) {
	    std::cout <<"Mac1609_4.cc Initialization! "<<std::endl;
		phy11p = FindModule<Mac80211pToPhy11pInterface*>::findSubModule(
		             getParentModule());
		assert(phy11p);
		//this is required to circumvent double precision issues with constants from CONST80211p.h
		assert(simTime().getScaleExp() == -12);

		sigChannelBusy = registerSignal("sigChannelBusy");
		sigCollision = registerSignal("sigCollision");

		txPower = par("txPower").doubleValue();
		bitrate = par("bitrate").longValue();
		n_dbps = 0;
		setParametersForBitrate(bitrate);

		// unicast parameters
		dot11RTSThreshold = par("dot11RTSThreshold");
		dot11ShortRetryLimit = par("dot11ShortRetryLimit");
		dot11LongRetryLimit = par("dot11LongRetryLimit");
		ackLength = par("ackLength");
		useAcks = par("useAcks").boolValue();
		ackErrorRate = par("ackErrorRate").doubleValue();
		rxStartIndication = false;
		ignoreChannelState = false;
		waitUntilAckRXorTimeout = false;
		stopIgnoreChannelStateMsg = new cMessage("ChannelStateMsg");
        maxIvTThresh = par("maxivtThresh").doubleValue();
        minIvTThresh = par("minivtThresh").doubleValue();
        maxT2CThresh = par("maxt2cThresh").doubleValue();
        minT2CThresh = par("mint2cThresh").doubleValue();

		//mac-adresses
//		myMacAddress = getParentModule()->getParentModule()->getIndex();

		//XIANG ZHONG
		myMacAddress = myMacAddress.generateAutoAddress();

		myId = getParentModule()->getParentModule()->getFullPath();
		//create frequency mappings
		frequency.insert(std::pair<int, double>(Channels::CRIT_SOL, 5.86e9));
		frequency.insert(std::pair<int, double>(Channels::SCH1, 5.87e9));
		frequency.insert(std::pair<int, double>(Channels::SCH2, 5.88e9));
		frequency.insert(std::pair<int, double>(Channels::CCH, 5.89e9));
		frequency.insert(std::pair<int, double>(Channels::SCH3, 5.90e9));
		frequency.insert(std::pair<int, double>(Channels::SCH4, 5.91e9));
		frequency.insert(std::pair<int, double>(Channels::HPPS, 5.92e9));

		//create two edca systems

		myEDCA[type_CCH] = std::unique_ptr<EDCA>(new EDCA(this, type_CCH,par("queueSize").longValue()));
		myEDCA[type_CCH]->myId = myId;
		myEDCA[type_CCH]->myId.append(" CCH");
		myEDCA[type_CCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_CCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_CCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_CCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

		myEDCA[type_SCH] = std::unique_ptr<EDCA>(new EDCA(this, type_SCH,par("queueSize").longValue()));
		myEDCA[type_SCH]->myId = myId;
		myEDCA[type_SCH]->myId.append(" SCH");
		myEDCA[type_SCH]->createQueue(2,(((CWMIN_11P+1)/4)-1),(((CWMIN_11P +1)/2)-1),AC_VO);
		myEDCA[type_SCH]->createQueue(3,(((CWMIN_11P+1)/2)-1),CWMIN_11P,AC_VI);
		myEDCA[type_SCH]->createQueue(6,CWMIN_11P,CWMAX_11P,AC_BE);
		myEDCA[type_SCH]->createQueue(9,CWMIN_11P,CWMAX_11P,AC_BK);

		useSCH = par("useServiceChannel").boolValue();
		if (useSCH) {
			if (useAcks) throw cRuntimeError("Unicast model does not support channel switching");
			//set the initial service channel
			switch (par("serviceChannel").longValue()) {
				case 1: mySCH = Channels::SCH1; break;
				case 2: mySCH = Channels::SCH2; break;
				case 3: mySCH = Channels::SCH3; break;
				case 4: mySCH = Channels::SCH4; break;
				default: throw cRuntimeError("Service Channel must be between 1 and 4"); break;
			}
		}

		headerLength = par("headerLength");

		nextMacEvent = new cMessage("next Mac Event");
        stopvehicle  = new cMessage ("artificial accident");
        triggerManeuverEvt = new cMessage("Emergency maneuver");
        planManEvt = new cMessage("maneuver");

        mobility = (Veins::TraCIMobility*)getOwner()->getOwner()->findObject("veinsmobility",true);
        std::cout << " -- -- - "<< findHost()->getName() <<std::endl;
        if (std::strcmp(findHost()->getName(), "node") ==0){
            scheduleAt(simTime() + 20, planManEvt);
            std::cout <<"+++++++++ Position is: ("
                    <<mobility->getCurrentPosition().x <<"; "
                    <<mobility->getCurrentPosition().z <<"; "
                    <<std::endl;
        }

		if (useSCH) {
			uint64_t currenTime = simTime().raw();
			uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
			double timeToNextSwitch = (double)(switchingTime
							   - (currenTime % switchingTime)) / simTime().getScale();
			if ((currenTime / switchingTime) % 2 == 0) {
				setActiveChannel(type_CCH);
			}
			else {
				setActiveChannel(type_SCH);
			}

			// channel switching active
			nextChannelSwitch = new cMessage("Channel Switch");
			//add a little bit of offset between all vehicles, but no more than syncOffset
			simtime_t offset = dblrand() * par("syncOffset").doubleValue();
			scheduleAt(simTime() + offset + timeToNextSwitch, nextChannelSwitch);
		}
		else {
			// no channel switching
			nextChannelSwitch = 0;
			setActiveChannel(type_CCH);
		}

	    /*-------------------------------XIANG ZHONG----------------------------------*/

		registerInterface();

	    /*------------------------------END-----------------------------------------*/

		/* -----    BAM     ------*/


        if (strcmp(findHost()->getFullName(), "node[2]") == 0){
            scheduleAt(simTime() + 20, stopvehicle);
        }
        if (strcmp(findHost()->getName(), "node") == 0){//Assign Newly injected Vehicles to a Corresponding Cluster
            InjTime = simTime().dbl();
            if  (!isFirstClustVeh){
                std::cout<<"-------------------> " << findHost()->getFullName() <<std::endl;
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
//            std::cout <<std::endl<<std::endl;
//            std::cout << " 111111111  Vehicle  "<<mobility->getExternalId() <<"  Cluster ID = " << AllClustInfo.back().cluster_id<<std::endl;
        }


        /* -----    BAM     ------*/

		//stats
		statsReceivedPackets = 0;
		statsReceivedBroadcasts = 0;
		statsSentPackets = 0;
		statsSentAcks = 0;
		statsTXRXLostPackets = 0;
		statsSNIRLostPackets = 0;
		statsDroppedPackets = 0;
		statsNumTooLittleTime = 0;
		statsNumInternalContention = 0;
		statsNumBackoff = 0;
		statsSlotsBackoff = 0;
		statsTotalBusyTime = 0;

		idleChannel = true;
		lastBusy = simTime();
		channelIdle(true);
	}

}



/*-----------------------------XIANG ZHONG-----------------------------------------*/

inet::InterfaceEntry *Mac1609_4::createInterfaceEntry()
{
    inet::InterfaceEntry *ie = inet::getContainingNicModule(this);
    // capabilities
    ie->setMtu(par("mtu"));
//    macAddr.setAddress("0A-AA-00-00-00-01");
    ie->setMacAddress(myMacAddress);
    auto mac = ie->getMacAddress();
    EV_INFO << "My Mac Address is: " << mac << std::endl;

    ie->setInterfaceToken(myMacAddress.formInterfaceIdentifier());
    ie->setBroadcast(true);
    ie->setMulticast(true);

    return ie;
}

/*--------------------------------END--------------------------------------------*/


void Mac1609_4::registerInterface()
{
    /*-----------------------------XIANG ZHONG-----------------------------------------*/

    ASSERT(interfaceEntry == nullptr);
       inet::IInterfaceTable *ift = inet::findModuleFromPar<inet::IInterfaceTable>(par("interfaceTableModule"), this);
       if (ift) {
           interfaceEntry = createInterfaceEntry();
           ift->addInterface(interfaceEntry);
           auto module = inet::getContainingNicModule(this);
           inet::registerInterface(*interfaceEntry, module->gate("upperLayerIn"), module->gate("upperLayerOut"));
       }

    /*--------------------------------END--------------------------------------------*/
}




void Mac1609_4::handleSelfMsg(cMessage* msg) {
	if (msg == stopIgnoreChannelStateMsg) {
		ignoreChannelState = false;
		return;
	}

	if (AckTimeOutMessage* ackTimeOutMsg = dynamic_cast<AckTimeOutMessage*>(msg)) {
		handleAckTimeOut(ackTimeOutMsg);
		return;
	}

	if (msg == nextChannelSwitch) {
		ASSERT(useSCH);

		scheduleAt(simTime() + SWITCHING_INTERVAL_11P, nextChannelSwitch);

		switch (activeChannel) {
			case type_CCH:
				DBG_MAC << "CCH --> SCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_SCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[mySCH]);
				break;
			case type_SCH:
				DBG_MAC << "SCH --> CCH" << std::endl;
				channelBusySelf(false);
				setActiveChannel(type_CCH);
				channelIdle(true);
				phy11p->changeListeningFrequency(frequency[Channels::CCH]);
				break;
		}
		//schedule next channel switch in 50ms

	}
	else if (msg ==  nextMacEvent) {

		//we actually came to the point where we can send a packet
		channelBusySelf(true);
		cPacket* pktToSend = myEDCA[activeChannel]->initiateTransmit(lastIdle);
		ASSERT(pktToSend);

		inet::Packet* pkt;
		WSM_ipwave* wsm;

		inet::MacAddress destMac;

        Mac80211Pkt* mac = new Mac80211Pkt(pktToSend->getName(), pktToSend->getKind());

		if (pkt = dynamic_cast <inet::Packet*>(pktToSend))
		{

            lastAC = AC_BK;
            lastWSM = pkt;
            DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;
            //send the packet

            destMac =pkt->getTag<MacAddressReq>()->getDestAddress();

            if (destMac != inet::MacAddress::UNSPECIFIED_ADDRESS) { // -1 to unspecified
                mac->setDestAddr(destMac);
            } else {
                mac->setDestAddr(inet::MacAddress::BROADCAST_ADDRESS);
            }
            mac->encapsulate(pkt->dup());
		}

		else if (wsm = dynamic_cast <WSM_ipwave*>(pktToSend))
		{
            lastAC = mapUserPriority(wsm->getUserPriority());
            lastWSM = wsm;
            DBG_MAC << "MacEvent received. Trying to send packet with priority" << lastAC << std::endl;
            //send the packet


            destMac =wsm->getRecipientAddress();

            if (destMac != inet::MacAddress::UNSPECIFIED_ADDRESS) { // -1 to unspecified
                mac->setDestAddr(destMac);
            } else {
                mac->setDestAddr(inet::MacAddress::BROADCAST_ADDRESS);
            }

            mac->encapsulate(wsm->dup());

		}


//----------------------------------------------------------------------------------------------//
		mac->setSrcAddr(myMacAddress);


		enum PHY_MCS mcs;
		double txPower_mW;
		uint64_t datarate;
		PhyControlMessage *controlInfo = dynamic_cast<PhyControlMessage *>(pktToSend->getControlInfo());
		if (controlInfo) {
			//if MCS is not specified, just use the default one
			mcs = (enum PHY_MCS)controlInfo->getMcs();
			if (mcs != MCS_DEFAULT) {
				datarate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);
			}
			else {
				datarate = bitrate;
			}
			//apply the same principle to tx power
			txPower_mW = controlInfo->getTxPower_mW();
			if (txPower_mW < 0) {
				txPower_mW = txPower;
			}
		}
		else {
			mcs = MCS_DEFAULT;
			txPower_mW = txPower;
			datarate = bitrate;
		}

		simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength(), mcs);
		DBG_MAC << "Sending duration will be" << sendingDuration << std::endl;
		if ((!useSCH) || (timeLeftInSlot() > sendingDuration)) {
			if (useSCH) DBG_MAC << " Time in this slot left: " << timeLeftInSlot() << std::endl;

			double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];

			DBG_MAC << "Sending a Packet. Frequency " << freq << " Priority" << lastAC << std::endl;
			sendFrame(mac, RADIODELAY_11P, freq, datarate, txPower_mW);

			// schedule ack timeout for unicast packets
			if (destMac != inet::MacAddress::UNSPECIFIED_ADDRESS && useAcks) {
				waitUntilAckRXorTimeout = true;
				// PHY-RXSTART.indication should be received within ackWaitTime
				// sifs + slot + rx_delay: see 802.11-2012 9.3.2.8 (32us + 13us + 49us = 94us)
				simtime_t ackWaitTime(94, SIMTIME_US);
				// update id in the retransmit timer
				myEDCA[activeChannel]->myQueues[lastAC].ackTimeOut->setWsmId(pktToSend->getTreeId());
				simtime_t timeOut = sendingDuration + ackWaitTime;
				scheduleAt(simTime() + timeOut, myEDCA[activeChannel]->myQueues[lastAC].ackTimeOut);
			}
		}
		else {   //not enough time left now
			DBG_MAC << "Too little Time left. This packet cannot be send in this slot." << std::endl;
			statsNumTooLittleTime++;
			//revoke TXOP
			myEDCA[activeChannel]->revokeTxOPs();
			delete mac;
			channelIdle();
			//do nothing. contention will automatically start after channel switch
		}
	}
	else if (msg == stopvehicle){
        mobility->getVehicleCommandInterface()->setSpeed(0);
        reorganizethisCluster(mobility->getExternalId());

        Coord pos;
        pos.x = mobility->getVehicleCommandInterface()->ReturnSumoCurrentposition(mobility->getCurrentPosition()).x;
        pos.y = mobility->getVehicleCommandInterface()->ReturnSumoCurrentposition(mobility->getCurrentPosition()).y;
        ObstacleInfo.vehicleId = mobility->getExternalId();
        ObstacleInfo.vehiclePosition = pos;
        int li = mobility->getVehicleCommandInterface()->getLaneIndex();
        ObstacleInfo.laneID = li;
        obMob = mobility;
//        scheduleAt(simTime() + 0.1, triggerAccEvt);
    }
	else if (msg == planManEvt){
	    if (checkIsCH(mobility->getExternalId())){
	        memberManeuver();
	    }
	}
    else if (msg == triggerManeuverEvt){
        clusterManeuverHandler();
    }
}

void Mac1609_4::handleUpperControl(cMessage* msg) {
	assert(false);
}

void Mac1609_4::handleUpperMsg(cMessage* msg) {

    /*--------------------------------------------XIANG ZHONG--------------------------------------*/

	inet::Packet* pkt;
	WSM_ipwave* wsm;
	cPacket* thisMsg;
	t_access_category ac;
	t_channel chan;

	int num;

	if ((pkt = check_and_cast<inet::Packet*>(msg)) != NULL) {
        chan = type_CCH;
        ac = AC_BK;

	    DBG_MAC << "Received a message from upper layer for channel "
	            << chan << " Access Category (Priority):  "
	            << ac << std::endl;


	    if ((thisMsg = dynamic_cast <cPacket*> (pkt)) != NULL){
	        num = myEDCA[chan]->queuePacket(ac,thisMsg);
	    }
	    else
	        error("WaveMac can not cast this Packet!");

	}

	else if ((wsm = check_and_cast <WSM_ipwave*>(msg))!=NULL ) {
	    ac = mapUserPriority(wsm->getUserPriority());

	    DBG_MAC << "Received a message from upper layer for channel "
	            << wsm->getChannelNumber() << " Access Category (Priority):  "
	            << ac << std::endl;


	    if (wsm->getChannelNumber() == Channels::CCH) {
	        chan = type_CCH;
	    }
	    else {
	        ASSERT(useSCH);
	        wsm->setChannelNumber(mySCH);
	        chan = type_SCH;
	    }

        if ((thisMsg = dynamic_cast <cPacket*> (wsm)) != NULL){
            num = myEDCA[chan]->queuePacket(ac,thisMsg);
        }
        else
            error("WaveMac can not cast this WaveShortMessage!");

	}

    else
        error("WaveMac only accepts WaveShortMessages and inet Packets");


	//XIANG ZHONG
	//delete pkt;


//	--------------------------------------------END-----------------------------------------------------------//

	//packet was dropped in Mac
	if (num == -1) {
		statsDroppedPackets++;
		return;
	}

	//if this packet is not at the front of a new queue we dont have to reevaluate times
	DBG_MAC << "sorted packet into queue of EDCA " << chan << " this packet is now at position: " << num << std::endl;

	if (chan == activeChannel) {
		DBG_MAC << "this packet is for the currently active channel" << std::endl;
	}
	else {
		DBG_MAC << "this packet is NOT for the currently active channel" << std::endl;
	}

	if (num == 1 && idleChannel == true && chan == activeChannel) {

		simtime_t nextEvent = myEDCA[chan]->startContent(lastIdle,guardActive());

		if (nextEvent != -1) {
			if ((!useSCH) || (nextEvent <= nextChannelSwitch->getArrivalTime())) {
				if (nextMacEvent->isScheduled()) {
					cancelEvent(nextMacEvent);
				}
				scheduleAt(nextEvent,nextMacEvent);
				DBG_MAC << "Updated nextMacEvent:" << nextMacEvent->getArrivalTime().raw() << std::endl;
			}
			else {
				DBG_MAC << "Too little time in this interval. Will not schedule nextMacEvent" << std::endl;
				//it is possible that this queue has an txop. we have to revoke it
				myEDCA[activeChannel]->revokeTxOPs();
				statsNumTooLittleTime++;
			}
		}
		else {
			cancelEvent(nextMacEvent);
		}
	}
	if (num == 1 && idleChannel == false && myEDCA[chan]->myQueues[ac].currentBackoff == 0 && chan == activeChannel) {
		myEDCA[chan]->backoff(ac);
	}


}

void Mac1609_4::handleLowerControl(cMessage* msg) {
	if (msg->getKind() == MacToPhyInterface::PHY_RX_START) {
		rxStartIndication = true;
	} else if (msg->getKind() == MacToPhyInterface::PHY_RX_END_WITH_SUCCESS) {
		// PHY_RX_END_WITH_SUCCESS will get packet soon! Nothing to do here
	} else if (msg->getKind() == MacToPhyInterface::PHY_RX_END_WITH_FAILURE) {
		// RX failed at phy. Time to retransmit
		phy11p->notifyMacAboutRxStart(false);
		rxStartIndication = false;
		handleRetransmit(lastAC);
	} else if (msg->getKind() == MacToPhyInterface::TX_OVER) {

		DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;

		phy->setRadioState(Radio::RX);

		if (!dynamic_cast<Mac80211Ack*>(lastMac.get())) {
			//message was sent
			//update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
			myEDCA[activeChannel]->postTransmit(lastAC, lastWSM, useAcks);
		}
		//channel just turned idle.
		//don't set the chan to idle. the PHY layer decides, not us.

		if (guardActive()) {
			throw cRuntimeError("We shouldnt have sent a packet in guard!");
		}
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_BUSY) {
		channelBusy();
	}
	else if (msg->getKind() == Mac80211pToPhy11pInterface::CHANNEL_IDLE) {
		// Decider80211p::processSignalEnd() sends up the received packet to MAC followed by control message CHANNEL_IDLE in the same timestamp.
		// If we received a unicast frame (first event scheduled by Decider), MAC immediately schedules an ACK message and wants to switch the radio to TX mode.
		// So, the notification for channel idle from phy is undesirable and we skip it here.
		// After ACK TX is over, PHY will inform the channel status again.
		if (ignoreChannelState) {
			// Skipping channelidle because we are about to send an ack regardless of the channel state
		} else {
			channelIdle();
		}
	}
	else if (msg->getKind() == Decider80211p::BITERROR || msg->getKind() == Decider80211p::COLLISION) {
		statsSNIRLostPackets++;
		DBG_MAC << "A packet was not received due to biterrors" << std::endl;
	}
	else if (msg->getKind() == Decider80211p::RECWHILESEND) {
		statsTXRXLostPackets++;
		DBG_MAC << "A packet was not received because we were sending while receiving" << std::endl;
	}
	else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
		DBG_MAC << "Phylayer said radio switching is done" << std::endl;
	}
	else if (msg->getKind() == BaseDecider::PACKET_DROPPED) {
		phy->setRadioState(Radio::RX);
		DBG_MAC << "Phylayer said packet was dropped" << std::endl;
	}
	else {
		DBG_MAC << "Invalid control message type (type=NOTHING) : name=" << msg->getName() << " modulesrc=" << msg->getSenderModule()->getFullPath() << "." << std::endl;
		assert(false);
	}

	if (msg->getKind() == Decider80211p::COLLISION) {
		emit(sigCollision, true);
	}

	delete msg;
}

void Mac1609_4::setActiveChannel(t_channel state) {
	activeChannel = state;
	assert(state == type_CCH || (useSCH && state == type_SCH));
}

void Mac1609_4::finish() {
	for (auto&& p : myEDCA) {
		statsNumInternalContention += p.second->statsNumInternalContention;
		statsNumBackoff += p.second->statsNumBackoff;
		statsSlotsBackoff += p.second->statsSlotsBackoff;
	}

	recordScalar("ReceivedUnicastPackets", statsReceivedPackets);
	recordScalar("ReceivedBroadcasts", statsReceivedBroadcasts);
	recordScalar("SentPackets", statsSentPackets);
	recordScalar("SentAcknowledgements", statsSentAcks);
	recordScalar("SNIRLostPackets", statsSNIRLostPackets);
	recordScalar("RXTXLostPackets", statsTXRXLostPackets);
	recordScalar("TotalLostPackets", statsSNIRLostPackets+statsTXRXLostPackets);
	recordScalar("DroppedPacketsInMac", statsDroppedPackets);
	recordScalar("TooLittleTime", statsNumTooLittleTime);
	recordScalar("TimesIntoBackoff", statsNumBackoff);
	recordScalar("SlotsBackoff", statsSlotsBackoff);
	recordScalar("NumInternalContention", statsNumInternalContention);
	recordScalar("totalBusyTime", statsTotalBusyTime.dbl());
}

Mac1609_4::~Mac1609_4() {
	if (nextMacEvent) {
		cancelAndDelete(nextMacEvent);
		nextMacEvent = nullptr;
	}

	if (nextChannelSwitch) {
		cancelAndDelete(nextChannelSwitch);
		nextChannelSwitch = nullptr;
	}

	if (stopIgnoreChannelStateMsg) {
		cancelAndDelete(stopIgnoreChannelStateMsg);
		stopIgnoreChannelStateMsg = nullptr;
	}
};

void Mac1609_4::sendFrame(Mac80211Pkt* frame, simtime_t delay, double frequency, uint64_t datarate, double txPower_mW) {
	phy->setRadioState(Radio::TX); // give time for the radio to be in Tx state before transmitting

	delay = std::max(delay, RADIODELAY_11P); // wait at least for the radio to switch

	//XIANG ZHONG 19.06.28 add uniform interval
	//+ uniform(0.0001,0.003)
	attachSignal(frame, simTime() + delay, frequency, datarate, txPower_mW);
	MacToPhyControlInfo* phyInfo = dynamic_cast<MacToPhyControlInfo*>(frame->getControlInfo());
	ASSERT(phyInfo);

	lastMac.reset(frame->dup());
	sendDelayed(frame, delay, lowerLayerOut);

	if (dynamic_cast<Mac80211Ack*>(frame)) {
		statsSentAcks += 1;
	} else {
		statsSentPackets += 1;
	}
}

void Mac1609_4::attachSignal(Mac80211Pkt* mac, simtime_t startTime, double frequency, uint64_t datarate, double txPower_mW) {

	simtime_t duration = getFrameDuration(mac->getBitLength());

	Signal* s = createSignal(startTime, duration, txPower_mW, datarate, frequency);
	MacToPhyControlInfo* cinfo = new MacToPhyControlInfo(s);

	mac->setControlInfo(cinfo);
}

Signal* Mac1609_4::createSignal(simtime_t start, simtime_t length, double power, uint64_t bitrate, double frequency) {
	simtime_t end = start + length;
	//create signal with start at current simtime and passed length
	Signal* s = new Signal(start, length);

	//create and set tx power mapping
	ConstMapping* txPowerMapping = createSingleFrequencyMapping(start, end, frequency, 5.0e6, power);
	s->setTransmissionPower(txPowerMapping);

	Mapping* bitrateMapping = MappingUtils::createMapping(DimensionSet::timeDomain(), Mapping::STEPS);

	Argument pos(start);
	bitrateMapping->setValue(pos, bitrate);

	pos.setTime(phyHeaderLength / bitrate);
	bitrateMapping->setValue(pos, bitrate);

	s->setBitrate(bitrateMapping);

	return s;
}

/* checks if guard is active */
bool Mac1609_4::guardActive() const {
	if (!useSCH) return false;
	if (simTime().dbl() - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P)
		return true;
	return false;
}

/* returns the time until the guard is over */
simtime_t Mac1609_4::timeLeftTillGuardOver() const {
	ASSERT(useSCH);
	simtime_t sTime = simTime();
	if (sTime - nextChannelSwitch->getSendingTime() <= GUARD_INTERVAL_11P) {
		return GUARD_INTERVAL_11P
		       - (sTime - nextChannelSwitch->getSendingTime());
	}
	else
		return 0;
}

/* returns the time left in this channel window */
simtime_t Mac1609_4::timeLeftInSlot() const {
	ASSERT(useSCH);
	return nextChannelSwitch->getArrivalTime() - simTime();
}

/* Will change the Service Channel on which the mac layer is listening and sending */
void Mac1609_4::changeServiceChannel(int cN) {
	ASSERT(useSCH);
	if (cN != Channels::SCH1 && cN != Channels::SCH2 && cN != Channels::SCH3 && cN != Channels::SCH4) {
		throw cRuntimeError("This Service Channel doesnt exit: %d",cN);
	}

	mySCH = cN;

	if (activeChannel == type_SCH) {
		//change to new chan immediately if we are in a SCH slot,
		//otherwise it will switch to the new SCH upon next channel switch
		phy11p->changeListeningFrequency(frequency[mySCH]);
	}
}

void Mac1609_4::setTxPower(double txPower_mW) {
	txPower = txPower_mW;
}
void Mac1609_4::setMCS(enum PHY_MCS mcs) {
	ASSERT2(mcs != MCS_DEFAULT, "invalid MCS selected");
	bitrate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);
	setParametersForBitrate(bitrate);
}

void Mac1609_4::setCCAThreshold(double ccaThreshold_dBm) {
	phy11p->setCCAThreshold(ccaThreshold_dBm);
}

void Mac1609_4::handleLowerMsg(cMessage* msg) {

    Mac80211Pkt* macPkt = check_and_cast<Mac80211Pkt*>(msg);

    //pass information about received frame to the upper layers
    DeciderResult80211 *macRes = check_and_cast<DeciderResult80211 *>(PhyToMacControlInfo::getDeciderResult(msg));
    DeciderResult80211 *res = new DeciderResult80211(*macRes);

    inet::MacAddress dest = macPkt->getDestAddr();

    DBG_MAC << "Received frame name= " << macPkt->getName()
            << ", myState=" << " src=" << macPkt->getSrcAddr()
            << " dst=" << macPkt->getDestAddr() << " myAddr="
            << myMacAddress << std::endl;



    if (dest == myMacAddress) {
        if (auto* ack = dynamic_cast<Mac80211Ack*>(macPkt)) {
            std::cout <<"111111111111111" <<std::endl;
            if (useAcks){
                handleAck(ack);
            }
        }

        //handle lower packet
        else if (inet::Packet* pkt = dynamic_cast<inet::Packet*>(macPkt->decapsulate()))
        {
            pkt->setControlInfo(new PhyToMacControlInfo(res));
            handleUnicastPkt(std::move(pkt));
            EV_INFO<<"handle unicast Pkt"<< endl;
        }

        //handle lower wsm
        else if (WSM_ipwave* wsm = dynamic_cast<WSM_ipwave*>(macPkt->decapsulate()))
        {
            wsm->setControlInfo(new PhyToMacControlInfo(res));
            handleUnicastWSM(std::move(wsm));

            //
            EV_INFO<<"handle unicast WSM"<< endl;


        }
    }
    else if (dest == inet::MacAddress::BROADCAST_ADDRESS) {
        statsReceivedBroadcasts++;

        //handle lower packet
        if (inet::Packet* pkt = dynamic_cast<inet::Packet*>(macPkt->decapsulate()))
        {
            std::cout <<"22222222222222222222" <<std::endl;
            EV_INFO<<"handle broadcast Pkt"<< endl;
            pkt->setControlInfo(new PhyToMacControlInfo(res));
            pkt->addTagIfAbsent<inet::PacketProtocolTag>()->setProtocol(&inet::Protocol::ipv6);
            pkt->addTagIfAbsent<inet::DispatchProtocolReq>()->setProtocol(&inet::Protocol::ipv6);
            pkt->addTagIfAbsent<inet::InterfaceInd>()->setInterfaceId(interfaceEntry->getInterfaceId());
            sendUp(pkt);
        }

        //handle lower wsm
        else if (WSM_ipwave* wsm = dynamic_cast<WSM_ipwave*>(macPkt->decapsulate()))
        {
            std::cout <<"3333333333333" <<std::endl;
            EV_INFO<<"handle broadcast wsm"<< endl;
            wsm->setControlInfo(new PhyToMacControlInfo(res));
            sendUp(wsm);

        }
    }
    else {
        DBG_MAC << "Packet not for me" << std::endl;
    }

    delete macPkt;

    if (rxStartIndication) {
        // We have handled/processed the incoming packet
        // Since we reached here, we were expecting an ack but we didnt get it, so retransmission should take place
        phy11p->notifyMacAboutRxStart(false);
        rxStartIndication = false;
        handleRetransmit(lastAC);
    }

}

/*-----------------------------------XIANG ZHONG-----------------------------------------*/


int Mac1609_4::EDCA::queuePacket(t_access_category ac,cPacket* msg) {

	if (maxQueueSize && myQueues[ac].queue.size() >= maxQueueSize) {
		delete msg;
		return -1;
	}
	myQueues[ac].queue.push(msg);
	return myQueues[ac].queue.size();
}

void Mac1609_4::EDCA::createQueue(int aifsn, int cwMin, int cwMax,t_access_category ac) {

	if (myQueues.find(ac) != myQueues.end()) {
		throw cRuntimeError("You can only add one queue per Access Category per EDCA subsystem");
	}

	EDCAQueue newQueue(aifsn,cwMin,cwMax,ac);
	myQueues[ac] = newQueue;
}

Mac1609_4::t_access_category Mac1609_4::mapUserPriority(int prio) {
	// Map user priority to access category, based on IEEE Std 802.11-2012, Table 9-1
	switch (prio) {
		case 1: return AC_BK;
		case 2: return AC_BK;
		case 0: return AC_BE;
		case 3: return AC_BE;
		case 4: return AC_VI;
		case 5: return AC_VI;
		case 6: return AC_VO;
		case 7: return AC_VO;
		default: throw cRuntimeError("MacLayer received a packet with unknown priority"); break;
	}
	return AC_VO;
}

cPacket* Mac1609_4::EDCA::initiateTransmit(simtime_t lastIdle) {

	//iterate through the queues to return the packet we want to send
    cPacket* pktToSend = NULL;

	simtime_t idleTime = simTime() - lastIdle;

	DBG_MAC << "Initiating transmit at " << simTime() << ". I've been idle since " << idleTime << std::endl;

	// As t_access_category is sorted by priority, we iterate back to front.
	// This realizes the behavior documented in IEEE Std 802.11-2012 Section 9.2.4.2; that is, "data frames from the higher priority AC" win an internal collision.
	// The phrase "EDCAF of higher UP" of IEEE Std 802.11-2012 Section 9.19.2.3 is assumed to be meaningless.
	for (auto iter = myQueues.rbegin(); iter != myQueues.rend(); iter++) {
		if (iter->second.queue.size() != 0 && !iter->second.waitForAck) {
			if (idleTime >= iter->second.aifsn* SLOTLENGTH_11P + SIFS_11P && iter->second.txOP == true) {

				DBG_MAC << "Queue " << iter->first << " is ready to send!" << std::endl;

				iter->second.txOP = false;
				//this queue is ready to send
				if (pktToSend == NULL) {
					pktToSend = iter->second.queue.front();
				}
				else {
					//there was already another packet ready. we have to go increase cw and go into backoff. It's called internal contention and its wonderful

					statsNumInternalContention++;
					iter->second.cwCur = std::min(iter->second.cwMax,(iter->second.cwCur+1)*2-1);
					iter->second.currentBackoff = OWNER intuniform(0,iter->second.cwCur);
					DBG_MAC << "Internal contention for queue " << iter->first  << " : "<< iter->second.currentBackoff << ". Increase cwCur to " << iter->second.cwCur << std::endl;
				}
			}
		}
	}

	if (pktToSend == NULL) {
		throw cRuntimeError("No packet was ready");
	}
	return pktToSend;
}

simtime_t Mac1609_4::EDCA::startContent(simtime_t idleSince,bool guardActive) {

	DBG_MAC << "Restarting contention." << std::endl;

	simtime_t nextEvent = -1;

	simtime_t idleTime = SimTime().setRaw(std::max((int64_t)0,(simTime() - idleSince).raw()));;

	lastStart = idleSince;

	DBG_MAC << "Channel is already idle for:" << idleTime << " since " << idleSince << std::endl;

	//this returns the nearest possible event in this EDCA subsystem after a busy channel

	for (auto&& p : myQueues) {
		auto &accessCategory = p.first;
		auto &edcaQueue = p.second;
		if (edcaQueue.queue.size() != 0 && !edcaQueue.waitForAck) {

			/* 1609_4 says that when attempting to send (backoff == 0) when guard is active, a random backoff is invoked */

			if (guardActive == true && edcaQueue.currentBackoff == 0) {
				//cw is not increased
				edcaQueue.currentBackoff = OWNER intuniform(0,edcaQueue.cwCur);
				statsNumBackoff++;
			}

			simtime_t DIFS = edcaQueue.aifsn * SLOTLENGTH_11P + SIFS_11P;

			//the next possible time to send can be in the past if the channel was idle for a long time, meaning we COULD have sent earlier if we had a packet
			simtime_t possibleNextEvent = DIFS + edcaQueue.currentBackoff * SLOTLENGTH_11P;


			DBG_MAC << "Waiting Time for Queue " << accessCategory <<  ":" << possibleNextEvent << "=" << edcaQueue.aifsn << " * "  << SLOTLENGTH_11P << " + " << SIFS_11P << "+" << edcaQueue.currentBackoff << "*" << SLOTLENGTH_11P << "; Idle time: " << idleTime << std::endl;

			if (idleTime > possibleNextEvent) {
				DBG_MAC << "Could have already send if we had it earlier" << std::endl;
				//we could have already sent. round up to next boundary
				simtime_t base = idleSince + DIFS;
				possibleNextEvent =  simTime() - simtime_t().setRaw((simTime() - base).raw() % SLOTLENGTH_11P.raw()) + SLOTLENGTH_11P;
			}
			else {
				//we are gonna send in the future
				DBG_MAC << "Sending in the future" << std::endl;
				possibleNextEvent =  idleSince + possibleNextEvent;
			}
			nextEvent == -1? nextEvent =  possibleNextEvent : nextEvent = std::min(nextEvent,possibleNextEvent);
		}
	}
	return nextEvent;
}

void Mac1609_4::EDCA::stopContent(bool allowBackoff, bool generateTxOp) {
	//update all Queues

	DBG_MAC << "Stopping Contention at " << simTime().raw() << std::endl;

	simtime_t passedTime = simTime() - lastStart;

	DBG_MAC << "Channel was idle for " << passedTime << std::endl;

	lastStart = -1; //indicate that there was no last start

	for (auto&& p : myQueues) {
		auto &accessCategory = p.first;
		auto &edcaQueue = p.second;
		if ((edcaQueue.currentBackoff != 0 || edcaQueue.queue.size() != 0) && !edcaQueue.waitForAck) {
			//check how many slots we already waited until the chan became busy

			int64_t oldBackoff = edcaQueue.currentBackoff;

			std::string info;
			if (passedTime < edcaQueue.aifsn * SLOTLENGTH_11P + SIFS_11P) {
				//we didnt even make it one DIFS :(
				info.append(" No DIFS");
			}
			else {
				//decrease the backoff by one because we made it longer than one DIFS
				edcaQueue.currentBackoff -= 1;

				//check how many slots we waited after the first DIFS
				int64_t passedSlots = (int64_t)((passedTime - SimTime(edcaQueue.aifsn * SLOTLENGTH_11P + SIFS_11P)) / SLOTLENGTH_11P);

				DBG_MAC << "Passed slots after DIFS: " << passedSlots << std::endl;


				if (edcaQueue.queue.size() == 0) {
					//this can be below 0 because of post transmit backoff -> backoff on empty queues will not generate macevents,
					//we dont want to generate a txOP for empty queues
					edcaQueue.currentBackoff -= std::min(edcaQueue.currentBackoff,passedSlots);
					info.append(" PostCommit Over");
				}
				else {
					edcaQueue.currentBackoff -= passedSlots;
					if (edcaQueue.currentBackoff <= -1) {
						if (generateTxOp) {
							edcaQueue.txOP = true; info.append(" TXOP");
						}
						//else: this packet couldnt be sent because there was too little time. we could have generated a txop, but the channel switched
						edcaQueue.currentBackoff = 0;
					}

				}
			}
			DBG_MAC << "Updating backoff for Queue " << accessCategory << ": " << oldBackoff << " -> " << edcaQueue.currentBackoff << info <<std::endl;
		}
	}
}
void Mac1609_4::EDCA::backoff(t_access_category ac) {
	myQueues[ac].currentBackoff = OWNER intuniform(0,myQueues[ac].cwCur);
	statsSlotsBackoff += myQueues[ac].currentBackoff;
	statsNumBackoff++;
	DBG_MAC << "Going into Backoff because channel was busy when new packet arrived from upperLayer" << std::endl;
}

void Mac1609_4::EDCA::postTransmit(t_access_category ac, cPacket* msg, bool useAcks) {

    //XIANG ZHONG
    // change wsm->getRecipientAddress to destMac

    WSM_ipwave* wsm;
    inet::Packet* pkt;
    inet::MacAddress destMac;

//    bool isWSM = (wsm = dynamic_cast<WSM*>(msg));
//    bool isIP = (ip = dynamic_cast<inet::Packet*>(msg));

    if (wsm = dynamic_cast<WSM_ipwave*>(msg))
        destMac = wsm->getRecipientAddress();
    else if (pkt = dynamic_cast<inet::Packet*>(msg))
        destMac = pkt->getTag<inet::MacAddressReq>()->getDestAddress();
    else
        throw cRuntimeError("No dest address received!");

	bool holBlocking = (destMac!= inet::MacAddress::UNSPECIFIED_ADDRESS) && useAcks;
	if (holBlocking) {
		//mac->waitUntilAckRXorTimeout = true; // set in handleselfmsg()
		// Head of line blocking, wait until ack timeout
		myQueues[ac].waitForAck = true;
		myQueues[ac].waitOnUnicastID = msg->getTreeId();
		((Mac1609_4*)owner)->phy11p->notifyMacAboutRxStart(true);
	} else {
		myQueues[ac].waitForAck = false;
		delete myQueues[ac].queue.front();
		myQueues[ac].queue.pop();
		myQueues[ac].cwCur = myQueues[ac].cwMin;
		//post transmit backoff
		myQueues[ac].currentBackoff = OWNER intuniform(0,myQueues[ac].cwCur);
		statsSlotsBackoff += myQueues[ac].currentBackoff;
		statsNumBackoff++;
		DBG_MAC << "Queue " << ac << " will go into post-transmit backoff for " << myQueues[ac].currentBackoff << " slots" << std::endl;
	}
}

Mac1609_4::EDCA::EDCA(cSimpleModule *owner, t_channel channelType, int maxQueueLength)
	: owner(owner),
	  maxQueueSize(maxQueueLength),
	  channelType(channelType),
	  statsNumInternalContention(0),
	  statsNumBackoff(0),
	  statsSlotsBackoff(0) {}

Mac1609_4::EDCA::~EDCA() {
	for (auto &q : myQueues) {
		auto& ackTimeout = q.second.ackTimeOut;
		if (ackTimeout) {
			owner->cancelAndDelete(ackTimeout);
			ackTimeout = nullptr;
		}
	}
}

void Mac1609_4::EDCA::revokeTxOPs() {
	for (auto&& p : myQueues) {
		auto &edcaQueue = p.second;
		if (edcaQueue.txOP == true) {
			edcaQueue.txOP = false;
			edcaQueue.currentBackoff = 0;
		}
	}
}

void Mac1609_4::channelBusySelf(bool generateTxOp) {

	//the channel turned busy because we're sending. we don't want our queues to go into backoff
	//internal contention is already handled in initiateTransmission

	if (!idleChannel) return;
	idleChannel = false;
	DBG_MAC << "Channel turned busy: Switch or Self-Send" << std::endl;

	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(false, generateTxOp);

	emit(sigChannelBusy, true);
}

void Mac1609_4::channelBusy() {

	if (!idleChannel) return;

	//the channel turned busy because someone else is sending
	idleChannel = false;
	DBG_MAC << "Channel turned busy: External sender" << std::endl;
	lastBusy = simTime();

	//channel turned busy
	if (nextMacEvent->isScheduled() == true) {
		cancelEvent(nextMacEvent);
	}
	else {
		//the edca subsystem was not doing anything anyway.
	}
	myEDCA[activeChannel]->stopContent(true,false);

	emit(sigChannelBusy, true);
}

void Mac1609_4::channelIdle(bool afterSwitch) {

	DBG_MAC << "Channel turned idle: Switch: " << afterSwitch << std::endl;
	if (waitUntilAckRXorTimeout) {
		return;
	}

	if (nextMacEvent->isScheduled() == true) {
		//this rare case can happen when another node's time has such a big offset that the node sent a packet although we already changed the channel
		//the workaround is not trivial and requires a lot of changes to the phy and decider
		return;
		//throw cRuntimeError("channel turned idle but contention timer was scheduled!");
	}

	idleChannel = true;

	simtime_t delay = 0;

	//account for 1609.4 guards
	if (afterSwitch) {
		//	delay = GUARD_INTERVAL_11P;
	}
	if (useSCH) {
		delay += timeLeftTillGuardOver();
	}

	//channel turned idle! lets start contention!
	lastIdle = delay + simTime();
	statsTotalBusyTime += simTime() - lastBusy;

	//get next Event from current EDCA subsystem
	simtime_t nextEvent = myEDCA[activeChannel]->startContent(lastIdle,guardActive());
	if (nextEvent != -1) {
		if ((!useSCH) || (nextEvent < nextChannelSwitch->getArrivalTime())) {
			scheduleAt(nextEvent,nextMacEvent);
			DBG_MAC << "next Event is at " << nextMacEvent->getArrivalTime().raw() << std::endl;
		}
		else {
			DBG_MAC << "Too little time in this interval. will not schedule macEvent" << std::endl;
			statsNumTooLittleTime++;
			myEDCA[activeChannel]->revokeTxOPs();
		}
	}
	else {
		DBG_MAC << "I don't have any new events in this EDCA sub system" << std::endl;
	}

	emit(sigChannelBusy, false);

}

void Mac1609_4::setParametersForBitrate(uint64_t bitrate) {
	for (unsigned int i = 0; i < NUM_BITRATES_80211P; i++) {
		if (bitrate == BITRATES_80211P[i]) {
			n_dbps = N_DBPS_80211P[i];
			return;
		}
	}
	throw cRuntimeError("Chosen Bitrate is not valid for 802.11p: Valid rates are: 3Mbps, 4.5Mbps, 6Mbps, 9Mbps, 12Mbps, 18Mbps, 24Mbps and 27Mbps. Please adjust your omnetpp.ini file accordingly.");
}

bool Mac1609_4::isChannelSwitchingActive() {
    return useSCH;
}

simtime_t Mac1609_4::getSwitchingInterval() {
    return SWITCHING_INTERVAL_11P;
}

bool Mac1609_4::isCurrentChannelCCH() {
    return (activeChannel ==  type_CCH);
}

simtime_t Mac1609_4::getFrameDuration(int payloadLengthBits, enum PHY_MCS mcs) const {
    simtime_t duration;
    if (mcs == MCS_DEFAULT) {
        // calculate frame duration according to Equation (17-29) of the IEEE 802.11-2007 standard
        duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(n_dbps) );
    }
    else {
        uint32_t ndbps = getNDBPS(mcs);
        duration = PHY_HDR_PREAMBLE_DURATION + PHY_HDR_PLCPSIGNAL_DURATION + T_SYM_80211P * ceil( (16 + payloadLengthBits + 6)/(ndbps) );
    }

	return duration;
}

// Unicast

void Mac1609_4::sendAck(inet::MacAddress recpAddress, unsigned long wsmId) {
	ASSERT(useAcks);
	// 802.11-2012 9.3.2.8
	// send an ACK after SIFS without regard of busy/ idle state of channel
	ignoreChannelState = true;
	channelBusySelf(true);

	// send the packet
	auto* mac = new Mac80211Ack("ACK");
	mac->setDestAddr(recpAddress);
	mac->setSrcAddr(myMacAddress);
	mac->setMessageId(wsmId);
	mac->setBitLength(ackLength);

	enum PHY_MCS mcs = MCS_DEFAULT;
	uint64_t datarate = getOfdmDatarate(mcs, BW_OFDM_10_MHZ);

	simtime_t sendingDuration = RADIODELAY_11P + getFrameDuration(mac->getBitLength(), mcs);
	DBG_MAC << "Ack sending duration will be " << sendingDuration << std::endl;

	// TODO: check ack procedure when channel switching is allowed
	// double freq = (activeChannel == type_CCH) ? frequency[Channels::CCH] : frequency[mySCH];
	double freq = frequency[Channels::CCH];

	DBG_MAC << "Sending an ack. Frequency " << freq << " at time : " << simTime() + SIFS_11P << std::endl;
	sendFrame(mac, SIFS_11P, freq, datarate, txPower);
	scheduleAt(simTime() + SIFS_11P, stopIgnoreChannelStateMsg);
}


/*-------------------------XIANG ZHONG------------------------------------------*/

void Mac1609_4::handleUnicastWSM(WSM_ipwave* wsm) {

	if (useAcks) {
		sendAck(wsm->getSenderAddress(), wsm->getTreeId());
	}

	if (handledUnicastToApp.find(wsm->getTreeId()) == handledUnicastToApp.end()) {
		handledUnicastToApp.insert(wsm->getTreeId());
		DBG_MAC << "Received a data packet addressed to me." << std::endl;
		statsReceivedPackets++;
		sendUp(wsm);
	}
}


void Mac1609_4::handleUnicastPkt(inet::Packet* pkt) {

    if (useAcks) {
        //XIANG ZHONG
        EV_INFO <<"Send Ack: "<<pkt->getTag<inet::MacAddressReq>()->getSrcAddress()<<endl;
        sendAck(pkt->getTag<inet::MacAddressReq>()->getSrcAddress(), pkt->getTreeId());
    }

    if (handledUnicastToApp.find(pkt->getTreeId()) == handledUnicastToApp.end()) {
        handledUnicastToApp.insert(pkt->getTreeId());
        DBG_MAC << "Received a data packet addressed to me." << std::endl;
        statsReceivedPackets++;

//         auto interfaceReq = wsm->findTag<inet::InterfaceReq>()->getInterfaceId(); //nic ID:100
//         auto protocol = wsm->getTag<inet::PacketProtocolTag>()->getProtocol(); //ipv6

        pkt->addTagIfAbsent<inet::PacketProtocolTag>()->setProtocol(&inet::Protocol::ipv6);
        pkt->addTagIfAbsent<inet::DispatchProtocolReq>()->setProtocol(&inet::Protocol::ipv6);
        pkt->addTagIfAbsent<inet::InterfaceInd>()->setInterfaceId(interfaceEntry->getInterfaceId());

        sendUp(pkt);
    }
}
/*---------------------------------------------END-------------------------------*/


void Mac1609_4::handleAck(const Mac80211Ack* ack) {
	ASSERT2(rxStartIndication, "Not expecting ack");
	phy11p->notifyMacAboutRxStart(false);
	rxStartIndication = false;

	t_channel chan = type_CCH;
	bool queueUnblocked = false;
	for (auto&& p : myEDCA[chan]->myQueues) {
		auto &accessCategory = p.first;
		auto &edcaQueue = p.second;
		if (edcaQueue.queue.size() > 0 && edcaQueue.waitForAck && (edcaQueue.waitOnUnicastID == ack->getMessageId())) {
		    cPacket* wsm = edcaQueue.queue.front();
			edcaQueue.queue.pop();
			delete wsm;
			myEDCA[chan]->myQueues[accessCategory].cwCur = myEDCA[chan]->myQueues[accessCategory].cwMin;
			myEDCA[chan]->backoff(accessCategory);
			edcaQueue.ssrc = 0;
			edcaQueue.slrc = 0;
			edcaQueue.waitForAck = false;
			edcaQueue.waitOnUnicastID = -1;
			if (myEDCA[chan]->myQueues[accessCategory].ackTimeOut->isScheduled()) {
				cancelEvent(myEDCA[chan]->myQueues[accessCategory].ackTimeOut);
			}
			queueUnblocked = true;
		}
	}
	if (!queueUnblocked) {
		throw cRuntimeError("Could not find WSM in EDCA queues with WSM ID received in ACK");
	} else {
		waitUntilAckRXorTimeout = false;
	}
}

void Mac1609_4::handleAckTimeOut(AckTimeOutMessage* ackTimeOutMsg) {
	if (rxStartIndication) {
		// Rx is already in process. Wait for it to complete.
		// In case it is not an ack, we will retransmit
		// This assigning might be redundant as it was set already in handleSelfMsg but no harm in reassigning here.
		lastAC = (t_access_category)(ackTimeOutMsg->getKind());
		return;
	}
	// We did not start receiving any packet.
	// stop receiving notification for rx start as we will retransmit
	phy11p->notifyMacAboutRxStart(false);
	// back off and try retransmission again
	handleRetransmit((t_access_category)(ackTimeOutMsg->getKind()));
	// Phy was requested not to send channel idle status on TX_OVER
	// So request the channel status now. For the case when we receive ACK, decider updates channel status itself after ACK RX
	phy11p->requestChannelStatusIfIdle();
}

void Mac1609_4::handleRetransmit(t_access_category ac) {
	// cancel the acktime out
	if (myEDCA[type_CCH]->myQueues[ac].ackTimeOut->isScheduled()) {
		// This case is possible if we received PHY_RX_END_WITH_SUCCESS or FAILURE even before ack timeout
		cancelEvent(myEDCA[type_CCH]->myQueues[ac].ackTimeOut);
	}
	if(myEDCA[type_CCH]->myQueues[ac].queue.size() == 0) {
		throw cRuntimeError("Trying retransmission on empty queue...");
	}
	cPacket* appPkt = myEDCA[type_CCH]->myQueues[ac].queue.front();
	bool contend = false;
	bool retriesExceeded = false;
	// page 879 of IEEE 802.11-2012
	if (appPkt->getBitLength() <= dot11RTSThreshold) {
		myEDCA[type_CCH]->myQueues[ac].ssrc++;
		if (myEDCA[type_CCH]->myQueues[ac].ssrc <= dot11ShortRetryLimit) {
			retriesExceeded = false;
		} else {
			retriesExceeded = true;
		}
	} else {
		myEDCA[type_CCH]->myQueues[ac].slrc++;
		if (myEDCA[type_CCH]->myQueues[ac].slrc <= dot11LongRetryLimit) {
			retriesExceeded = false;
		} else {
			retriesExceeded = true;
		}
	}
	if (!retriesExceeded) {
		// try again!
		myEDCA[type_CCH]->myQueues[ac].cwCur = std::min(myEDCA[type_CCH]->myQueues[ac].cwMax, (myEDCA[type_CCH]->myQueues[ac].cwCur*2)+1);
		myEDCA[type_CCH]->backoff(ac);
		contend = true;
		// no need to reset wait on id here as we are still retransmitting same packet
		myEDCA[type_CCH]->myQueues[ac].waitForAck = false;
	} else {
		// enough tries!
		myEDCA[type_CCH]->myQueues[ac].queue.pop();
		if (myEDCA[type_CCH]->myQueues[ac].queue.size() > 0) {
			// start contention only if there are more packets in the queue
			contend = true;
		}
		delete appPkt;
		myEDCA[type_CCH]->myQueues[ac].cwCur = myEDCA[type_CCH]->myQueues[ac].cwMin;
		myEDCA[type_CCH]->backoff(ac);
		myEDCA[type_CCH]->myQueues[ac].waitForAck = false;
		myEDCA[type_CCH]->myQueues[ac].waitOnUnicastID = -1;
		myEDCA[type_CCH]->myQueues[ac].ssrc = 0;
		myEDCA[type_CCH]->myQueues[ac].slrc = 0;
	}
	waitUntilAckRXorTimeout = false;
	if (contend && idleChannel && !ignoreChannelState) {
		// reevaluate times -- if channel is not idle, then contention would start automatically
		cancelEvent(nextMacEvent);
		simtime_t nextEvent = myEDCA[type_CCH]->startContent(lastIdle, guardActive());
		scheduleAt(nextEvent, nextMacEvent);
	}
}

Mac1609_4::EDCA::EDCAQueue::EDCAQueue(int aifsn,int cwMin, int cwMax, t_access_category ac)
	: aifsn(aifsn),
	  cwMin(cwMin),
	  cwMax(cwMax),
	  cwCur(cwMin),
	  currentBackoff(0),
	  txOP(false),
	  ssrc(0),
	  slrc(0),
	  waitForAck(false),
	  waitOnUnicastID(-1),
	  ackTimeOut(new AckTimeOutMessage("AckTimeOut")) {
	ackTimeOut->setKind(ac);
}

Mac1609_4::EDCA::EDCAQueue::~EDCAQueue() {
	while (!queue.empty()) {
		delete queue.front();
		queue.pop();
	}
	// ackTimeOut needs to be deleted in EDCA
}


void Mac1609_4::clusterManeuverHandler(){
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
//        Alternative needed
        /*if (continueClusterManeuvers){
            scheduleAt(simTime() + 0.1, triggerManeuverEvt);
        }*/
     /*   else{
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
        }*/
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
/*                        collisionRecords tmpc;
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
                        flag1 = true;*/
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

void Mac1609_4::CMManeuverPlanningBasedLaneProbability (Clusterstruct CMembers, double CP2Obst, TraCIMobility*  emvmob, double ManTime){
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
cp2obstacle Mac1609_4::CP2Obstacle(TraCIMobility* vehMob){
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
std::map<int,double>  Mac1609_4::CPManeuverLanes(Clusterstruct CMs, double CP2Obst, TraCIMobility*  emvmob, double MT){
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

std::string Mac1609_4::getClusterHeadID (std::map<std::string,Veins::TraCIMobility*> Allvehicles){
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
void Mac1609_4::UpdateMyClusterMembers(){//Remove from Clusters vehicles that exited from Road
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        if(AllClustInfo[i].cluster_id == mycluster.cluster_id){
            mycluster = AllClustInfo[i];
        }
    }
}
std::string Mac1609_4::getInterVehiclesAveragedistances(std::map<std::string,Veins::TraCIMobility*> Allvehicles){
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
void Mac1609_4::classifyClusterMembersbyLane(Clusterstruct clust, int nl){
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
    }
}
void Mac1609_4::unregisteringFromClusters(){
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
}
void Mac1609_4::reorganizethisCluster(std::string MID){
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
void Mac1609_4::memberManeuver(){
    classifyClusterMembersbyLane(mycluster, 3);
    std::map<int,  std::vector<Veins::TraCIMobility*>>::iterator it = ClusterMembersbyLane.find(ObstacleInfo.laneID);
    if (it != ClusterMembersbyLane.end()){
        if (ObstacleInfo.laneID == 0){

            for (auto i:it->second){
                std::map<std::string, Veins::TraCIMobility*>::iterator it2;
                it2 = myMembersInAccident.find(i->getExternalId());
                if (it2 == myMembersInAccident.end()){
                    i->getVehicleCommandInterface()->changeLane(ObstacleInfo.laneID+1, 2000);
                }
            }
        }
        else if (ObstacleInfo.laneID == 2){

            for (auto i:it->second){
                std::map<std::string, Veins::TraCIMobility*>::iterator it2;
                it2 = myMembersInAccident.find(i->getExternalId());
                if (it2 == myMembersInAccident.end()){
                    i->getVehicleCommandInterface()->changeLane(ObstacleInfo.laneID-1, 2000);
                }
            }
        }
        else{
            int nexl = ObstacleInfo.laneID - 1;
            for (auto i:it->second){
                std::map<std::string, Veins::TraCIMobility*>::iterator it2;
                it2 = myMembersInAccident.find(i->getExternalId());
                if (it2 == myMembersInAccident.end()){
                    if (nexl == ObstacleInfo.laneID - 1){
                        i->getVehicleCommandInterface()->changeLane(nexl, 2000);
                        nexl = ObstacleInfo.laneID + 1;
                    }
                    else {
                        i->getVehicleCommandInterface()->changeLane(nexl, 2000);
                        nexl = ObstacleInfo.laneID - 1;
                    }
                }
            }
        }
    }
}
bool Mac1609_4::checkIsCH(std::string vehID){
    bool flag = false;
    for (unsigned int i = 0; i < AllClustInfo.size(); i++){
        if (strcmp(vehID.c_str(), AllClustInfo[i].cluster_Hid.c_str()) == 0){
            flag = true;
            mycluster = AllClustInfo[i];
            break;
        }
    }
   return flag;
}
