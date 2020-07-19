cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;
class noncobject Coord;

packet ECM extends WaveShortMessage {
	Coord senderPos;
//	coord senderSpeed;
	double accVehSpeed;
	string accidentLane; 
	int accLInd;
	string senderid;
	simtime_t sendingTime;
	int SeqNum = 0;
//	Emergency vehicle Forwarder
	string emergVID;
	string emergManeuvLane;
	double emergDrivSpeed;
	Coord emergVehCurPos;
	Coord emergStartPos;
	Coord EmergEndPos;
	bool isForwarded = false;
	
}