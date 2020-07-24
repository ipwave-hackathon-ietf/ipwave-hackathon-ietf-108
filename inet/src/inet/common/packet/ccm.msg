import inet.common.INETDefs;
import inet.common.Units;
import inet.common.packet.chunk.Chunk;

cplusplus {{
#include "inet/common/packet/Packet.h"
}}
namespace inet;
//
class ccm
{
	string senderID;
	double sendingSpeed;
	int CurrentLane;
	int headingLane;
	bool isrsu; 
}