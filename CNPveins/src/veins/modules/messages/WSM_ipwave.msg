//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// TODO generated message class
//


cplusplus {{
#include "inet/linklayer/common/MacAddress.h"
using namespace inet;
}}

class MacAddress extends void;
//namespace inet;

packet WSM_ipwave {
	//Version of the Wave Short Message
	int wsmVersion = 0;
	//Determine which security mechanism was used
	int securityType = 0;
	//Channel Number on which this packet was sent
	int channelNumber;
	//Data rate with which this packet was sent
	int dataRate = 1;
	//User priority with which this packet was sent (note the AC mapping rules in Mac1609_4::mapUserPriority)
	int userPriority = 7;
	//Unique number to identify the service
	int psid = 0;
	//Provider Service Context
	string psc = "Service with some Data";
	//Length of Wave Short Message
	int wsmLength;
	//Data of Wave Short Message
	string wsmData = "Some Data";

	MacAddress senderAddress = MacAddress::UNSPECIFIED_ADDRESS;
	MacAddress recipientAddress = MacAddress::UNSPECIFIED_ADDRESS;
	int serial = 0;
	simtime_t timestamp = 0;
}