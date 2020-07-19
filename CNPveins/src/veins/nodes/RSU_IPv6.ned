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

package org.car2x.veins.nodes;

import inet.node.ipv6.Router6;
import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;

module RSU_IPv6 extends Router6
{
    parameters:
        hasTcp = true;
        hasUdp = true;
		
        @figure[submodules];
        string nicType = default("Nic80211p"); // type of network interface card
        numApps = 1;
        string applType; //type of the application layer
//        string veinsmobilityType = default("org.car2x.veins.modules.mobility.traci.TraCIMobility");


    gates:
        input veinsradioIn; // gate for sendDirect     

    submodules:

        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=660,555");
        }

        mobility: BaseMobility {
            parameters:
                @display("p=124.424995,160.65;i=block/cogwheel");
        }

//        appl: <applType> like org.car2x.veins.base.modules.IBaseApplLayer {
//            parameters:
//                @display("p=996.975,349.65");
//        }


    connections:
        veinsradioIn --> nic.radioIn;
        nic.upperLayerOut --> nl.in++;
        nic.upperLayerIn <-- nl.out++;
        nic.upperControlOut --> nl.in++;
        nic.upperControlIn <-- nl.out++;

//        nl.out++ --> appl.lowerLayerIn;
//        nl.in++ <-- appl.lowerLayerOut ;
//        nl.out++ --> appl.lowerControlIn ;
//        nl.in++ <-- appl.lowerControlOut;


}
