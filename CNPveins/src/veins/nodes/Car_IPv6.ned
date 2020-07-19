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

import inet.node.ipv6.StandardHost6;
import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;

module Car_IPv6 extends StandardHost6
{
    parameters:       
        hasTcp = true;
        forwarding = true;
        isRouter = false;
        string applType; //type of the application layer
        bool hasAppl = default(false);

        @figure[submodules];
        string nicType = default("Nic80211p"); // type of network interface card
        numApps = 1;
        string veinsmobilityType = default("org.car2x.veins.modules.mobility.traci.TraCIMobility");
        
        //XIANG ZHONG 19.09.05
        ipv6.xMIPv6Support = true;
        ipv6.mipv6support.isMobileNode = true; //is Mobile Node
    	ipv6.mipv6support.isHomeAgent = false; //is Home Agent


        @display("bgb=1376.5499,778.05");
    gates:
        input veinsradioIn @directIn; // gate for sendDirect     

    submodules:
//        appl: <applType> like org.car2x.veins.base.modules.IBaseApplLayer {
//            parameters:
//                @display("p=1124.5499,349.65");
//        }

        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=634.725,559.125");
        }
        veinsmobility: <veinsmobilityType> like org.car2x.veins.base.modules.IMobility {
            parameters:
                @display("p=124.424995,163.79999;i=block/cogwheel");
        }



    connections:
        veinsradioIn --> nic.radioIn;

        nic.upperLayerOut --> nl.in++;
        nic.upperLayerIn <-- nl.out++;
        nic.upperControlOut --> nl.in++;
        nic.upperControlIn <-- nl.out++;

//        nl.out++ --> appl.lowerLayerIn if hasAppl;
//        nl.in++ <-- appl.lowerLayerOut if hasAppl;
//        nl.out++ --> appl.lowerControlIn if hasAppl;
//        nl.in++ <-- appl.lowerControlOut if hasAppl;


}
