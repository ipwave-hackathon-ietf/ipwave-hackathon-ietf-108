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

package org.car2x.veins.modules.linklayer;



import inet.common.packet.recorder.PcapRecorder;
import inet.linklayer.contract.IEthernetInterface;
import inet.linklayer.contract.IExternalInterface;
import inet.linklayer.contract.ILoopbackInterface;
import inet.linklayer.contract.IPppInterface;
import inet.linklayer.contract.ITunnelInterface;
import inet.linklayer.contract.IWirelessInterface;
import inet.networklayer.common.InterfaceTable;


module linkLayer
{
    parameters:
//        string nicType = default("Nic80211p"); // type of network interface card
        @networkNode();
        int numLoInterfaces = default(1);
        @display("p=542,277");
        *.interfaceTableModule = default(absPath(".interfaceTable"));

    gates:
//        input veinsradioIn @directIn; // gate for sendDirect


    submodules:
        interfaceTable: InterfaceTable {
            @display("p=228,235;is=s");
        }
        
        lo[numLoInterfaces]: <default("LoopbackInterface")> like ILoopbackInterface {
            parameters:
                @display("p=750,526,row,150");
        }

//        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
//            parameters:
//                @display("p=660,555");
//        }

connections allowunconnected:    
    
//      veinsradioIn --> nic.radioIn;




}
