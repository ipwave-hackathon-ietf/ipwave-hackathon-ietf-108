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

import org.car2x.veins.nodes.CNPRSU;
import org.car2x.veins.nodes.CNPScenario;
import org.car2x.veins.nodes.MobilityAnchor_IPv6;

import inet.node.ipv6.StandardHost6;



network CNPNet extends CNPScenario
{
    parameters:
    int n;
    
    submodules:
        rsu[n]: CNPRSU {
            @display("p=150,140;i=veins/sign/yellowdiamond;is=vs");
        }
        
		MA[1]: MobilityAnchor_IPv6 {
            @display("p=450.36751,41.445;i=veins/sign/yellowdiamond;is=vs");
        }
	connections allowunconnected:
//	    MA[0].ethg$o++ --> ethernetline --> rsu[0].ethg$i++;
//        MA[0].ethg$i++ <-- ethernetline <-- rsu[0].ethg$o++;

        for i=0..n-1 {
            MA[0].ethg$o++ --> ethernetline --> rsu[i].ethg$i++;
            MA[0].ethg$i++ <-- ethernetline <-- rsu[i].ethg$o++;
        }
        rsu[0].ethg$i++ <-- ethernetline <-- rsu[1].ethg$o++;
        rsu[0].ethg$o++ --> ethernetline --> rsu[1].ethg$i++;
	 	rsu[1].ethg$i++ <-- ethernetline <-- rsu[2].ethg$o++;
        rsu[1].ethg$o++ --> ethernetline --> rsu[2].ethg$i++;
}