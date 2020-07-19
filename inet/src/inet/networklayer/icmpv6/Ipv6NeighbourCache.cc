/**
 * Copyright (C) 2005 Andras Varga
 * Copyright (C) 2005 Wei Yang, Ng
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "inet/networklayer/icmpv6/Ipv6NeighbourCache.h"

namespace inet {

void Ipv6NeighbourCache::DefaultRouterList::add(Neighbour& router)
{
    ASSERT(router.isRouter);
    ASSERT(!router.nextDefaultRouter);
    ASSERT(!router.prevDefaultRouter);

    if (head) {
        // insert before head
        head->prevDefaultRouter->nextDefaultRouter = &router;
        router.prevDefaultRouter = head;
        head->prevDefaultRouter = &router;
        router.nextDefaultRouter = head;
    }
    else {
        head = router.nextDefaultRouter = router.prevDefaultRouter = &router;
    }
}

void Ipv6NeighbourCache::DefaultRouterList::remove(Neighbour& router)
{
    ASSERT(router.isDefaultRouter());
    if (router.nextDefaultRouter == &router) {
        head = nullptr;
    }
    else {
        if (head == &router)
            head = router.nextDefaultRouter;
        router.nextDefaultRouter->prevDefaultRouter = router.prevDefaultRouter;
        router.prevDefaultRouter->nextDefaultRouter = router.nextDefaultRouter;
    }
    router.nextDefaultRouter = router.prevDefaultRouter = nullptr;
}

std::ostream& operator<<(std::ostream& os, const Ipv6NeighbourCache::Key& e)
{
    return os << "if=" << e.interfaceID << " " << e.address;    //FIXME try printing interface name
}

std::ostream& operator<<(std::ostream& os, const Ipv6NeighbourCache::Neighbour& e)
{
    os << e.macAddress;
    if (e.isRouter)
        os << " ROUTER";
    if (e.isDefaultRouter())
        os << "DefaultRtr";
    if (e.isHomeAgent)
        os << " Home Agent";
    os << " " << Ipv6NeighbourCache::stateName(e.reachabilityState);
    os << " reachabilityExp:" << e.reachabilityExpires;
    os << " registerTime: " << e.registLifeTime;
    //os << " linkLocalAddr: " << e.linkLocalAddr;
    os << " relayAddr: " << e.relayAddr;
    os << " globalAddr: " << e.globalAddr;
    os << " hopDistance: " << e.hopDistance;
    if (e.numProbesSent)
        os << " probesSent:" << e.numProbesSent;
    if (e.isRouter)
        os << " rtrExp:" << e.routerExpiryTime;
    return os;
}


Ipv6NeighbourCache::Ipv6NeighbourCache(cSimpleModule& neighbourDiscovery)
    : neighbourDiscovery(neighbourDiscovery)
{
    WATCH_MAP(neighbourMap);
    WATCH_MAP(multihopDadMap);
    WATCH_MAP(neighborRouteMap);

}

Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::lookup(const Ipv6Address& addr, int interfaceID)
{
    Key key(addr, interfaceID);
    auto i = neighbourMap.find(key);
    return i == neighbourMap.end() ? nullptr : &(i->second);
}

const Ipv6NeighbourCache::Key *Ipv6NeighbourCache::lookupKeyAddr(Key& key)
{
    auto i = neighbourMap.find(key);
    return &(i->first);
}

Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addNeighbour(const Ipv6Address& addr, int interfaceID)
{
    Key key(addr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];

    nbor.nceKey = lookupKeyAddr(key);
    nbor.isRouter = false;
    nbor.isHomeAgent = false;
    nbor.reachabilityState = INCOMPLETE;
    return &nbor;
}


//XIANG ZHONG
Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addMultihopNCE(const Ipv6Address& addr, int interfaceID)
{
    //XIANG ZHONG 2020.01.03
    //Set the expire time to a large number in MA since vehicles will just do address registration to serving RSU
    Key key(addr, interfaceID);
    ASSERT(multihopDadMap.find(key) == multihopDadMap.end());    // entry must not exist yet
    Neighbour& nbor = multihopDadMap[key];
    nbor.registLifeTime = simTime()+500;
    nbor.reachabilityExpires = simTime()+500;
    nbor.nceKey = lookupKeyAddr(key);
    nbor.isRouter = false;
    nbor.isHomeAgent = false;
    nbor.reachabilityState = REACHABLE;
    return &nbor;
}

Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::lookupDadTable(const Ipv6Address& addr, int interfaceID)
{
    Key entry(addr, interfaceID);
    auto i = multihopDadMap.find(entry);
    return i == multihopDadMap.end() ? nullptr : &(i->second);
}



Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addTentativeNeighbour(const Ipv6Address& globalAddr, const Ipv6Address& linkLocalAddr, int interfaceID, MacAddress macAddress, short registLifetime)
{
    Key key(globalAddr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];
    nbor.registLifeTime = simTime() + registLifetime;
    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.isRouter = false;
    nbor.isHomeAgent = false;
    nbor.reachabilityState = TENTATIVE;
    nbor.reachabilityExpires = simTime() + registLifetime;
    nbor.linkLocalAddr = linkLocalAddr;
    return &nbor;
}

Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addRelayNeighbour(const Ipv6Address& globalAddr,const Ipv6Address& linkLocalAddr, int interfaceID, MacAddress macAddress, short registLifetime, const Ipv6Address& relayAddr)
{
    Key key(globalAddr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];
    nbor.registLifeTime = simTime() + registLifetime;
    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.isRouter = false;
    nbor.isHomeAgent = false;
    nbor.reachabilityState = REACHABLE;
    nbor.relayAddr = relayAddr;
    nbor.reachabilityExpires = simTime() + registLifetime;
    nbor.linkLocalAddr = linkLocalAddr;
    return &nbor;


}


Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addNeighbour(const Ipv6Address& addr, int interfaceID, MacAddress macAddress)
{
    Key key(addr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];

    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.isRouter = false;
    nbor.isHomeAgent = false;
    nbor.reachabilityState = STALE;
    return &nbor;
}

/**
 * Creates and initializes a router entry (isRouter=isDefaultRouter=true), MAC address and state=STALE.
 *
 * Update by CB: Added an optional parameter which is false by default. Specifies whether a router is also a home agent.
 */
Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addRouter(const Ipv6Address& addr,
        int interfaceID, MacAddress macAddress, simtime_t expiryTime, bool isHomeAgent)
{
    Key key(addr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];

    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.isRouter = true;
    nbor.isHomeAgent = isHomeAgent;
    //XIANG ZHONG 19.12.21
//    nbor.reachabilityState = STALE;
    nbor.reachabilityState = REACHABLE;
    nbor.routerExpiryTime = expiryTime;
    defaultRouterList.add(nbor);

    return &nbor;
}

//XIANG ZHONG 19.05.20
Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addRouterWithGlobalAddr(const Ipv6Address& addr,
        int interfaceID, MacAddress macAddress, simtime_t expiryTime, const Ipv6Address& globalAddr, bool isHomeAgent)
{
    Key key(addr, interfaceID);
    ASSERT(neighbourMap.find(key) == neighbourMap.end());    // entry must not exist yet
    Neighbour& nbor = neighbourMap[key];

    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.isRouter = true;
    nbor.isHomeAgent = isHomeAgent;
    //XIANG ZHONG 19.12.16
//    nbor.reachabilityState = STALE;
    nbor.reachabilityState = REACHABLE;
    nbor.routerExpiryTime = expiryTime;
    nbor.globalAddr = globalAddr;

    //XIANG ZHONG 19.06.11
    nbor.reachabilityExpires = expiryTime;
    defaultRouterList.add(nbor);

    return &nbor;
}

//XIANG ZHONG 19.06.11
Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::addNeighborRoute (const Ipv6Address &addr, int interfaceID, MacAddress macAddress,
        const Ipv6Address& relayAddr, int hopDistance)
{
    Key key(addr, interfaceID);
    ASSERT(neighborRouteMap.find(key) == neighborRouteMap.end());    // entry must not exist yet
    Neighbour& nbor = neighborRouteMap[key];
    nbor.reachabilityState = REACHABLE;
    nbor.nceKey = lookupKeyAddr(key);
    nbor.macAddress = macAddress;
    nbor.relayAddr = relayAddr;
    nbor.hopDistance = hopDistance;

    return &nbor;

}

//XIANG ZHONG 19.06.11
Ipv6NeighbourCache::Neighbour *Ipv6NeighbourCache::lookupNeighborRoute(const Ipv6Address& addr, int interfaceID)
{
    Key key(addr, interfaceID);
    auto i = neighborRouteMap.find(key);
    return i == neighborRouteMap.end() ? nullptr : &(i->second);
}



void Ipv6NeighbourCache::remove(const Ipv6Address& addr, int interfaceID)
{
    Key key(addr, interfaceID);
    auto it = neighbourMap.find(key);
    ASSERT(it != neighbourMap.end());    // entry must exist
    remove(it);
}



//XIANG ZHONG
void Ipv6NeighbourCache::removeFromDadTable(const Ipv6Address& addr, int interfaceID)
{
    Key key(addr, interfaceID);
    auto it = multihopDadMap.find(key);
    ASSERT(it != multihopDadMap.end());    // entry must exist
//    remove(it);
    multihopDadMap.erase(it);
}



void Ipv6NeighbourCache::remove(NeighbourMap::iterator it)
{
    neighbourDiscovery.cancelAndDelete(it->second.nudTimeoutEvent);    // 20.9.07 - CB
    it->second.nudTimeoutEvent = nullptr;
    if (it->second.isDefaultRouter())
        defaultRouterList.remove(it->second);
    neighbourMap.erase(it);
}




// Added by CB
void Ipv6NeighbourCache::invalidateEntriesForInterfaceID(int interfaceID)
{
    for (auto & elem : neighbourMap) {
        if (elem.first.interfaceID == interfaceID) {
            elem.second.reachabilityState = PROBE;    // we make sure this neighbour is not used anymore in the future, unless reachability can be confirmed
            neighbourDiscovery.cancelAndDelete(elem.second.nudTimeoutEvent);    // 20.9.07 - CB
            elem.second.nudTimeoutEvent = nullptr;
        }
    }
}

// Added by CB
void Ipv6NeighbourCache::invalidateAllEntries()
{
    while (!neighbourMap.empty()) {
        auto it = neighbourMap.begin();
        remove(it);
    }
    defaultRouterList.clear();

    /*
       int size = neighbourMap.size();
       EV << "size: " << size << endl;
       for (auto it = neighbourMap.begin(); it != neighbourMap.end(); it++)
       {
        it->second.reachabilityState = PROBE; // we make sure this neighbour is not used anymore in the future, unless reachability can be confirmed
       }
     */
}

const char *Ipv6NeighbourCache::stateName(ReachabilityState state)
{
    switch (state) {
        case INCOMPLETE:
            return "INCOMPLETE";

        case REACHABLE:
            return "REACHABLE";

        case STALE:
            return "STALE";

        case DELAY:
            return "DELAY";

        case PROBE:
            return "PROBE";

        case TENTATIVE:
            return "TENTATIVE";

        case REGISTERED:
            return "REGISTERED";
        case GARBAGE:
            return "GARBAGE";

        default:
            return "???";
    }
}

} // namespace inet

