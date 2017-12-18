/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Konstantinos Katsaros
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Konstantinos Katsaros <K.Katsaros@surrey.ac.uk>
 */

///
/// \file	clwpr-repositories.h
/// \brief	Here are defined all data structures needed by an CLWPR node.
///

#ifndef __CLWPR_REPOSITORIES_H__
#define __CLWPR_REPOSITORIES_H__

#include <set>
#include <vector>
#include "ns3/vector.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"

#include "ns3/mobility-helper.h"
#include "ns3/vector.h"

namespace ns3 { namespace clwpr {



/// An Interface Association Tuple.
struct IfaceAssocTuple
{
  /// Interface address of a node.
  Ipv4Address ifaceAddr;
  /// Main address of the node.
  Ipv4Address mainAddr;
  /// Time at which this tuple expires and must be removed.
  Time time;
};

static inline bool
operator == (const IfaceAssocTuple &a, const IfaceAssocTuple &b)
{
  return (a.ifaceAddr == b.ifaceAddr
          && a.mainAddr == b.mainAddr);
}

static inline std::ostream&
operator << (std::ostream &os, const IfaceAssocTuple &tuple)
{
  os << "IfaceAssocTuple(ifaceAddr=" << tuple.ifaceAddr
     << ", mainAddr=" << tuple.mainAddr
     << ", time=" << tuple.time << ")";
  return os;
}


// Not really used in CLWPR -- legacy from OLSR
/// A Link Tuple.
struct LinkTuple
{
  /// Interface address of the local node.
  Ipv4Address localIfaceAddr;
  /// Interface address of the neighbor node.
  Ipv4Address neighborIfaceAddr;
  /// The link is considered bidirectional until this time.
  Time symTime;
  /// The link is considered unidirectional until this time.
  Time asymTime;
  /// Time at which this tuple expires and must be removed.
  Time time;
};

static inline bool
operator == (const LinkTuple &a, const LinkTuple &b)
{
  return (a.localIfaceAddr == b.localIfaceAddr
          && a.neighborIfaceAddr == b.neighborIfaceAddr);
}

static inline std::ostream&
operator << (std::ostream &os, const LinkTuple &tuple)
{
  os << "LinkTuple(localIfaceAddr=" << tuple.localIfaceAddr
     << ", neighborIfaceAddr=" << tuple.neighborIfaceAddr
     << ", symTime=" << tuple.symTime
     << ", asymTime=" << tuple.asymTime
     << ", expTime=" << tuple.time
     << ")";
  return os;
}

/// A Neighbor Tuple.
struct NeighborTuple
{
  /// Main address of a neighbor node.
  Ipv4Address neighborMainAddr;
  /// Neighbor Interface
  uint32_t neighborInterface; ///< Interface index.
//  Ipv4Address neighborInterface;
  
  /// Neighbor Type and Link Type at the four less significative digits.
  enum Status {
    STATUS_NOT_SYM = 0, // "not symmetric"
    STATUS_SYM = 1, // "symmetric"
  } status;
  /// Neighbor Position (x,y,z)
  Vector neighborPosition;
  /// Neighbor Velocity (x,y,z)
  Vector neighborVelocity;
  /// Neighor Heading in degrees
  double neighborHeading;
  /// Neighbor's Road ID
  uint8_t neighborRoadId;
  /// Neighbor's utilization as in queue length
  uint32_t neighborUtilization;
  /// Neighbor's MAC layer information
  uint32_t neighborMacInfo;
  /// Neighbor's SNR
  double neighborSNR;
  /// Neighbour's CnF
  uint16_t neighborCnF;
  /// Timestamp of this tuple
  Time neighborTimestamp;
  Time neighborExpireTime;

  ///This is a counter of the consecutive hello messages
  /// used only for statistics at the moment.
  int neighborHelloCount;
};

static inline bool
operator == (const NeighborTuple &a, const NeighborTuple &b)
{
  return (a.neighborMainAddr == b.neighborMainAddr);
}

static inline std::ostream&
operator << (std::ostream &os, const NeighborTuple &tuple)
{
  os << "NeighborTuple(neighborMainAddr=" << tuple.neighborMainAddr
     << ", neighbor Interface=" << tuple.neighborInterface
     << ", Position = (" << tuple.neighborPosition.x
     << ", "<< tuple.neighborPosition.y << ")"
     << ", Velocity = (" << tuple.neighborVelocity.x
     << ", "<< tuple.neighborVelocity.y << ")"
     << ", Heading=" << tuple.neighborHeading
     << ", road ID=" <<  (int) tuple.neighborRoadId
     << ", Utilization =" << (int) tuple.neighborUtilization
     << ", MAC info =" << (int) tuple.neighborMacInfo
     << ", SNR = " << tuple.neighborSNR
     << ", CnF = " << tuple.neighborCnF
     << ", Expiration Time=" <<  tuple.neighborExpireTime ;

  return os;
}


/// The type "list of interface addresses"
//typedef std::vector<nsaddr_t> addr_list_t;

/// A Duplicate Tuple
struct DuplicateTuple
{
  /// Originator address of the message.
  Ipv4Address address;
  /// Message sequence number.
  uint16_t sequenceNumber;
  /// Indicates whether the message has been retransmitted or not.
  bool retransmitted;
  /// List of interfaces which the message has been received on.
  std::vector<Ipv4Address> ifaceList;
  /// Time at which this tuple expires and must be removed.
  Time expirationTime;
};

static inline bool
operator == (const DuplicateTuple &a, const DuplicateTuple &b)
{
  return (a.address == b.address
          && a.sequenceNumber == b.sequenceNumber);
}


/// Association
struct Association
{
  Ipv4Address networkAddr;
  Ipv4Mask netmask;
};

static inline bool
operator == (const Association &a, const Association &b)
{
  return (a.networkAddr == b.networkAddr
          && a.netmask == b.netmask);
}

static inline std::ostream&
operator << (std::ostream &os, const Association &tuple)
{
  os << "Association(networkAddr=" << tuple.networkAddr
     << ", netmask=" << tuple.netmask
     << ")";
  return os;
}

/// An Association Tuple
struct AssociationTuple
{
  /// Main address of the gateway.
  Ipv4Address gatewayAddr;
  /// Network Address of network reachable through gatewayAddr
  Ipv4Address networkAddr;
  /// Netmask of network reachable through gatewayAddr
  Ipv4Mask netmask;
  /// Time at which this tuple expires and must be removed
  Time expirationTime;
};

static inline bool
operator == (const AssociationTuple &a, const AssociationTuple &b)
{
  return (a.gatewayAddr == b.gatewayAddr
          && a.networkAddr == b.networkAddr
          && a.netmask == b.netmask);
}

static inline std::ostream&
operator << (std::ostream &os, const AssociationTuple &tuple)
{
  os << "AssociationTuple(gatewayAddr=" << tuple.gatewayAddr
     << ", networkAddr=" << tuple.networkAddr
     << ", netmask=" << tuple.netmask
     << ", expirationTime=" << tuple.expirationTime
     << ")";
  return os;
}

/// A Position Association Tuple
/// Here we keep a track of the destinations with their latest position information
// It is updated with hellos or with the location service when integrate one

struct PosAssociationTuple
{
  /// Main address of the node.
  Ipv4Address nodeMainAddr;
  /// The position of the node.
  Vector nodePosition;
  /// Node Velocity (x,y,z)
  Vector nodeVelocity;
  /// Neighor Heading in degrees
  double nodeHeading;
  /// Neighbor's Road ID
  uint8_t nodeRoadId;
  /// Timestamp
  Time nodeTimestamp;
};


static inline bool
operator == (const PosAssociationTuple &a, const PosAssociationTuple &b)
{
  return (a.nodeMainAddr == b.nodeMainAddr);
}

static inline std::ostream&
operator << (std::ostream &os, const PosAssociationTuple &tuple)
{
  os << "PositionAssociationTuple(nodeMainAddr=" << tuple.nodeMainAddr
     << ", position = (" << tuple.nodePosition.x 
     << ", "<< tuple.nodePosition.y 
     << ", "<< tuple.nodePosition.z << ")"
     << ", velocity = (" << tuple.nodeVelocity.x 
     << ", "<< tuple.nodeVelocity.y 
     << ", "<< tuple.nodeVelocity.z << ")"
     << ", heading=" << tuple.nodeHeading
     << ", road ID = " << tuple.nodeRoadId     
     << ", node Timestamp=" << tuple.nodeTimestamp
     << ")";
  return os;
}


typedef std::vector<NeighborTuple>		NeighborSet;	///< Neighbor Set type.
typedef std::vector<DuplicateTuple>		DuplicateSet;	///< Duplicate Set type.
typedef std::vector<IfaceAssocTuple>		IfaceAssocSet; ///< Interface Association Set type.
typedef std::vector<AssociationTuple>		AssociationSet; ///< Association Set type.
typedef std::vector<Association>		Associations; ///< Association Set type.
typedef std::vector<PosAssociationTuple>		PosAssociationSet; ///< IP - Position Association Set type.

}}; // namespace ns3, clwpr

#endif  /* __CLWPR_REPOSITORIES_H__ */
