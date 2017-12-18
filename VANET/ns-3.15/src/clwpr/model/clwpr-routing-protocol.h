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


#ifndef __CLWPR_AGENT_IMPL_H__
#define __CLWPR_AGENT_IMPL_H__

#include "clwpr-header.h"
#include "ns3/test.h"
#include "clwpr-state.h"
#include "clwpr-repositories.h"
#include "ns3/clwpr-map.h"
#include "clwpr-rqueue.h"
#include "ns3/snr-tag.h"
#include "deferred-tag.h"

#include "ns3/qos-tag.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/pointer.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4-address.h"
#include "ns3/vector.h" //for the Position and Velocity (Vector3D)
#include "ns3/node.h"
#include <vector>
#include <map>
#include <math.h>
#include <utility>
#include "ns3/node-list.h"

namespace ns3 {
namespace clwpr {


/// An %CLWPR's routing table entry.
struct RoutingTableEntry
{
  Ipv4Address destAddr;	///< Address of the destination node.
  Ipv4Address nextAddr;	///< Address of the next hop.
  uint32_t interface;
  double distance; ///< Distance in meters to the destination.
  double weight; ///< Output from the weighting function.
  
  RoutingTableEntry () : // default values
    destAddr (), nextAddr (),
    interface (0), distance (0), weight(0) {};
};



///
/// \ingroup clwpr
///
/// \brief CLWPR routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
//  friend class ClwprMprTestCase;
  static TypeId GetTypeId (void);

  RoutingProtocol ();
  virtual ~RoutingProtocol ();

  ///
  /// \brief Set the CLWPR main address to the first address on the indicated
  ///        interface 
  /// \param interface IPv4 interface index
  ///
  void SetMainInterface (uint32_t interface);

  /// 
  /// Dump the neighbor table, two-hop neighbor table, and routing table
  /// to logging output (NS_LOG_DEBUG log level).  If logging is disabled,
  /// this function does nothing.
  /// 
  void Dump (void);

  /**
   * Return the list of routing table entries discovered by CLWPR
   **/
  std::vector<RoutingTableEntry> GetRoutingTableEntries () const;

  void SetGridMap (Ptr<GridMap> map);


private:
  std::set<uint32_t> m_interfaceExclusions;
  Ptr<Ipv4StaticRouting> m_routingTableAssociation;

public:
  std::set<uint32_t> GetInterfaceExclusions () const
    {
      return m_interfaceExclusions;
    }
  void SetInterfaceExclusions (std::set<uint32_t> exceptions);

  /// Inject Association to be sent in HNA message
  void AddHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask);

  /// Inject Associations from an Ipv4StaticRouting instance
  void SetRoutingTableAssociation (Ptr<Ipv4StaticRouting> routingTable);
//  static void MacRxDrop(Ptr <const Packet> p);

  Time GetMaxQueueTime () const { return MaxQueueTime; }
  void SetMaxQueueTime (Time t);
  uint32_t GetMaxQueueLen () const { return MaxQueueLen; }
  void SetMaxQueueLen (uint32_t len);

  void PrintNeighbourTable (Ptr<OutputStreamWrapper> stream) const;
  void PrintNeighbourTableCompact (Ptr<OutputStreamWrapper> stream) const;
protected:
  virtual void DoStart (void);
private:
  // Need Multimap because for each Ipv4Adress multiple entries are added. 
  // For each destination EVERY neighbor gets an entry!!
  std::multimap<Ipv4Address, RoutingTableEntry> m_table; ///< Data structure for the routing table.

  Ptr<Ipv4StaticRouting> m_hnaRoutingTable;

  EventGarbageCollector m_events;

  /// Address of the routing agent.
  Ipv4Address m_routingAgentAddr;
	
  /// Packets sequence number counter.
  uint16_t m_packetSequenceNumber;
  /// Messages sequence number counter.
  uint16_t m_messageSequenceNumber;
  /// Advertised Neighbor Set sequence number.
  uint16_t m_ansn;
  
  /// HELLO messages' emission interval.
  Time m_helloInterval;

  /// HNA messages' emission interval.
  Time m_hnaInterval;

  /// Components Integration
  /// 1 : true, 0 : false
  bool m_mapFlag; // Use curvementric distance
  bool m_emapFlag; // Use navigation info
  bool m_predictFlag; // Use prediction
  bool m_cacheFlag; // Use carry-n-forward
  bool m_normFlag; // Used for testing weighting function
  bool m_ahpFlag; // Use AHP to calculate weights
  double m_txTh; // Used as a threshold to reduce communication range
  double temp_snr;

  /// Distance weighting Factor
  double m_Dfact;
  /// Angle weighting Factor
  double m_Afact;
  /// Utilization weighting Factor
  double m_Ufact;
  /// MAC information weighting Factor
  double m_Mfact;
  /// CnF weighting Factor
  double m_Cfact;
  /// SNR (Channel State) weighting Factor
  double m_Sfact;
  /// Road weighting Factor
  double m_Rfact;
  /// Hello Count factor
  double m_Hfact;


   /// Position of the node
   Vector m_position;
   /// Velocity of the node
   Vector m_velocity;
   /// Heading of the node
   uint8_t m_heading;
   /// Road Id of the node
   uint8_t m_roadId;
   /// MAC info of the node
   uint32_t m_macInfo;
   /// Utilization of the node
   uint32_t m_utilization;
   /// Number of Cached Packets
   uint16_t m_CnFinfo;

  /// Internal state with all needed data structs.
  ClwprState m_state;

  /// Internal class for the MAP manipulation
  Ptr<GridMap> m_map;

  Ptr<Ipv4> m_ipv4;
  void TestPosition();
  void Clear ();
  uint32_t GetSize () const { return m_table.size (); }
  void RemoveEntry (const Ipv4Address &dest);
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &next,
                 uint32_t interface,
                 double distance,
                 double weight);
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &next,
                 const Ipv4Address &interfaceAddress,
                 double distance,
                 double weight);
  bool Lookup (const Ipv4Address &dest,
               RoutingTableEntry &outEntry) const;
  bool FindSendEntry (const RoutingTableEntry &entry,
                      RoutingTableEntry &outEntry,
                      Ipv4Address const &dest,
                      Ipv4Address const &src);

  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
  virtual bool RouteInput  (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                             UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                             LocalDeliverCallback lcb, ErrorCallback ecb);  
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;


  void DoDispose ();

  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);
	
  /// Increments packet sequence number and returns the new value.
  inline uint16_t GetPacketSequenceNumber ();
  /// Increments message sequence number and returns the new value.
  inline uint16_t GetMessageSequenceNumber ();
	
  void RecvClwpr (Ptr<Socket> socket);

  void RoutingTableComputation ();


  Ipv4Address GetMainAddress (Ipv4Address iface_addr) const;
  
  Vector GetNodePosition (Ptr<Ipv4> ipv4);
  Vector GetNodeVelocity (Ptr<Ipv4> ipv4);
  double GetNodeHeading (Ptr<Ipv4> ipv4);
  uint8_t GetNodeRoadId (Ptr<Ipv4> ipv4);
  uint32_t GetNodeUtilization (Ptr<Ipv4> ipv4);
  uint32_t GetNodeMacInfo (Ptr<Ipv4> ipv4);
  uint16_t GetNodeCnFInfo (Ptr<Ipv4> ipv4);
  

  void CalculateNodeNormUtil ();
  uint32_t GetNodeNormUtil(void);
  uint64_t m_macTxTemp;
  
  /// Calculates the weight of a neighbouring node towards the destination
  double WeightingFunction(NeighborTuple const &nb_tuple, PosAssociationTuple const &ds_tuple);
  /// Calculates the weight of 'me' towards the destination
  double WeightingFunction(PosAssociationTuple const &ds_tuple);

  /// Weighted SRN value
  double SNRFunction(double n_SNIR);

  /// Weighted Hello Count value
  double HelloFunction(int count);

  /// Calculates the distance between two nodes
  // If 'map-flag' is true then 'curvemetric' distance is calculated
  // For courent node
  double DistanceComputation(PosAssociationTuple const &pos_tuple);
  // For a neighbour
  double DistanceComputation(NeighborTuple const &nb_tuple, PosAssociationTuple const &pos_tuple);

//  Vector PredictPosition(Vector pos, Vector vel, double heading, Time t);
  Vector PredictPositionGeneric(Vector pos, Vector vel, double heading, Time t);

  /// Calculates the angle between two nodes' heading
  /// @param pos_a The position of first node
  /// @param heading_a the heading of first node (0-360)
  /// @param vel_a the velocity of first node
  /// @param pos_b The position of second node
  /// return a value {-1,1} : -1 approaching, 1 moving away (assuming that node B is static)
  double AngleComputation (Vector pos_a, double heading_a, Vector vel_a, Vector pos_b);
  double AngleComputationGeneric(Vector pos_a, double heading_a, Vector vel_a, Vector pos_b);

  // Timer handlers
  Timer m_helloTimer;
  void HelloTimerExpire ();

  Timer m_hnaTimer;
  void HnaTimerExpire ();

  Timer m_utilTimer;
  void UtilTimerExpire();

  void DupTupleTimerExpire (Ipv4Address address, uint16_t sequenceNumber);
  void NeighborTupleTimerExpire (Ipv4Address neighborMainAddr);
  void IfaceAssocTupleTimerExpire (Ipv4Address ifaceAddr);
  void AssociationTupleTimerExpire (Ipv4Address gatewayAddr, Ipv4Address networkAddr, Ipv4Mask netmask);

  void IncrementAnsn ();

  /// A list of pending messages which are buffered awaiting for being sent.
  clwpr::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; // timer for throttling outgoing messages

  void ForwardDefault (clwpr::MessageHeader clwprMessage,
                       DuplicateTuple *duplicated,
                       const Ipv4Address &localIface,
                       const Ipv4Address &senderAddress);
  void QueueMessage (const clwpr::MessageHeader &message, Time delay);
  void SendQueuedMessages ();
  void SendHello ();
  void SendHna ();

  void AddDuplicateTuple (const DuplicateTuple &tuple);
  void RemoveDuplicateTuple (const DuplicateTuple &tuple);
  void AddNeighborTuple (const NeighborTuple &tuple);
  void RemoveNeighborTuple (const NeighborTuple &tuple);
  void AddIfaceAssocTuple (const IfaceAssocTuple &tuple);
  void RemoveIfaceAssocTuple (const IfaceAssocTuple &tuple);
  void AddAssociationTuple (const AssociationTuple &tuple);
  void RemoveAssociationTuple (const AssociationTuple &tuple);

  void AddPosAssociationTuple (const PosAssociationTuple &tuple);
  void RemovePosAssociationTuple (const PosAssociationTuple &tuple);


  void ProcessHello (const clwpr::MessageHeader &msg,
                     const Ipv4Address &receiverIface,
                     const Ipv4Address &senderIface);
  void ProcessHna (const clwpr::MessageHeader &msg,
                   const Ipv4Address &senderIface);

  void PopulateNeighborSet (const clwpr::MessageHeader &msg,
                            const clwpr::MessageHeader::Hello &hello,
                            const Ipv4Address &iface);
                            

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;

  Ipv4Address m_mainAddress;

  // One socket per interface, each bound to that interface's address
  // (reason: for CLWPR Link Sensing we need to know on which interface
  // HELLO messages arrive)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;

  TracedCallback <const PacketHeader &,
                  const MessageList &> m_rxPacketTrace;
  TracedCallback <const PacketHeader &,
                  const MessageList &> m_txPacketTrace;
  TracedCallback <uint32_t> m_routingTableChanged;

  TracedCallback <const Ipv4Header &,
                  Ptr<const Packet> > m_queueTrace;
  TracedCallback <const Ipv4Header &,
                  Ptr<const Packet> > m_dequeueTrace;

  TracedCallback <const Ipv4Header &,
                  Ptr<const Packet>,
                  Ptr<Ipv4> > m_dropCnFTrace;

//  void IncreaseMacInfo();
  Ptr <Node> m_node;

  std::map <Ipv4Address, Ptr<Ipv4> > m_nodeList;
  void InitNodeList();
  bool m_init_list;
  void LocService(Ipv4Address dest);

  /// CLWPR Packets Qos Tag == BK
  QosTag m_qosTag;

  ///< The maximum number of packets that we allow a routing protocol to buffer.
  uint32_t MaxQueueLen;
  ///< The maximum period of time that a routing protocol is allowed to buffer a packet for.
  Time MaxQueueTime;
  /// A "drop-front" queue used by the routing layer to buffer packets to which it does not have a route.
  RequestQueue m_queue;

  /// Create loopback route for given header
  Ptr<Ipv4Route> LoopbackRoute (const Ipv4Header & header, Ptr<NetDevice> oif) const;
  /// Queue packet and send route request
  void DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header & header, UnicastForwardCallback ucb, ErrorCallback ecb);
  void SendPacketFromQueue (Ipv4Address dst, Ptr<Ipv4Route> route);

};

}} // namespace ns3

#endif
