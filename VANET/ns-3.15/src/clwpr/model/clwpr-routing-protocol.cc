/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) Konstantinos Katsaros
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
/// \file       CLWPR.cc
/// \brief      Implementation of CLWPR agent and related classes.
///
/// This is the main file of this software because %CLWPR's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "clwpr-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/random-variable.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"
#include "ns3/vector.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/boolean.h"

#include "ns3/ipv4-address.h"
#include "ns3/callback.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/config.h"
#include "ns3/pointer.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-mac-queue.h"
#include <map>
#include <utility>
#include <iostream>
#include <sstream>
#include <cmath>
#include <iomanip> // used for formatting output
/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))

#define DELAY_FLAG 0
//#define MAP_FLAG 0
///
/// \brief Period at which a node must cite every link and every neighbor.
///
/// We only use this value in order to define CLWPR_NEIGHB_HOLD_TIME.
///
//#define CLWPR_REFRESH_INTERVAL        Seconds (5)


/********** Holding times **********/

/// Neighbor holding time.
#define CLWPR_NEIGHB_HOLD_TIME  Time(2.5 * m_helloInterval)
//#define CLWPR_NEIGHB_HOLD_TIME        Seconds (5)
/// Dup holding time.
#define CLWPR_DUP_HOLD_TIME     Seconds (30)
/// HNA holding time. -- not currently used
#define CLWPR_HNA_HOLD_TIME  Time(3.5 * m_hnaInterval)

/********** Link types -- not really used **********/

/// Unspecified link type.
#define CLWPR_UNSPEC_LINK       0
/// Asymmetric link type.
#define CLWPR_ASYM_LINK         1
/// Symmetric link type.
#define CLWPR_SYM_LINK          2
/// Lost link type.
#define CLWPR_LOST_LINK         3

/********** Neighbor types **********/

/// Not neighbor type.
#define CLWPR_NOT_NEIGH         0
/// Symmetric neighbor type.
#define CLWPR_SYM_NEIGH         1
/// Asymmetric neighbor type.
#define CLWPR_MPR_NEIGH         2


/********** Miscellaneous constants **********/

/// Velocity Error
#define VEL_ERR         0.001

/// SNR LIMIT -- For weighted SNIR function. Depends on propagation model
#define SNIRmin			9.75
#define PREF_SNIR        15
#define ALPHA           -0.03628
#define BETA            -5.25

/// Max Vehicle Speed for the road
///--> Should come from navigation
#define MAX_Vel			50

/// Maximum allowed jitter.
#define CLWPR_MAXJITTER         (m_helloInterval.GetSeconds () / 4)
/// Maximum allowed sequence number.
#define CLWPR_MAX_SEQ_NUM       65535
/// Random number between [0-CLWPR_MAXJITTER] used to jitter CLWPR packet transmission.
#define JITTER (Seconds (UniformVariable().GetValue (0, CLWPR_MAXJITTER)))


#define CLWPR_PORT_NUMBER 2011
/// Maximum number of messages per packet.
#define CLWPR_MAX_MSGS          64

/// Maximum number of hellos per message. Only ONE.
#define CLWPR_MAX_HELLOS                1

/// Maximum number of addresses advertised on a message.
#define CLWPR_MAX_ADDRS         64


namespace ns3 {
namespace clwpr {

NS_LOG_COMPONENT_DEFINE ("ClwprRoutingProtocol");

/********** CLWPR class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId 
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::clwpr::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
                   TimeValue (Seconds (1.5)),
                   MakeTimeAccessor (&RoutingProtocol::m_helloInterval),
                   MakeTimeChecker ())
    .AddAttribute ("HnaInterval", "HNA messages emission interval.",
                   TimeValue (Seconds (5)),
                   MakeTimeAccessor (&RoutingProtocol::m_hnaInterval),
                   MakeTimeChecker ())
    .AddAttribute ("MapFlag","Activate Map Integration",
                   BooleanValue(false),
                   MakeBooleanAccessor(&RoutingProtocol::m_mapFlag),
                   MakeBooleanChecker())
    .AddAttribute ("EMapFlag","Enchanted Map Integration",
                   BooleanValue(false),
                   MakeBooleanAccessor(&RoutingProtocol::m_emapFlag),
                   MakeBooleanChecker())
    .AddAttribute ("TxThreshold","Max Tx Range for neighbour selection",
                   DoubleValue(500),
                   MakeDoubleAccessor(&RoutingProtocol::m_txTh),
                   MakeDoubleChecker<double>())
    .AddAttribute ("PredictFlag","Activate Position Prediction",
                    BooleanValue(false),
                    MakeBooleanAccessor(&RoutingProtocol::m_predictFlag),
                    MakeBooleanChecker())
    .AddAttribute ("CacheFlag","Activate Carry'n'Forward ",
                    BooleanValue(false),
                    MakeBooleanAccessor(&RoutingProtocol::m_cacheFlag),
                    MakeBooleanChecker())
    .AddAttribute ("NormFlag","Normalize the Distance ",
                   BooleanValue(false),
                   MakeBooleanAccessor(&RoutingProtocol::m_normFlag),
                   MakeBooleanChecker())
    .AddAttribute ("MaxQueueLen", "Maximum number of packets that we allow a routing protocol to buffer.",
                   UintegerValue (50),
                   MakeUintegerAccessor (&RoutingProtocol::SetMaxQueueLen,
                                         &RoutingProtocol::GetMaxQueueLen),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("MaxQueueTime", "Maximum time packets can be queued (in seconds)",
                   TimeValue (Seconds (10)),
                   MakeTimeAccessor (&RoutingProtocol::SetMaxQueueTime,
                                     &RoutingProtocol::GetMaxQueueTime),
                   MakeTimeChecker ())
    .AddAttribute ("DistFact","Distance Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Dfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("AngFact","Angle Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Afact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("UtilFact","Utilazation (Tier 2) Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Ufact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("MACFact","MAC Frame Error Rate Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Mfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("CnFFact","Carry-N-Forward Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Cfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("SNRFact","SNIR Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Sfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("HelloFact","Hello Count Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Hfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("RoadFact","Road Relevance Weighting Factor",
                   DoubleValue(1),
                   MakeDoubleAccessor(&RoutingProtocol::m_Rfact),
                   MakeDoubleChecker<double>())
    .AddAttribute ("RoadMap","Road Map pointer",
                    PointerValue(0),
                    MakePointerAccessor(&RoutingProtocol::SetGridMap),
                    MakePointerChecker<GridMap>())
    .AddTraceSource ("Rx", "Receive CLWPR packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_rxPacketTrace))
    .AddTraceSource ("Tx", "Send CLWPR packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_txPacketTrace))
    .AddTraceSource ("RoutingTableChanged", "The CLWPR routing table has changed.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_routingTableChanged))
    .AddTraceSource ("CarryNForward", "A packet has been queued due to local maximum",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_queueTrace))
    .AddTraceSource ("Dequeue", "A packet has been dequeued from local maximum",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_dequeueTrace))
    .AddTraceSource ("DropCache", "Drop a cached packet",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_dropCnFTrace))
    ;
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  : m_routingTableAssociation (0),
    m_ipv4 (0),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_hnaTimer (Timer::CANCEL_ON_DESTROY),
    m_utilTimer (Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
    MaxQueueLen (50),
    MaxQueueTime (Seconds(10)),
    m_queue (MaxQueueLen , MaxQueueTime)
{
  m_hnaRoutingTable = Create<Ipv4StaticRouting> ();
}

RoutingProtocol::~RoutingProtocol ()
{}

void RoutingProtocol::SetMaxQueueLen(uint32_t len){
  MaxQueueLen = len;
  m_queue.SetMaxQueueLen(len);
}

void RoutingProtocol::SetMaxQueueTime(Time t){
  MaxQueueTime = t;
  m_queue.SetQueueTimeout(t);
}


void
RoutingProtocol::SetGridMap (Ptr<GridMap> map)
{
  m_map = map;
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created clwpr::RoutingProtocol");
  m_helloTimer.SetFunction (&RoutingProtocol::HelloTimerExpire, this);
  m_hnaTimer.SetFunction (&RoutingProtocol::HnaTimerExpire, this);
  m_utilTimer.SetFunction(&RoutingProtocol::UtilTimerExpire, this);
  m_queuedMessagesTimer.SetFunction (&RoutingProtocol::SendQueuedMessages, this);

  m_packetSequenceNumber = CLWPR_MAX_SEQ_NUM;
  m_messageSequenceNumber = CLWPR_MAX_SEQ_NUM;
  m_ansn = CLWPR_MAX_SEQ_NUM;

//  m_linkTupleTimerFirstTime = true;

  m_ipv4 = ipv4;
  
  m_node = m_ipv4->GetObject<Node>();
  m_init_list = false;

  m_hnaRoutingTable->SetIpv4 (ipv4);
//  m_state.InitPosAssociationSet();
  m_position = GetNodePosition( m_ipv4 );
  m_velocity = GetNodeVelocity( m_ipv4 );
  m_heading = GetNodeHeading( m_ipv4 );
  if (m_mapFlag){
      TestPosition();
  }

}

void RoutingProtocol::InitNodeList(){
        uint32_t nn = NodeList::GetNNodes();
        NS_LOG_DEBUG(nn);
        for (uint32_t i =0; i<nn ; i++){
                Ptr<Node> node = NodeList::GetNode(i);
                Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
                NS_LOG_DEBUG("Ipv4 : " << ipv4);
                Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal ();
                NS_LOG_DEBUG("IP: " << addr);
                m_nodeList.insert(std::pair<Ipv4Address, Ptr<Ipv4> > (addr,ipv4));
        }
}

void RoutingProtocol::TestPosition(){
        m_map->SetRoadXFromVehicle(GetNodePosition( m_ipv4 ).x);
        m_map->SetRoadYFromVehicle(GetNodePosition( m_ipv4 ).y);
        GetNodeRoadId(m_ipv4);
}

void RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  m_hnaRoutingTable = 0;
  m_routingTableAssociation = 0;

  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter = m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();

  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream();
  *os << "Destination\tNextHop\t\tInterface\tWeight\tDistance\n";

  for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
    iter != m_table.end (); iter++)
    {
      *os << (*iter).first << "\t";
      *os << (*iter).second.nextAddr << "\t";
      *os << (*iter).second.interface << "\t\t";
      *os << (*iter).second.weight << "\t";
      *os << (*iter).second.distance << "\t";
      *os << "\n";
    }
}


void
RoutingProtocol::PrintNeighbourTable (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream();

  *os << "Node: " << m_ipv4->GetObject<Node> ()->GetId ()
                        << " Time: " << Simulator::Now ().GetSeconds () << "s "
                        << "Neighbouring table" << std::endl;

  const NeighborSet &neighborset = m_state.GetNeighbors ();

  *os << "Neighbour Degree = " << neighborset.size() << "\n";

  *os << "Neighbour\tPosition\t\tVelocity\t\tHeading\tRoadId\tUtilization\tMacInfo\tSNR\t\tCnF\tHelloCounter\n";

  if (neighborset.size()!= 0){
	  for (uint32_t i=0; i<neighborset.size(); i++)
		{
		  *os << neighborset.at(i).neighborMainAddr <<"\t";
		  *os << neighborset.at(i).neighborPosition <<"\t\t";
		  *os << neighborset.at(i).neighborVelocity <<"\t\t";
		  *os << neighborset.at(i).neighborHeading <<"\t";
		  *os << (int)neighborset.at(i).neighborRoadId <<"\t";
		  *os << (int)neighborset.at(i).neighborUtilization <<"\t\t";
		  *os << (int)neighborset.at(i).neighborMacInfo <<"\t";
		  *os << neighborset.at(i).neighborSNR <<"\t\t";
		  *os << (int)neighborset.at(i).neighborCnF <<"\t";
		  *os << neighborset.at(i).neighborHelloCount <<"\t";
		  *os << "\n";
		}
  }
  else {
	  *os << "Neighbor Set empty";
  }

  *os << std::endl;
}

void
RoutingProtocol::PrintNeighbourTableCompact (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream();
  const NeighborSet &neighborset = m_state.GetNeighbors ();

  *os << "Node: " << m_ipv4->GetObject<Node> ()->GetId ()
                        << " Time: " << Simulator::Now ().GetSeconds () << "s "
                        << "Neighbour Degree = " << neighborset.size() << std::endl;
}

void RoutingProtocol::DoStart ()
{
  if (m_mainAddress == Ipv4Address ())
    {
      Ipv4Address loopback ("127.0.0.1");
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
        {
          // Use primary address, if multiple
          Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
          if (addr != loopback)
            {
              m_mainAddress = addr;
              break;
            }
        }

      NS_ASSERT (m_mainAddress != Ipv4Address ());
    }

  NS_LOG_DEBUG ("Starting CLWPR on node " << m_mainAddress);

  Ipv4Address loopback ("127.0.0.1");

  bool canRunClwpr = false;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
      if (addr == loopback)
        continue;

      if (addr != m_mainAddress)
        {
          // Create never expiring interface association tuple entries for our
          // own network interfaces, so that GetMainAddress () works to
          // translate the node's own interface addresses into the main address.
          IfaceAssocTuple tuple;
          tuple.ifaceAddr = addr;
          tuple.mainAddr = m_mainAddress;
          AddIfaceAssocTuple (tuple);
          NS_ASSERT (GetMainAddress (addr) == m_mainAddress);
        }

      if(m_interfaceExclusions.find (i) != m_interfaceExclusions.end ())
        continue;

      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (), 
                                                 UdpSocketFactory::GetTypeId());
      socket->SetAllowBroadcast (true);
      InetSocketAddress inetAddr (m_ipv4->GetAddress (i, 0).GetLocal (), CLWPR_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvClwpr,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() CLWPR socket");
        }
      m_socketAddresses[socket] = m_ipv4->GetAddress (i, 0);

      canRunClwpr = true;
    }

  if(canRunClwpr)
   {
      HelloTimerExpire ();
      HnaTimerExpire ();
      if (m_normFlag){
        UtilTimerExpire();
      }
      NS_LOG_DEBUG ("CLWPR on node " << m_mainAddress << " started");
   }
}

void RoutingProtocol::SetMainInterface (uint32_t interface)
{
  m_mainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
}

void RoutingProtocol::SetInterfaceExclusions (std::set<uint32_t> exceptions)
{
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %CLWPR packet 
void
RoutingProtocol::RecvClwpr (Ptr<Socket> socket)
{
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("CLWPR node " << m_mainAddress << " received a CLWPR packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr);
  
  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == CLWPR_PORT_NUMBER);
  
  Ptr<Packet> packet = receivedPacket;

  // ***** SNR **** //
  SnrTag tag;
  if (packet->PeekPacketTag(tag)){
	  NS_LOG_DEBUG ("Received Packet with SRN = " << tag.snr);
      temp_snr = tag.snr;
  }
  // ***** SNR **** //

  clwpr::PacketHeader clwprPacketHeader;
  packet->RemoveHeader (clwprPacketHeader);
  NS_ASSERT (clwprPacketHeader.GetPacketLength () >= clwprPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = clwprPacketHeader.GetPacketLength () - clwprPacketHeader.GetSerializedSize ();

  MessageList messages;
  
  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);
      
      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("Clwpr Msg received with type "
                << std::dec << int (messageHeader.GetMessageType ())
                << " TTL=" << int (messageHeader.GetTimeToLive ())
                << " origAddr=" << messageHeader.GetOriginatorAddress ());
      messages.push_back (messageHeader);
    }

  m_rxPacketTrace (clwprPacketHeader, messages);

  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); messageIter++)
    {
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      if (messageHeader.GetTimeToLive () == 0
          || messageHeader.GetOriginatorAddress () == m_mainAddress)
        {
          packet->RemoveAtStart (messageHeader.GetSerializedSize ()
                                 - messageHeader.GetSerializedSize ());
          continue;
        }

      // If the message has been processed it must not be processed again
      bool do_forwarding = true;
      DuplicateTuple *duplicated = m_state.FindDuplicateTuple
        (messageHeader.GetOriginatorAddress (),
         messageHeader.GetMessageSequenceNumber ());

      
      if (duplicated == NULL)
        {
          switch (messageHeader.GetMessageType ())
            {
            case clwpr::MessageHeader::HELLO_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s CLWPR node " << m_mainAddress
                            << " received HELLO message of size " << int (messageHeader.GetSerializedSize ()));
              ProcessHello (messageHeader, receiverIfaceAddr, senderIfaceAddr);
              break;

            case clwpr::MessageHeader::HNA_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s CLWPR node " << m_mainAddress
                            <<  " received HNA message of size " << int (messageHeader.GetSerializedSize ()));
              ProcessHna (messageHeader, senderIfaceAddr);
              break; 

            default:
              NS_LOG_DEBUG ("CLWPR message type " <<
                        int (messageHeader.GetMessageType ()) <<
                        " not implemented");
            }
        }
      else
        {
          NS_LOG_DEBUG ("CLWPR message is duplicated, not reading it.");
      
          // If the message has been considered for forwarding, it should
          // not be retransmitted again
          for (std::vector<Ipv4Address>::const_iterator it = duplicated->ifaceList.begin ();
               it != duplicated->ifaceList.end(); it++)
            {
              if (*it == receiverIfaceAddr)
                {
                  do_forwarding = false;
                  break;
                }
            }
        }
      
      if (do_forwarding)
        {
          // HELLO messages are never forwarded.
          // Remaining messages are also forwarded using the default algorithm.
          if (messageHeader.GetMessageType ()  != clwpr::MessageHeader::HELLO_MESSAGE)
            {
              ForwardDefault (messageHeader, duplicated,
                              receiverIfaceAddr, inetSourceAddr.GetIpv4 ());
            }
        }

    }

  // After processing all CLWPR messages, we must recompute the routing table
  RoutingTableComputation ();


  // Dequeue any packet if the route has changed!

  if (m_cacheFlag){
      if (m_queue.GetSize()!= 0)
        {
          NS_LOG_DEBUG("Starting Queue Size = "<<m_queue.GetSize());
          Ptr<Ipv4Route> route;
                RoutingTableEntry entry1, entry2;
//              bool found = false;
                NS_LOG_DEBUG("TESTING dequeue Messages");


                std::vector<QueueEntry> qv = m_queue.GetQueue();
                for (std::vector<QueueEntry>::iterator i = qv.begin(); i != qv.end (); ++i)
                    {
                            Ipv4Address destination = i->GetIpv4Header().GetDestination();
                            Ipv4Address source = i->GetIpv4Header().GetSource();
                            NS_LOG_DEBUG ("Checking packet with ID " << i->GetIpv4Header().GetIdentification());
                            if (Lookup (destination , entry1) != 0)
                            {
                              bool foundSendEntry = FindSendEntry (entry1, entry2, destination, source);
                              if (!foundSendEntry)
                              {
                                  NS_FATAL_ERROR ("FindSendEntry failure");
                              }

               // If the forwarding node is not this node OR this node is the destination..
                              if (entry2.nextAddr != m_mainAddress )
                              {
                                      NS_LOG_DEBUG (" Local Maximum Problem SOLVED!!! (Dequeue all packets for that dst) ");
                                      route = Create<Ipv4Route> ();
                                      route->SetDestination (destination);
                                      // the source address is the interface address that matches
                                          // the destination address (when multiple are present on the
                                          // outgoing interface, one is selected via scoping rules)
                                          NS_ASSERT (m_ipv4);
                                          uint32_t interfaceIdx = entry2.interface;
                                          uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
                                          NS_ASSERT (numOifAddresses > 0);
                                          Ipv4InterfaceAddress ifAddr;
                                          if (numOifAddresses == 1) {
                                              ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
                                          } else {
                                              NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and CLWPR");
                                          }
                                          route->SetSource(i->GetIpv4Header().GetSource());
//                                          route->SetSource (ifAddr.GetLocal ());
                                          route->SetGateway (entry2.nextAddr);
                                          route->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
                                          //                    sockerr = Socket::ERROR_NOTERROR;
                                          NS_LOG_DEBUG ("Clwpr node " << m_mainAddress
                                                      << ": RouteOutput for dest=" << destination
                                                      << " --> nextHop=" << entry2.nextAddr
                                                      << " interface=" << entry2.interface);
                                          NS_LOG_DEBUG ("Found route to " << destination
                                                      << " via nh " << route->GetGateway ()
                                                      << " with source addr " << route->GetSource ()
                                                      << " and output dev "   << route->GetOutputDevice());
                                          //found = true;
                                          SendPacketFromQueue(destination, route);
                              }
                            }

              }
        }
  }
}


/// Weight Function for Neighbor node
double RoutingProtocol::WeightingFunction(NeighborTuple const &nb_tuple,
                                        PosAssociationTuple const &ds_tuple)
{
  double weight;
  double nb_dist = DistanceComputation(nb_tuple, ds_tuple);
  double my_dist = DistanceComputation(ds_tuple);
  double road_weight = 0;
  double angle = 0;
  Vector nb_pos = nb_tuple.neighborPosition;

  if (m_emapFlag){
    if (m_map->GetRoadXFromVehicle(nb_tuple.neighborPosition.x) != m_map->GetRoadXFromVehicle(ds_tuple.nodePosition.x)){
      road_weight += 0.5;
    }
    if (m_map->GetRoadYFromVehicle(nb_tuple.neighborPosition.y) != m_map->GetRoadYFromVehicle(ds_tuple.nodePosition.y)){
      road_weight += 0.5;
    }

    if (m_predictFlag){
      NS_LOG_INFO("Prediction ON");
      nb_pos = PredictPositionGeneric(nb_tuple.neighborPosition, nb_tuple.neighborVelocity,
    		                                 nb_tuple.neighborHeading,nb_tuple.neighborTimestamp);

    }

    angle = AngleComputation(nb_pos,
                             nb_tuple.neighborHeading,
                             nb_tuple.neighborVelocity,
                             ds_tuple.nodePosition);

   }

  if (m_normFlag){
  angle = AngleComputationGeneric(nb_pos,
	                       nb_tuple.neighborHeading,
	                       nb_tuple.neighborVelocity,
	                       ds_tuple.nodePosition);
  weight = m_Dfact * (nb_dist-my_dist)/m_txTh +
           m_Afact * angle +
           m_Ufact * nb_tuple.neighborUtilization +
           m_Mfact * nb_tuple.neighborMacInfo +
           m_Cfact * nb_tuple.neighborCnF +
           m_Sfact * SNRFunction(nb_tuple.neighborSNR)+
           m_Rfact * road_weight +
           m_Hfact * HelloFunction(nb_tuple.neighborHelloCount);
  }
  else {
      weight = m_Dfact * nb_dist +
               m_Afact * angle +
               m_Ufact * nb_tuple.neighborUtilization +
               m_Mfact * nb_tuple.neighborMacInfo +
               m_Cfact * nb_tuple.neighborCnF +
               m_Sfact * SNRFunction(nb_tuple.neighborSNR) +
               m_Rfact * road_weight +
               m_Hfact * HelloFunction(nb_tuple.neighborHelloCount);
  }
  return weight;
}

/// Weight function for current node
double RoutingProtocol::WeightingFunction(PosAssociationTuple const &ds_tuple)
{
  double weight;
  double my_dist = DistanceComputation(ds_tuple);
  double road_weight = 0;
  double angle = 0;
  Vector m_pos = GetNodePosition(m_ipv4);
  double heading = GetNodeHeading(m_ipv4);
  Vector m_vel = GetNodeVelocity(m_ipv4);

  if (m_emapFlag){
    if (m_map->GetRoadXFromVehicle(m_pos.x) != m_map->GetRoadXFromVehicle(ds_tuple.nodePosition.x)){
      road_weight += 0.5;
    }
    if (m_map->GetRoadYFromVehicle(m_pos.y) != m_map->GetRoadYFromVehicle(ds_tuple.nodePosition.y)){
      road_weight += 0.5;
    }

    angle = AngleComputation(m_pos,
                           heading,
                           m_vel,
                           ds_tuple.nodePosition);
  }

  if (m_normFlag){
//	  m_Dfact * (my_dist-my_dist)/m_txTh --> Distance == 0!!
  angle = AngleComputationGeneric(m_pos,
                                  heading,
                                  m_vel,
                                  ds_tuple.nodePosition);
  weight = m_Afact * angle +
           m_Ufact * GetNodeNormUtil () +
           m_Mfact * GetNodeMacInfo (m_ipv4) +
           m_Cfact * GetNodeCnFInfo (m_ipv4) +
           m_Rfact * road_weight;
  }
  else {
  weight = m_Dfact * my_dist +
           m_Afact * angle +
           m_Ufact * GetNodeUtilization (m_ipv4) +
           m_Mfact * GetNodeMacInfo (m_ipv4) +
           m_Cfact * GetNodeCnFInfo (m_ipv4) +
           m_Rfact * road_weight;
  }
  return weight;
}

/// This auxiliary function weights the SNR value of a link

double RoutingProtocol::SNRFunction(double n_SNIR)
{
  NS_LOG_DEBUG("SRN Value = " << n_SNIR);

  double snir_func =0;

  if (n_SNIR < PREF_SNIR){
        snir_func = ALPHA*(n_SNIR - SNIRmin)*(n_SNIR -SNIRmin);
  }
  else {
//        snir_func = BETA/(PREF_SNIR - SNIRmin);
        snir_func = BETA/(n_SNIR - SNIRmin);
  }
  return snir_func;
}

/// This auxiliary function weights the Hello count for a node
double RoutingProtocol::HelloFunction(int count)
{
  if (count <= 2) {
	  return 1;
  }
  else if (count > 2 && count <= 4){
	  return 0.5;
  }
  else {
	  return 0;
  }
}

///
/// \brief This auxiliary function is used for calculating
/// the distance between two nodes.
/// \param  nb_tuple the neighbor touple.
/// \param  ds_tuple the destination tuple.
/// \return the distance between neighbor node and destination.
///
double RoutingProtocol::DistanceComputation(NeighborTuple const &nb_tuple, PosAssociationTuple const &ds_tuple)
{
        double distance;
        Vector nb_pos = nb_tuple.neighborPosition;
        Vector nb_vel = nb_tuple.neighborVelocity;
        Vector ds_pos = ds_tuple.nodePosition;
        //Vector ds_vel = ds_tuple.nodeVelocity;
        //Time now = Simulator::Now();

        if (m_mapFlag){
          if (m_predictFlag){
              NS_LOG_INFO("Prediction ON");
              double nb_heading = nb_tuple.neighborHeading;
              nb_pos = PredictPositionGeneric(nb_pos, nb_vel, nb_heading, nb_tuple.neighborTimestamp);
          }

          int nb_X = m_map->GetRoadXFromVehicle(nb_pos.x);
          int nb_Y = m_map->GetRoadYFromVehicle(nb_pos.y);

          int pos_X = m_map->GetRoadXFromVehicle(ds_pos.x);
          int pos_Y = m_map->GetRoadYFromVehicle(ds_pos.y);

          distance = m_map->GetCurvemetricDistance(nb_pos,
                                                  nb_X,
                                                  nb_Y,
                                                  ds_pos,
                                                  pos_X,
                                                  pos_Y);
        }
        else{
            if (m_predictFlag){
                NS_LOG_INFO("Prediction ON");
                double nb_heading = nb_tuple.neighborHeading;
                nb_pos = PredictPositionGeneric(nb_pos, nb_vel, nb_heading, nb_tuple.neighborTimestamp);
            }
            distance = CalculateDistance(nb_pos,ds_pos);
        }

        return distance;
}

Vector RoutingProtocol::PredictPositionGeneric(Vector pos, Vector vel, double heading, Time t){
  Vector new_pos;
  Time now = Simulator::Now();
  Time dt = now - t;
  
  if (vel.x == 0 && vel.y == 0){
	  new_pos.x = pos.x;
	  new_pos.y = pos.y;
	  }
  else {
	  if (heading >=0 && heading <= 360){
		  if (cos(heading * M_PI / 180) >= 0){
			  new_pos.x = pos.x + vel.x * dt.GetSeconds();
		  }
		  else {
			  new_pos.x = pos.x - vel.x * dt.GetSeconds();
		  }
		  if (sin(heading * M_PI / 180) >= 0){
			  new_pos.y = pos.y + vel.y * dt.GetSeconds();
		  }
		  else {
			  new_pos.y = pos.y - vel.y * dt.GetSeconds();
		  }
		}
	}

  return new_pos;
}

///
/// \brief This auxiliary function is used for calculating 
/// the distance between two nodes.
///
/// \param  ds_tuple the destination tuple.
/// \return the distance between current node and destination.
///

double RoutingProtocol::DistanceComputation(PosAssociationTuple const &ds_tuple)
{
  double distance =0;
  Vector ds_pos = ds_tuple.nodePosition;
  //Vector ds_vel = ds_tuple.nodeVelocity;
  //Time now = Simulator::Now();

  m_position = GetNodePosition( m_ipv4 );
  if (m_mapFlag){
    int pos_X = m_map->GetRoadXFromVehicle(ds_pos.x);
	int pos_Y = m_map->GetRoadYFromVehicle(ds_pos.y);

	int m_X = m_map->GetRoadXFromVehicle(m_position.x);
	int m_Y = m_map->GetRoadYFromVehicle(m_position.y);

	distance = m_map->GetCurvemetricDistance(m_position,
                                             m_X,
                                             m_Y,
                                             ds_pos,
                                             pos_X,
                                             pos_Y);
  }
  else{
    distance = CalculateDistance(GetNodePosition( m_ipv4 ),ds_pos);
  }

  return distance;
}


/// Compute if a vehicle is approaching the destination or it is moving away
/// Return Value {-1, 1}
/// Without nornFlag, return value is quantised {-1, -0.5, 0, 0.5, 1}
/// With normFlag it can take any value {-1, 1}
double RoutingProtocol::AngleComputation(Vector pos_a, double heading_a, Vector vel_a, Vector pos_b){

  double Dx = pos_a.x - pos_b.x;
  double Dy = pos_a.y - pos_b.y;
  double angle =0;
  if (heading_a < 0 || heading_a > 360){
      //Static Node
      return angle;
  }

  // For X-axis
  if (Dx > 0 ){ // Node A on the right of node B
      if (heading_a > 90 && heading_a < 270){ // Moving towards left
          // Vehicle A approaching Vehicle B
          if (m_normFlag){
        	  angle = angle - 0.5*(std::abs(vel_a.x) / MAX_Vel);
          }
          else {
        	  angle = angle - 0.5;
          }
      }
      else{ // Moving towards Right
          // Vehicle A moving away from Vehicle B
          if (m_normFlag){
        	  angle = angle + 0.5*(std::abs(vel_a.x) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
  }
  else if (Dx <0 ) { // Node A on the left of node B
      if (heading_a < 90 || heading_a > 270){ // Moving towards Right
          // Vehicle A approaching Vehicle B
          if (m_normFlag){
        	  angle = angle - 0.5*(std::abs(vel_a.x) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
      else{ // Moving towards Left
          // Vehicle A moving away from Vehicle B
          if (m_normFlag){
        	  angle = angle + 0.5*(std::abs(vel_a.x) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
  }

  // For Y-axis
  if (Dy > 0 ){ // Node A on the above node B
      if (heading_a > 180 && heading_a < 360){ // Moving downwards
          // Vehicle A approaching Vehicle B
          if (m_normFlag){
        	  angle = angle - 0.5*(std::abs(vel_a.y) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
      else{ // Moving upwards
          // Vehicle A moving away from Vehicle B
          if (m_normFlag){
        	  angle = angle + 0.5*(std::abs(vel_a.y) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
  }
  else if (Dy < 0){ // Node A on the above node B
      if (heading_a > 0 && heading_a < 180){ // Moving upwards
          // Vehicle A approaching Vehicle B
          if (m_normFlag){
        	  angle = angle - 0.5*(std::abs(vel_a.y) / MAX_Vel);
          }
          else {
              angle = angle - 0.5;
          }
      }
      else{ // Moving downwards
          // Vehicle A moving away from Vehicle B
          if (m_normFlag){
        	  angle = angle + 0.5*(std::abs(vel_a.y) / MAX_Vel);
          }
          else {
              angle = angle + 0.5;
          }
      }
  }

  return angle;
}


/// Compute if a vehicle is approaching the destination or it is moving away
/// Return Value {-1, 1}
double RoutingProtocol::AngleComputationGeneric(Vector pos_a, double heading_a, Vector vel_a, Vector pos_b)
{
  // If theta is the angle between Velocity_A and the vector AB (assuming node B is static)
  // cos(theta) = adjacent / hypotenuse
  // return value = -cos(theta) / MAX_value;
  // cos(theta) > 0 --> theta < 90 (approaching) therefore return opposite sign

  Vector AB;
  AB.x = pos_b.x - pos_a.x;
  AB.y = pos_b.y - pos_a.y;
  double ABmag = sqrt(AB.x * AB.x + AB.y * AB.y);
  if (ABmag == 0) return 0;

  if (m_normFlag){
	  return -((vel_a.x*AB.x + vel_a.y*AB.y)/ABmag)/MAX_Vel;
  }
  else {
	  return -(vel_a.x*AB.x + vel_a.y*AB.y)/ABmag;
  }
}
///
/// \brief Gets the main address associated with a given interface address.
///
/// \param iface_addr the interface address.
/// \return the corresponding main address.
///
Ipv4Address
RoutingProtocol::GetMainAddress (Ipv4Address iface_addr) const
{
  const IfaceAssocTuple *tuple =
    m_state.FindIfaceAssocTuple (iface_addr);
  
  if (tuple != NULL)
    return tuple->mainAddr;
  else
    return iface_addr;
}

///
/// \brief Creates the routing table of the node 
///
void
RoutingProtocol::RoutingTableComputation ()
{
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << " s: Node " << m_mainAddress
                << ": RoutingTableComputation begin...");

  // 1. All the entries from the routing table are removed.
  Clear ();
  Time now = Simulator::Now();
  // For each Destination in the Position Association Set
  // calculate the weight of each neighbor (1-hop)

  const PosAssociationSet &posAssociationSet = m_state.GetPosAssociationSet();
  const NeighborSet &neighborSet = m_state.GetNeighbors ();
  
  if (posAssociationSet.size() != 0){
  for (PosAssociationSet::const_iterator it1 = posAssociationSet.begin ();
                it1 != posAssociationSet.end(); it1++)
        {
            PosAssociationTuple const &pos_tuple = *it1;
            NS_LOG_DEBUG ("Looking at position association tuple: " << pos_tuple);

            // ADD MY SELF
            // if i'm the destination
            if (pos_tuple.nodeMainAddr == m_mainAddress){
                  AddEntry (pos_tuple.nodeMainAddr,
                            Ipv4Address("127.0.0.0"),
                            0,
                            0,
                            -1000
                            );
            }
            //if i'm not the destination
            else{
                  AddEntry (pos_tuple.nodeMainAddr,
                            m_mainAddress,
                            1,
                            DistanceComputation(pos_tuple),
                            WeightingFunction(pos_tuple)
                            );
            }
            if (neighborSet.size() != 0){
                for (NeighborSet::const_iterator it = neighborSet.begin ();
                        it != neighborSet.end(); it++)
                        {
                          NeighborTuple const &nb_tuple = *it;
                          NS_LOG_DEBUG ("Looking at neighbor tuple: " << nb_tuple);
//                        bool nb_main_addr = false;

//                         if  ( (now - nb_tuple.neighborTimestamp).GetSeconds()<= CLWPR_NEIGHB_HOLD_TIME.GetSeconds() )
                          if  ( nb_tuple.neighborExpireTime >= now )
                          {
                              // Check for position / SNR etc
                              // If dist (node1, node2) > threshold
                              // do not add it.
                              if (m_predictFlag){
                                  Vector pred_pos = PredictPositionGeneric(nb_tuple.neighborPosition,
                                                             nb_tuple.neighborVelocity,
                                                             nb_tuple.neighborHeading,
                                                             nb_tuple.neighborTimestamp);
                                  double distance = CalculateDistance(pred_pos, GetNodePosition( m_ipv4 ));
                                  NS_LOG_DEBUG("Predicted position = "<< pred_pos << " Predicted distance = " << distance);
                                  if (distance >= m_txTh){
                                      NS_LOG_DEBUG(" Predicted position out of range of " << m_txTh <<"m. Node not added to routing table");
                                  }
                                  else {
                                      NS_LOG_LOGIC ("Adding routing table entry");
                                      AddEntry (pos_tuple.nodeMainAddr,
                                                nb_tuple.neighborMainAddr,
                                                1,
                                                DistanceComputation(nb_tuple, pos_tuple),
                                                WeightingFunction(nb_tuple, pos_tuple));
                                  }
                              }


                              //////////////////////////////////////
                              else
                                {
                                NS_LOG_LOGIC ("Adding routing table entry");
                                AddEntry (pos_tuple.nodeMainAddr,
                                          nb_tuple.neighborMainAddr,
                                          1,
                                          DistanceComputation(nb_tuple, pos_tuple),
                                          WeightingFunction(nb_tuple, pos_tuple));
                                }
                           }
                           else
                                {
//                                 std::cout << "Neighbor tuple:"<< nb_tuple << " for neighborMainAddr =  " << nb_tuple.neighborMainAddr << " expired => IGNORE @ " <<now << "\n";
                                  NS_LOG_LOGIC ("Neighbor tuple:"<< nb_tuple << " for neighborMainAddr =  " << nb_tuple.neighborMainAddr << " expired => IGNORE");
                                }
//}

                        }
                }
                else {
                    NS_LOG_DEBUG (" Neighbor Set is EMPTY " );
                }
        }
  }
  else {
      NS_LOG_DEBUG (" Position Association Set is EMPTY " );
  }

  // 5. For each tuple in the association set,
  //    If there is no entry in the routing table with:
  //        R_dest_addr     == A_network_addr/A_netmask
  //   then a new routing entry is created.
  const AssociationSet &associationSet = m_state.GetAssociationSet ();
  for (AssociationSet::const_iterator it = associationSet.begin ();
       it != associationSet.end (); it++)
    {
      AssociationTuple const &tuple = *it;
      RoutingTableEntry gatewayEntry;
      
      bool gatewayEntryExists = Lookup (tuple.gatewayAddr, gatewayEntry);
      bool addRoute = false;
      
      uint32_t routeIndex = 0;
      
      for (routeIndex = 0; routeIndex < m_hnaRoutingTable->GetNRoutes (); routeIndex++)
        {
          Ipv4RoutingTableEntry route = m_hnaRoutingTable->GetRoute (routeIndex);
          if (route.GetDestNetwork () == tuple.networkAddr &&
              route.GetDestNetworkMask () == tuple.netmask)
            {
              break;
            }
        }
    
      if (routeIndex == m_hnaRoutingTable->GetNRoutes ())
        {
          addRoute = true;
        }
      else if(gatewayEntryExists && m_hnaRoutingTable->GetMetric (routeIndex) > gatewayEntry.distance)
        {
          m_hnaRoutingTable->RemoveRoute(routeIndex);
          addRoute = true;
        }
        
      if(addRoute && gatewayEntryExists)
        {
//          m_hnaRoutingTable->AddNetworkRouteTo (tuple.networkAddr,
//                                                tuple.netmask,
//                                                gatewayEntry.nextAddr,
//                                                gatewayEntry.interface,
//                                                gatewayEntry.distance);
                                             
        }
    }

  NS_LOG_DEBUG ("Node " << m_mainAddress << ": RoutingTableComputation end.");
  m_routingTableChanged (GetSize ());
}


///
/// \brief Processes a HELLO message 
///
/// Population of the Neighbor Set is performed
///
/// \param msg the %CLWPR message which contains the HELLO message.
/// \param receiver_iface the address of the interface where the message was received from.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessHello (const clwpr::MessageHeader &msg,
                         const Ipv4Address &receiverIface,
                         const Ipv4Address &senderIface)
{
  NS_LOG_FUNCTION (msg << receiverIface << senderIface);
  
  const clwpr::MessageHeader::Hello &hello = msg.GetHello ();
PopulateNeighborSet (msg, hello, receiverIface);
  
#ifdef NS3_LOG_ENABLE
  {
    const NeighborSet &Neighbors = m_state.GetNeighbors ();
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << "s ** BEGIN dump Neighbor Set for CLWPR Node " << m_mainAddress);
    for (NeighborSet::const_iterator tuple = Neighbors.begin ();
         tuple != Neighbors.end (); tuple++)
      {
        NS_LOG_DEBUG(*tuple);
      }
    NS_LOG_DEBUG ("** END dump Neighbor Set for CLWPR Node " << m_mainAddress);
  }
#endif // NS3_LOG_ENABLE
  

  // If neighbor exists in position association tuple, update information
  PosAssociationTuple *pos_tuple = m_state.FindPosAssociationTuple(msg.GetOriginatorAddress ());
  if ( pos_tuple != NULL){
      pos_tuple->nodePosition = hello.position;
      pos_tuple->nodeVelocity = hello.velocity;
      pos_tuple->nodeHeading = hello.heading;
      pos_tuple->nodeRoadId = hello.roadId;
      pos_tuple->nodeTimestamp= Simulator::Now();
  }

}

///
/// \brief Processes a HNA message following RFC 3626 specification.
///
/// The Host Network Association Set is updated (if needed) with the information
/// of the received HNA message.
///
/// \param msg the %CLWPR message which contains the HNA message.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessHna (const clwpr::MessageHeader &msg,
                       const Ipv4Address &senderIface)
{

  const clwpr::MessageHeader::Hna &hna = msg.GetHna ();
  Time now = Simulator::Now ();
  
  // 1. If the sender interface of this message is not in the symmetric
  // 1-hop neighborhood of this node, the message MUST be discarded.
//  const LinkTuple *link_tuple = m_state.FindSymLinkTuple (senderIface, now);
//  if (link_tuple == NULL)
//    return;
  
  // 2. Otherwise, for each (network address, netmask) pair in the
  // message:
  
  for (std::vector<clwpr::MessageHeader::Hna::Association>::const_iterator it = hna.associations.begin();
       it != hna.associations.end() ; it++)
    {
      AssociationTuple *tuple = m_state.FindAssociationTuple(msg.GetOriginatorAddress(),it->address,it->mask);
  
      // 2.1  if an entry in the association set already exists, where:
      //          A_gateway_addr == originator address
      //          A_network_addr == network address
      //          A_netmask      == netmask
      //      then the holding time for that tuple MUST be set to:
      //          A_time         =  current time + validity time
      if(tuple != NULL)
        {
          tuple->expirationTime = now + msg.GetVTime ();
        }
        
      // 2.2 otherwise, a new tuple MUST be recorded with:
      //          A_gateway_addr =  originator address
      //          A_network_addr =  network address
      //          A_netmask      =  netmask
      //          A_time         =  current time + validity time
      else
        {
          AssociationTuple assocTuple = {
            msg.GetOriginatorAddress(),
            it->address,
            it->mask,
            now + msg.GetVTime ()
          };
          AddAssociationTuple (assocTuple);
          
          //Schedule Association Tuple deletion
          if (DELAY_FLAG){
          Simulator::Schedule (DELAY (assocTuple.expirationTime),&RoutingProtocol::AssociationTupleTimerExpire,
                               this, assocTuple.gatewayAddr,assocTuple.networkAddr,assocTuple.netmask);
          }
          else{
          Simulator::Schedule (assocTuple.expirationTime,&RoutingProtocol::AssociationTupleTimerExpire,
                               this, assocTuple.gatewayAddr,assocTuple.networkAddr,assocTuple.netmask);
          }
        }
        
    }
}

///
/// \brief CLWPR's default forwarding algorithm is Greedy Forwarding
/// based on weight information calculated for each destination
///
///
/// \param p the %CLWPR packet which has been received.
/// \param msg the %CLWPR message which must be forwarded.
/// \param dup_tuple NULL if the message has never been considered for forwarding,
/// or a duplicate tuple in other case.
/// \param local_iface the address of the interface where the message was received from.
///
void
RoutingProtocol::ForwardDefault (clwpr::MessageHeader clwprMessage,
                               DuplicateTuple *duplicated,
                               const Ipv4Address &localIface,
                               const Ipv4Address &senderAddress)
{
  Time now = Simulator::Now ();
  
  // If the sender interface address is not in the symmetric
  // 1-hop neighborhood the message must not be forwarded
//  const LinkTuple *linkTuple = m_state.FindSymLinkTuple (senderAddress, now);
//  if (linkTuple == NULL)
//   return;

  // If the message has already been considered for forwarding,
  // it must not be retransmitted again
  if (duplicated != NULL && duplicated->retransmitted)
    {
      NS_LOG_LOGIC (Simulator::Now () << "Node " << m_mainAddress << " does not forward a message received"
                    " from " << clwprMessage.GetOriginatorAddress () << " because it is duplicated");
      return;
    }

  // If the sender interface address is an interface address
  // of a MPR selector of this node and ttl is greater than 1,
  // the message must be retransmitted
  
  // There is no MPR selector. We forward ALL messages
  bool retransmitted = false;
  if (clwprMessage.GetTimeToLive () > 1)
    {
          clwprMessage.SetTimeToLive (clwprMessage.GetTimeToLive () - 1);
          clwprMessage.SetHopCount (clwprMessage.GetHopCount () + 1);
          // We have to introduce a random delay to avoid
          // synchronization with neighbors.
          QueueMessage (clwprMessage, JITTER);
          retransmitted = true;
    }

  // Update duplicate tuple...
  if (duplicated != NULL)
    {
      duplicated->expirationTime = now + CLWPR_DUP_HOLD_TIME;
      duplicated->retransmitted = retransmitted;
      duplicated->ifaceList.push_back (localIface);
    }
  // ...or create a new one
  else
    {
      DuplicateTuple newDup;
      newDup.address = clwprMessage.GetOriginatorAddress ();
      newDup.sequenceNumber = clwprMessage.GetMessageSequenceNumber ();
      newDup.expirationTime = now + CLWPR_DUP_HOLD_TIME;
      newDup.retransmitted = retransmitted;
      newDup.ifaceList.push_back (localIface);
      AddDuplicateTuple (newDup);

//      Simulator::Schedule (CLWPR_DUP_HOLD_TIME,
//                           &RoutingProtocol::DupTupleTimerExpire, this,
//                           newDup.address, newDup.sequenceNumber);
    }
}

///
/// \brief Enques an %CLWPR message which will be sent with a delay of (0, delay].
///
/// This buffering system is used in order to piggyback several %CLWPR messages in
/// a same %CLWPR packet.
///
/// \param msg the %CLWPR message which must be sent.
/// \param delay maximum delay the %CLWPR message is going to be buffered.
///
void
RoutingProtocol::QueueMessage (const clwpr::MessageHeader &message, Time delay)
{
  m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}

void
RoutingProtocol::SendPacket (Ptr<Packet> packet, 
                       const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("CLWPR node " << m_mainAddress << " sending a CLWPR packet");

  // Add a header
  clwpr::PacketHeader header;
  header.SetPacketLength (header.GetSerializedSize () + packet->GetSize ());
  header.SetPacketSequenceNumber (GetPacketSequenceNumber ());
  packet->AddHeader (header);

  // Add Qos Tag
  // CLWPR packets (only HELLO at the moment) are send with low priority
  m_qosTag.SetTid(UP_BK);
  packet->AddPacketTag(m_qosTag);

  // Trace it
  m_txPacketTrace (header, containedMessages);

  // Send it
  for (std::map<Ptr<Socket> , Ipv4InterfaceAddress>::const_iterator i =
      m_socketAddresses.begin (); i != m_socketAddresses.end (); i++)
    {
      Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, CLWPR_PORT_NUMBER));
    }
}

///
/// \brief Creates as many %CLWPR packets as needed in order to send all buffered
/// %CLWPR messages.
///
/// Maximum number of messages which can be contained in an %CLWPR packet is
/// dictated by CLWPR_MAX_MSGS constant.
///
void
RoutingProtocol::SendQueuedMessages ()
{
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("Clwpr node " << m_mainAddress << ": SendQueuedMessages");

  MessageList msglist;

  for (std::vector<clwpr::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       message++)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == CLWPR_MAX_MSGS)
        {
          SendPacket (packet, msglist);
          msglist.clear ();
          // Reset variables for next packet
          numMessages = 0;
          packet = Create<Packet> ();
        }
    }

  if (packet->GetSize ())
    {
      SendPacket (packet, msglist);
    }

  m_queuedMessages.clear ();
}

///
/// \brief Creates a new %CLWPR HELLO message which is buffered for being sent later on.
///
void
RoutingProtocol::SendHello ()
{
  NS_LOG_FUNCTION (this);
    
  clwpr::MessageHeader msg;
  //Time now = Simulator::Now ();

  msg.SetVTime (CLWPR_NEIGHB_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (1);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  clwpr::MessageHeader::Hello &hello = msg.GetHello ();

  hello.position = GetNodePosition(m_ipv4);
  hello.velocity = GetNodeVelocity(m_ipv4);
  hello.heading = GetNodeHeading(m_ipv4);
  hello.roadId = GetNodeRoadId(m_ipv4);
  if (m_normFlag){
    hello.utilization = GetNodeNormUtil();
  }
  else {
    hello.utilization = GetNodeUtilization(m_ipv4);
  }
  hello.SetHTime (m_helloInterval);
  hello.macInfo = GetNodeMacInfo(m_ipv4);
  hello.CnFinfo = GetNodeCnFInfo(m_ipv4);
  NS_LOG_FUNCTION (this << hello.position << hello.velocity <<  hello.heading  << hello.roadId );
  NS_LOG_DEBUG ("CLWPR HELLO message size: " << int (msg.GetSerializedSize ()));
  QueueMessage (msg, JITTER);
}


Vector RoutingProtocol::GetNodePosition (Ptr<Ipv4> ipv4)
{
    Vector pos = ipv4->GetObject<MobilityModel>()->GetPosition(); 
    NS_LOG_DEBUG (" Node " << ipv4 << " position =" << pos);
        return pos;
}

Vector RoutingProtocol::GetNodeVelocity (Ptr<Ipv4> ipv4)
{
        Vector vel = ipv4->GetObject<MobilityModel>()->GetVelocity();
        NS_LOG_DEBUG (" Node " << ipv4 << " velocity =" << vel);
        return vel;
}

/***
 * This method calculated the heading of a specific node
 */
double RoutingProtocol::GetNodeHeading (Ptr<Ipv4> ipv4)
{
    double angle = 0;
    Vector vel = GetNodeVelocity( ipv4 );

    if (vel.x == 0 && vel.y == 0) {
        NS_LOG_DEBUG("Node is stationary");
        angle = -1;
    }
    else {
        // Angle in radians
        angle = atan2(vel.y, vel.x);
        if (angle < 0) {
            angle = 2* M_PI + angle;
        }
        NS_LOG_DEBUG("Node's Heading in radians =" << angle);
        // Angle in degrees
        angle = angle * 180 / M_PI;

        NS_LOG_DEBUG("Node's Heading in degrees =" << angle);
    }

    return angle;
}

/***
 * This function returns the road id on which the vehicle is traveling
 */

uint8_t RoutingProtocol::GetNodeRoadId (Ptr<Ipv4> ipv4)
{
        m_map->SetRoadXFromVehicle(GetNodePosition( ipv4 ).x);
        m_map->SetRoadYFromVehicle(GetNodePosition( ipv4 ).y);
//      m_roadId = m_map->FindRoadID(m_map->GetRoadXId(), m_map->GetRoadYId());
        NS_LOG_DEBUG (" Node " << ipv4 << " road X =" << m_map->GetRoadXId() << " road Y =" << m_map->GetRoadYId() );
        return m_map->FindRoadID(m_map->GetRoadXId(), m_map->GetRoadYId());
        }

/***
 * This function returns the number of packets in MAC queue as a metric of node utilization
 */
uint32_t RoutingProtocol::GetNodeUtilization (Ptr<Ipv4> ipv4)
{
//      Show how to print the Wifi MAC queue size at a particular time
//      Ptr<WifiNetDevice> dev = ipv4->GetObject<ns3::WifiNetDevice>();
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_node->GetDevice(0));
        NS_ASSERT (dev != NULL);
        PointerValue ptr;
        dev->GetAttribute ("Mac",ptr);
        Ptr<AdhocWifiMac> mac = ptr.Get<AdhocWifiMac> ();
        NS_ASSERT (mac != NULL);
        mac->GetAttribute ("DcaTxop",ptr);
        Ptr<DcaTxop> dca = ptr.Get<DcaTxop> ();
        NS_ASSERT (dca != NULL);
        Ptr<WifiMacQueue> queue = dca->GetQueue ();
        NS_ASSERT (queue != NULL);
        NS_LOG_INFO ("Queue size: " << queue->GetSize ());
        m_utilization = queue->GetSize();
        return m_utilization;

}


/***
 * This function calculates the node's utilization every second.
 * Use the MacTxBytes from wifi-mac to measure the size
 * of packets and wifi-remote-station-manager to get the nominal data rate
 * The function is scheduled for recalculation.
 */
void RoutingProtocol::CalculateNodeNormUtil ()
{
  uint64_t temp;

  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_node->GetDevice(0));
  NS_ASSERT (dev != NULL);
  PointerValue ptr;
  PointerValue ptr2;
  dev->GetAttribute ("Mac",ptr);
  Ptr<AdhocWifiMac> mac = ptr.Get<AdhocWifiMac> ();
  NS_ASSERT (mac != NULL);

  NS_LOG_DEBUG("Current MAC Tx Bytes = "<< mac->GetTxBytes());
  // temp = current value - previous value
  temp = mac->GetTxBytes() - m_macTxTemp;
  // current value set as previous value
  m_macTxTemp = mac->GetTxBytes();

  dev->GetAttribute("RemoteStationManager", ptr2);
  Ptr<WifiRemoteStationManager>  _wifiRemSt = ptr2.Get<WifiRemoteStationManager>();
  NS_ASSERT (_wifiRemSt != NULL);

  NS_LOG_DEBUG ("MAC data rate = " << _wifiRemSt->GetDefaultMode().GetDataRate());
  _wifiRemSt->GetDefaultMode().GetDataRate();

  m_utilization = temp * 8 / _wifiRemSt->GetDefaultMode().GetDataRate();
  return;
}

uint32_t RoutingProtocol::GetNodeNormUtil(void)
{
	return m_utilization;
}
/***
 * This function returns the MAC Frame Error Rate for a specific ipv4
 * Not sure if it works .. haven't seen it return any actual value
 *
 */

uint32_t RoutingProtocol::GetNodeMacInfo (Ptr<Ipv4> ipv4)
{
          Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_node->GetDevice(0));
          NS_ASSERT (dev != NULL);
          PointerValue ptr;
          PointerValue ptr2;
          dev->GetAttribute ("Mac",ptr);
          Ptr<AdhocWifiMac> mac = ptr.Get<AdhocWifiMac> ();
          NS_ASSERT (mac != NULL);
          dev->GetAttribute("RemoteStationManager", ptr2);
          Ptr<WifiRemoteStationManager>  _wifiRemSt = ptr2.Get<WifiRemoteStationManager>();
          NS_ASSERT (_wifiRemSt != NULL);
          WifiRemoteStationInfo _wifiInfo = _wifiRemSt->GetInfo(mac->GetAddress());
          NS_LOG_INFO ("Frame Error Rate = " <<_wifiInfo.GetFrameErrorRate());
          m_macInfo = (uint32_t) _wifiInfo.GetFrameErrorRate();
          return m_macInfo;
        }

/***
 * This function returns the number of cached packets from the routing protocol
 * when the carry-n-forward is employed
 */
uint16_t RoutingProtocol::GetNodeCnFInfo(Ptr<Ipv4> ipv4)
{
//  NS_LOG_UNCOND ("CnF Queue = " << m_queue.GetSize());
  m_CnFinfo =  (m_queue.GetSize()) & 0x10F;
  return m_CnFinfo;
  }

///
/// \brief Creates a new %CLWPR HNA message which is buffered for being sent later on.
///
void
RoutingProtocol::SendHna ()
{

  clwpr::MessageHeader msg;

  msg.SetVTime (CLWPR_HNA_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (255);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  clwpr::MessageHeader::Hna &hna = msg.GetHna ();
  
  std::vector<clwpr::MessageHeader::Hna::Association>
    &associations = hna.associations;
      
  if (m_routingTableAssociation != 0)
    {
      // Add (NetworkAddr, Netmask) entries from Associated Routing Table to HNA message.
      for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes (); i++)
        {
          Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute (i);
          
          std::set<uint32_t>::const_iterator ci = m_interfaceExclusions.find (route.GetInterface ());
                  
          if (ci != m_interfaceExclusions.end ())
            {
              clwpr::MessageHeader::Hna::Association assoc = {route.GetDestNetwork (), route.GetDestNetworkMask ()};
              associations.push_back(assoc);
            }
        }
    }
    
  int size = associations.size ();

  // Add (NetworkAddr, Netmask) entries specified using AddHostNetworkAssociation () to HNA message.
  for (Associations::const_iterator it = m_state.GetAssociations ().begin ();
        it != m_state.GetAssociations ().end (); it++)
    {
      // Check if the entry has already been added from the Associated Routing Table
      std::vector<clwpr::MessageHeader::Hna::Association>::const_iterator ci = associations.begin ();
      bool found = false;
      for (int i = 0; i < size; i++)
        {
          if (it->networkAddr == ci->address && it->netmask == ci->mask)
            {
              found = true;
              break;
            }
          ci++;
        }
      
      if(!found)
        {
          clwpr::MessageHeader::Hna::Association assoc = {it->networkAddr,it->netmask};
          associations.push_back(assoc);
        }
    }
    
  if(associations.size () == 0)
    return;
  
  QueueMessage (msg, JITTER);
}

///
/// \brief Injects a (networkAddr, netmask) tuple for which the node
///        can generate an HNA message for
///
void
RoutingProtocol::AddHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask)
{
  m_state.InsertAssociation ((Association) {networkAddr, netmask});
}

///
/// \brief Adds an Ipv4StaticRouting protocol Association
///        can generate an HNA message for
///
void
RoutingProtocol::SetRoutingTableAssociation (Ptr<Ipv4StaticRouting> routingTable)
{
  m_routingTableAssociation = routingTable;
}


///
/// \brief      Updates the Neighbor Set according to the information contained in a new received
///             HELLO message (following RFC 3626).
void
RoutingProtocol::PopulateNeighborSet (const clwpr::MessageHeader &msg,
                                const clwpr::MessageHeader::Hello &hello,
                                const Ipv4Address &iface)
{
  NeighborTuple *nb_tuple = m_state.FindNeighborTuple (msg.GetOriginatorAddress ());
  Time now = Simulator::Now();

  // Update information of the neighbor...
  if (nb_tuple != NULL)
    {
      nb_tuple->neighborPosition = hello.position;
	  nb_tuple->neighborVelocity = hello.velocity;
	  ///TODO
	  /// Check heading to make velocity changes because velocity is transmitted as unsigned integer

      if (hello.heading >= 90 || hello.heading <= 270)
      {
    	  nb_tuple->neighborVelocity.x = -hello.velocity.x;
      }
      if (hello.heading >= 180 )
      {
    	  nb_tuple->neighborVelocity.y = -hello.velocity.y;
      }

      if (hello.heading >= 360) {
          nb_tuple->neighborHeading = -1;
      }
      else {
          nb_tuple->neighborHeading = hello.heading;
      }
      nb_tuple->neighborRoadId = hello.roadId;
      nb_tuple->neighborUtilization = hello.utilization;
      nb_tuple->neighborMacInfo = hello.macInfo;
      nb_tuple->neighborSNR = temp_snr;
      nb_tuple->neighborCnF = hello.CnFinfo;
      nb_tuple->neighborTimestamp = now;
      nb_tuple->neighborExpireTime = now + msg.GetVTime();
      nb_tuple->neighborHelloCount++;
//      std::cout << "Valid time =" <<   nb_tuple->neighborExpireTime << "\n";
//      m_events.Track (Simulator::Schedule (DELAY (CLWPR_NEIGHB_HOLD_TIME),
//                      &RoutingProtocol::NeighborTupleTimerExpire , this,
//                      new_tuple.neighborMainAddr));
    }
  //Make new entry
  else
    {
      NeighborTuple new_tuple;
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
        {
          for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
            {
              if (m_ipv4->GetAddress (i,j).GetLocal () == iface)
                {
                  new_tuple.neighborInterface = i;
                  break;
                }
            }
        }
      new_tuple.neighborMainAddr = msg.GetOriginatorAddress ();

      new_tuple.status = NeighborTuple::STATUS_SYM;
      new_tuple.neighborPosition = hello.position;
      new_tuple.neighborVelocity = hello.velocity;
	  ///TODO
	  /// Check heading to make velocity changes because velocity is transmitted as unsigned integer

      if (hello.heading >= 90 || hello.heading <= 270)
      {
    	  new_tuple.neighborVelocity.x = -hello.velocity.x;
      }
      if (hello.heading >= 180 )
      {
    	  new_tuple.neighborVelocity.y = -hello.velocity.y;
      }

      if (hello.heading >= 360) {
          new_tuple.neighborHeading = -1;
      }
      else {
          new_tuple.neighborHeading = hello.heading;
      }
      new_tuple.neighborRoadId = hello.roadId;
      new_tuple.neighborUtilization = hello.utilization;
      new_tuple.neighborMacInfo = hello.macInfo;
      new_tuple.neighborSNR = temp_snr;
      new_tuple.neighborCnF = hello.CnFinfo;
      new_tuple.neighborTimestamp= now;
      new_tuple.neighborExpireTime= now + msg.GetVTime();
      new_tuple.neighborHelloCount = 1;
      AddNeighborTuple(new_tuple);
      m_events.Track (Simulator::Schedule (CLWPR_NEIGHB_HOLD_TIME,
                                           &RoutingProtocol::NeighborTupleTimerExpire , this,
                                           new_tuple.neighborMainAddr));
  }
}


#if 0
///
/// \brief      Drops a given packet because it couldn't be delivered to the corresponding
///             destination by the MAC layer. This may cause a neighbor loss, and appropiate
///             actions are then taken.
///
/// \param p the packet which couldn't be delivered by the MAC layer.
///
void
CLWPR::mac_failed(Ptr<Packet> p) {
        double now              = Simulator::Now ();
        struct hdr_ip* ih       = HDR_IP(p);
        struct hdr_cmn* ch      = HDR_CMN(p);

        debug("%f: Node %d MAC Layer detects a breakage on link to %d\n",
                now,
                CLWPR::node_id(ra_addr()),
                CLWPR::node_id(ch->next_hop()));

        if ((u_int32_t)ih->daddr() == IP_BROADCAST) {
                drop(p, DROP_RTR_MAC_CALLBACK);
                return;
        }

        CLWPR_link_tuple* link_tuple = state_.find_link_tuple(ch->next_hop());
        if (link_tuple != NULL) {
                link_tuple->lost_time() = now + CLWPR_NEIGHB_HOLD_TIME;
                link_tuple->time()      = now + CLWPR_NEIGHB_HOLD_TIME;
                nb_loss(link_tuple);
        }
        drop(p, DROP_RTR_MAC_CALLBACK);
}
#endif



///
/// \brief Adds a duplicate tuple to the Duplicate Set.
///
/// \param tuple the duplicate tuple to be added.
///
void
RoutingProtocol::AddDuplicateTuple (const DuplicateTuple &tuple)
{
        /*debug("%f: Node %d adds dup tuple: addr = %d seq_num = %d\n",
                Simulator::Now (),
                CLWPR::node_id(ra_addr()),
                CLWPR::node_id(tuple->addr()),
                tuple->seq_num());*/
  m_state.InsertDuplicateTuple (tuple);
}

///
/// \brief Removes a duplicate tuple from the Duplicate Set.
///
/// \param tuple the duplicate tuple to be removed.
///
void
RoutingProtocol::RemoveDuplicateTuple (const DuplicateTuple &tuple)
{
  /*debug("%f: Node %d removes dup tuple: addr = %d seq_num = %d\n",
    Simulator::Now (),
    CLWPR::node_id(ra_addr()),
    CLWPR::node_id(tuple->addr()),
    tuple->seq_num());*/
  m_state.EraseDuplicateTuple (tuple);
}

///
/// \brief Adds a neighbor tuple to the Neighbor Set.
///
/// \param tuple the neighbor tuple to be added.
///
void
RoutingProtocol::AddNeighborTuple (const NeighborTuple &tuple)
{
//   debug("%f: Node %d adds neighbor tuple: nb_addr = %d status = %s\n",
//         Simulator::Now (),
//         CLWPR::node_id(ra_addr()),
//         CLWPR::node_id(tuple->neighborMainAddr),
//         ((tuple->status() == CLWPR_STATUS_SYM) ? "sym" : "not_sym"));
  
  m_state.InsertNeighborTuple (tuple);
  IncrementAnsn ();
}

///
/// \brief Removes a neighbor tuple from the Neighbor Set.
///
/// \param tuple the neighbor tuple to be removed.
///
void
RoutingProtocol::RemoveNeighborTuple (const NeighborTuple &tuple)
{
//   debug("%f: Node %d removes neighbor tuple: nb_addr = %d status = %s\n",
//         Simulator::Now (),
//         CLWPR::node_id(ra_addr()),
//         CLWPR::node_id(tuple->neighborMainAddr),
//         ((tuple->status() == CLWPR_STATUS_SYM) ? "sym" : "not_sym"));

  m_state.EraseNeighborTuple (tuple);
  IncrementAnsn ();
}


void
RoutingProtocol::IncrementAnsn ()
{
  m_ansn = (m_ansn + 1) % (CLWPR_MAX_SEQ_NUM + 1);
}



///
/// \brief Adds an interface association tuple to the Interface Association Set.
///
/// \param tuple the interface association tuple to be added.
///
void
RoutingProtocol::AddIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
//   debug("%f: Node %d adds iface association tuple: main_addr = %d iface_addr = %d\n",
//         Simulator::Now (),
//         CLWPR::node_id(ra_addr()),
//         CLWPR::node_id(tuple->main_addr()),
//         CLWPR::node_id(tuple->iface_addr()));

  m_state.InsertIfaceAssocTuple (tuple);
}

///
/// \brief Removes an interface association tuple from the Interface Association Set.
///
/// \param tuple the interface association tuple to be removed.
///
void
RoutingProtocol::RemoveIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
//   debug("%f: Node %d removes iface association tuple: main_addr = %d iface_addr = %d\n",
//         Simulator::Now (),
//         CLWPR::node_id(ra_addr()),
//         CLWPR::node_id(tuple->main_addr()),
//         CLWPR::node_id(tuple->iface_addr()));

  m_state.EraseIfaceAssocTuple (tuple);
}

///
/// \brief Adds a host network association tuple to the Association Set.
///
/// \param tuple the host network association tuple to be added.
///
void
RoutingProtocol::AddAssociationTuple (const AssociationTuple &tuple)
{
  m_state.InsertAssociationTuple (tuple);
}

///
/// \brief Removes a host network association tuple from the Association Set.
///
/// \param tuple the host network association tuple to be removed.
///
void
RoutingProtocol::RemoveAssociationTuple (const AssociationTuple &tuple)
{
  m_state.EraseAssociationTuple (tuple);
}



uint16_t RoutingProtocol::GetPacketSequenceNumber ()
{
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (CLWPR_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}

/// Increments message sequence number and returns the new value.
uint16_t RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (CLWPR_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}


///
/// \brief Sends a HELLO message and reschedules the HELLO timer.
/// \param e The event which has expired.
///
void
RoutingProtocol::HelloTimerExpire ()
{
  SendHello ();
  m_helloTimer.Schedule (m_helloInterval);
}

/*
 * \brief Calculates the Node Utilization and reschedules the Utilization timer
 * \param e The event which has expired.
 */
void
RoutingProtocol::UtilTimerExpire ()
{
  CalculateNodeNormUtil ();
  m_utilTimer.Schedule (Seconds (1.0));
}

///
/// \brief Sends an HNA message (if the node has associated hosts/networks) and reschedules the HNA timer.
/// \param e The event which has expired.
///
void
RoutingProtocol::HnaTimerExpire ()
{
  if (m_state.GetAssociations ().size () > 0 || m_routingTableAssociation !=0)
    {
      SendHna ();
    }
  else
    {
      NS_LOG_DEBUG ("Not sending any HNA, no associations to advertise.");
    }
  m_hnaTimer.Schedule (m_hnaInterval);
}

///
/// \brief Removes tuple if expired. Else timer is rescheduled to expire at tuple.expirationTime.
///
/// The task of actually removing the tuple is left to the CLWPR agent.
///
/// \param tuple The tuple which has expired.
///
void
RoutingProtocol::DupTupleTimerExpire (Ipv4Address address, uint16_t sequenceNumber)
{
  DuplicateTuple *tuple =
    m_state.FindDuplicateTuple (address, sequenceNumber);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveDuplicateTuple (*tuple);
    }
  else
    {
      if (DELAY_FLAG){
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::DupTupleTimerExpire, this,
                                           address, sequenceNumber));
      }
      else{
          m_events.Track (Simulator::Schedule (tuple->expirationTime,
                                               &RoutingProtocol::DupTupleTimerExpire, this,
                                               address, sequenceNumber));
      }
    }
}

/////
///// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
/////
///// The task of actually removing the tuple is left to the CLWPR agent.
/////
///// \param e The event which has expired.
/////
void
RoutingProtocol::NeighborTupleTimerExpire (Ipv4Address neighborMainAddr)
{
  NeighborTuple *tuple = m_state.FindNeighborTuple (neighborMainAddr);
  Time now = Simulator::Now();
  if (tuple == NULL)
    {
      return;
    }
//  if ((now - tuple->neighborTimestamp).GetSeconds() > CLWPR_NEIGHB_HOLD_TIME.GetSeconds() )
  if (tuple->neighborExpireTime < now )
          {
      RemoveNeighborTuple (*tuple);
      // After removing a neighbor, we must recompute the routing table
      RoutingTableComputation ();
    }
  else
    {
      if (DELAY_FLAG){
      m_events.Track (Simulator::Schedule (DELAY (tuple->neighborExpireTime),
                                           &RoutingProtocol::NeighborTupleTimerExpire,
                                           this, neighborMainAddr));
      }
      else{
          m_events.Track (Simulator::Schedule (tuple->neighborExpireTime,
                                                     &RoutingProtocol::NeighborTupleTimerExpire,
                                                     this, neighborMainAddr));
      }
    }
}

///
/// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
/// \param e The event which has expired.
///
void
RoutingProtocol::IfaceAssocTupleTimerExpire (Ipv4Address ifaceAddr)
{
  IfaceAssocTuple *tuple = m_state.FindIfaceAssocTuple (ifaceAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->time < Simulator::Now ())
    {
      RemoveIfaceAssocTuple (*tuple);
    }
  else
    {
      if (DELAY_FLAG){
      m_events.Track (Simulator::Schedule (DELAY (tuple->time),
                                           &RoutingProtocol::IfaceAssocTupleTimerExpire,
                                           this, ifaceAddr));
      }
      else {
          m_events.Track (Simulator::Schedule (tuple->time,
                                               &RoutingProtocol::IfaceAssocTupleTimerExpire,
                                               this, ifaceAddr));
      }
    }
}

///// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
///// \param e The event which has expired.
///
void
RoutingProtocol::AssociationTupleTimerExpire (Ipv4Address gatewayAddr, Ipv4Address networkAddr, Ipv4Mask netmask)
{
  AssociationTuple *tuple = m_state.FindAssociationTuple (gatewayAddr, networkAddr, netmask);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveAssociationTuple (*tuple);
    }
  else
    {
      if (DELAY_FLAG){
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::AssociationTupleTimerExpire,
                                           this, gatewayAddr, networkAddr, netmask));
      }
      else {
          m_events.Track (Simulator::Schedule (tuple->expirationTime,
                                               &RoutingProtocol::AssociationTupleTimerExpire,
                                               this, gatewayAddr, networkAddr, netmask));
      }
    }
}

///
/// \brief Clears the routing table and frees the memory assigned to each one of its entries.
///
void
RoutingProtocol::Clear ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_table.clear ();
}

///
/// \brief Deletes the entry whose destination address is given.
/// \param dest address of the destination node.
///
void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  m_table.erase (dest);
}

///
/// \brief Looks up an entry for the specified destination address.
/// \param dest destination address.
/// \param outEntry output parameter to hold the routing entry result, if fuond
/// \return     true if found, false if not found
///
bool
RoutingProtocol::Lookup (Ipv4Address const &dest,
                      RoutingTableEntry &outEntry) const
{
  // Get the iterator at "dest" position
  std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator it =
    m_table.find (dest);
  // If there is no route to "dest", return NULL
  if (it == m_table.end ())
    return false;
  outEntry = (*it).second;
  return true;
}




///
/// \brief      Finds the appropiate entry which must be used in order to forward
///             a data packet to a next hop (given a destination).
/// 
/// This method searches all routing table entries for the specific destination
/// and returns the entry with the lowest distance and/or lowest weight
/// 
///
///\param entry the routing table entry which indicates the destination node
///                     we are interested in.
///\param outEntry the selected entry from the routing table
///\param dest the destination of the packet
/// \return             the appropiate routing table entry which indicates the next
///                     hop which must be used for forwarding a data packet, or NULL
///                     if there is no such entry.
///
bool
RoutingProtocol::FindSendEntry (RoutingTableEntry const &entry,
                                RoutingTableEntry &outEntry,
                                Ipv4Address const &dest,
                                Ipv4Address const &src)
{
  outEntry = entry;
  double min_weight = entry.weight;
  NS_LOG_DEBUG("Min Weight =" << min_weight);
  std::pair<std::multimap<Ipv4Address, RoutingTableEntry>::iterator,
                          std::multimap<Ipv4Address, RoutingTableEntry>::iterator> ppp;

// equal_range(dest) returns pair<iterator,iterator> representing the range
// of element with key dest
  ppp = m_table.equal_range(dest);



// Based on the MIN WEIGHT and avoid to send it back
  for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = ppp.first;
    iter != ppp.second; ++iter){
        NS_LOG_DEBUG("Dest : " << (*iter).second.destAddr << " Next : "
                        <<(*iter).second.nextAddr << " " <<(*iter).second.weight);

        if ((*iter).second.weight <= min_weight && (*iter).second.nextAddr != src){
                  outEntry = (*iter).second;
                  min_weight = (*iter).second.weight;
          }
        }

  NS_LOG_DEBUG("Dest : " << outEntry.destAddr << " Selected Next : "
                  << outEntry.nextAddr << " weight: " << outEntry.weight);


  return true;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
                              const Ipv4Header &header,
                              Ptr<NetDevice> oif,
                              Socket::SocketErrno &sockerr)
{  
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId()
                        << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  // Entry1 the first found in the table, Entry2 the selected with min dist/weight
  RoutingTableEntry entry1, entry2;
  bool found = false;
  Ipv4Address destination = header.GetDestination ();
  Ipv4Address source = header.GetSource();

  /*
   * Implementation of simple location service
   * Check destination Address. If it is in PosAssociation Table.. update position
   * Else... add new entry
   *
   */
  if (header.GetProtocol() == 17){
      LocService(destination);
  }
  NS_LOG_DEBUG("Packet from node : " << m_ipv4->GetObject<Node> ()->GetId()
                                                  << " to : " << destination);
  /******************************************************************/


  if (Lookup (destination , entry1) != 0)
    {
      bool foundSendEntry = FindSendEntry (entry1, entry2, destination, source);
      if (!foundSendEntry)
        {
          NS_FATAL_ERROR ("FindSendEntry failure");
        }

      /******************************************************/
      /*
       * Check if the forwarding entry is 'myself' --> Local Maximum Problem
       * Recovery Method --> Carry'n'Forward
       * Cache the packet for either for some time or
       * Cache limited # of packets
       *  --- > CGGC
       *
       */
      // This node is selected as next to route and is NOT the destination
      // --> Local Maximum
      if (m_cacheFlag){
        if (entry2.nextAddr == m_mainAddress && destination!=m_mainAddress){
            NS_LOG_DEBUG (" Local Maximum Problem ");

        // Valid route not found, in this case we return loopback.
        // Actual route request will be deferred until packet will be fully formed,
        // routed to loopback, received from loopback and passed to RouteInput (see below)
           uint32_t iif = (oif ? m_ipv4->GetInterfaceForDevice (oif) : -1);
           DeferredRouteOutputTag tag (iif);
           if (! p->PeekPacketTag (tag))
             {
               p->AddPacketTag (tag);
             }
           return LoopbackRoute (header, oif);
        }
      }
      /*****************************************************/

      uint32_t interfaceIdx = entry2.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing search
          // if the caller specifies the oif; we just enforce that 
          // that the found route matches the requested outbound interface 
          NS_LOG_DEBUG ("Clwpr node " << m_mainAddress 
                        << ": RouteOutput for dest=" << destination
                        << " Route interface " << interfaceIdx 
                        << " does not match requested output interface "
                        << m_ipv4->GetInterfaceForDevice (oif));
          sockerr = Socket::ERROR_NOROUTETOHOST;
          return rtentry;
        }
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (destination);
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the 
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);  
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
        ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
      } else {
        NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and CLWPR");
      }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry2.nextAddr);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      NS_LOG_DEBUG ("Clwpr node " << m_mainAddress 
                    << ": RouteOutput for dest=" << destination
                    << " --> nextHop=" << entry2.nextAddr
                    << " interface=" << entry2.interface);
      NS_LOG_DEBUG ("Found route to " << destination << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice());
      found = true;
    }
  else
    { 
      rtentry = m_hnaRoutingTable->RouteOutput (p, header, oif, sockerr);
      
      if (rtentry)
        {
          found = true;
          NS_LOG_DEBUG ("Found route to " << destination << " via nh " << rtentry->GetGateway ()
                                          << " with source addr " << rtentry->GetSource ()
                                          << " and output dev " << rtentry->GetOutputDevice());
        }
    }
    
  if (!found)
    {
      NS_LOG_DEBUG ("Clwpr node " << m_mainAddress 
                    << ": RouteOutput for dest=" << destination
                    << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
    }
  return rtentry;
}

Ptr<Ipv4Route>
RoutingProtocol::LoopbackRoute (const Ipv4Header & hdr, Ptr<NetDevice> oif) const
{
  NS_LOG_DEBUG (" Loopback Route called");
  NS_LOG_FUNCTION (this << hdr);
  NS_ASSERT (m_ipv4->GetNetDevice(0) != 0);
  Ptr<Ipv4Route> rt = Create<Ipv4Route> ();
  rt->SetDestination (hdr.GetDestination ());
  //
  // Source address selection here is tricky.  The loopback route is
  // returned when CLWPR does not have a route; this causes the packet
  // to be looped back and handled (cached) in RouteInput() method
  // while a route is found. However, connection-oriented protocols
  // like TCP need to create an endpoint four-tuple (src, src port,
  // dst, dst port) and create a pseudo-header for checksumming.  So,
  // CLWPR needs to guess correctly what the eventual source address
  // will be.
  //
  // For single interface, single address nodes, this is not a problem.
  // When there are possibly multiple outgoing interfaces, the policy
  // implemented here is to pick the first available CLWPR interface.
  // If RouteOutput() caller specified an outgoing interface, that
  // further constrains the selection of source address
  //
  std::map<Ptr<Socket> , Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
  if (oif)
    {
      // Iterate to find an address on the oif device
      for (j = m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
        {
          Ipv4Address addr = j->second.GetLocal ();
          int32_t interface = m_ipv4->GetInterfaceForAddress (addr);
          if (oif == m_ipv4->GetNetDevice (static_cast<uint32_t> (interface)))
            {
              rt->SetSource (addr);
              break;
            }
        }
    }
  else
    {
      rt->SetSource (j->second.GetLocal ());
    }
  NS_ASSERT_MSG (rt->GetSource() != Ipv4Address (), "Valid CLWPR source address not found");
  rt->SetGateway (Ipv4Address ("127.0.0.1"));
  rt->SetOutputDevice (m_ipv4->GetNetDevice(0));
  return rt;
}

void
RoutingProtocol::DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header & header,
    UnicastForwardCallback ucb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << p << header);
  NS_ASSERT (p != 0 && p != Ptr<Packet> ());

  QueueEntry newEntry (p, header, ucb, ecb);
  bool result = m_queue.Enqueue (newEntry);

  // We do not use RReq messages...
  if (result)
    {
      NS_LOG_LOGIC ("Add packet " << p->GetUid() << " to queue. Protocol " << (uint16_t) header.GetProtocol ());
      NS_LOG_DEBUG (" PACKET QUEUED ");
      m_queueTrace(header, p);
    }
}

bool RoutingProtocol::RouteInput  (Ptr<const Packet> p, 
  const Ipv4Header &header, Ptr<const NetDevice> idev,                            
  UnicastForwardCallback ucb, MulticastForwardCallback mcb,             
  LocalDeliverCallback lcb, ErrorCallback ecb)
{   
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId() << " " << header.GetDestination ());
  
  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  NS_LOG_DEBUG ("Packet # "<< header.GetIdentification()<<" received at node :" << m_ipv4->GetObject<Node>()->GetId() << " for :" << dst);

  // Deferred route request
  if (idev == m_ipv4->GetNetDevice (0))
    {
      DeferredRouteOutputTag tag;
      if (p->PeekPacketTag (tag))
        {
          NS_LOG_DEBUG("Queue Packet");
          DeferredRouteOutput (p, header, ucb, ecb);
          return true;
        }
    }

  // Consume self-originated packets
  if (IsMyOwnAddress (origin) == true)
    {
      return true; 
    }
  
  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Local delivery to " << dst);
          lcb (p, header, iif);
          return true;
        }
      else
        {
          // The local delivery callback is null.  This may be a multicast
          // or broadcast packet, so return false so that another 
          // multicast routing protocol can handle it.  It should be possible
          // to extend this to explicitly check whether it is a unicast
          // packet, and invoke the error callback if so
          return false;
        }
    }
  
  // Forwarding
  if (header.GetProtocol() == 17){
      LocService(dst);
  }
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry1, entry2; 
  if (Lookup (dst , entry1))
    { 
      bool foundSendEntry = FindSendEntry (entry1, entry2, dst, origin);
      if (!foundSendEntry)
        NS_FATAL_ERROR ("FindSendEntry failure");

      /******************************************************/
      /*
       * Check if the forwarding entry is 'myself' --> Local Maximum Problem
       * Recovery Method --> Carry'n'Forward
       * Cache the packet for either for some time or
       * Cache limited # of packets
       *  --- > CGGC
       *
       */
      // This node is selected as next to route and is NOT the destination
      // --> Local Maximum
      if (m_cacheFlag){
        if (entry2.nextAddr == m_mainAddress && dst!=m_mainAddress){
            NS_LOG_DEBUG (" Local Maximum Problem ");

        // Valid route not found, in this case we return loopback.
        // Actual route request will be deferred until packet will be fully formed,
        // routed to loopback, received from loopback and passed back to RouteInput
           uint32_t iif = (idev ? m_ipv4->GetInterfaceForDevice (idev) : -1);
           DeferredRouteOutputTag tag (iif);
           if (! p->PeekPacketTag (tag))
             {
               p->AddPacketTag (tag);
             }

           rtentry = Create<Ipv4Route> ();
           rtentry->SetDestination (dst);


           //std::map<Ptr<Socket> , Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
           // ----------- TESTING -----------//
           rtentry->SetSource(header.GetSource());

           NS_ASSERT_MSG (rtentry->GetSource() != Ipv4Address (), "Valid CLWPR source address not found");
           rtentry->SetGateway (Ipv4Address ("127.0.0.1"));
           rtentry->SetOutputDevice (m_ipv4->GetNetDevice(0));

           NS_LOG_DEBUG ("Carry'n'Forward: clwpr node " << m_mainAddress
                         << ": RouteOutput for dest=" << dst
                         << " --> nextHop=" << entry2.nextAddr
                         << " interface=" << entry2.interface);

           ucb (rtentry, p, header);
           return true;
        }
      }
      /*****************************************************/


      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (dst);
      uint32_t interfaceIdx = entry2.interface;
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
        ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
      } else {
        NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and CLWPR");
      }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry2.nextAddr);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      
      NS_LOG_DEBUG ("Clwpr node " << m_mainAddress
                    << ": RouteOutput for dest=" << dst
                    << " --> nextHop=" << entry2.nextAddr
                    << " interface=" << entry2.interface);
      
      ucb (rtentry, p, header);
      return true;
    }
  else
    {
      if(m_hnaRoutingTable->RouteInput (p, header, idev, ucb, mcb, lcb, ecb))
        {
          return true;
        }
      else
        {
        
#ifdef NS3_LOG_ENABLE
          NS_LOG_DEBUG ("Clwpr node " << m_mainAddress 
                    << ": RouteInput for dest=" << header.GetDestination ()
                    << " --> NOT FOUND; ** Dumping routing table...");      
                    
          for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
             iter != m_table.end (); iter++)
          { 
            NS_LOG_DEBUG ("dest=" << (*iter).first << " --> next=" << (*iter).second.nextAddr
                        << " via interface " << (*iter).second.interface);
          }
      
          NS_LOG_DEBUG ("** Routing table dump end.");
#endif // NS3_LOG_ENABLE

          return false;
        }
    }
}
void 
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void 
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void 
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void 
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}


///
/// \brief Adds a new entry into the routing table.
///
/// If an entry for the given destination existed, it is deleted and freed.
///
/// \param dest         address of the destination node.
/// \param next         address of the next hop node.
/// \param iface        address of the local interface.
/// \param dist         distance to the destination node.
/// \param weight       weight of the neighbor node.
///
void
RoutingProtocol::AddEntry (Ipv4Address const &dest,
                        Ipv4Address const &next,
                        uint32_t interface,
                        double distance,
                        double weight)
{
  NS_LOG_FUNCTION (this << dest << next << interface << distance << weight << m_mainAddress);

  NS_ASSERT_MSG (distance >= 0, distance);

  // Creates a new rt entry with specified values
//  RoutingTableEntry &entry = m_table[dest];
  RoutingTableEntry entry ;
  entry.destAddr = dest;
  entry.nextAddr = next;
  entry.interface = interface,
  entry.distance = distance;
  entry.weight = weight;

// Adds a new entry in the routing table
  m_table.insert( std::pair<Ipv4Address, RoutingTableEntry> (dest, entry) );

}

void
RoutingProtocol::AddEntry (Ipv4Address const &dest,
                        Ipv4Address const &next,
                        Ipv4Address const &interfaceAddress,
                        double distance,
                        double weight)
{
  NS_LOG_FUNCTION (this << dest << next << interfaceAddress << distance << weight << m_mainAddress);

//  NS_ASSERT (distance > 0);
  NS_ASSERT (m_ipv4);
  std::cout << " NODE : "<<  m_ipv4 << "at :("<< GetNodePosition( m_ipv4 ).x<<","<< GetNodePosition( m_ipv4 ).y<<","
      << GetNodePosition( m_ipv4 ).z<<") adds .." << std::endl;
  std::cout << "Destination = " <<dest <<std::endl;
  std::cout << "Next = " <<next << std::endl;
  std::cout << "Interface = " <<interfaceAddress << std::endl;
  std::cout << "Distance = " << distance << std::endl;

  RoutingTableEntry entry;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
        {
          if (m_ipv4->GetAddress (i,j).GetLocal () == interfaceAddress)
            {
              AddEntry (dest, next, i, distance, weight);
              std::cout << "interface index = "<< i<<std::endl;
              return;
            }
        }
    }
  NS_ASSERT (false); // should not be reached
  AddEntry (dest, next, interfaceAddress, distance, weight);
}


std::vector<RoutingTableEntry>
RoutingProtocol::GetRoutingTableEntries () const
{
  std::vector<RoutingTableEntry> retval;
  for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
       iter != m_table.end (); iter++)
    {
      retval.push_back ((*iter).second);
    }
  return retval;
}



static class ClwprProtocolTestSuite : public TestSuite
{
public:
  ClwprProtocolTestSuite ();
} g_clwprProtocolTestSuite;

ClwprProtocolTestSuite::ClwprProtocolTestSuite()
  : TestSuite("routing-clwpr", UNIT)
{
}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  for (std::map<Ptr<Socket> , Ipv4InterfaceAddress>::const_iterator j =
      m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (a == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
}

void
RoutingProtocol::Dump (void)
{
  //Time now = Simulator::Now ();
//  std::cout<< "Dumping for node with main address " << m_mainAddress<<std::endl;
//  std::cout<<" Neighbor set"<<std::endl;
//
//  for (NeighborSet::const_iterator iter = m_state.GetNeighbors ().begin ();
//         iter != m_state.GetNeighbors ().end (); iter++)
//    {
//      std::cout<<"  " << *iter <<std::endl;
//    }
//
//
// std::cout << "Routing table"<<std::endl;
//  for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
//                        iter != m_table.end (); iter++)
//    {
//      std::cout << "  dest=" << (*iter).first << " --> next=" << (*iter).second.nextAddr << " via interface " << (*iter).second.interface <<std::endl;
//    }
//  std::cout<< "";

#ifdef NS3_LOG_ENABLE
  NS_LOG_DEBUG ("Dumping for node with main address " << m_mainAddress);
  NS_LOG_DEBUG (" Neighbor set");
  for (NeighborSet::const_iterator iter = m_state.GetNeighbors ().begin ();
         iter != m_state.GetNeighbors ().end (); iter++)
    {
      NS_LOG_DEBUG ("  " << *iter);
    }
  //NS_LOG_DEBUG (" Two-hop neighbor set");
  //for (TwoHopNeighborSet::const_iterator iter = m_state.GetTwoHopNeighbors ().begin ();
         //iter != m_state.GetTwoHopNeighbors ().end (); iter++)
    //{
      //if (now < iter->expirationTime)
        //{ 
          //NS_LOG_DEBUG ("  " << *iter);
        //}
    //}
  NS_LOG_DEBUG (" Routing table");
  for (std::multimap<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin (); 
                        iter != m_table.end (); iter++)
    {
      NS_LOG_DEBUG ("  dest=" << (*iter).first << " --> next=" << (*iter).second.nextAddr << " via interface " << (*iter).second.interface);
    }
  NS_LOG_DEBUG ("");
#endif  //NS3_LOG_ENABLE
}

void RoutingProtocol::LocService(Ipv4Address dest){

  NS_LOG_DEBUG(this << "Location Service");

  if (!m_init_list){
      InitNodeList();
      m_init_list = true;
  }
  Ptr<Ipv4> dest_ipv4;
  std::map <Ipv4Address, Ptr<Ipv4> >::iterator it;
  it = m_nodeList.find(dest);

  dest_ipv4 = (*it).second;
  PosAssociationTuple *tuple = m_state.FindPosAssociationTuple(dest);
  NS_LOG_DEBUG("Pos. Ass tuple " << tuple);

  if (tuple != NULL){ // Already exists.. update location
          NS_LOG_DEBUG (" Updating Pos.Ass. Entry");
          tuple->nodePosition = GetNodePosition(dest_ipv4);
          tuple->nodeHeading = GetNodeHeading(dest_ipv4);
          tuple->nodeRoadId = GetNodeRoadId(dest_ipv4);
          tuple->nodeVelocity = GetNodeVelocity(dest_ipv4);
          tuple->nodeTimestamp = Simulator::Now();
  }
  else { // New entry
          NS_LOG_DEBUG(" Adding new PositionAss. Entry");
          PosAssociationTuple dst;
          dst.nodeHeading = GetNodeHeading(dest_ipv4);
          NS_LOG_DEBUG("Position " << GetNodePosition(dest_ipv4));
          dst.nodeMainAddr = dest;
          dst.nodePosition = GetNodePosition(dest_ipv4);
          dst.nodeRoadId = GetNodeRoadId(dest_ipv4);
          dst.nodeVelocity = GetNodeVelocity(dest_ipv4);
          dst.nodeTimestamp = Simulator::Now();
          m_state.InsertPosAssociationTuple(dst);
  }
  RoutingTableComputation();
}

void
RoutingProtocol::SendPacketFromQueue (Ipv4Address dst, Ptr<Ipv4Route> route)
{
  NS_LOG_FUNCTION (this);
//  LocService(dst);
  QueueEntry queueEntry;

  //  while (m_queue.Dequeue (dst, queueEntry))

  if (m_queue.Dequeue (dst, queueEntry))
    {
      NS_LOG_DEBUG("Queue Size = " << m_queue.GetSize());
      DeferredRouteOutputTag tag;
      Ptr<Packet> p = ConstCast<Packet> (queueEntry.GetPacket ());
      if (p->RemovePacketTag (tag) &&
          tag.oif != -1 &&
          tag.oif != m_ipv4->GetInterfaceForDevice (route->GetOutputDevice ()))
        {
          NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
          m_dropCnFTrace( queueEntry.GetIpv4Header (), p, m_ipv4);
          return;
        }
      UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback ();
      Ipv4Header header = queueEntry.GetIpv4Header ();
      header.SetSource (route->GetSource ());
      header.SetTtl (header.GetTtl() + 1); // compensate extra TTL decrement by fake loopback routing
      ucb (route, p, header);
      m_dequeueTrace(header, p);
    }
}

}} // namespace clwpr, ns3



