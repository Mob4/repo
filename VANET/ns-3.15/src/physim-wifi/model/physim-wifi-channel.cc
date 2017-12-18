/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA, 2009-2010 Jens Mittag, Stylianos Papanastasiou
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
 * Based on yans-wifi-channel.cc by:
 *      Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Major modifications by:
 *      Jens Mittag <jens.mittag@kit.edu>
 *      Stylianos Papanastasiou <stylianos@gmail.com>
 */

// FD: Display information about packet where the destination is not the interface
//#define ALLDISPLAY

#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/object-factory.h"
#include "physim-wifi-channel.h"
#include "physim-wifi-phy.h"
#include "physim-propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"

#include <ns3/cross-layer-tag.h>
#include <ns3/wifi-mac-header.h>
#include <ns3/wifi-net-device.h>

NS_LOG_COMPONENT_DEFINE ("PhySimWifiChannel");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PhySimWifiChannel);

TypeId
PhySimWifiChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PhySimWifiChannel")
    .SetParent<WifiChannel> ();
  return tid;
}


NS_OBJECT_ENSURE_REGISTERED (PhySimWifiUniformChannel);

TypeId
PhySimWifiUniformChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PhySimWifiUniformChannel")
    .SetParent<PhySimWifiChannel> ()
    .AddConstructor<PhySimWifiUniformChannel> ()
    .AddAttribute ("PhySimPropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&PhySimWifiUniformChannel::m_loss),
                   MakePointerChecker<PhySimPropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&PhySimWifiUniformChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
  ;
  return tid;
}

PhySimWifiUniformChannel::PhySimWifiUniformChannel ()
{
}
PhySimWifiUniformChannel::~PhySimWifiUniformChannel ()
{
  m_phyList.clear ();
}

uint32_t
PhySimWifiUniformChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}

Ptr<NetDevice>
PhySimWifiUniformChannel::GetDevice (uint32_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
PhySimWifiUniformChannel::Add (Ptr<PhySimWifiPhy> phy)
{
  m_phyList.push_back (phy);
}

void
PhySimWifiUniformChannel::SetPropagationLossModel (Ptr<PhySimPropagationLossModel> loss)
{
  m_loss = loss;
}

void
PhySimWifiUniformChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay)
{
  m_delay = delay;
}

void
PhySimWifiUniformChannel::Send (Ptr<PhySimWifiPhy> sender, Ptr<const Packet> packet, Ptr<const PhySimWifiPhyTag> tag)
{
  // To decode Mac Address
  WifiMacHeader wifiMacHeader;
  packet->PeekHeader (wifiMacHeader);
  Mac48Address dest = wifiMacHeader.GetAddr1() ;
  //Mac48Address src = wifiMacHeader.GetAddr2() ;
  int packetNumberBrd = -1 ;

  // BH added 24/08/2011
  Ptr<Object> srcNetDevice=sender->GetDevice();
  uint32_t srcNode = srcNetDevice->GetObject<NetDevice>()->GetNode()->GetId();

  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  NS_ASSERT (senderMobility != 0);
  uint32_t j = 0;
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++)
    {
      if (sender != (*i))
        {
          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          Time delay = m_delay->GetDelay (senderMobility, receiverMobility);

          // copy packet and tag object
          Ptr<Packet> packetcopy = packet->Copy ();
          Ptr<PhySimWifiPhyTag> tagcopy = Create<PhySimWifiPhyTag> (*tag);

          // set rx net device object on the tag
          tagcopy->SetRxNetDevice ((*i)->GetDevice ()->GetObject<NetDevice> ());

          // determine receiver node id
          uint32_t dstNode = (*i)->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();

          // apply propagation loss models
          m_loss->CalcRxPower (tagcopy, senderMobility, receiverMobility);

          // display only for correct destination
          bool display = true ;
          Ptr<NetDevice> dev = (*i)->GetDevice ()->GetObject<NetDevice> () ;
          Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();

          if (wifi != NULL) {
            if (!dest.IsBroadcast()) {
              // Not a broadcast and not the destination
              if (wifi->GetAddress() != dest) display = false ;
            }
            else {

#ifdef ALLDISPLAY
              packetNumberBrd = -1  ;
#endif
              // Is a broadcast => display only once the 
              if (packetNumberBrd == -1) packetNumberBrd = (int)packet->GetUid() ;
              else display = false ;
            }
          }

          if (display) {

            if (!dest.IsBroadcast()) {
              NS_LOG_UNCOND ("PhySimWifiUniformChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
                             << " CalcRxPower ran for packet " << packet->GetUid()
                             << " (" << srcNode << "->" << dstNode
                             << ") Pos(" << srcNode
                             << ")(" << senderMobility->GetPosition() 
                             << ") Pos(" << dstNode
                             << ")(" << receiverMobility->GetPosition()
                             << ") txPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetTxedSamples()))
                             << " rxPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetRxedSamples()))
                             << " dist: " << senderMobility->GetDistanceFrom (receiverMobility)
                             << " relSpeed: " << senderMobility->GetRelativeSpeed (receiverMobility)
                             << " size: " << packet->GetSize()
                             << " type " << wifiMacHeader.GetTypeString()
                             ) ;
            }
            else {
              NS_LOG_UNCOND ("PhySimWifiUniformChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
                             << " CalcRxPower ran for packet " << packet->GetUid()
                             << " (" << srcNode << "->" << dest
                             << ") Pos(" << srcNode
                             << ")(" << senderMobility->GetPosition() 
                             //<< ") Pos(" << dstNode
                             //<< ")(" << receiverMobility->GetPosition()
                             << ") txPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetTxedSamples()))
                             //<< " rxPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetRxedSamples()))
                             //<< " dist: " << senderMobility->GetDistanceFrom (receiverMobility)
                             //<< " relSpeed: " << senderMobility->GetRelativeSpeed (receiverMobility)
                             << " size: " << packet->GetSize()
                             << " type " << wifiMacHeader.GetTypeString()
                             ) ;
            }
          }
#ifdef ALLDISPLAY
          else {
            NS_LOG_UNCOND ("PhySimWifiUniformChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
                           << " -- not the destination -- "
                           << " CalcRxPower ran for packet " << packet->GetUid()
                           << " (" << srcNode << "->" << dstNode
                           << ") Pos(" << srcNode
                           << ")(" << senderMobility->GetPosition()
                           << ") Pos(" << dstNode
                           << ")(" << receiverMobility->GetPosition()
                           << ") txPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetTxedSamples()))
                           << " rxPower: " << PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetRxedSamples()))
                           << " dist: " << senderMobility->GetDistanceFrom (receiverMobility)
                           << " relSpeed: " << senderMobility->GetRelativeSpeed (receiverMobility)
                           << " size: " << packet->GetSize()
                           << " type " << wifiMacHeader.GetTypeString()
                           ) ;
          }
#endif
          // In iv4-raw-socket-impl.cc, to add a Ipv4PacketInfoTag they remove the tag from the copy and than add it.
          // If not, simulations crash
          ClTagNode clTN;
          packetcopy->RemovePacketTag(clTN);
          ClTagTrans clTT;
          packetcopy->RemovePacketTag(clTT);
          ClTagInfo clTI;
          packetcopy->RemovePacketTag(clTI);
          // Set the nodes information to Node tag
          clTN.SetSrcNode (srcNode);
          clTN.SetDstNode (dstNode);
          clTN.SetSrcXYZ (senderMobility->GetPosition());
          clTN.SetDstXYZ (receiverMobility->GetPosition());
          packetcopy->AddPacketTag(clTN);
          // Set the transmission information of Trans tag
          clTT.SetTxPower (PhySimHelper::PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetTxedSamples())));
          clTT.SetRxPower (PhySimHelper::PhySimHelper::WToDbm (PhySimHelper::GetOFDMSymbolSignalStrength (tagcopy->GetRxedSamples())));
          packetcopy->AddPacketTag(clTT);
          // Set the size and speed information of Info tag
          clTI.SetRelSpeed (senderMobility->GetRelativeSpeed (receiverMobility));
          clTI.SetPacketSize (packet->GetSize());
          packetcopy->AddPacketTag(clTI);

          // schedule the reception event at the receiver
          Simulator::ScheduleWithContext (dstNode, delay, &PhySimWifiUniformChannel::Receive, this,
                                          j, packetcopy, tagcopy);
        }
      j++;
    }
}

void
PhySimWifiUniformChannel::Receive (uint32_t i, Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag) const
{
  m_phyList[i]->StartReceivePacket (packet, tag);
}






NS_OBJECT_ENSURE_REGISTERED (PhySimWifiManualChannel);

TypeId
PhySimWifiManualChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PhySimWifiManualChannel")
    .SetParent<PhySimWifiChannel> ()
    .AddConstructor<PhySimWifiManualChannel> ()
    .AddAttribute ("DefaultPropagationLossModel", "A pointer to the default propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&PhySimWifiManualChannel::m_defaultLoss),
                   MakePointerChecker<PhySimPropagationLossModel> ())
    .AddAttribute ("DefaultPropagationDelayModel", "A pointer to the default propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&PhySimWifiManualChannel::m_defaultDelay),
                   MakePointerChecker<PropagationDelayModel> ())
  ;
  return tid;
}

PhySimWifiManualChannel::PhySimWifiManualChannel ()
{
}
PhySimWifiManualChannel::~PhySimWifiManualChannel ()
{
  m_phyList.clear ();
}

uint32_t
PhySimWifiManualChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}

Ptr<NetDevice>
PhySimWifiManualChannel::GetDevice (uint32_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
PhySimWifiManualChannel::Add (Ptr<PhySimWifiPhy> phy)
{
  m_phyList.push_back (phy);
}

void
PhySimWifiManualChannel::SetPropagationLossModel (Ptr<PhySimPropagationLossModel> loss)
{
  NS_LOG_FUNCTION (this << loss);
  m_defaultLoss = loss;
}
void
PhySimWifiManualChannel::SetExplicitPropagationLossModel (uint32_t sender, uint32_t receiver, Ptr<PhySimPropagationLossModel> loss)
{
  m_loss[sender].insert ( std::pair<uint32_t, Ptr<PhySimPropagationLossModel> > (receiver, loss) );
}

void
PhySimWifiManualChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay)
{
  NS_LOG_FUNCTION (this << delay);
  m_defaultDelay = delay;
}
void
PhySimWifiManualChannel::SetExplicitPropagationDelayModel (uint32_t sender, uint32_t receiver, Ptr<PropagationDelayModel> delay)
{
  m_delay[sender].insert ( std::pair<uint32_t, Ptr<PropagationDelayModel> > (receiver, delay) );
}

void
PhySimWifiManualChannel::Send (Ptr<PhySimWifiPhy> sender, Ptr<const Packet> packet, Ptr<const PhySimWifiPhyTag> tag)
{
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  const uint32_t senderId = sender->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();
  NS_ASSERT (senderMobility != 0);
  uint32_t j = 0;
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++)
    {
      if (sender != (*i))
        {
          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();

          const uint32_t receiverId = (*i)->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();

          // Check if we have a manually configured delay model
          Time delay;
          if ( m_delay.find (senderId) != m_delay.end ()
               && (m_delay.find (senderId)->second).find (receiverId) != (m_delay.find (senderId)->second).end () )
            {
              NS_LOG_DEBUG ("PhySimWifiManualChannel::Send() - using manually selected delay model.");
              delay = ((m_delay.find (senderId)->second).find (receiverId)->second)->GetDelay (senderMobility, receiverMobility);
            }
          else
            {
              NS_LOG_DEBUG ("PhySimWifiManualChannel::Send() - using default delay model.");
              delay = m_defaultDelay->GetDelay (senderMobility, receiverMobility);
            }

          // copy packet and tag object
          Ptr<Packet> packetcopy = packet->Copy ();
          Ptr<PhySimWifiPhyTag> tagcopy = Create<PhySimWifiPhyTag> (*tag);

          // Check if we have a manually configured propagation loss models
          if ( m_loss.find (senderId) != m_loss.end ()
               && (m_loss.find (senderId)->second).find (receiverId) != (m_loss.find (senderId)->second).end () )
            {
              NS_LOG_DEBUG ("PhySimWifiManualChannel::Send() - using manually selected propagation loss model(s).");
              ((m_loss.find (senderId)->second).find (receiverId)->second)->CalcRxPower (tagcopy, senderMobility, receiverMobility);
            }
          else
            {
              NS_LOG_DEBUG ("PhySimWifiManualChannel::Send() - using default propagation loss model(s).");
              m_defaultLoss->CalcRxPower (tagcopy, senderMobility, receiverMobility);
            }

          // set rx net device object on the tag
          tagcopy->SetRxNetDevice ((*i)->GetDevice ()->GetObject<NetDevice> ());

          uint32_t dstNode = (*i)->GetDevice ()->GetObject<NetDevice> ()->GetNode ()->GetId ();
          Simulator::ScheduleWithContext (dstNode, delay, &PhySimWifiManualChannel::Receive, this,
                                          j, packetcopy, tagcopy);
        }
      j++;
    }
}

void
PhySimWifiManualChannel::Receive (uint32_t i, Ptr<Packet> packet, Ptr<PhySimWifiPhyTag> tag) const
{
  m_phyList[i]->StartReceivePacket (packet, tag);
}

} // namespace ns3
