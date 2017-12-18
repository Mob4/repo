/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
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
 * Author: Mathieu Lacage, <mathieu.lacage@sophia.inria.fr>
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
#include "yans-wifi-channel.h"
#include "yans-wifi-phy.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"

#include <ns3/cross-layer-tag.h>
#include <ns3/wifi-mac-header.h>
#include <ns3/wifi-net-device.h>

NS_LOG_COMPONENT_DEFINE ("YansWifiChannel");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (YansWifiChannel);

TypeId
YansWifiChannel::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::YansWifiChannel")
    		.SetParent<WifiChannel> ()
    		.AddConstructor<YansWifiChannel> ()
    		.AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
    				PointerValue (),
    				MakePointerAccessor (&YansWifiChannel::m_loss),
    				MakePointerChecker<PropagationLossModel> ())
    				.AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
    						PointerValue (),
    						MakePointerAccessor (&YansWifiChannel::m_delay),
    						MakePointerChecker<PropagationDelayModel> ())
    						;
	return tid;
}

YansWifiChannel::YansWifiChannel ()
{
}
YansWifiChannel::~YansWifiChannel ()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_phyList.clear ();
}

void
YansWifiChannel::SetPropagationLossModel (Ptr<PropagationLossModel> loss)
{
	m_loss = loss;
}
void
YansWifiChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay)
{
	m_delay = delay;
}

void
YansWifiChannel::Send (Ptr<YansWifiPhy> sender, Ptr<const Packet> packet, double txPowerDbm,
		WifiMode wifiMode, WifiPreamble preamble) const
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
	for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
	{
		if (sender != (*i))
		{
			// For now don't account for inter channel interference
			if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
			{
				continue;
			}

			Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
			Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
			double rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
			NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
					"distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);
			Ptr<Packet> copy = packet->Copy ();
			Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
			uint32_t dstNode;
			if (dstNetDevice == 0)
			{
				dstNode = 0xffffffff;
			}
			else
			{
				dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
			}


			// -----------------------------------------------------------------------------
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
					NS_LOG_UNCOND ("YansWifiChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
							<< " CalcRxPower ran for packet " << packet->GetUid()
							<< " (" << srcNode << "->" << dstNode
							<< ") Pos(" << srcNode
							<< ")(" << senderMobility->GetPosition()
							<< ") Pos(" << dstNode
							<< ")(" << receiverMobility->GetPosition()
							<< ") txPower: " << txPowerDbm
							<< " rxPower: " << rxPowerDbm
							<< " dist: " << senderMobility->GetDistanceFrom (receiverMobility)
							<< " relSpeed: " << senderMobility->GetRelativeSpeed (receiverMobility)
							<< " size: " << packet->GetSize()
							<< " type " << wifiMacHeader.GetTypeString()
					) ;
				}
				else {
					NS_LOG_UNCOND ("YansWifiChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
							<< " CalcRxPower ran for packet " << packet->GetUid()
							<< " (" << srcNode << "->" << dest
							<< ") Pos(" << srcNode
							<< ")(" << senderMobility->GetPosition()
							//<< ") Pos(" << dstNode
							//<< ")(" << receiverMobility->GetPosition()
							<< ") txPower: " << txPowerDbm
							//<< " rxPower: " << rxPowerDbm
							//<< " dist: " << senderMobility->GetDistanceFrom (receiverMobility)
							//<< " relSpeed: " << senderMobility->GetRelativeSpeed (receiverMobility)
							<< " size: " << packet->GetSize()
							<< " type " << wifiMacHeader.GetTypeString()
					) ;
				}
			}
#ifdef ALLDISPLAY
			else {
				NS_LOG_UNCOND ("YansWifiChannel::Send" << ": +AtPhy " << Simulator::Now().GetSeconds()
						<< " -- not the destination -- "
						<< " CalcRxPower ran for packet " << packet->GetUid()
						<< " (" << srcNode << "->" << dstNode
						<< ") Pos(" << srcNode
						<< ")(" << senderMobility->GetPosition()
						<< ") Pos(" << dstNode
						<< ")(" << receiverMobility->GetPosition()
						<< ") txPower: " << txPowerDbm
						<< " rxPower: " << rxPowerDbm
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
			copy->RemovePacketTag(clTN);
			ClTagTrans clTT;
			copy->RemovePacketTag(clTT);
			ClTagInfo clTI;
			copy->RemovePacketTag(clTI);
			// Set the nodes information to Node tag
			clTN.SetSrcNode (srcNode);
			clTN.SetDstNode (dstNode);
			clTN.SetSrcXYZ (senderMobility->GetPosition());
			clTN.SetDstXYZ (receiverMobility->GetPosition());
			copy->AddPacketTag(clTN);
			// Set the transmission information of Trans tag
			clTT.SetTxPower (txPowerDbm) ;
			clTT.SetRxPower (rxPowerDbm) ;
			copy->AddPacketTag(clTT);
			// Set the size and speed information of Info tag
			clTI.SetRelSpeed (senderMobility->GetRelativeSpeed (receiverMobility));
			clTI.SetPacketSize (packet->GetSize());
			copy->AddPacketTag(clTI);

			Simulator::ScheduleWithContext (dstNode,
					delay, &YansWifiChannel::Receive, this,
					j, copy, rxPowerDbm, wifiMode, preamble);
		}
	}
}

void
YansWifiChannel::Receive (uint32_t i, Ptr<Packet> packet, double rxPowerDbm,
		WifiMode txMode, WifiPreamble preamble) const
{
	m_phyList[i]->StartReceivePacket (packet, rxPowerDbm, txMode, preamble);
}

uint32_t
YansWifiChannel::GetNDevices (void) const
{
	return m_phyList.size ();
}
Ptr<NetDevice>
YansWifiChannel::GetDevice (uint32_t i) const
{
	return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
YansWifiChannel::Add (Ptr<YansWifiPhy> phy)
{
	m_phyList.push_back (phy);
}

int64_t
YansWifiChannel::AssignStreams (int64_t stream)
{
	int64_t currentStream = stream;
	currentStream += m_loss->AssignStreams (stream);
	return (currentStream - stream);
}

} // namespace ns3
