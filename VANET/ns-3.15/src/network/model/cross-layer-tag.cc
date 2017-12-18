/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/cross-layer-tag.h"
#include "ns3/log.h"

#include "ns3/uinteger.h"
#include "ns3/integer.h"

namespace ns3 {

// ClTagNode implementation ---------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (ClTagNode);

ClTagNode::ClTagNode () { }

TypeId
ClTagNode::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::ClTagNode")
    						  .SetParent<Tag> ()
    						  .AddConstructor<ClTagNode> ()
    						  ;
	return tid;
}

TypeId
ClTagNode::GetInstanceTypeId (void) const {
	return GetTypeId ();
}

uint32_t
ClTagNode::GetSerializedSize (void) const {
	return (
			sizeof(m_srcNode) +
			sizeof(m_dstNode) +
			sizeof(m_srcX) +
			sizeof(m_srcY) +
			sizeof(m_srcZ) +
			sizeof(m_dstX) +
			sizeof(m_dstY) +
			sizeof(m_dstZ) ) ;
}

void
ClTagNode::Serialize (TagBuffer i) const {
	i.WriteU16 (m_srcNode);
	i.WriteU16 (m_dstNode);
	i.WriteU16 (m_srcX);
	i.WriteU16 (m_srcY);
	i.WriteU16 (m_srcZ);
	i.WriteU16 (m_dstX);
	i.WriteU16 (m_dstY);
	i.WriteU16 (m_dstZ);
}

void ClTagNode::Deserialize (TagBuffer i) {
	m_srcNode=i.ReadU16 ();
	m_dstNode=i.ReadU16 () ;
	m_srcX=i.ReadU16 ();
	m_srcY=i.ReadU16 ();
	m_srcZ=i.ReadU16 ();
	m_dstX=i.ReadU16 ();
	m_dstY=i.ReadU16 ();
	m_dstZ=i.ReadU16 ();
}

void ClTagNode::Print (std::ostream &os) const
{
	os << " from node "
			<< m_srcNode
			<< " (" <<  m_srcX << ", " << m_srcY << ", " << m_srcZ << ")"
			<< " to node "
			<< m_dstNode
			<< " (" <<  m_dstX << ", " << m_dstY  << ", " << m_dstZ << ")";
}

void ClTagNode::SetSrcNode (uint16_t srcNode) { m_srcNode=srcNode; }
void ClTagNode::SetDstNode (uint16_t dstNode) { m_dstNode=dstNode; }
void ClTagNode::SetSrcX(uint16_t x) { m_srcX = x ; }
void ClTagNode::SetSrcY(uint16_t y) { m_srcY = y ; }
void ClTagNode::SetSrcZ(uint16_t z) { m_srcZ = z ; }
void ClTagNode::SetDstX(uint16_t x) { m_dstX = x ; }
void ClTagNode::SetDstY(uint16_t y) { m_dstY = y ; }
void ClTagNode::SetDstZ(uint16_t z) { m_dstZ = z ; }
void ClTagNode::SetSrcXYZ(Vector v) { m_srcX = v.x ; m_srcY = v.y ; m_srcZ = v.z ; }
void ClTagNode::SetDstXYZ(Vector v) { m_dstX = v.x ; m_dstY = v.y ; m_dstZ = v.z ; }

uint16_t ClTagNode::GetSrcNode (void) const { return m_srcNode; }
uint16_t ClTagNode::GetDstNode (void) const { return m_dstNode; }
uint16_t ClTagNode::GetSrcX(void) const{ return m_srcX; }
uint16_t ClTagNode::GetSrcY(void) const{ return m_srcY; }
uint16_t ClTagNode::GetSrcZ(void) const{ return m_srcZ; }
uint16_t ClTagNode::GetDstX(void) const{ return m_dstX; }
uint16_t ClTagNode::GetDstY(void) const{ return m_dstY; }
uint16_t ClTagNode::GetDstZ(void) const{ return m_dstZ; }

double ClTagNode::GetDistance() const {
	return sqrt((m_srcX - m_dstX)*(m_srcX - m_dstX)
			+(m_srcY - m_dstY)*(m_srcY - m_dstY)
			+(m_srcZ - m_dstZ)*(m_srcZ - m_dstZ)
	) ;
}

// ClTagTrans implementation ---------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (ClTagTrans);

ClTagTrans::ClTagTrans() { }

TypeId
ClTagTrans::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::ClTagTrans")
    						  .SetParent<Tag> ()
    						  .AddConstructor<ClTagTrans> ()
    						  ;
	return tid;
}

TypeId
ClTagTrans::GetInstanceTypeId (void) const
{
	return GetTypeId ();
}

uint32_t
ClTagTrans::GetSerializedSize (void) const
{
	return sizeof (m_txPowerDbm)
			+ sizeof (m_rxPowerDbm) ;
}

void
ClTagTrans::Serialize (TagBuffer i) const
{
	i.WriteDouble (m_txPowerDbm);
	i.WriteDouble (m_rxPowerDbm);
}

void
ClTagTrans::Deserialize (TagBuffer i)
{
	m_txPowerDbm=i.ReadDouble ();
	m_rxPowerDbm=i.ReadDouble ();
}

void
ClTagTrans::Print (std::ostream &os) const
{
	os << "ClTagTrans:"
			<< " TxPower: " << m_txPowerDbm
			<< " RxPower: " << m_rxPowerDbm ;
}

void
ClTagTrans::SetTxPower (double txpowerDbm){
	m_txPowerDbm=txpowerDbm;
}
void
ClTagTrans::SetRxPower (double rxpowerDbm){
	m_rxPowerDbm=rxpowerDbm;
}

double
ClTagTrans::GetTxPower (void) const{
	return m_txPowerDbm;
}

double
ClTagTrans::GetRxPower (void) const{
	return m_rxPowerDbm;
}

// ClTagSnr implementation ---------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (ClTagSnr);

ClTagSnr::ClTagSnr() { }

TypeId
ClTagSnr::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::ClTagSnr")
    						  .SetParent<Tag> ()
    						  .AddConstructor<ClTagSnr> ()
    						  ;
	return tid;
}

TypeId
ClTagSnr::GetInstanceTypeId (void) const
{
	return GetTypeId ();
}

uint32_t
ClTagSnr::GetSerializedSize (void) const
{
	return sizeof (m_snr)
			+ sizeof (m_teb) ;
}

void
ClTagSnr::Serialize (TagBuffer i) const
{
	i.WriteDouble (m_snr);
	i.WriteDouble (m_teb);
}

void
ClTagSnr::Deserialize (TagBuffer i)
{
	m_snr=i.ReadDouble ();
	m_teb=i.ReadDouble ();
}

void
ClTagSnr::Print (std::ostream &os) const
{
	os << "ClTagSnr:"
			<< " snr: " << m_snr
			<< " teb: " << m_teb ;
}

void
ClTagSnr::SetSnr (double snr) {
	m_snr=snr;
}

void
ClTagSnr::SetPer (double teb) {
	m_teb=teb;
}

double
ClTagSnr::GetSnr (void) const{
	return m_snr;
}

double
ClTagSnr::GetPer (void) const{
	return m_teb;
}

// ClTagInfo implementation ---------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (ClTagInfo);

ClTagInfo::ClTagInfo() { }

TypeId
ClTagInfo::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::ClTagInfo")
    						  .SetParent<Tag> ()
    						  .AddConstructor<ClTagInfo> ()
    						  ;
	return tid;
}

TypeId
ClTagInfo::GetInstanceTypeId (void) const
{
	return GetTypeId ();
}

uint32_t
ClTagInfo::GetSerializedSize (void) const
{
	return sizeof (m_packetSize)
			+ sizeof (m_relSpeed) ;
}

void
ClTagInfo::Serialize (TagBuffer i) const
{
	i.WriteU16 (m_packetSize);
	i.WriteDouble (m_relSpeed);
}

void
ClTagInfo::Deserialize (TagBuffer i)
{
	m_packetSize=i.ReadU16 ();
	m_relSpeed=i.ReadDouble ();
}

void
ClTagInfo::Print (std::ostream &os) const
{
	os << "ClTagInfo:"
			<< " packetSize: " << m_packetSize
			<< " relSpeed: " << m_relSpeed ;
}

void
ClTagInfo::SetPacketSize (uint16_t pktSize) {
	m_packetSize=pktSize;
}

void
ClTagInfo::SetRelSpeed (double relSpeed) {
	m_relSpeed=relSpeed;
}

uint16_t
ClTagInfo::GetPacketSize (void) const{
	return m_packetSize;
}

double
ClTagInfo::GetRelSpeed (void) const{
	return m_relSpeed;
}

// ClTagXLayerInformation class definition ---------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (ClTagXLayerInformation);

ClTagXLayerInformation::ClTagXLayerInformation() { }

TypeId
ClTagXLayerInformation::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::ClTagXLayerInformation")
    						  .SetParent<Tag> ()
    						  .AddConstructor<ClTagXLayerInformation> ()
    						  ;
	return tid;
}

TypeId
ClTagXLayerInformation::GetInstanceTypeId (void) const
{
	return GetTypeId ();
}

uint32_t
ClTagXLayerInformation::GetSerializedSize (void) const
{
	return sizeof (m_rxPower) +
			sizeof (m_snr) +
			sizeof (m_retransmission) ;
}

void
ClTagXLayerInformation::Serialize (TagBuffer i) const
{
	i.WriteDouble (m_rxPower);
	i.WriteDouble (m_snr);
	i.WriteU32 (m_retransmission);
}

void
ClTagXLayerInformation::Deserialize (TagBuffer i)
{
	m_rxPower=i.ReadDouble ();
	m_snr=i.ReadDouble ();
	m_retransmission=i.ReadU32 ();
}

void
ClTagXLayerInformation::Print (std::ostream &os) const
{
	os << "ClTagXLayerInformation:"
			<< " RX Power: " << m_rxPower
			<< " SNR: " << m_snr
			<< " Retransmission: " << m_retransmission ;
}


}; // namespace ns3
