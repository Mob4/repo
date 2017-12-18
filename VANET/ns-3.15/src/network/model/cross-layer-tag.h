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

#ifndef CROSS_LAYER_TAG_H
#define CROSS_LAYER_TAG_H

#include "ns3/tag.h"
#include "ns3/ref-count-base.h"

#include "ns3/address.h"
#include "ns3/nstime.h"

#include "ns3/vector.h"
#include <ns3/wifi-mac-header.h>

namespace ns3 {

// ClTagNode class definition ---------------------------------------------
class ClTagNode: public Tag, public RefCountBase
{
public:
	ClTagNode() ;
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	uint16_t GetSrcNode (void) const;
	uint16_t GetDstNode (void) const;

	uint16_t GetSrcX(void) const;
	uint16_t GetSrcY(void) const;
	uint16_t GetSrcZ(void) const;
	uint16_t GetDstX(void) const;
	uint16_t GetDstY(void) const;
	uint16_t GetDstZ(void) const;

	void SetSrcNode (uint16_t srcNode);
	void SetDstNode (uint16_t dstNode);

	void SetSrcX(uint16_t x);
	void SetSrcY(uint16_t y);
	void SetSrcZ(uint16_t z);
	void SetDstX(uint16_t x);
	void SetDstY(uint16_t y);
	void SetDstZ(uint16_t z);
	void SetSrcXYZ(Vector v);
	void SetDstXYZ(Vector v);

	double GetDistance() const;

private:
	uint16_t m_srcNode;
	uint16_t m_dstNode;
	uint16_t m_srcX;
	uint16_t m_srcY;
	uint16_t m_srcZ;
	uint16_t m_dstX;
	uint16_t m_dstY;
	uint16_t m_dstZ;
};

// ClTagTrans class definition ---------------------------------------------
class ClTagTrans : public Tag, public RefCountBase
{
public:
	ClTagTrans ();
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	void SetTxPower (double txPowerDbm);
	void SetRxPower (double rxPowerDbm);

	double GetTxPower (void) const;
	double GetRxPower (void) const;

private:
	double m_txPowerDbm;
	double m_rxPowerDbm;
};

// ClTagPower class definition ---------------------------------------------
class ClTagSnr : public Tag, public RefCountBase
{
public:
	ClTagSnr ();
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	void SetSnr (double);
	double GetSnr (void) const;
	void SetPer (double);
	double GetPer (void) const;

private:
	double m_snr;
	double m_teb;
};

// ClTagInfo class definition ---------------------------------------------
class ClTagInfo : public Tag, public RefCountBase
{
public:
	ClTagInfo ();
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	void SetPacketSize (uint16_t);
	uint16_t GetPacketSize (void) const;
	void SetRelSpeed (double);
	double GetRelSpeed (void) const;

private:
	uint16_t m_packetSize;
	double m_relSpeed;
};

/**
 * ClTagXLayerInformation class definition ---------------------------------------------
 * This class is used to transmit X layer information the destination (used only by AODV modified by F. Drouhin
 */
class ClTagXLayerInformation: public Tag, public RefCountBase
{
public:
	ClTagXLayerInformation() ;

	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	double GetRxPower() const {
		return m_rxPower;
	}

	void SetRxPower(double rxPower) {
		this->m_rxPower = rxPower;
	}

	double GetSnr() const {
		return m_snr;
	}

	void SetSnr(double snr) {
		this->m_snr = snr;
	}

	uint32_t GetRetransmission() const {
		return m_retransmission;
	}

	void SetRetransmission(uint32_t retransmission) {
		m_retransmission = retransmission;
	}

	void AddRetransmission() {
		m_retransmission = m_retransmission + 1 ;
	}

private:
	double m_rxPower ; // the RX power transmit is the minimum one of the different links used
	double m_snr     ; // the SNR value is the minimum one of the different links used
	uint32_t m_retransmission ; // the retransmission number of message
};
}; // namespace ns3

#endif /* CROSS_LAYER_TAG_H */
