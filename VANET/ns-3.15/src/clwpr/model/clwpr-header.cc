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
 * Author: Konstantinos Katsaros <K.Katsaros@surrey.ac.uk>
 */

#include "ns3/assert.h"

#include "clwpr-header.h"
#include "ns3/log.h"

#define IPV4_ADDRESS_SIZE 4
#define CLWPR_MSG_HEADER_SIZE 12 
#define CLWPR_PKT_HEADER_SIZE 4
#define CLWPR_HELLO_MESSAGE_SIZE 52

namespace ns3 {
namespace clwpr {


NS_LOG_COMPONENT_DEFINE("ClwprHeader");

/// Scaling factor used in RFC 3626.
#define CLWPR_C 0.0625

///
/// \brief Converts a decimal number of seconds to the mantissa/exponent format.
///
/// \param seconds decimal number of seconds we want to convert.
/// \return the number of seconds in mantissa/exponent format.
///
uint8_t
SecondsToEmf (double seconds)
{
  int a, b = 0;
  
  // find the largest integer 'b' such that: T/C >= 2^b
  for (b = 0; (seconds/CLWPR_C) >= (1 << b); ++b)
    ;
  NS_ASSERT ((seconds/CLWPR_C) < (1 << b));
  b--;
  NS_ASSERT ((seconds/CLWPR_C) >= (1 << b));

  // compute the expression 16*(T/(C*(2^b))-1), which may not be a integer
  double tmp = 16*(seconds/(CLWPR_C*(1<<b))-1);

  // round it up.  This results in the value for 'a'
  a = (int) ceil (tmp);

  // if 'a' is equal to 16: increment 'b' by one, and set 'a' to 0
  if (a == 16)
    {
      b += 1;
      a = 0;
    }

  // now, 'a' and 'b' should be integers between 0 and 15,
  NS_ASSERT (a >= 0 && a < 16);
  NS_ASSERT (b >= 0 && b < 16);

  // the field will be a byte holding the value a*16+b
  return (uint8_t) ((a << 4) | b);
}

///
/// \brief Converts a number of seconds in the mantissa/exponent format to a decimal number.
///
/// \param clwpr_format number of seconds in mantissa/exponent format.
/// \return the decimal number of seconds.
///
double
EmfToSeconds (uint8_t clwprFormat)
{
  int a = (clwprFormat >> 4);
  int b = (clwprFormat & 0xf);
  // value = C*(1+a/16)*2^b [in seconds]
  return CLWPR_C * (1 + a/16.0) * (1 << b);
}



// ---------------- CLWPR Packet -------------------------------

NS_OBJECT_ENSURE_REGISTERED (PacketHeader);

PacketHeader::PacketHeader ()
{}

PacketHeader::~PacketHeader ()
{}

TypeId
PacketHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::clwpr::PacketHeader")
    .SetParent<Header> ()
    .AddConstructor<PacketHeader> ()
    ;
  return tid;
}
TypeId
PacketHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t 
PacketHeader::GetSerializedSize (void) const
{
  return CLWPR_PKT_HEADER_SIZE;
}

void 
PacketHeader::Print (std::ostream &os) const
{
  // TODO
}

void
PacketHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  i.WriteHtonU16 (m_packetLength);
  i.WriteHtonU16 (m_packetSequenceNumber);
}

uint32_t
PacketHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_packetLength  = i.ReadNtohU16 ();
  m_packetSequenceNumber = i.ReadNtohU16 ();
  return GetSerializedSize ();
}


// ---------------- CLWPR Message -------------------------------

NS_OBJECT_ENSURE_REGISTERED (MessageHeader);

MessageHeader::MessageHeader ()
  : m_messageType (MessageHeader::MessageType (0))
{}

MessageHeader::~MessageHeader ()
{}

TypeId
MessageHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::clwpr::MessageHeader")
    .SetParent<Header> ()
    .AddConstructor<MessageHeader> ()
    ;
  return tid;
}
TypeId
MessageHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
MessageHeader::GetSerializedSize (void) const
{
  uint32_t size = CLWPR_MSG_HEADER_SIZE;
  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      NS_LOG_DEBUG ("Hello Message Size: " << size << " + " << m_message.hello.GetSerializedSize ());
      size += m_message.hello.GetSerializedSize ();
      break;
    case HNA_MESSAGE:
      size += m_message.hna.GetSerializedSize ();
      break;
    default:
      NS_ASSERT (false);
    }
  return size;
}

void 
MessageHeader::Print (std::ostream &os) const
{
  // TODO
}

void
MessageHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  i.WriteU8 (m_messageType);
  i.WriteU8 (m_vTime);
  i.WriteHtonU16 (GetSerializedSize ());
  i.WriteU8 (m_timeToLive);
  i.WriteU8 (m_hopCount);
  i.WriteHtonU16 (m_messageSequenceNumber);
  i.WriteHtonU32 (m_originatorAddress.Get ());

  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      m_message.hello.Serialize (i);
      break;
    case HNA_MESSAGE:
      m_message.hna.Serialize (i);
      break;
    default:
      NS_ASSERT (false);
    }

}

uint32_t
MessageHeader::Deserialize (Buffer::Iterator start)
{
  uint32_t size;
  Buffer::Iterator i = start;
  m_messageType  = (MessageType) i.ReadU8 ();
  NS_ASSERT (m_messageType >= HELLO_MESSAGE && m_messageType <= HNA_MESSAGE);
  m_vTime  = i.ReadU8 ();
  m_messageSize  = i.ReadNtohU16 ();
  m_timeToLive  = i.ReadU8 ();
  m_hopCount  = i.ReadU8 ();
  m_messageSequenceNumber = i.ReadNtohU16 ();
  m_originatorAddress = Ipv4Address (i.ReadNtohU32 ());
  size = CLWPR_MSG_HEADER_SIZE;
  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      size += m_message.hello.Deserialize (i, CLWPR_HELLO_MESSAGE_SIZE);
      break;
    case HNA_MESSAGE:
      size += m_message.hna.Deserialize (i, m_messageSize - CLWPR_MSG_HEADER_SIZE);
      break;
    default:
      NS_ASSERT (false);
    }
  return size;
}

// ---------------- CLWPR HELLO Message -------------------------------

uint32_t 
MessageHeader::Hello::GetSerializedSize (void) const
{
  return CLWPR_HELLO_MESSAGE_SIZE;
}

void 
MessageHeader::Hello::Print (std::ostream &os) const
{
  // TODO
}

/***
 * Heading needed since we have only abs(velocity)
 * Z-axis not needed since we work in 2D
 * Devise a way to send double within integers -->
 * multiply and divide by 1000 (accuracy 3 decimals)
 * Better Solution use mantissa like in time.
 */
void
MessageHeader::Hello::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU64  ((uint64_t)(this->position.x*1000));
  i.WriteHtonU64  ((uint64_t)(this->position.y*1000));
//  i.WriteHtonU64  ((uint64_t)this->position.z);
  i.WriteHtonU64  ((uint64_t)abs(this->velocity.x*1000));
  i.WriteHtonU64  ((uint64_t)abs(this->velocity.y*1000));
//  i.WriteHtonU64  ((uint64_t)abs(this->velocity.z));
  i.WriteHtonU64  ((uint64_t)this->heading*1000);
  i.WriteU8 (this->roadId);
  i.WriteU8 (this->hTime);
  i.WriteU32 (this->utilization);
  i.WriteU32 (this->macInfo*1000);
  i.WriteU16 (this->CnFinfo);
}

uint32_t
MessageHeader::Hello::Deserialize (Buffer::Iterator start, uint32_t messageSize)
{
  Buffer::Iterator i = start;

  NS_ASSERT (messageSize == 52);

  this->position.x =(double) (i.ReadNtohU64 ()/1000.0);
  this->position.y =(double) (i.ReadNtohU64 ()/1000.0);
//  this->position.z =(double) i.ReadNtohU64 ();
  this->velocity.x =(double) (i.ReadNtohU64 ()/1000.0);
  this->velocity.y =(double) (i.ReadNtohU64 ()/1000.0);
//  this->velocity.z =(double) i.ReadNtohU64 ();
  this->heading =(double) (i.ReadNtohU64 ()/1000.0);
  this->roadId = i.ReadU8 ();
  this->hTime = i.ReadU8 ();
  this->utilization = i.ReadU32 ();
  this->macInfo = i.ReadU32 ()/1000;
  this->CnFinfo = i.ReadU16();
  return messageSize;
}


// ---------------- CLWPR HNA Message -------------------------------

uint32_t 
MessageHeader::Hna::GetSerializedSize (void) const
{
  return 2*this->associations.size () * IPV4_ADDRESS_SIZE;
}

void 
MessageHeader::Hna::Print (std::ostream &os) const
{
  // TODO
}

void
MessageHeader::Hna::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  for (size_t n = 0; n < this->associations.size (); ++n)
    {
      i.WriteHtonU32 (this->associations[n].address.Get ());
      i.WriteHtonU32 (this->associations[n].mask.Get ());
    }
}

uint32_t
MessageHeader::Hna::Deserialize (Buffer::Iterator start, uint32_t messageSize)
{
  Buffer::Iterator i = start;

  NS_ASSERT (messageSize % (IPV4_ADDRESS_SIZE*2) == 0);
  int numAddresses = messageSize / IPV4_ADDRESS_SIZE / 2;
  this->associations.clear ();
  for (int n = 0; n < numAddresses; ++n)
    {
      Ipv4Address address (i.ReadNtohU32 ());
      Ipv4Mask mask (i.ReadNtohU32 ());
      this->associations.push_back ((Association) {address, mask});
    }
  return messageSize;
}

}} // namespace clwpr, ns3

#include "ns3/test.h"
#include "ns3/packet.h"

namespace ns3 {

class ClwprEmfTestCase : public TestCase {
public:
  ClwprEmfTestCase ();
  virtual void DoRun (void);
};

ClwprEmfTestCase::ClwprEmfTestCase ()
  : TestCase ("Check Emf clwpr time conversion")
{}
void
ClwprEmfTestCase::DoRun (void)
{
  for (int time = 1; time <= 30; time++)
    {
      uint8_t emf = clwpr::SecondsToEmf (time);
      double seconds = clwpr::EmfToSeconds (emf);
      NS_TEST_ASSERT_MSG_EQ((seconds < 0 || fabs (seconds - time) > 0.1), false,
                            "XXX");
    }
  return;
}


class ClwprHelloTestCase : public TestCase {
public:
  ClwprHelloTestCase ();
  virtual void DoRun (void);
};

ClwprHelloTestCase::ClwprHelloTestCase ()
  : TestCase ("Check Hello clwpr messages")
{}
void
ClwprHelloTestCase::DoRun (void)
{
  Packet packet;
  clwpr::MessageHeader msgIn;
  clwpr::MessageHeader::Hello &helloIn = msgIn.GetHello ();
  
  Vector pos = Vector (0.5, 0.5, 0.0);
  Vector vel = Vector (10.5, 0.5, 0.0);
  
  helloIn.SetHTime (Seconds (7));
  helloIn.position = pos ;
  helloIn.velocity = vel ;
  helloIn.heading = 0;
  helloIn.roadId = 1;
  helloIn.utilization = 60;
  helloIn.macInfo = 3;
  
  packet.AddHeader (msgIn);

  clwpr::MessageHeader msgOut;
  packet.RemoveHeader (msgOut);
  clwpr::MessageHeader::Hello &helloOut = msgOut.GetHello ();
    
  NS_TEST_ASSERT_MSG_EQ (helloOut.GetHTime (), Seconds (7), "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.position.x, 0.5, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.position.y, 0.5, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.position.z, 0.0, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.velocity.x, 10.5, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.velocity.y, 0.5, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.velocity.z, 0.0, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.heading, 0, "XXX");
  
  NS_TEST_ASSERT_MSG_EQ (helloOut.roadId, 1, "XXX");
  NS_TEST_ASSERT_MSG_EQ (helloOut.macInfo, 3, "XXX");
  
  NS_TEST_ASSERT_MSG_EQ (packet.GetSize (), 0, "All bytes in packet were not read");

  return;
}

class ClwprHnaTestCase : public TestCase {
public:
  ClwprHnaTestCase ();
  virtual void DoRun (void);
};

ClwprHnaTestCase::ClwprHnaTestCase ()
  : TestCase ("Check Hna clwpr messages")
{}

void
ClwprHnaTestCase::DoRun (void)
{
  Packet packet;
  clwpr::MessageHeader msgIn;
  clwpr::MessageHeader::Hna &hnaIn = msgIn.GetHna ();
  
  hnaIn.associations.push_back ((clwpr::MessageHeader::Hna::Association)
                                { Ipv4Address ("1.2.3.4"), Ipv4Mask ("255.255.255.0")});
  hnaIn.associations.push_back ((clwpr::MessageHeader::Hna::Association)
                                {Ipv4Address ("1.2.3.5"), Ipv4Mask ("255.255.0.0")});
  packet.AddHeader (msgIn);
  
  clwpr::MessageHeader msgOut;
  packet.RemoveHeader (msgOut);
  clwpr::MessageHeader::Hna &hnaOut = msgOut.GetHna ();
  
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations.size (), 2, "XXX");
  
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[0].address,
                        Ipv4Address ("1.2.3.4"), "XXX");
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[0].mask,
                        Ipv4Mask ("255.255.255.0"), "XXX");

  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[1].address,
                        Ipv4Address ("1.2.3.5"), "XXX");
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[1].mask,
                        Ipv4Mask ("255.255.0.0"), "XXX");
  
  NS_TEST_ASSERT_MSG_EQ (packet.GetSize (), 0, "All bytes in packet were not read");

  return;
}


static class ClwprTestSuite : public TestSuite
{
public:
  ClwprTestSuite ();
} g_clwprTestSuite;

ClwprTestSuite::ClwprTestSuite()
  : TestSuite("routing-Clwpr-header", UNIT)
{
  AddTestCase(new ClwprHnaTestCase());
  AddTestCase(new ClwprHelloTestCase());
  AddTestCase(new ClwprEmfTestCase());
}

} // namespace ns3

