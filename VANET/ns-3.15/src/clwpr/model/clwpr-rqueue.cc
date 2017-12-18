/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Based on 
 *      NS-2 AODV model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 * 
 *      AODV-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/AODV-UU
 *
 * These methods were used 'as it is' for CLWPR
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#include "clwpr-rqueue.h"
#include <algorithm>
#include <functional>
#include "ns3/ipv4-route.h"
#include "ns3/socket.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("ClwprRequestQueue");

namespace ns3
{
namespace clwpr
{
uint32_t
RequestQueue::GetSize ()
{
  Purge ();
  return m_rqueue.size ();
}

bool
RequestQueue::Enqueue (QueueEntry & entry)
{
  Purge ();
  for (std::vector<QueueEntry>::const_iterator i = m_rqueue.begin (); i
      != m_rqueue.end (); ++i)
    {
      if ((i->GetPacket ()->GetUid () == entry.GetPacket ()->GetUid ())
          && (i->GetIpv4Header ().GetDestination ()
              == entry.GetIpv4Header ().GetDestination ()))
        return false;
    }
  entry.SetExpireTime (m_queueTimeout);
  if (m_rqueue.size () == m_maxLen)
    {
      Drop (m_rqueue.front (), "Drop the most aged packet"); // Drop the most aged packet
      m_rqueue.erase (m_rqueue.begin ());
    }
  m_rqueue.push_back (entry);
  return true;
}

void
RequestQueue::DropPacketWithDst (Ipv4Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  Purge ();
  //const Ipv4Address addr = dst;
  for (std::vector<QueueEntry>::iterator i = m_rqueue.begin (); i
      != m_rqueue.end (); ++i)
    {
      if (IsEqual (*i, dst))
        {
          Drop (*i, "DropPacketWithDst ");
        }
    }
  m_rqueue.erase (std::remove_if (m_rqueue.begin (), m_rqueue.end (),
      std::bind2nd (std::ptr_fun (RequestQueue::IsEqual), dst)), m_rqueue.end ());
}

bool
RequestQueue::Dequeue (Ipv4Address dst, QueueEntry & entry)
{
  Purge ();
  for (std::vector<QueueEntry>::iterator i = m_rqueue.begin (); i != m_rqueue.end (); ++i)
    {
      if (i->GetIpv4Header ().GetDestination () == dst)
        {
          entry = *i;
          m_rqueue.erase (i);
          return true;
        }
    }
  return false;
}

bool
RequestQueue::Find (Ipv4Address dst)
{
  for (std::vector<QueueEntry>::const_iterator i = m_rqueue.begin (); i
      != m_rqueue.end (); ++i)
    {
      if (i->GetIpv4Header ().GetDestination () == dst)
        return true;
    }
  return false;
}

struct IsExpired
{
  bool
  operator() (QueueEntry const & e) const
  {
    return (e.GetExpireTime () < Seconds (0));
  }
};

void
RequestQueue::Purge ()
{
  IsExpired pred;
  for (std::vector<QueueEntry>::iterator i = m_rqueue.begin (); i
      != m_rqueue.end (); ++i)
    {
      if (pred (*i))
        {
          Drop (*i, "Drop outdated packet ");
        }
    }
  m_rqueue.erase (std::remove_if (m_rqueue.begin (), m_rqueue.end (), pred),
      m_rqueue.end ());
}

void
RequestQueue::Drop (QueueEntry en, std::string reason)
{
  NS_LOG_LOGIC (reason << en.GetPacket ()->GetUid () << " " << en.GetIpv4Header ().GetDestination ());
  en.GetErrorCallback () (en.GetPacket (), en.GetIpv4Header (),
      Socket::ERROR_NOROUTETOHOST);
  return;
}

}
}
