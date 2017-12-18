/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Konstantinos Katsaros <K.Katsaros@surrey.ac.uk> // adaptation to CLWPR
 */
#include "clwpr-helper.h"
#include "ns3/clwpr-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"


namespace ns3 {

ClwprHelper::ClwprHelper ()
{
  m_agentFactory.SetTypeId ("ns3::clwpr::RoutingProtocol");
}

ClwprHelper::ClwprHelper (const ClwprHelper &o)
  : m_agentFactory (o.m_agentFactory)
{
  m_interfaceExclusions = o.m_interfaceExclusions;
}

ClwprHelper* 
ClwprHelper::Copy (void) const 
{
  return new ClwprHelper (*this); 
}

void
ClwprHelper::ExcludeInterface (Ptr<Node> node, uint32_t interface)
{
  std::map< Ptr<Node>, std::set<uint32_t> >::iterator it = m_interfaceExclusions.find (node);

  if(it == m_interfaceExclusions.end ())
    {
      std::set<uint32_t> interfaces;
      interfaces.insert (interface);
    
      m_interfaceExclusions.insert (std::make_pair (node, std::set<uint32_t> (interfaces) ));
    }
  else
    {
      it->second.insert (interface);
    }
}

Ptr<Ipv4RoutingProtocol> 
ClwprHelper::Create (Ptr<Node> node) const
{
  Ptr<clwpr::RoutingProtocol> agent = m_agentFactory.Create<clwpr::RoutingProtocol> ();

  std::map<Ptr<Node>, std::set<uint32_t> >::const_iterator it = m_interfaceExclusions.find (node);

  if(it != m_interfaceExclusions.end ())
    {
      agent->SetInterfaceExclusions (it->second);
    }

  node->AggregateObject (agent);
  return agent;
}

void 
ClwprHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}


void
ClwprHelper::PrintNeighbourTableAllEvery (Time printInterval, Ptr<OutputStreamWrapper> stream) const
{
  for (uint32_t i = 0; i < NodeList::GetNNodes (); i++)
    {
      Ptr<Node> node = NodeList::GetNode (i);
      Simulator::Schedule (printInterval, &ClwprHelper::PrintNeighborEvery, this, printInterval, node, stream);
    }
}

void
ClwprHelper::PrintNeighborEvery (Time printInterval, Ptr<Node> node, Ptr<OutputStreamWrapper> stream) const
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  Ptr<clwpr::RoutingProtocol> rp = ipv4->GetObject<clwpr::RoutingProtocol>();
  NS_ASSERT (rp);
  rp->PrintNeighbourTable (stream);
  Simulator::Schedule (printInterval, &ClwprHelper::PrintNeighborEvery, this, printInterval, node, stream);
}

void
ClwprHelper::PrintNeighbourTableCompactEvery (Time printInterval, Ptr<OutputStreamWrapper> stream) const
{
  for (uint32_t i = 0; i < NodeList::GetNNodes (); i++)
    {
      Ptr<Node> node = NodeList::GetNode (i);
      Simulator::Schedule (printInterval, &ClwprHelper::PrintCompactNeighborEvery, this, printInterval, node, stream);
    }
}

void
ClwprHelper::PrintCompactNeighborEvery (Time printInterval, Ptr<Node> node, Ptr<OutputStreamWrapper> stream) const
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  Ptr<clwpr::RoutingProtocol> rp = ipv4->GetObject<clwpr::RoutingProtocol>();
  NS_ASSERT (rp);
  rp->PrintNeighbourTableCompact (stream);
  Simulator::Schedule (printInterval, &ClwprHelper::PrintCompactNeighborEvery, this, printInterval, node, stream);
}

} // namespace ns3
