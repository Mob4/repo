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
 * Authors: Konstantinos Katsaros  <K.Katsaros@surrey.ac.uk>
 *
 */

///
/// \file	ClwprState.cc
/// \brief	Implementation of all functions needed for manipulating the internal
///		state of an CLWPR node.
///

#include "clwpr-state.h"
#include "ns3/simulator.h"
#include "ns3/vector.h"

namespace ns3 {

/********** Neighbor Set Manipulation **********/

NeighborTuple*
ClwprState::FindNeighborTuple (Ipv4Address const &mainAddr)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr)
        return &(*it);
    }
  return NULL;
}

//const NeighborTuple*
//ClwprState::FindSymNeighborTuple (Ipv4Address const &mainAddr) const
//{
//  for (NeighborSet::const_iterator it = m_neighborSet.begin ();
//       it != m_neighborSet.end (); it++)
//    {
//      if (it->neighborMainAddr == mainAddr && it->status == NeighborTuple::STATUS_SYM)
//        return &(*it);
//    }
//  return NULL;
//}


void
ClwprState::EraseNeighborTuple (const NeighborTuple &tuple)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_neighborSet.erase (it);
          break;
        }
    }
}

void
ClwprState::EraseNeighborTuple (const Ipv4Address &mainAddr)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr)
        {
          it = m_neighborSet.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertNeighborTuple (NeighborTuple const &tuple)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == tuple.neighborMainAddr)
        {
          // Update it
          *it = tuple;
          return;
        }
    }
  m_neighborSet.push_back (tuple);
}


/********** Duplicate Set Manipulation **********/

DuplicateTuple*
ClwprState::FindDuplicateTuple (Ipv4Address const &addr, uint16_t sequenceNumber)
{
  for (DuplicateSet::iterator it = m_duplicateSet.begin ();
       it != m_duplicateSet.end(); it++)
    {
      if (it->address == addr && it->sequenceNumber == sequenceNumber)
        return &(*it);
    }
  return NULL;
}

void
ClwprState::EraseDuplicateTuple (const DuplicateTuple &tuple)
{
  for (DuplicateSet::iterator it = m_duplicateSet.begin ();
       it != m_duplicateSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_duplicateSet.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertDuplicateTuple (DuplicateTuple const &tuple)
{
  m_duplicateSet.push_back (tuple);
}


///********** Interface Association Set Manipulation **********/

IfaceAssocTuple*
ClwprState::FindIfaceAssocTuple (Ipv4Address const &ifaceAddr)
{
  for (IfaceAssocSet::iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (it->ifaceAddr == ifaceAddr)
        return &(*it);
    }
  return NULL;
}

const IfaceAssocTuple*
ClwprState::FindIfaceAssocTuple (Ipv4Address const &ifaceAddr) const
{
 for (IfaceAssocSet::const_iterator it = m_ifaceAssocSet.begin ();
      it != m_ifaceAssocSet.end (); it++)
   {
     if (it->ifaceAddr == ifaceAddr)
        return &(*it);
   }
 return NULL;
}

void
ClwprState::EraseIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
  for (IfaceAssocSet::iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (*it == tuple)
       {
          m_ifaceAssocSet.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
  m_ifaceAssocSet.push_back (tuple);
}

std::vector<Ipv4Address>
ClwprState::FindNeighborInterfaces (const Ipv4Address &neighborMainAddr) const
{
  std::vector<Ipv4Address> retval;
  for (IfaceAssocSet::const_iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (it->mainAddr == neighborMainAddr)
       retval.push_back (it->ifaceAddr);
    }
  return retval;
}


/********** IP - Position Association Set Manipulation **********/

//void ClwprState::InitPosAssociationSet (){

//// FOR STATIC 5x5 25 nodes
//  PosAssociationTuple dst;
//  dst.nodeHeading =0;
//  dst.nodeMainAddr = Ipv4Address("10.1.1.121");
//  dst.nodePosition =Vector(0,0,0);
//  dst.nodeRoadId = 0;
//  dst.nodeTimestamp = Simulator::Now();
//  dst.nodeVelocity =Vector(0,0,0);
//  m_posAssociationSet.push_back (dst);

// FOR MOVING bonnmotion 200nodes
//  PosAssociationTuple dst;
//  dst.nodeHeading =0;
//  dst.nodeMainAddr = Ipv4Address("10.1.1.201");
//  dst.nodePosition =Vector(0,0,0);
//  dst.nodeRoadId = 0;
//  dst.nodeTimestamp = Simulator::Now();
//  dst.nodeVelocity =Vector(0,0,0);
//  m_posAssociationSet.push_back (dst);

//// FOR SUMO 600 nodes
//	PosAssociationTuple dst;
//	dst.nodeHeading =0;
//	dst.nodeMainAddr = Ipv4Address("10.1.2.92");
//	dst.nodePosition =Vector(2500,2500,0);
//	dst.nodeRoadId = 0;
//	dst.nodeTimestamp = Simulator::Now();
//	dst.nodeVelocity =Vector(0,0,0);
//	m_posAssociationSet.push_back (dst);

//	return ;
//}


PosAssociationTuple*
ClwprState::FindPosAssociationTuple (const Ipv4Address &nodeMainAddr)
{
  for (PosAssociationSet::iterator it = m_posAssociationSet.begin ();
	it != m_posAssociationSet.end (); it++)
    {
      if (it->nodeMainAddr == nodeMainAddr)
        {
          return &(*it);
        }
    }
   return NULL;
}

void
ClwprState::ErasePosAssociationTuple (const PosAssociationTuple &tuple)
{
  for (PosAssociationSet::iterator it = m_posAssociationSet.begin ();
       it != m_posAssociationSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_posAssociationSet.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertPosAssociationTuple (const PosAssociationTuple &tuple)
{
  m_posAssociationSet.push_back (tuple);
}


/********** Host-Network Association Set Manipulation **********/

// HNA will be used in the future, so keep this

AssociationTuple*
ClwprState::FindAssociationTuple (const Ipv4Address &gatewayAddr, const Ipv4Address &networkAddr, const Ipv4Mask &netmask)
{
  for (AssociationSet::iterator it = m_associationSet.begin ();
	it != m_associationSet.end (); it++)
    {
      if (it->gatewayAddr == gatewayAddr and it->networkAddr == networkAddr and it->netmask == netmask)
        {
          return &(*it);
        }
    }
   return NULL;
}

void
ClwprState::EraseAssociationTuple (const AssociationTuple &tuple)
{
  for (AssociationSet::iterator it = m_associationSet.begin ();
       it != m_associationSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_associationSet.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertAssociationTuple (const AssociationTuple &tuple)
{
  m_associationSet.push_back (tuple);
}

void
ClwprState::EraseAssociation (const Association &tuple)
{
  for (Associations::iterator it = m_associations.begin ();
       it != m_associations.end (); it++)
    {
      if (*it == tuple)
        {
          m_associations.erase (it);
          break;
        }
    }
}

void
ClwprState::InsertAssociation (const Association &tuple)
{
  m_associations.push_back(tuple);
}

} // namespace ns3
