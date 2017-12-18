/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Konstantinos Katsaros, University of Surrey
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

#ifndef CLWPR_H
#define CLWPR_H

/**
 * \ingroup routing
 * \defgroup clwpr CLWPR
 *
 * \section model Model
 *
 * This model implements the base specification of the Cross-Layer,
 * Weighted, Position-Based Routing Protocol (CLWPR), which is a
 * dynamic mobile ad hoc unicast routing protocol for vehicular networks.
 * It has been developed at the University of Surrey (UK) by Konstantinos Katsaros for NS-3.
 *
 * Initial results are presented in : K.Katsaros, M.Dianati, R.Tafazolli, R.Kernchen
 * "CLWPR - A Novel Cross-Layer Optimized Position Based Routing Protocol for VANETs",
 * in IEEE Vehicular Networking Conference, 2011
 *
 * Here is a summary of software's main features:
 *
 * \section api API and Usage
 * 
 * A helper class for CLWPR has been written.  After an IPv4 topology
 * has been created and unique IP addresses assigned to each node, the
 * simulation script writer can call one of three overloaded functions
 * with different scope to enable CLWPR: ns3::ClwprHelper::Install
 * (NodeContainer container); ns3::ClwprHelper::Install (Ptr<Node>
 * node); or ns3::ClwprHelper::InstallAll (void);
 *
 * In addition, the behavior of CLWPR can be modified by changing certain
 * attributes.  The method ns3::ClwprHelper::Set () can be used
 * to set CLWPR attributes.  These include HelloInterval, Weighting Factors,
 * a pointer to the underlying road topology map etc. The complete list of attributes
 * can be found under RoutingProtocol::GetTypeId (). There are also 6 tracesources that
 * can be monitored including received, transmitted, cached packets etc.
 *
 * Host Network Association (HNA) is not fully supported in this implementation
 * of CLWPR. Since this started from OLSR implementation, any reference to HNA
 * is from the former implementation of OLSR and not tested with CLWPR.
 * Refer to examples/routing/clwpr-hna.cc to see how the API is used.
 *
 * \section list Open Issues
 * 
 * 1. DCF error with some scenarios : assert failed. cond="m_lastNavStart < Simulator::Now ()",
 * file=../src/wifi/model/dcf-manager.cc, line=722
 *
 * 2. HNA is not tested/supported yet
 *
 * 3. Currently there is no actual location service implemented, which is needed in all position based
 * routing protocols. We use the global knowledge of ns-3 to get such information.
 *
 * 4. There is not optimal configuration for Weighting parameters yet.
 *
 *
 */


#endif /* CLWPR_H */
