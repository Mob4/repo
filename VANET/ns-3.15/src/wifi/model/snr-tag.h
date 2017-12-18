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

#ifndef SNR_TAG_H
#define SNR_TAG_H

#include "ns3/tag.h"
#include <stdint.h>

namespace ns3{

///Tag used to get SNR value from PHY
struct SnrTag : public Tag
{
  /// Positive if output device is fixed in RouteOutput
  double snr;

  SnrTag (double s = -1) : Tag(), snr (s) {}

  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::clwpr::SnrTag").SetParent<Tag> ();
    return tid;
  }

  TypeId  GetInstanceTypeId () const
  {
    return GetTypeId ();
  }

  uint32_t GetSerializedSize () const
  {
    return sizeof(double);
  }

  void  Serialize (TagBuffer i) const
  {
    i.WriteDouble(snr);
  }

  void  Deserialize (TagBuffer i)
  {
    snr = i.ReadDouble();
  }

  void  Print (std::ostream &os) const
  {
    os << "SNRTag: SNR value of received packet = " << snr;
  }
};

} // namespace
#endif
