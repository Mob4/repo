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
 *
 */


#ifndef CLWPR_MAP_H_
#define CLWPR_MAP_H_

// MAX Road Network 15 X 15
#define MAX_DIAMENSION 15
#define ROADERR 15

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include "ns3/vector.h"
#include <utility>
#include "ns3/object.h"
namespace ns3 {


class GridMap : public Object
{
       private:
         double m_x; //x co-ordinate of vehicle
         double m_y; // Data member of vegicle
		 uint8_t m_roadID;
		 int m_xd; //x co-ordinate of road
		 int m_yd; //y co-ordinate of road
		 int m_xMax, m_yMax;
		 int m_dist; //Distance between blocks
		 std::pair<double, double> m_waypoints[MAX_DIAMENSION][MAX_DIAMENSION];

		 void InitWayPoitns(); //Set waypoits
		 int FindWPX (Vector p, int r_index);
		 int FindWPY (Vector p, int r_index);

     public:
		  static TypeId GetTypeId (void);
		  GridMap();
		  GridMap(int x, int y, int d);
		  ~GridMap();
		  uint8_t FindRoadID(int r_x, int r_y);
		  int GetRoadXFromID (uint8_t r_id);
		  int GetRoadYFromID (uint8_t r_id);
		  void SetRoadXFromVehicle(double v_x);
		  void SetRoadYFromVehicle(double v_y);
		  int GetRoadXId();
		  int GetRoadYId();
		  double GetCurvemetricDistance(Vector pos_a, int x_a, int y_a, Vector pos_b, int x_b, int y_b);
		  void GetNLOSDistance(Vector pos_a, int x_a, int y_a, Vector pos_b, int x_b, int y_b, double &dist1, double &dist2);
		  int GetRoadXFromVehicle(double v_x);
		  int GetRoadYFromVehicle(double v_y);
};
} // namespace ns3

#endif /* CLWPR_MAP_H_ */
