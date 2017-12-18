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

#include "clwpr-map.h"
#include "ns3/log.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("ClwprMap");

TypeId
GridMap::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GridMap")
    	.SetParent<Object> ()
        .AddConstructor<GridMap> ()
        ;
  return tid;
}

GridMap::GridMap() //Default Constructor
{ }

GridMap::GridMap(int x, int y, int d) // Constructor for (x,y) grid with distance d
{
  m_xMax = x;
  m_yMax = y;
  m_dist = d;
  InitWayPoitns();
}

GridMap::~GridMap() //Destructor
{ }

/// Input : (x,y) co-ordinates of road IDs
/// Output: ID of the road 8bits --
/// 4 LSB y co-ordinate
/// 4 MSB x co-ordinate
uint8_t GridMap::FindRoadID(int r_x, int r_y)
{
	return m_roadID = (r_x<<4)|(r_y);
}

/// Get the X-Axis form Road_ID
int GridMap::GetRoadXFromID (uint8_t r_id)
{
  return r_id >> 4;
}
/// Get the Y-Axis form Road ID
int GridMap::GetRoadYFromID (uint8_t r_id)
{
  return r_id & 0x4F;

}

int GridMap::GetRoadXId()
{
 return m_xd;
}
int GridMap::GetRoadYId()
{
 return m_yd;
}


/// Set the local X-axis from vehicle position
void GridMap::SetRoadXFromVehicle(double v_x)
{
  for (int i=0; i<m_xMax; i++){
  // i*dist - err << x << i*dist + err
	if (( v_x >= (i*m_dist - ROADERR)) && (v_x <= (i*m_dist + ROADERR)))
	{
	  m_xd = i;
	  return;
	}
  }
	m_xd = 15;
	NS_LOG_DEBUG ("SET: Clwpr node not on a road for X-axis. Veh. Pos = " << v_x);
//	std::cout << "Clwpr node not on a road for Y-axis";
	return;

}
/// Set the local Y-axis from vehicle position
void GridMap::SetRoadYFromVehicle(double v_y){
  for (int i=0; i<m_yMax; i++){
  // i*dist - err << x << i*dist + err
	if (( v_y >= (i*m_dist - ROADERR)) && (v_y <= (i*m_dist + ROADERR)))
	{
	  m_yd = i;
	  return;
	}
  }
	m_yd = 15;
	NS_LOG_DEBUG ("SET: Clwpr node not on a road for Y-axis. Veh. Pos = " << v_y);
//	std::cout << "Clwpr node not on a road for Y-axis";
	return;
}


int GridMap::GetRoadXFromVehicle(double v_x)
{
//  SetRoadXFromVehicle(v_x);
  for (int i=0; i<m_xMax; i++){
  // i*dist - err << x << i*dist + err
        if (( v_x >= (i*m_dist - ROADERR)) && (v_x <= (i*m_dist + ROADERR)))
        {
//          m_xd = i;
          return i;
        }
  }
//  m_xd = 15;
  NS_LOG_DEBUG ("GET: Clwpr node not on a road for X-axis");
//  std::cout << "Clwpr node not on a road for X-axis";
  return 15;
//	return m_xd;
}

int GridMap::GetRoadYFromVehicle(double v_y)
{
//	SetRoadYFromVehicle(v_y);
//	return m_yd;
  for (int i=0; i<m_yMax; i++){
  // i*dist - err << x << i*dist + err
        if (( v_y >= (i*m_dist - ROADERR)) && (v_y <= (i*m_dist + ROADERR)))
        {
//          m_xd = i;
          return i;
        }
  }
//  m_xd = 15;
  NS_LOG_DEBUG ("GET: Clwpr node not on a road for Y-axis");
//  std::cout << "Clwpr node not on a road for Y-axis";
  return 15;
}

void GridMap::InitWayPoitns(){ //Set waypoits

    NS_LOG_DEBUG ("Initialize WayPoints at clwpr MAP");
	for (int i=0; i<m_xMax; i++){
		for (int j=0; j<m_yMax; j++){
			m_waypoints[i][j].first = i*m_dist;
			m_waypoints[i][j].second = j*m_dist;
		}
	}
//        for (int i=0; i<m_xMax; i++){
//                for (int j=0; j<m_yMax; j++){
//                        std::cout << "WP" << i <<","<<j<<" : ";
//                        std::cout << m_waypoints[i][j].first << " , ";
//                        std::cout << m_waypoints[i][j].second << "\n";
//                }
//        }

}

/// Calculates the distance between two nodes
/// Using map information -- Curvemetric distance
double GridMap::GetCurvemetricDistance(Vector pos_a,
                                        int x_a,
                                        int y_a,
                                        Vector pos_b,
                                        int x_b,
                                        int y_b)
{
	double dist = 0.0;
	int a_wp_x = x_a;
	int a_wp_y = y_a;
	int b_wp_x = x_b;
	int b_wp_y = y_b;

//	if (((a_x == b_x )&&(a_x!=15))||((a_y == b_y)&&(a_y!=15))){
//		// On the same X-axis or Y-axis (road)
//		// Curvemetric distance is the euclidian distance
//		dist = CalculateDistance(pos_a, pos_b);
//	}
//	if (x_a == MAX_DIAMENSION && y_a == MAX_DIAMENSION) std::cout << "ERROR!!!!! Invalid position A";
	NS_ASSERT_MSG (!(x_a == MAX_DIAMENSION && y_a == MAX_DIAMENSION), " Invalid Position for node A" );
//	if (x_b == MAX_DIAMENSION && y_b == MAX_DIAMENSION) std::cout << "ERROR!!!!! Invalid position B";
	NS_ASSERT_MSG (!(x_b == MAX_DIAMENSION && y_b == MAX_DIAMENSION), " Invalid Position for node B");
	NS_LOG_DEBUG("Curvemetric Distance Calc");
	if ((x_a == x_b )&&(x_a!=MAX_DIAMENSION)){
		// On the same X-axis
	    NS_LOG_DEBUG(" Same X-axis ");
	    dist = abs(pos_a.y - pos_b.y);
	    return dist;
	}
	else if ((y_a == y_b)&&(y_a!=MAX_DIAMENSION)){
		// On the same Y-axis
            NS_LOG_DEBUG(" Same Y-axis ");
	    dist = abs(pos_a.x - pos_b.x);
	    return dist;
	}
	else if (y_a != MAX_DIAMENSION && x_b != MAX_DIAMENSION)
	  {
	    // the distance is a "L shape"
	    // Closest WP == [X_B, Y_A]
            NS_LOG_DEBUG(" L-shape : X-b / Y-a ");

	    dist = abs(m_waypoints[b_wp_x][a_wp_y].first - pos_a.x);
            dist += abs(m_waypoints[b_wp_x][a_wp_y].second - pos_b.y);
            return dist;
	}
	else if (x_a != MAX_DIAMENSION && y_b != MAX_DIAMENSION){
	        // the distance is a "L shape"
	        // Closest WP == [X_A, Y_B]
	    NS_LOG_DEBUG(" L-shape : X-a / Y-b ");
            dist = abs(m_waypoints[a_wp_x][b_wp_y].second - pos_a.y);
            dist += abs(m_waypoints[a_wp_x][b_wp_y].first - pos_b.x);
            return dist;
	}
	else if (x_a == x_b && x_a== MAX_DIAMENSION){
	    // at the same Vertical box
            NS_LOG_DEBUG(" Same Vertical Box ");

	    a_wp_x = FindWPX (pos_a, y_a);
	    b_wp_x = FindWPX (pos_b, y_b);

	    // Vertical Distance
	    dist = abs(pos_a.y - pos_b.y);

	    // Minimum Horizontal
	    double tmp1 , tmp2;
	    tmp1 = abs(m_waypoints[a_wp_x][y_a].first - pos_a.x);
            tmp1 += abs(m_waypoints[a_wp_x][y_b].first - pos_b.x);

            tmp2 = abs(m_waypoints[b_wp_x][y_a].first - pos_a.x);
            tmp2 += abs(m_waypoints[b_wp_x][y_b].first - pos_b.x);

            if (tmp1 < tmp2){
                dist += tmp1;
            }
            else{
                dist += tmp2;
            }
            return dist;
	}
        else if (y_a == y_b && y_a== MAX_DIAMENSION){
            // at the same Horizontal box
            NS_LOG_DEBUG(" Same Horizontal box ");

            a_wp_y = FindWPY (pos_a, x_a);
            b_wp_y = FindWPY (pos_b, x_b);

            // Horizontal Distance
            dist = abs(pos_a.x - pos_b.x);

            // Minimum Vertical
            double tmp1 , tmp2;
            tmp1 = abs(m_waypoints[x_a][a_wp_y].second - pos_a.y);
            tmp1 += abs(m_waypoints[x_b][a_wp_y].second - pos_b.y);

            tmp2 = abs(m_waypoints[x_a][b_wp_y].second - pos_a.y);
            tmp2 += abs(m_waypoints[x_b][b_wp_y].second - pos_b.y);

            if (tmp1 < tmp2){
                dist += tmp1;
            }
            else{
                dist += tmp2;
            }
            return dist;
        }

	else {
	    NS_LOG_UNCOND("Scenario not implemented!!");
	    }

	return dist;
}

int GridMap::FindWPY (Vector p, int r_index)
{

	int wpY=0;
	double min_dist = abs(m_waypoints[r_index][wpY].second - p.y);

	for (int i=1; i < m_yMax; i++){
		if (abs(m_waypoints[r_index][i].second - p.y) <= min_dist){
			wpY = i;
		}
	}
	return wpY;
}

int GridMap::FindWPX (Vector p, int r_index)
{

	int wpX=0;
	double min_dist = abs(m_waypoints[wpX][r_index].first - p.x);

	for (int i=1; i < m_xMax; i++){
		if (abs(m_waypoints[i][r_index].first - p.x) <= min_dist){
			wpX = i;
		}
	}
	return wpX;
}

/// Calculates the distance between two nodes
/// Using map information -- Curvemetric distance
void GridMap::GetNLOSDistance(Vector pos_a,
							  int x_a,
							  int y_a,
							  Vector pos_b,
							  int x_b,
							  int y_b,
							  double &dist1,
							  double &dist2)
{
	int a_wp_x = x_a;
	int a_wp_y = y_a;
	int b_wp_x = x_b;
	int b_wp_y = y_b;

	NS_ASSERT_MSG (!(x_a == MAX_DIAMENSION && y_a == MAX_DIAMENSION), " Invalid Position for node A" );
	NS_ASSERT_MSG (!(x_b == MAX_DIAMENSION && y_b == MAX_DIAMENSION), " Invalid Position for node B");

	NS_LOG_DEBUG("NLOS Distance Calc");
	if ((x_a == x_b )&&(x_a!=MAX_DIAMENSION)){
		// On the same X-axis
	    NS_LOG_DEBUG(" Same X-axis ");
	    dist1 = abs(pos_a.y - pos_b.y);
	    return;
	}
	else if ((y_a == y_b)&&(y_a!=MAX_DIAMENSION)){
		// On the same Y-axis
            NS_LOG_DEBUG(" Same Y-axis ");
	    dist1 = abs(pos_a.x - pos_b.x);
	    return;
	}
	else if (y_a != MAX_DIAMENSION && x_b != MAX_DIAMENSION)
	  {
	    // the distance is a "L shape"
	    // Closest WP == [X_B, Y_A]
        NS_LOG_DEBUG(" L-shape : X-b / Y-a ");

	    dist1 = abs(m_waypoints[b_wp_x][a_wp_y].first - pos_a.x);
        dist2 = abs(m_waypoints[b_wp_x][a_wp_y].second - pos_b.y);
        return;
	}
	else if (x_a != MAX_DIAMENSION && y_b != MAX_DIAMENSION)
	  {
	     // the distance is a "L shape"
	     // Closest WP == [X_A, Y_B]
	     NS_LOG_DEBUG(" L-shape : X-a / Y-b ");
         dist1 = abs(m_waypoints[a_wp_x][b_wp_y].second - pos_a.y);
         dist2 = abs(m_waypoints[a_wp_x][b_wp_y].first - pos_b.x);
         return;
	}
	else if (x_a == x_b && x_a== MAX_DIAMENSION)
	  {
	     // at the same Vertical box " C shape"
         NS_LOG_DEBUG(" Same Vertical Box: C shape ");
  	     a_wp_x = FindWPX (pos_a, y_a);
	     b_wp_x = FindWPX (pos_b, y_b);

	     // Minimum Horizontal
	     double tmp1 , tmp2;
	     tmp1 = abs(m_waypoints[a_wp_x][y_a].first - pos_a.x);
         tmp1 += abs(m_waypoints[a_wp_x][y_b].first - pos_b.x);

         tmp2 = abs(m_waypoints[b_wp_x][y_a].first - pos_a.x);
         tmp2 += abs(m_waypoints[b_wp_x][y_b].first - pos_b.x);

         if (tmp1 < tmp2)
         {
             dist1 =  abs(m_waypoints[a_wp_x][y_a].first - pos_a.x);
             dist2 = abs(m_waypoints[a_wp_x][y_b].first - pos_b.x) + abs(pos_a.y - pos_b.y);
         }
         else
         {
             dist1 =  abs(m_waypoints[b_wp_x][y_a].first - pos_a.x);
             dist2 = abs(m_waypoints[b_wp_x][y_b].first - pos_b.x) + abs(pos_a.y - pos_b.y);
         }
         return;
	  }
    else if (y_a == y_b && y_a== MAX_DIAMENSION){
         // at the same Horizontal box
         NS_LOG_DEBUG(" Same Horizontal box: Π shape ");

         a_wp_y = FindWPY (pos_a, x_a);
         b_wp_y = FindWPY (pos_b, x_b);

         // Minimum Vertical
         double tmp1 , tmp2;
         tmp1 = abs(m_waypoints[x_a][a_wp_y].second - pos_a.y);
         tmp1 += abs(m_waypoints[x_b][a_wp_y].second - pos_b.y);

         tmp2 = abs(m_waypoints[x_a][b_wp_y].second - pos_a.y);
         tmp2 += abs(m_waypoints[x_b][b_wp_y].second - pos_b.y);

         if (tmp1 < tmp2)
         {
              dist1 = abs(m_waypoints[x_a][a_wp_y].second - pos_a.y);
              dist2 = abs(m_waypoints[x_b][a_wp_y].second - pos_b.y) + abs(pos_a.x - pos_b.x);
         }
         else
         {
              dist1 = abs(m_waypoints[x_a][b_wp_y].second - pos_a.y);
              dist2 = abs(m_waypoints[x_b][b_wp_y].second - pos_b.y) + abs(pos_a.x - pos_b.x);
         }
         return ;
        }

	else {
	    NS_LOG_UNCOND("Scenario not implemented!!");
	    }

	return;
}



}
