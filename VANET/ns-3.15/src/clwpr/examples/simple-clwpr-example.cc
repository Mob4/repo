/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Konstantinos Katsaros (University of Surrey)
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
 */

/* 
 * This example demonstrated the use of CLWPR routing protocol, as well as the use of
 * Winner Propagation Loss module for urban vehicular ad-hoc scenarios.
 * Using Bonnmotion we generated the trace files which are loaded with the use of Ns2TransmobilityHelper
 * class to work with mobility. The scenario is a 5x5 ManhattanGrid network with 200 nodes, mean speed 15m/s, 
 * minimum speed 5m/s and grid distance of 500m.
 *
 * Detailed example description.
 *
 *  - intended usage: this should be used in order to test and configure CLWPR routing protocol in an urban scenario.
 *                    Moreover, test the use of realistic propagation loss models which take into consideration buildings.
 *  - behavior:
 *      - GridMap object is created which specifies the underlying roadtopology. Currently only Manhattan Grid networks are supported.
 *      - BuildingMapModel and Shadowing objects are created afterwards.
 *      - According the the command line attributes we can use either WinnerB1LossModel or TwoRayGround propagation.
 *      - Ns2TransmobilityHelperTrace object is created, whith the specified trace file. At this moment, only
 *      specify the file, and no movements are scheduled.
 *      - A node container is created with the correct node number specified in the command line.
 *      - Use Install method of Ns2TransmobilityHelperTrace to set mobility to nodes. At this moment, file is
 *        read line by line, and the movement is scheduled in the simulator.
 *      - ClwprHelper object is created, and  is configured based on the command line attributes.
 *        It is then installed to all nodes in the container.
 *      - 10 concurrent connections are created using UDP connections. The size of the packets and the frequency can be configured with command line
 *  - expected output: example used flowmonitor to print packet delivery ratio, end-to-end delay, hop count, drop packets and other metrics
 *                     for these 10 concurrent connections. Also, using callbacks, we can monitor when a packet has been cached by the routing protocol
 *                     and when it has found a new route.
 *
 * Usage of clwpr, winner modules and ns2-mobility-trace:
 *
 *          ./waf --run "scratch/simple-clwpr-example --numNodes=200  --traceFile=./scratch/mobility.ns_movements --movingNodes=200 --duration=50   //
 *           --map=0 --predict=0 --TxR=500 --hello=1.5 --CnF=0 --cT=0 --DistFactor=1 --SNRFactor=0 --CnFFactor=0 --AngFactor=0 --RoadFactor=0 //
 *			--MACFactor=0 --UtilFactor=0 --vanet=1 --numPackets=400 --interval=0.1 --winner=1" 
 *
 *          NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements
 *                included in the same directory that the present file.
 *          NOTE 2: Number of nodes present in the trace file must match with the command line argument.
 *                  Note that you must know it before to be able to load it.
 *          NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.
 */



#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/clwpr-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/winner-models-module.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


/********** Useful macros **********/

NS_LOG_COMPONENT_DEFINE ("ClwprMovingWinner");

using namespace ns3;

double DropCount = 0;
double RxCount = 0;
double SNRtmp = 0;
double SNRtmp2 = 0;

void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_DEBUG ("Received one packet!");
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, 
                           socket, pktSize,pktCount-1, pktInterval);
//      std::cout<< "Packet #" << pktCount << " queued."<<std::endl;
    }
  else
    {
      socket->Close ();
    }
}

static void PacketCnF (const Ipv4Header &header, Ptr< const Packet> p){

  NS_LOG_INFO ("A packet from: "<< header.GetIdentification() <<" to: " << header.GetDestination() << " has been queued (CnF)");
  NS_LOG_INFO ( "CnF packet" );
}

static void PacketDequeue (const Ipv4Header &header, Ptr< const Packet> p){

  NS_LOG_INFO ("A packet from: "<< header.GetIdentification() <<" to: " << header.GetDestination() << " has been queued (CnF)");
  NS_LOG_INFO (" Route Found ");
}

int main (int argc, char *argv[])
{
  std::string traceFile;
  double duration=50.0;
  int movingNodes; //number of moving nodes used for ns2Trace incase there are other static nodes

  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  double distance = 500;  // m
  uint32_t packetSize = 512; // bytes
  uint32_t numPackets = 20;
  uint32_t numNodes = 200;  // 
  double interval = 2.0; // seconds
  bool verbose = true;
  bool tracing = true;
  bool map = false; // Enable curvemetric distance in CLWPR
  bool debug = true; 
  bool predict = false; // Enable prediction in CLWPR
  bool cache = false; // Enable Carry-n-Forward in CLWPR
  bool enchance = false; // Enable map information (road, angle)
  bool winner = true; // Enable winner propagation models
  double cacheTime = 10.0; // max time to keep in cache
  int range = 500; // Communication range
  double hello = 1.5; // Hello interval time
  bool vanet = true; // Enable 802.11p or 802.11b
// Configuration of CLWPR factors for weighting function
  double AngFactor = 1; 
  double DistFactor = 1;
  double UtilFactor = 1;
  double MACFactor = 1;
  double CnFFactor = 1;
  double RoadFactor = 1;
  double SNRFactor = -1;
  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("distance", "distance (m)", distance);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
  cmd.AddValue ("numNodes", "number of nodes", numNodes);
  cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
  cmd.AddValue ("movingNodes", "Number of moving nodes", movingNodes);
  cmd.AddValue ("duration", "Duration of Simulation", duration);

  cmd.AddValue ("map", "Enable the MAP integration", map);
  cmd.AddValue ("emap", "Enable the MAP integration", enchance);
  cmd.AddValue ("predict", "Enable the Position prediction", predict);
  cmd.AddValue ("dbg", "turn on Debug tracing", debug);
  cmd.AddValue ("TxR", "Select TxRange -- 250 or 500m", range);
  cmd.AddValue ("hello", "Hello Interval time in seconds", hello);
  cmd.AddValue ("cT", "Caching time in seconds", cacheTime);
  cmd.AddValue ("CnF", "Enabling Carry'n'Forward mechanism", cache);

  cmd.AddValue ("AngFactor", "Enabling Carry'n'Forward mechanism", AngFactor);
  cmd.AddValue ("DistFactor", "Enabling Carry'n'Forward mechanism", DistFactor);
  cmd.AddValue ("UtilFactor", "Enabling Carry'n'Forward mechanism", UtilFactor);
  cmd.AddValue ("MACFactor", "Enabling Carry'n'Forward mechanism", MACFactor);
  cmd.AddValue ("CnFFactor", "Enabling Carry'n'Forward mechanism", CnFFactor);
  cmd.AddValue ("RoadFactor", "Enabling Carry'n'Forward mechanism", RoadFactor);
  cmd.AddValue ("SNRFactor", "Enabling Carry'n'Forward mechanism", SNRFactor);

  cmd.AddValue ("vanet", " Set 802.11p ", vanet);
  cmd.AddValue ("winner", " ENABLE WINNEL PROPAGATION LOSS MODELS ", winner);
  cmd.Parse (argc, argv);


  if (debug)  LogComponentEnable ("ClwprRoutingProtocol",LOG_LEVEL_DEBUG);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("500"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel ;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

// Set up the road topology
  GridMap gridMap = GridMap(5, 5, distance);
// Set up visibility and shadowing models
  BuildingMapModel vis = BuildingMapModel("./scratch/visibility.txt", &gridMap);
  ShadowingModel shadow = ShadowingModel();

// Set up propagation models
  if (winner){
    wifiChannel.AddPropagationLoss("ns3::WinnerB1LossModel",
		   	   	   	   	   	   	   "Frequency", DoubleValue(5.9e9),
		   	   	   	   	   	   	   "EffEnvironmentHeight", DoubleValue(1),
		   	   	   	   	   	   	   "VisibilityModel", PointerValue(&vis),
		   	   	   	   	   	   	   "ShadowingModel", PointerValue(&shadow));
	if (range == 500){
   	  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-96));
	  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-99));
	}
	else if (range == 250){
	  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-83));
	  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-86));
	}
  }
  else {
    wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
	 	  	  	  	  	  	  	    "SystemLoss", DoubleValue(1),
		  	  	  	  	  	  	    "HeightAboveZ", DoubleValue(1.5));
    if (range == 500){
	  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-68));
	  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-71));
    }
    else if (range == 250){
	  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-61.8));
	  wifiPhy.Set ("CcaMode1Threshold", DoubleValue(-64.8));
    }
  }

  // Values for typical VANET scenarios according to 802.11p
  wifiPhy.Set ("TxPowerStart", DoubleValue(33));
  wifiPhy.Set ("TxPowerEnd", DoubleValue(33));
  wifiPhy.Set ("TxPowerLevels", UintegerValue(1));
  wifiPhy.Set ("TxGain", DoubleValue(0));
  wifiPhy.Set ("RxGain", DoubleValue(0));

  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  if (vanet){
       wifi.SetStandard (WIFI_PHY_STANDARD_80211p_CCH);
  }
  else {
      wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  }
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue(phyMode),
                                   "ControlMode",StringValue(phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);


  // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  ns2.Install (); // configure movements for each node, while reading trace file

  // Enable CLWPR
  ClwprHelper clwpr;
  
  clwpr.Set("MapFlag", BooleanValue(map));
  clwpr.Set("PredictFlag", BooleanValue (predict));
  clwpr.Set("HelloInterval", TimeValue(Seconds(hello)));
  clwpr.Set("CacheFlag", BooleanValue(cache));
  clwpr.Set("MaxQueueTime", TimeValue(Seconds(cacheTime)));
  clwpr.Set("EMapFlag", BooleanValue(enchance));

  clwpr.Set("DistFact", DoubleValue(DistFactor));
  clwpr.Set("AngFact", DoubleValue(AngFactor));
  clwpr.Set("UtilFact", DoubleValue(UtilFactor));
  clwpr.Set("MACFact", DoubleValue(MACFactor));
  clwpr.Set("CnFFact", DoubleValue(CnFFactor));
  clwpr.Set("SNRFact", DoubleValue(SNRFactor));
  clwpr.Set("RoadFact", DoubleValue(RoadFactor));
  
  clwpr.Set("RoadMap", PointerValue(&gridMap));

//  Ipv4StaticRoutingHelper staticRouting;
//  LogComponentEnableAll(LOG_PREFIX_NODE);

  Ipv4ListRoutingHelper list;
//  list.Add (staticRouting, 0);
  list.Add (clwpr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);


  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // Number of Connections = 10
  // Interval = 2.000000
    Ptr<Socket> receivers[10];
    Ptr<Socket> sources[10];
    receivers[0]=Socket::CreateSocket (c.Get (2), tid);
    sources[0]=Socket::CreateSocket (c.Get (1), tid);
    receivers[1]=Socket::CreateSocket (c.Get (3), tid);
    sources[1]=Socket::CreateSocket (c.Get (2), tid);
    receivers[2]=Socket::CreateSocket (c.Get (4), tid);
    sources[2]=Socket::CreateSocket (c.Get (2), tid);
    receivers[3]=Socket::CreateSocket (c.Get (10), tid);
    sources[3]=Socket::CreateSocket (c.Get (9), tid);
    receivers[4]=Socket::CreateSocket (c.Get (13), tid);
    sources[4]=Socket::CreateSocket (c.Get (12), tid);
    receivers[5]=Socket::CreateSocket (c.Get (14), tid);
    sources[5]=Socket::CreateSocket (c.Get (13), tid);
    receivers[6]=Socket::CreateSocket (c.Get (17), tid);
    sources[6]=Socket::CreateSocket (c.Get (16), tid);
    receivers[7]=Socket::CreateSocket (c.Get (19), tid);
    sources[7]=Socket::CreateSocket (c.Get (18), tid);
    receivers[8]=Socket::CreateSocket (c.Get (20), tid);
    sources[8]=Socket::CreateSocket (c.Get (19), tid);
    receivers[9]=Socket::CreateSocket (c.Get (23), tid);
    sources[9]=Socket::CreateSocket (c.Get (22), tid);

    for (int i=0; i < 10; i++){
       InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
       receivers[i]->Bind (local);
       receivers[i]->SetRecvCallback (MakeCallback (&ReceivePacket));
    }

    InetSocketAddress remote[10] = InetSocketAddress (i.GetAddress (1, 0), 80);
    remote[0] = InetSocketAddress (i.GetAddress (2, 0), 80);
    remote[1] = InetSocketAddress (i.GetAddress (3, 0), 80);
    remote[2] = InetSocketAddress (i.GetAddress (4, 0), 80);
    remote[3] = InetSocketAddress (i.GetAddress (10, 0), 80);
    remote[4] = InetSocketAddress (i.GetAddress (13, 0), 80);
    remote[5] = InetSocketAddress (i.GetAddress (14, 0), 80);
    remote[6] = InetSocketAddress (i.GetAddress (17, 0), 80);
    remote[7] = InetSocketAddress (i.GetAddress (19, 0), 80);
    remote[8] = InetSocketAddress (i.GetAddress (20, 0), 80);
    remote[9] = InetSocketAddress (i.GetAddress (23, 0), 80);

    for (int i=0; i<10 ; i++){
      sources[i]->Connect(remote[i]);
    }


  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("simple-clwpr-example.tr"));
      wifiPhy.EnablePcap ("simple-clwpr-example", devices);
      // Trace routing tables
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("simple-clwpr-example.routes", std::ios::out);
      clwpr.PrintRoutingTableAllEvery (Seconds (2), routingStream);

      // To do-- enable an IP-level trace that shows forwarding events only
    }
  
  Simulator::Schedule ((Seconds (5.561746)), &GenerateTraffic, sources[0], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (7.103547)), &GenerateTraffic, sources[1], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (8.474123)), &GenerateTraffic, sources[2], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (7.022612)), &GenerateTraffic, sources[3], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (4.581176)), &GenerateTraffic, sources[4], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (9.449938)), &GenerateTraffic, sources[5], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (5.633268)), &GenerateTraffic, sources[6], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (6.152299)), &GenerateTraffic, sources[7], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (7.682519)), &GenerateTraffic, sources[8], packetSize, numPackets, interPacketInterval);
  Simulator::Schedule ((Seconds (9.224302)), &GenerateTraffic, sources[9], packetSize, numPackets, interPacketInterval);


  Config::ConnectWithoutContext("/NodeList/*/$ns3::clwpr::RoutingProtocol/CarryNForward", MakeCallback (&PacketCnF));
  Config::ConnectWithoutContext("/NodeList/*/$ns3::clwpr::RoutingProtocol/Dequeue", MakeCallback (&PacketDequeue));


  // 8. Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(1000));


  Simulator::Stop (Seconds (duration));
  Simulator::Run ();


  // 10. Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  uint32_t totalRx =0;
  uint32_t totalTx =0;
  uint32_t totalDrop =0;
  double delay = 0;
  double count =0;
  double hopCount = 0;
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if ((t.sourceAddress=="10.1.1.2" && t.destinationAddress == "10.1.1.3") ||
          (t.sourceAddress=="10.1.1.3" && t.destinationAddress == "10.1.1.4") ||
          (t.sourceAddress=="10.1.1.3" && t.destinationAddress == "10.1.1.5") ||
          (t.sourceAddress=="10.1.1.10" && t.destinationAddress == "10.1.1.11") ||
          (t.sourceAddress=="10.1.1.13" && t.destinationAddress == "10.1.1.14") ||
          (t.sourceAddress=="10.1.1.14" && t.destinationAddress == "10.1.1.15") ||
          (t.sourceAddress=="10.1.1.17" && t.destinationAddress == "10.1.1.18") ||
          (t.sourceAddress=="10.1.1.19" && t.destinationAddress == "10.1.1.20") ||
          (t.sourceAddress=="10.1.1.20" && t.destinationAddress == "10.1.1.21") ||
          (t.sourceAddress=="10.1.1.23" && t.destinationAddress == "10.1.1.24")  )
        {
          if (i->second.rxPackets != 0){
        	  totalRx +=  i->second.rxPackets;
        	  totalTx +=  i->second.txPackets;
        	  hopCount += (i->second.timesForwarded / i->second.rxPackets +1);
        	  delay += (i->second.delaySum.GetSeconds() / i->second.rxPackets);
        	  count++;
        	  for (uint32_t j=0; j < i->second.packetsDropped.size() ; j++){
       	           totalDrop += i->second.packetsDropped[j];
     	      }
          }
        }
    }
  std::cout << "  PDR = " << ((double)totalRx / (double)totalTx)*100.0  << " \n";
  std::cout << "  Mean HOP count = " << hopCount / count << "\n";
  std::cout << "  Mean Delay = " << delay / count << "\n";
  std::cout << "  L3 Drop Packets = " << totalDrop << "\n" ;
//  monitor->SerializeToXmlFile("simple-clwpr-example.flowmon", true, true);
  // 11. Cleanup
  Simulator::Destroy ();
  return 0;
}

