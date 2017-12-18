/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */


// From NS3 web site
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

// NetAnim
#include "ns3/netanim-module.h"

// --------------------------------------------- AODV
#include "ns3/aodv-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/v4ping-helper.h"
#include "ns3/aodv-rtable.h" // for define AOFDV

// --------------------------------------------- OLSR
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-helper.h"

// -------------------------------------------- Physimwifi
#include "ns3/physim-wifi-module.h"
#include "ns3/physim-wifi-phy.h"

// -------------------------------------------- C++
#include <iostream>
#include <cmath>
#include <map>
#include <vector>
#include <sys/time.h>

NS_LOG_COMPONENT_DEFINE ("simulation");

using namespace ns3;
using namespace std;

class EmitterReceiver ;

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                          Global variables                                              */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */

// Interval entre deux packets en secondes
double p_packetInterval = 1 ;

// Taille du packet
int p_packetSize = 256 ;

// Protocole à utiliser soit NOPROTOCOL soit AODV
string p_protocol = "NOPROTOCOL" ;

// taille des paquets en dessous de laquelle le RTS/CTS est mis en oeuvre
int p_valRtsCtsThreshold = 2200 ;

// Puissance d'émission
int p_txPower = 0 ;

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                          Ne pas changer                                                */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */

// Just to have a map between the nodeContainer in class and have access to a static method GenerateTraffic
NodeContainer s_nodeContainer;

/**
 */
NetDeviceContainer s_netDeviceContainer;

/**
 */
Ipv4InterfaceContainer s_ipv4InterfaceContainer;

//
Ns2MobilityHelper *s_ns2Helper ;

// Emitter/receiver links with size of data to be sent
vector<EmitterReceiver> s_vlinks ;

// Fichier de simulation
std::string s_scenarioFileName ;

// Fichier de traffic
std::string s_trafficFileName  ;

// Nombre de noeuds déterminé par la lecture fichier de scénatrio
unsigned int s_nNodes = 0 ;

// numéro du test
unsigned int s_testNumber = 0 ;

// Répertoire de sortie
std::ostringstream s_output ;

// fichier de log
std::ostringstream s_logfile ;

// Durée de la simulation
int s_dureeSimulation ;

// Quand des packets commencent par etre envoyé
int s_startSendTime = 1 ;

// PCAP
bool p_pcap = true ;

// Physical mode
#define DEFAULT_PHYSICALMODE "OfdmRate6MbpsBW10MHz"

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                          Class to build nodes link                                     */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */
/**
 * Class to store information about links between nodes with emitter / receiver / packet size
 * and messages sent and received
 */
class EmitterReceiver {

public:

	EmitterReceiver(int emitter, int receiver) {

		this->emitter = emitter ;
		this->receiver = receiver ;
		this->packetSize = p_packetSize ;
		this->msgSent = 0 ;
		this->msgReceived = 0 ;
	}

	EmitterReceiver(int emitter, int receiver, int packetSize) {

		this->emitter = emitter ;
		this->receiver = receiver ;
		this->packetSize = packetSize ;
		this->msgSent = 0 ;
		this->msgReceived = 0 ;
	}

	void MessageSent() {

		this->msgSent ++ ;
	}

	void MessageReceived() {

		this->msgReceived ++ ;
	}

	uint32_t GetEmitter() {
		return this->emitter ;
	}

	uint32_t GetReceiver() {
		return this->receiver ;
	}

	uint32_t GetPacketSize() {
		return this->packetSize ;
	}

	uint32_t GetMessageSent() {
		return this->msgSent ;
	}

	uint32_t GetMessageReceived() {
		return this->msgReceived ;
	}

private:
	/** Emitter
	 */
	uint32_t emitter ;
	/** Receiver
	 */
	uint32_t receiver ;
	/** Packet size
	 */
	uint32_t packetSize ;
	/** Messages sent
	 */
	uint32_t msgSent ;
	/** Messages received
	 */
	uint32_t msgReceived ;
} ;

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                                        STATIC METHODS                                  */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */
/**
 * Retreive the TTL value from a packet. 
 * To use this socket option Socket::SetRecvPktInfo(true) must be set during the creation of socket receiver
 * \return value TTL value, if no TTL value then -1 returned if TTL not set or -2 if no ipv4PacketInfoTag added, if !socket->IsRecvPktInfo() then TTL value = -3
 */
static int GetTtlValue(Ptr<Packet const> packet) {

	int ttlValue = -1;

	Ipv4PacketInfoTag ipv4PacketInfoTag;
	if (packet->PeekPacketTag (ipv4PacketInfoTag)) {
		ttlValue = (int)ipv4PacketInfoTag.GetTtl() ;
	}

	return ttlValue ;
}


/**
 * Generate the traffic
 */
static void GenerateTraffic (uint32_t nReceiver, uint32_t nEmitter, Ptr<Socket> socket, uint32_t pktSize, double pktInterval ) {

	// -------------------------------------------------------
	// Packet to be sent
	Ptr<Packet> packet = Create<Packet> (pktSize);

	Ptr<MobilityModel> mob = s_nodeContainer.Get(nReceiver)->GetObject<MobilityModel>();
	Vector pos = mob->GetPosition ();
	//std::cout << "Node " << nReceiver << ": POS: x=" << pos.x << ", y=" << pos.y << std::endl;

	Ptr<MobilityModel> mob1 = s_nodeContainer.Get(nEmitter)->GetObject<MobilityModel>();
	Vector pos1 = mob1->GetPosition ();
	//std::cout << "Node " << nEmitter << ": POS: x=" << pos1.x << ", y=" << pos1.y << std::endl;
	double distance = sqrt((pos.x - pos1.x)*(pos.x - pos1.x)+(pos.y - pos1.y)*(pos.y - pos1.y)+(pos.z - pos1.z)*(pos.z - pos1.z));

	// ----------------------------------------------------------------------------
	// Display information about sending => does not work for tag to be checked
	double speed = mob1->GetRelativeSpeed (mob) ;

	NS_LOG_UNCOND ("+AtApp " << Simulator::Now().GetSeconds()
			<< " packet " << packet->GetUid()
			<< " sent (" << nEmitter
			<< "->" << nReceiver
			<< "), dist: " << distance
			<< " relSpeed: " << speed
			<< " txSizeNet: " << pktSize
	);


	// -------------------------------------------------------
	// Send the packet
	socket->Send (packet);

	// -------------------------------------------------------
	// Schedule next packet
	// Adding new random variable
	//double offset = myRandom->GetValue(pktInterval/100 - pktInterval, pktInterval - pktInterval/100) ;

	if (Simulator::Now().GetSeconds() < (s_dureeSimulation-1))  {
		double offset = 0 ;
		Simulator::Schedule (Seconds(pktInterval + offset), &GenerateTraffic, nReceiver, nEmitter, socket, pktSize, pktInterval);
	}
}


/**
 * Analyse arguments and assign protocol and physical layer.
 * \param argc number of arguments issued from main args
 * \param argv arguments issued from main args
 * \return true if all parameters were correctly Configured
 */
bool Configure (int argc, char **argv) {

	// -------------------------------------------------
	CommandLine cmd;

	cmd.AddValue ("scenario", "Fichier de scénario", s_scenarioFileName) ;
	cmd.AddValue ("traffic", "Fichier de communication", s_trafficFileName) ;
        cmd.AddValue ("duration", "Durée de la simulation", s_dureeSimulation);
        cmd.AddValue ("nodes", "nombre de noeuds dans la simulation", s_nNodes) ;
        cmd.AddValue ("test", "numéro du test", s_testNumber) ;

	cmd.Parse (argc, argv);

        NS_LOG_INFO("Scenario utilisé : " << s_scenarioFileName << " avec un traffic issu du fichier " << s_trafficFileName << " pour " << s_dureeSimulation << " s") ;

	return true;
}

/** Load the scenario
 * \return true if scenario correctly loaded
 * \see ns3::Ns2MobilityHelper Class
 */
bool CreateNodes() {

	s_ns2Helper = new Ns2MobilityHelper(s_scenarioFileName) ;
	s_nodeContainer.Create (s_nNodes) ;
	s_ns2Helper->Install() ; 

	return true ;
}

/** Physim wifi installation
 * See https://svn.ensisa.uha.fr/redmine/projects/vanet-ns3/wiki/Configurations_Specifiques
 */
void InstallPhysimWifi() {

	// Configuration of defaults values
	string phyMode=DEFAULT_PHYSICALMODE;
	int valRtsCtsThreshold = p_valRtsCtsThreshold ; // 2200 ;
	double txPower = p_txPower ; // 33
	double txGain = 1.0 ;
	double rxGain = 0.0 ;
	double txPowerLevels = 1.0 ;
	double noiseFloor = -99.0 ; // default value
	double edDetectionThreshold = -104 ; // default value

	// SetDefault configuration
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", UintegerValue (valRtsCtsThreshold));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (valRtsCtsThreshold)) ;
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

	// Configure PhySimWifiPhy power parameters
	Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerEnd", DoubleValue (txPower));
	Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerStart", DoubleValue (txPower));
	Config::SetDefault ("ns3::PhySimWifiPhy::TxPowerLevels", UintegerValue (txPowerLevels) );
	Config::SetDefault ("ns3::PhySimWifiPhy::TxGain", DoubleValue (txGain));
	Config::SetDefault ("ns3::PhySimWifiPhy::RxGain", DoubleValue (rxGain));

	// Do not use a fixed scrambler (from Yens example code)
	Config::SetDefault ("ns3::PhySimScrambler::UseFixedScrambler", BooleanValue (false) );
	// Set the correct Channel Estimator implementation
	Config::SetDefault ("ns3::PhySimWifiPhy::ChannelEstimator", StringValue ("ns3::PhySimChannelFrequencyOffsetEstimator") );
	// Set noise floor of -99 dBm - Default value
	Config::SetDefault ("ns3::PhySimInterferenceHelper::NoiseFloor", DoubleValue (noiseFloor));
	// Set energy detection threshold to -104 dBm
	Config::SetDefault ("ns3::PhySimWifiPhy::EnergyDetectionThreshold", DoubleValue (edDetectionThreshold));
	// Set CcaModelThreshold to -65dBm - Default value for 10MHz channel
	Config::SetDefault ("ns3::PhySimWifiPhy::CcaModelThreshold", DoubleValue (-65.0) );

	// Enable/Disable Soft-Decision Viterbi, Method 1: Soft Decision
	Config::SetDefault ("ns3::PhySimConvolutionalEncoder::SoftViterbiDecision", BooleanValue (false));
	Config::SetDefault ("ns3::PhySimOFDMSymbolCreator::SoftViterbiDecision", BooleanValue (false));
	// In order to get the SNR
	Config::SetDefault ("ns3::PhySimWifiPhy::CalculateHeaderSinr", BooleanValue (false));
	Config::SetDefault ("ns3::PhySimWifiPhy::CalculatePayloadSinr", BooleanValue (false));
	Config::SetDefault ("ns3::PhySimWifiPhy::CalculateOverallSinr", BooleanValue (true));

	// Channel
	PhySimWifiChannelHelper wifiChannel; // Pas de défaut !!

        wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

	// Dummy propagationLossModel
	wifiChannel.AddPropagationLoss ("ns3::PhySimPropagationLossModel");

	/* *************************************************************************************** */
	/* CANAL DE PROPAGATION, LES DIFFERENTS EFFETS                                             */
	/* *************************************************************************************** */
	
	// Friis
        wifiChannel.AddPropagationLoss ("ns3::PhySimFriisSpacePropagationLoss");

	// TwoRayGround
        // wifiChannel.AddPropagationLoss ("ns3::PhySimTwoRayGroundPropagationLossModel", "SystemLoss", DoubleValue (1.0), "HeightAboveZ", DoubleValue(m_heightAboveZ), "SpeedLight", DoubleValue (299792458.0) );

	// PhySimShadowingPropagationLoss
        //wifiChannel.AddPropagationLoss ("ns3::PhySimShadowingPropagationLoss", "StandardDeviation", DoubleValue (6.0), "MinimumDistance", DoubleValue (10.0) );

	// Rice depending on LineOfSightPower value at 50 km/h
        //double lineOfSightDoppler = (13.33*5.9e9)/(0.3e9*10e6);
        //double phySimRicianPropagationLoss = 7.0 ;
        //wifiChannel.AddPropagationLoss ("ns3::PhySimRicianPropagationLoss", "MinimumRelativeSpeed", DoubleValue (2.0),"UseShortcut", BooleanValue (false),"LineOfSightPower", DoubleValue(phySimRicianPropagationLoss), "LineOfSightDoppler", DoubleValue (lineOfSightDoppler) );

	// Rayleigh
        // double lineOfSightDoppler = (13.33*5.9e9)/(0.3e9*10e6);
        // wifiChannel.AddPropagationLoss ("ns3::PhySimRicianPropagationLoss", "MinimumRelativeSpeed", DoubleValue (2.0), "UseShortcut", BooleanValue (false), "LineOfSightPower", DoubleValue(0.0), "LineOfSightDoppler", DoubleValue (lineOfSightDoppler) );

	/* *************************************************************************************** */

	// Configure the wireless channel characteristics
	PhySimWifiPhyHelper wifiPhy = PhySimWifiPhyHelper::Default ();

	wifiPhy.SetPcapDataLinkType(PhySimWifiPhyHelper::DLT_IEEE802_11_RADIO);
	wifiPhy.SetChannel (wifiChannel.Create ());

	WifiHelper wifi = WifiHelper::Default ();
	wifi.SetStandard(WIFI_PHY_STANDARD_80211p_SCH);

	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (phyMode),
                                      "NonUnicastMode", StringValue (phyMode));

	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	wifiMac.SetType ("ns3::AdhocWifiMac");

	// ------------------- Installation sur les noeuds
	s_netDeviceContainer = wifi.Install (wifiPhy, wifiMac, s_nodeContainer);

	// Add Ascii trace helper for Yans, PhySim or Network level
	if (p_pcap) {
          std::ostringstream fileName ;
          fileName << s_output.str().c_str() << "/traces" ;
          AsciiTraceHelper ascii;
          wifiPhy.EnableAscii(ascii.CreateFileStream (fileName.str()), s_netDeviceContainer);
          wifiPhy.EnablePcapAll (fileName.str());
	}
}

/** Create the devices
 */
void CreateDevices () {

  InstallPhysimWifi() ;
}

/**
 * No protocol installed for internet stack
 * \see MobilityIntegrationTest::InstallInternetStack
 */
void InstallNoProtocol () {

	InternetStackHelper stack;
	stack.Install (s_nodeContainer);
	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.0.0.0");
	s_ipv4InterfaceContainer = address.Assign (s_netDeviceContainer);
}

/**
 * No protocol installed for internet stack
 * \see MobilityIntegrationTest::InstallInternetStack
 */
void InstallAODV () {


  // -------------------------------------------------------------------
  // you can configure AODV attributes here using aodv.Set(name, value)
  /*
    Paramètres de AODV avec SN-2
    RReqRetries : 3
    NodeTraversalTime : 0.03
    ActiveRouteTimeOut : 10
    MyRouteTimeout : 10
    NetworkDiameter : 30
    AllowedHelloLoss : 2
    Pour le reste, je ne connais pas, où je ne vois pas de différence.
    Pour le enable Hello et Broadcast, c'est écrit dans le code que c'est pour indiquer l'envoie des messages.
  */

  /* 
  // Paramètrage avec les paramètres RFC
  bool gratuitous = true ;
  bool destinationOnly = true ;
  bool EnableHello = true ;
  bool EnableBroadcast = false ;
  int RreqRetries = 2 ;
  
  int ActiveRouteTimeout = 3 ; // in s
  int MyRouteTimeout = 2 * ActiveRouteTimeout ; // in s 
  
  int NetDiameter = 35 ; 
  int NodeTraversalTime = 40 ;  // in ms 
  int NetTraversalTime = 2 * NodeTraversalTime * NetDiameter ;
  
  int AllowedHelloLoss = 2 ;*/

  // Paramètrage avec les paramètres de Jonathan
  bool gratuitous = true ;
  bool destinationOnly = true ;
  bool EnableHello = true ;
  bool EnableBroadcast = false ;
  int RreqRetries = 3 ;
  int NodeTraversalTime = 30 ;
  int ActiveRouteTimeout = 10 ;
  int MyRouteTimeout = 10 ;
  int NetDiameter = 30 ;
  int AllowedHelloLoss = 2 ;
  int HelloInterval = 1 ; // in s

  AodvHelper aodv;
  aodv.Set("EnableHello", BooleanValue (EnableHello));
  aodv.Set("HelloInterval", TimeValue (Seconds(HelloInterval))) ;
  aodv.Set("EnableBroadcast", BooleanValue (EnableBroadcast));
  aodv.Set("RreqRetries", UintegerValue (RreqRetries)) ;
  aodv.Set("NodeTraversalTime", TimeValue (MilliSeconds (NodeTraversalTime))) ;
  aodv.Set("ActiveRouteTimeout", TimeValue (Seconds (ActiveRouteTimeout))) ;
  aodv.Set("MyRouteTimeout", TimeValue (Seconds (MyRouteTimeout))) ;
  aodv.Set("NetDiameter", UintegerValue (NetDiameter)) ;
  aodv.Set("AllowedHelloLoss", UintegerValue (AllowedHelloLoss)) ;
  aodv.Set("GratuitousReply", BooleanValue (gratuitous));
  aodv.Set("DestinationOnly", BooleanValue (destinationOnly));


  // -------------------------------------------------------------------
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (s_nodeContainer);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  s_ipv4InterfaceContainer = address.Assign (s_netDeviceContainer);
}

/**
 * Les adresses IP et les stacks IP
 */
void InstallInternetStack () {

  if (p_protocol.compare("NOPROTOCOL") == 0) {
    InstallNoProtocol() ;
  }
  else if (p_protocol.compare("AODV") == 0) {
    InstallAODV() ;
  }
  else {
    NS_FATAL_ERROR ("Mauvais protocole " << p_protocol) ;
  }
}

/**
 * Retrieve the IPV4 address from a node
 * \return IP address
 * \param index index of the node in NodeContainer
 */
Ipv4InterfaceAddress GetIpv4Address(uint32_t index) {

	for (uint32_t indexDevice = 0 ; indexDevice < s_netDeviceContainer.GetN() ; indexDevice ++) {

		Ptr<NetDevice> netDevice=s_netDeviceContainer.Get(indexDevice);
		Ptr<Node> node=netDevice->GetNode ();
		if (node != NULL) {
			Ptr<Ipv4> ipv4=node->GetObject<Ipv4> ();

			int32_t interface = ipv4->GetInterfaceForDevice (netDevice);
			uint32_t nb = ipv4->GetNAddresses(interface);

			for(uint32_t indexInterface = 0 ;  indexInterface < nb ; indexInterface ++) {
				Ipv4InterfaceAddress addr = ipv4->GetAddress(interface, indexInterface);

				if (index == node->GetId() ) {
					//std::cerr << "FD::Found the correct node " << node->GetId() << " has address " << addr.GetLocal() << " Mask " << addr.GetMask() << " Broadcast "<< addr.GetBroadcast() << std::endl;
					return addr ;
				}
			}
		}
		else {
			std::cerr << "FD:Node is null" << std::endl ;
		}
	}

	return Ipv4InterfaceAddress() ;
}

/** Socket application
 * \author Benoit Hilt
 */
void ReceivePacket (Ptr<Socket> socket) {

	// For the NODE id
	// Ptr<Node> node = socket->GetNode() ;

	Ptr<Packet> packet;

	while ((packet = socket->Recv ())) {

		uint64_t packetNb = packet->GetUid() ;

		ClTagTrans clTagTrans;
		if (packet->PeekPacketTag(clTagTrans)){

			double distance = -1 ;
			long dstNode = -1 ;
			long srcNode = -1 ;

			ClTagNode clTagNode ;
			if (packet->PeekPacketTag(clTagNode)) {
				distance = clTagNode.GetDistance() ;
				dstNode =  clTagNode.GetDstNode() ;
				srcNode =  clTagNode.GetSrcNode() ;
			}

			std::ostringstream os ;
			os << "" ;

			ClTagSnr clTagSnr ;
			if (packet->PeekPacketTag(clTagSnr)) {
				os << " SNR = " << clTagSnr.GetSnr() << " and Packet Error Rate = " << clTagSnr.GetPer() ;
			}

			// TTL : time to leave (packet) and time of received packet
			int ttlValue = -1 ;
			if (socket->IsRecvPktInfo()) {
				ttlValue = GetTtlValue(packet) ; // TTL value
			}

			// TTL difference to be displayed
			std::ostringstream ttl ;
			//ttl << "TTL: " << pInfo->second.GetTtlValueEnd() << "/" << ttlValue << " HOP count = " << (pInfo->second.GetTtlValueEnd() - ttlValue);
			ttl << "TTL: " << 64 << "/" << ttlValue << " HOP count = " << (64 - ttlValue);

			NS_LOG_UNCOND ("-AtApp " << Simulator::Now().GetSeconds()
					<< " Node " << dstNode /*<< "/" << ((node==NULL)?(int)-1:(int)node->GetId())*/
					<< " packet " << packet->GetUid()
					<< " received from node " << srcNode
					<< " (distance = " << distance << " m)"
					<< " with"
					<< os.str()
					<< " RX Power: " << clTagTrans.GetRxPower() << " dB"
					<< " Tx Power: " << clTagTrans.GetTxPower() << " dB"
					<< " " << ttl.str()) ;
		}
		else {
			NS_LOG_DEBUG ("-AtApp " << Simulator::Now().GetSeconds() <<" Packet "<<packetNb <<" received without ClTagTrans");
		}
	}
}

/**
 * Install socket for node communication
 */
void InstallSocketApplications() {

  uint32_t port = 4000;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  
  if (s_vlinks.size() == 0) {
    NS_FATAL_ERROR ("No traffic defined, please used or a file or random (see parameters: NodesEmitter or TrafficFileName)");
  }
  
  // Configuration of the applications
  // UdpClient -> UdpServer
  int i = 1 ;
  for (vector<EmitterReceiver>::iterator it = s_vlinks.begin() ; it != s_vlinks.end() ; it ++, i++) {
    NS_LOG_UNCOND("Link " << i << " : node " << it->GetEmitter() << " send information to " << it->GetReceiver()) ;

    uint32_t nNodeServer = it->GetReceiver() ;
    uint32_t nNodeClient = it->GetEmitter() ;
    uint32_t packetSize = it->GetPacketSize() ;

    // Configure the server
    Ipv4InterfaceAddress destAddress = GetIpv4Address(nNodeServer) ;
    Ptr<Socket> recvSink = Socket::CreateSocket (s_nodeContainer.Get (nNodeServer), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), port);
    recvSink->Bind (local);
    //recvSink->Bind();
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
    recvSink->SetRecvPktInfo(true); // added to have TTL information

    // Configure the client
    Ptr<Socket> source = Socket::CreateSocket ( s_nodeContainer.Get(nNodeClient), tid);
    InetSocketAddress remote = InetSocketAddress (destAddress.GetLocal(), port);
    source->SetAllowBroadcast (true);
    source->Bind();
    source->Connect (remote);  // Used to set the destination address of the outgoing packets

    double offset = 0.001*(i-1) ; //* nNodeClient ;

    Ipv4InterfaceAddress srcAddress = GetIpv4Address(nNodeClient) ;
    NS_LOG_UNCOND("\tClient " << srcAddress.GetLocal() << " (" << nNodeClient << ") sends information to " << destAddress.GetLocal() << " (" << nNodeServer << ")" << " (start time = " << Seconds(s_startSendTime + offset).GetSeconds() << " s)") ;

    Simulator::Schedule (Seconds(1 + offset), &GenerateTraffic, nNodeServer, nNodeClient, source, packetSize, p_packetInterval);
  }
}

/**
 * Les applications => depend of the traffic file name => DEFAULT -> RANDOM applications, NOT DEFAULT => take the filename and parse it
 */
void InstallApplications () {

  std::ifstream trafficFile ;
  trafficFile.open(s_trafficFileName.c_str()) ;
  if (!trafficFile.is_open()) {
    NS_FATAL_ERROR("Cannot read traffic file " << s_trafficFileName << " please check the file name") ;
  }
  else {
    int iNodeEmit, iNodeRec, packetSize ;
    string line ;
    while (std::getline(trafficFile,line)) {
      iNodeEmit = -1 ; iNodeRec = -1 ; packetSize = -1 ;
      if (line.find("#") == string::npos && !line.empty()) {
        std::istringstream iss(line);
        iss >> iNodeEmit >> iNodeRec >> packetSize ;
        
        if ((iNodeEmit < 0) || (iNodeRec < 0)) {
          trafficFile.close() ;
          NS_FATAL_ERROR("Incorrect traffic file, should be node number and found \"" << line << "\", please review traffic file") ;
        }
        else {
          if (((uint32_t)iNodeEmit >= s_nNodes) || ((uint32_t)iNodeRec >= s_nNodes)) {
            trafficFile.close() ;
            NS_FATAL_ERROR("Incorrect traffic file, node number should be number of nodes, found emitter = " << iNodeEmit << " and receiver = " << iNodeRec << " regarding total node number of " << s_nNodes) ;
          }
          else {
            if (packetSize <= 0) packetSize = p_packetSize ;
            s_vlinks.push_back(EmitterReceiver(iNodeEmit, iNodeRec, packetSize)) ;
          }
        }
      }
    }
    
    trafficFile.close();
  }

  InstallSocketApplications() ;
}

/**
 * Prepare the configuration before running
 */
void Prepare () {

  // Répertoire de sortie
  s_output << "output/test" << s_testNumber << "/" << s_scenarioFileName ;
  std::ostringstream os ; 
  os << "mkdir -p " << s_output.str() ;
  system(os.str().c_str()) ;

  // ------ Load scenario file
  CreateNodes() ;

  NS_LOG_DEBUG("Devices creation") ;
  CreateDevices() ;

  NS_LOG_DEBUG("IP addresses and stacks") ;

  InstallInternetStack ();

  NS_LOG_DEBUG("Applications on nodes");

  InstallApplications();
}

/**
 * Load the scenario file => Create nodes 
 * Create devices
 * Create internet stacks
 * create application
 * run the simulation
 */
void Run () {

	// -------------------------------------------------------------------------------------
	// Prepare simulation
	Prepare() ;

	// -------------------------------------------------------------------------------------
	// NetAnim => available on NS-3.15
	std::ostringstream logFileNameAnim ;
	logFileNameAnim << s_output.str().c_str() << "/animation.xml" ;
	AnimationInterface anim (logFileNameAnim.str().c_str()); // Mandatory

	// ---------------------------------------------------------------------------------------------------------------
	// Start the simulation
	Simulator::Stop (Seconds (s_dureeSimulation)) ;

	// Run the simulation
	Simulator::Run ();

	// ---------------------------------------------------------------------------------------------------------------
	// Destroy simulation
	Simulator::Destroy ();

	// ---------------------------------------------------------------------------------------------------------------
	// Stop animation
	// anim.StopAnimation ();
}


/**
 *
 */
int main (int argc, char **argv) {

	LogComponentEnable("simulation", LOG_LEVEL_DEBUG);

	// Change the seed manager
	SeedManager::SetSeed(time(NULL)) ;

	// -------------------------------------------------- Configuration
	if (!Configure (argc, argv))
          NS_FATAL_ERROR ("Configuration failed. Aborted.");
        
	NS_LOG_INFO("Configuration done, starting the simulation") ;
        
	Run() ;

	return 0;
}
