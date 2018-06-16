/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
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
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
 */
#include <atomic>
#include <mutex>
#include <map>
#include <vector>

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"

#include <iostream>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/internet-module.h"

#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"

using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

WaveBsmHelper m_waveBsmHelper; ///< helper
int m_nodeSpeed; ///< in m/s
int m_nodePause; ///< in s
//int average_speed;
int speed_threshold;
/*
 * In WAVE module, there is no net device class named like "Wifi80211pNetDevice",
 * instead, we need to use Wifi80211pHelper to create an object of
 * WifiNetDevice class.
 *
 * usage:
 *  NodeContainer nodes;
 *  NetDeviceContainer devices;
 *  nodes.Create (2);
 *  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
 *  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
 *  wifiPhy.SetChannel (wifiChannel.Create ());
 *  NqosWaveMacHelper wifi80211pMac = NqosWave80211pMacHelper::Default();
 *  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
 *  devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);
 *
 * The reason of not providing a 802.11p class is that most of modeling
 * 802.11p standard has been done in wifi module, so we only need a high
 * MAC class that enables OCB mode.
 */
 ///-----> Start copy

 class MyTag : public Tag
 {
 public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  /**
   * Set the tag value
   * \param value The tag value.
   */
  void SetSimpleValue (uint32_t value);
  /**
   * Get the tag value
   * \return the tag value.
   */
  uint32_t GetSimpleValue (void) const;
 private:
  uint32_t m_simpleValue;  //!< tag value
 };


 uint32_t m_phyTxPkts; ///< phy transmit packets
 uint32_t m_phyTxBytes; ///< phy transmit bytes

 //NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");

 TypeId
 MyTag::GetTypeId (void)
 {
   static TypeId tid = TypeId ("ns3::MyTag")
     .SetParent<Tag> ()
     .AddConstructor<MyTag> ()
     .AddAttribute ("SimpleValue",
                    "A simple value",
                    EmptyAttributeValue (),
                    MakeUintegerAccessor (&MyTag::GetSimpleValue),
                    MakeUintegerChecker<uint32_t> ())
   ;
   return tid;
 }
 TypeId
 MyTag::GetInstanceTypeId (void) const
 {
   return GetTypeId ();
 }
 uint32_t
 MyTag::GetSerializedSize (void) const
 {
   return 4;
 }
 void
 MyTag::Serialize (TagBuffer i) const
 {
   i.WriteU32(m_simpleValue);
 }
 void
 MyTag::Deserialize (TagBuffer i)
 {
   m_simpleValue = i.ReadU32();
 }
 void
 MyTag::Print (std::ostream &os) const
 {
   os << "v=" << (uint32_t)m_simpleValue;
 }
 void
 MyTag::SetSimpleValue (uint32_t value)
 {
   m_simpleValue = value;
 }
 uint32_t
 MyTag::GetSimpleValue (void) const
 {
   return m_simpleValue;
 }


 struct NodePos {

 	NodePos() :
 			 m_send_rec_flag(' '),
 			 m_x(0),
 			 m_y(0),
 			 m_z(0) {}
 	NodePos(double x, double y, double z) :
 			 m_send_rec_flag(' '),
 			 m_x(x),
 			 m_y(y),
 			 m_z(z) {}
 	NodePos(char send_rec, double x, double y, double z) :
 			 m_send_rec_flag(send_rec),
 			 m_x(x),
 			 m_y(y),
 			 m_z(z) {}

 	~NodePos() {}

 	char m_send_rec_flag;
 	double m_x;
 	double m_y;
 	double m_z;
 };


 static uint32_t MaxNodes =500;

 static int NodeMoving[500] = {0};
 struct NodeState {
 	const static uint32_t _lcl_max_nodes = 500;
 	NodePos stats[_lcl_max_nodes];
 	uint32_t m_send_node_id;
 	NodeState() : m_send_node_id(0)
 	{
 		for (uint32_t i =0; i < _lcl_max_nodes; i++)
 		{
 			NodePos ns;
 			stats[i]= ns;
 		}

 	}
 	void SendPacket(uint32_t m_Node_Id, uint32_t m_Msg_Id,
 			double m_x, double m_y, double m_z)
 	{
 		NodePos ns('S', m_x, m_y, m_z);
 		stats[m_Node_Id] = ns;
 		m_send_node_id = m_Node_Id;
 	}
 	void RecvPacket(uint32_t m_Node_Id, uint32_t m_Msg_Id,
 			double m_x, double m_y, double m_z)
 	{
 		NodePos ns('R', m_x, m_y, m_z);
 		stats[m_Node_Id] = ns;
 	}
 	void SetNodeLocation(uint32_t m_Node_Id,
 			double m_x, double m_y, double m_z)
 	{
 		NodePos ns(' ', m_x, m_y, m_z);
 		stats[m_Node_Id] = ns;
 	}
 	void PrintDistFrom(uint32_t Node_Id)
 	{
 		for (uint32_t i =0; i < MaxNodes; i++)
 		{
 			double dx =stats[i].m_x - stats[Node_Id].m_x;
 			double dy =stats[i].m_y - stats[Node_Id].m_y;
 			double dz =stats[i].m_z - stats[Node_Id].m_z;
 			std::cout << "," << (dx*dx + dy*dy + dz*dz);
 		}
 	}

  vector<double> GetDistfrom(uint32_t Node_Id){
    vector<double> dist;
    double dist_sq;
    for (uint32_t i =0; i < MaxNodes; i++)
 		{
 			double dx =stats[i].m_x - stats[Node_Id].m_x;
 			double dy =stats[i].m_y - stats[Node_Id].m_y;
 			double dz =stats[i].m_z - stats[Node_Id].m_z;
      dist_sq=(dx*dx + dy*dy + dz*dz);
 			//std::cout << "," << (dx*dx + dy*dy + dz*dz);
      dist.push_back(dist_sq);

 		}
    return dist;
  }
  vector<uint32_t> NodesInRange(uint32_t Node_Id,double dist_from){
    vector<uint32_t> nodes;
    double dist_sq;
    for (uint32_t i =0; i < MaxNodes; i++)
    {
      double dx =stats[i].m_x - stats[Node_Id].m_x;
      double dy =stats[i].m_y - stats[Node_Id].m_y;
      double dz =stats[i].m_z - stats[Node_Id].m_z;
      dist_sq=(dx*dx + dy*dy + dz*dz);
      //std::cout << "," << (dx*dx + dy*dy + dz*dz);
      if(dist_from>dist_sq){
        nodes.push_back(i);
      }


    }
    return nodes;
  }
  void PrintNodesInRange(uint32_t Node_Id,double dist_from){
    int count=0;
    double dist_sq;
    for (uint32_t i =0; i < MaxNodes; i++)
    {
      double dx =stats[i].m_x - stats[Node_Id].m_x;
      double dy =stats[i].m_y - stats[Node_Id].m_y;
      double dz =stats[i].m_z - stats[Node_Id].m_z;
      dist_sq=(dx*dx + dy*dy + dz*dz);
      //std::cout << "," << (dx*dx + dy*dy + dz*dz);
      if(dist_sq<dist_from && NodeMoving[i] !=0 && Node_Id != i){
        //if(dist_sq<dist_from){
        count++;
      }
    }
    std::cout <<",count"<< count;
    for (uint32_t i =0; i < MaxNodes; i++)
    {
      double dx =stats[i].m_x - stats[Node_Id].m_x;
      double dy =stats[i].m_y - stats[Node_Id].m_y;
      double dz =stats[i].m_z - stats[Node_Id].m_z;
      dist_sq=(dx*dx + dy*dy + dz*dz);
      std::cout << "," << (dx*dx + dy*dy + dz*dz);
    }
  }
 };

 static NodeState Current_pos;

 static std::map<uint32_t, Ptr<YansWifiPhy> > NodeIdToDeviceMap;
 static  Ipv4InterfaceContainer adhocInterfaces;


 Ptr<Node>
 GetNode (int id)
 {
   //NS_LOG_FUNCTION (this);

   std::pair<Ptr<Ipv4>, uint32_t> interface = adhocInterfaces.Get (id);
   Ptr<Ipv4> pp = interface.first;
   Ptr<Node> node = pp->GetObject<Node> ();

   return node;
 }

 uint32_t port = 80;
 void ReceivePacket(Ptr<Socket> socket) {
 	Ptr<Packet> p;
  Ptr<MobilityModel> mobility =socket->GetNode()->GetObject<MobilityModel>();
  //Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity(); // Get velocity
  int node_speed;
  node_speed = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
  //std::cout << "/* speed */" <<node_speed<< '\n';
 	while ((p = socket->Recv())) {
 		MyTag tag;
 		p->PeekPacketTag(tag);
 		//	  Ptr<MobilityModel> mobility = socket->GetNode()->GetObject<MobilityModel>();
 		//	  Vector pos = mobility->GetPosition();
 		std::cout << Simulator::Now().GetMilliSeconds() << ",R,"
 				<< socket->GetNode()->GetId() << "," << tag.GetSimpleValue()<<","<<node_speed
 				//	  << "," << pos.x << "," <<pos.y << "," <<pos.z
 				<< std::endl;
 	}

 }

uint32_t MsgId = 0;
static int g_safetyDistSq = 10000;
static int g_power = 20;
static Time g_interval;
static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize,
		uint32_t pktCount, Time pktInterval) {

	if (NodeMoving[socket->GetNode()->GetId()] != 0) {
		Ptr<MobilityModel> mobility =
				socket->GetNode()->GetObject<MobilityModel>();
		//Vector pos = mobility->GetPosition (); // Get position
		Vector vel = mobility->GetVelocity(); // Get velocity
		int node_speed;
		node_speed = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
		int priority;
		if (node_speed < (speed_threshold * speed_threshold)) {
			//Ptr<NetDevice> device = socket->GetNode()->GetDevice(0);
			//Ptr<WifiNetDevice> wifiDevice_0 = DynamicCast<WifiNetDevice>(device);
			//Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice_0->GetPhy());
			//phy->SetTxPowerStart(g_power);
			//phy->SetTxPowerEnd(g_power);
			//pktInterval=g_interval;
			priority = 5;
		} else {
			//Ptr<NetDevice> device = socket->GetNode()->GetDevice(0);
			//Ptr<WifiNetDevice> wifiDevice_0 = DynamicCast<WifiNetDevice>(device);
			//Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice_0->GetPhy());
			//phy->SetTxPowerStart(0);
			//phy->SetTxPowerEnd(0);

			priority = 1;

		}
		vector<uint32_t> nodes_in_range;
		//nodes_in_range=Current_pos.NodesInRange(socket->GetNode()->GetId(),100);

		static std::atomic<int> MsgId(0);
		uint32_t lcal = MsgId++;

		Ptr<Packet> pkt = Create<Packet>(pktSize);
		MyTag tag;
		tag.SetSimpleValue(lcal);
		pkt->AddPacketTag(tag);
		std::cout << Simulator::Now().GetMilliSeconds() << ",S,"
				<< socket->GetNode()->GetId() << "," << tag.GetSimpleValue()
				<< ",prioirt" << priority << ",nspeed" <<node_speed<<",sd" <<g_safetyDistSq;
		//Current_pos.PrintDistFrom(socket->GetNode()->GetId());
		Current_pos.PrintNodesInRange(socket->GetNode()->GetId(),
				g_safetyDistSq);
		std::cout << std::endl;
		socket->Send(pkt);
	}
	Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize,
			pktCount - 1, pktInterval);

}

 //// ----- end copy

void CourseChange (std::ostream *os, std::string context, Ptr<const MobilityModel> mobility)
{
	  Vector pos = mobility->GetPosition (); // Get position
	  Vector vel = mobility->GetVelocity (); // Get velocity


	  pos.z = 1.5;

	  unsigned int nodeId = mobility->GetObject<Node> ()->GetId ();
	  Current_pos.SetNodeLocation(nodeId,pos.x, pos.y, pos.z);
	  double t = (Simulator::Now ()).GetSeconds ();
	  if (t >= 1.0)
	    {
		  NodeMoving[nodeId] = 1;
	      //WaveBsmHelper::GetNodesMoving ()[nodeId] = 1;
	    }
	  if (false)
		  std::cout << Simulator::Now () << " node:" << nodeId << " POS: x=" << pos.x << ", y=" << pos.y
	      	  << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
			  << ", z=" << vel.z << std::endl;
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1;
  double interval = 0.1; // seconds
  double hp_interval = 0.1; // seconds
  bool verbose = false;
  double safetyDist=500;
  int p_power=3;
  int num_nodes=10;
  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("hp_interval", "interval (seconds) between packets", hp_interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("safetyDist", "distance for safety range", safetyDist);
  cmd.AddValue ("p_power", "distance for safety range", p_power);
  cmd.AddValue ("num_nodes", "Number of nodes", num_nodes);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);
  g_safetyDistSq=safetyDist*safetyDist;
  g_power=p_power;
  g_interval=Seconds(hp_interval);
  MaxNodes = (uint32_t)num_nodes;

  NodeContainer nodes;
  nodes.Create (num_nodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  //double m_TxGain=5.0;
  //std::cout << "/* txgain */" << m_TxGain<<'\n';

  //wifiPhy.Set ("TxGain", DoubleValue (m_TxGain) );
  //m_TxGain=wifiPhy.GetPowerDbm();
  //wifiPhy.GetTxGain();
  //wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
  //wavePhy.Set ("Frequency", UintegerValue (m_freq));
  //wavePhy.Set ("EnergyDetectionThreshold", DoubleValue (m_sensitivity));
  //wavePhy.Set ("CcaMode1Threshold",DoubleValue(m_cca_sensitivity));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  /*
  wifi80211p.SetRemoteStationManager ("ns3::ParfWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));*/
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, nodes);

  // Tracing
  wifiPhy.EnablePcap ("wave-simple-80211p", devices);

  MobilityHelper mobility;
  int64_t m_streamIndex=0;

  /*
    Avik code
  */
  ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=30.0]"));
    // we need antenna height uniform [1.0 .. 2.0] for loss model
    pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex);
    m_nodeSpeed=20;
    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
    std::cout << ssSpeed.str() << std::endl;
    m_nodePause=0;
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";
    speed_threshold=m_nodeSpeed/2;

    mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue (ssSpeed.str ()),
                                    "Pause", StringValue (ssPause.str ()),
                                    "PositionAllocator", PointerValue (taPositionAlloc));
    mobility.SetPositionAllocator (taPositionAlloc);
    mobility.Install (nodes);
    m_streamIndex += mobility.AssignStreams (nodes, m_streamIndex);
    std::ofstream m_os; ///< output stream
    Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                 MakeBoundCallback (&CourseChange,&(std::cout)));


  /*
    End of Avik code
  */
  InternetStackHelper internet;
  internet.Install (nodes);

  int nSinks=num_nodes;
  double TotalTime = 100.0;
  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  //Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (devices);
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
  rv->SetAttribute ("Min", DoubleValue (0.0));
  rv->SetAttribute ("Max", DoubleValue (0.05));
  for (int i = 0; i < nSinks; i++)
    {
      //Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), nodes.Get (i));
      Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (i), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
      recvSink->Bind (local);
      recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));


      Ptr<Socket> source = Socket::CreateSocket (nodes.Get (i), tid);
      InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
      source->SetAllowBroadcast (true);
      source->Connect (remote);
      //std::cout << "/* in main loop about to enter schedule with context */" << '\n';
      Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                      Seconds (1.0+rv->GetValue()), &GenerateTraffic,
                                      source, packetSize, numPackets, interPacketInterval);
    }

  /*
  End of Avik's code
  */



  Simulator::Stop (Seconds (TotalTime));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
