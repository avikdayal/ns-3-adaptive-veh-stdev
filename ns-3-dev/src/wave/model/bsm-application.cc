/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 North Carolina State University
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
 * Author: Scott E. Carpenter <scarpen@ncsu.edu>
 *
 */

#include "ns3/bsm-application.h"
#include "ns3/log.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/seq-ts-header.h"
#include <atomic>
NS_LOG_COMPONENT_DEFINE ("BsmApplication");

// chnahe the followig values for experiment
#define ADAPTIVE_PRIO

#define TTC_THRESH 7.0
#define TTC_THRESH_UPPER 10.0
#define CRASH_THRESHOLD 2
#define PRIO_MANUEVER_THRESHOLD 4.5

namespace ns3 {

#include "packet_info.inc"
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

  class PacketInfo {
  public:
   PacketInfo();
   virtual ~PacketInfo();
   std::vector<uint32_t> NodesInRange;
   std::map<uint32_t, double> TTCInRange;
   std::map<uint32_t, double> DistInRange;
   void SetLagTime(uint64_t lag_time);
   //void SetSendPriority(uint32_t txpriority);
   //void SetRecvPriority(uint32_t rxpriority);
   void SetSendPriority(double txpriority);
   void SetRecvPriority(double rxpriority);
   void AddNodeInRange(uint32_t nodeId);
   void AddTTCInRange(uint32_t nodeId, double ttc);
   void AddDistInRange(uint32_t nodeId, double dist);
   //void RemoveNodeInRange(uint32_t nodeId);
   uint64_t GetLagTime();
   //uint32_t GetSendPriority();
   //uint32_t GetRecvPriority();
   double GetSendPriority();
   double GetRecvPriority();
   uint32_t m_lag_time;
   uint32_t m_txNodeId;
   uint32_t m_rxNodeId;
   uint32_t m_msgId;
   //uint32_t m_txpriority;
   //uint32_t m_rxpriority;
   double m_txpriority;
   double m_rxpriority;
   double m_txspeed;
   double m_rxspeed;
   double m_ttc;
   double m_dist;
   double m_beac;
   double m_sysTime;
   //float getPDR();
 };


  PacketInfo::PacketInfo() :m_lag_time(0), m_txNodeId(0), m_rxNodeId(0), m_msgId(0), m_txpriority(0), m_rxpriority(0), m_txspeed(0), m_rxspeed(0) {
   // TODO Auto-generated constructor stub

  }

  PacketInfo::~PacketInfo() {
   // TODO Auto-generated destructor stub
  }

  void PacketInfo::SetLagTime(uint64_t lag_time){
    m_lag_time=lag_time;
  }
  void PacketInfo::SetSendPriority(double txpriority){
    m_txpriority=txpriority;
  }
  void PacketInfo::SetRecvPriority(double rxpriority){
    m_rxpriority=rxpriority;
  }
  void PacketInfo::AddNodeInRange(uint32_t nodeId){
 	 NodesInRange.push_back(nodeId);
  }
  void PacketInfo::AddTTCInRange(uint32_t nodeId, double ttc){
 	 //TTCInRange.push_back(ttc);
   TTCInRange[nodeId]=ttc;
  }
  void PacketInfo::AddDistInRange(uint32_t nodeId, double dist){
 	 //TTCInRange.push_back(ttc);
   DistInRange[nodeId]=dist;
  }
  uint64_t PacketInfo::GetLagTime(){
    return m_lag_time;
  }
  /*uint32_t PacketInfo::GetSendPriority(){
    return m_txpriority;
  }
  uint32_t PacketInfo::GetRecvPriority(){
    return m_rxpriority;
  }*/
  double PacketInfo::GetSendPriority(){
    return m_txpriority;
  }
  double PacketInfo::GetRecvPriority(){
    return m_rxpriority;
  }

std::vector <PacketInfo *> PacketList;
//std::vector <PacketInfo> hp_PacketList;
std::map <uint32_t, PacketInfo *> hp_PacketList;
std::map <uint32_t, PacketInfo *> hp_missedPacketList;
double m_lane_thres=2.0;
double lane_ypos[3] = { 2, 6, 10 };
//const int m_lane_num = 3;
double highwaylength=ROAD_LENGTH_NUM;
//int crashes=0;
static std::atomic<int> crashes(0);
static std::atomic<int> manuevers(0);
//static std::atomic<int> MsgId(0);
// (Arbitrary) port for establishing socket to transmit WAVE BSMs
int BsmApplication::wavePort = 9080;
std::string m_CSVfileName3="transmit_output.csv";

NS_OBJECT_ENSURE_REGISTERED (BsmApplication);

TypeId
BsmApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BsmApplication")
    .SetParent<Application> ()
    .SetGroupName ("Wave")
    .AddConstructor<BsmApplication> ()
    ;
  return tid;
}

BsmApplication::BsmApplication ()
  : m_waveBsmStats (0),
    m_txSafetyRangesSq (),
    m_TotalSimTime (Seconds (10)),
    m_wavePacketSize (200),
    m_numWavePackets (1),
    m_waveInterval (MilliSeconds (100)),
    m_gpsAccuracyNs (10000),
    m_adhocTxInterfaces (0),
    m_nodesMoving (0),
    m_unirv (0),
    m_nodeId (0),
    m_chAccessMode (0),
    m_txMaxDelay (MilliSeconds (50)),
    m_prevTxDelay (MilliSeconds (0)),
    m_prioritytag (1)
    //m_CSVfileName3("transmit_output.csv")
{
  NS_LOG_FUNCTION (this);
}

BsmApplication::~BsmApplication ()
{
  NS_LOG_FUNCTION (this);
}

void
BsmApplication::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  // chain up
  Application::DoDispose ();
}

// Application Methods
void BsmApplication::StartApplication () // Called at time specified by Start
{
  NS_LOG_FUNCTION (this);
  std::cout << "inside of BSM start application" << '\n';
  // setup generation of WAVE BSM messages
  Time waveInterPacketInterval = m_waveInterval;

  // BSMs are not transmitted for the first second
  Time startTime = Seconds (1.0);
  // total length of time transmitting WAVE packets
  Time totalTxTime = m_TotalSimTime - startTime;
  // total WAVE packets needing to be sent
  m_numWavePackets = (uint32_t) (totalTxTime.GetDouble () / m_waveInterval.GetDouble ());
  m_numWavePackets=m_numWavePackets*2;

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // every node broadcasts WAVE BSM to potentially all other nodes
  Ptr<Socket> recvSink = Socket::CreateSocket (GetNode (m_nodeId), tid);
  //recvSink->SetRecvCallback (MakeCallback (&BsmApplication::ReceiveWavePacket, this));
  recvSink->SetRecvCallback (MakeCallback (&BsmApplication::ReceiveAdaptiveWavePacket, this));
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), wavePort);
  recvSink->BindToNetDevice (GetNetDevice (m_nodeId));
  recvSink->Bind (local);
  recvSink->SetAllowBroadcast (true);

  // dest is broadcast address
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), wavePort);
  recvSink->Connect (remote);

  // Transmission start time for each BSM:
  // We assume that the start transmission time
  // for the first packet will be on a ns-3 time
  // "Second" boundary - e.g., 1.0 s.
  // However, the actual transmit time must reflect
  // additional effects of 1) clock drift and
  // 2) transmit delay requirements.
  // 1) Clock drift - clocks are not perfectly
  // synchronized across all nodes.  In a VANET
  // we assume all nodes sync to GPS time, which
  // itself is assumed  accurate to, say, 40-100 ns.
  // Thus, the start transmission time must be adjusted
  // by some value, t_drift.
  // 2) Transmit delay requirements - The US
  // minimum performance requirements for V2V
  // BSM transmission expect a random delay of
  // +/- 5 ms, to avoid simultanous transmissions
  // by all vehicles congesting the channel.  Thus,
  // we need to adjust the start trasmission time by
  // some value, t_tx_delay.
  // Therefore, the actual transmit time should be:
  // t_start = t_time + t_drift + t_tx_delay
  // t_drift is always added to t_time.
  // t_tx_delay is supposed to be +/- 5ms, but if we
  // allow negative numbers the time could drift to a value
  // BEFORE the interval start time (i.e., at 100 ms
  // boundaries, we do not want to drift into the
  // previous interval, such as at 95 ms.  Instead,
  // we always want to be at the 100 ms interval boundary,
  // plus [0..10] ms tx delay.
  // Thus, the average t_tx_delay will be
  // within the desired range of [0..10] ms of
  // (t_time + t_drift)

  // WAVE devices sync to GPS time
  // and all devices would like to begin broadcasting
  // their safety messages immediately at the start of
  // the CCH interval.  However, if all do so, then
  // significant collisions occur.  Thus, we assume there
  // is some GPS sync accuracy on GPS devices,
  // typically 40-100 ns.
  // Get a uniformly random number for GPS sync accuracy, in ns.
  Time tDrift = NanoSeconds (m_unirv->GetInteger (0, m_gpsAccuracyNs));

  // When transmitting at a default rate of 10 Hz,
  // the subsystem shall transmit every 100 ms +/-
  // a random value between 0 and 5 ms. [MPR-BSMTX-TXTIM-002]
  // Source: CAMP Vehicle Safety Communications 4 Consortium
  // On-board Minimum Performance Requirements
  // for V2V Safety Systems Version 1.0, December 17, 2014
  // max transmit delay (default 10ms)
  // get value for transmit delay, as number of ns
  uint32_t d_ns = static_cast<uint32_t> (m_txMaxDelay.GetInteger ());
  // convert random tx delay to ns-3 time
  // see note above regarding centering tx delay
  // offset by 5ms + a random value.
  Time txDelay = NanoSeconds (m_unirv->GetInteger (0, d_ns));
  m_prevTxDelay = txDelay;

  Time txTime = startTime + tDrift + txDelay;
  // schedule transmission of first packet
  /*
  Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                  txTime, &BsmApplication::GenerateWaveTraffic, this,
                                  recvSink, m_wavePacketSize, m_numWavePackets, waveInterPacketInterval, m_nodeId);
                                  */
  //std::cout << "priority tag: " << m_prioritytag << '\n';
  if(m_prioritytag){
    //std::cout << "priority tag: " << m_prioritytag << "in generate prioirty"<< '\n';
    //std::string CSVfileName4 =m_CSVfileName3+"_txmsgStats.csv";
    //std::ofstream out2 (CSVfileName4.c_str ());
    std::string CSVfileName4 =m_CSVfileName3+"_txmsgStats.csv";
    std::ofstream out2;
    out2.open(CSVfileName4.c_str());
    out2 << "Msg ID" << "," << "TxNode"<< "," << "RxNode" << "," << "TxPriority" << "," << "RxPriority" << "," << "Delay" << "," << "ttc" << "," << "dist" <<"," << "tx_vel" << "," << "rx_vel" << "," << "Sys_time" << std::endl;
    out2.close();
    /*Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                    txTime, &BsmApplication::GeneratePriorityWaveTraffic, this,
                                    recvSink, m_wavePacketSize, m_numWavePackets, waveInterPacketInterval, m_nodeId);*/
    Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                    txTime, &BsmApplication::GeneratePriorityWaveTraffic, this,
                                    recvSink, m_wavePacketSize, totalTxTime.GetSeconds(), waveInterPacketInterval, m_nodeId);
    //out2.close();
  }
  else{
    //std::cout << "priority tag: " << m_prioritytag << "in generate wave"<< '\n';
    std::string CSVfileName4 =m_CSVfileName3+"_txmsgStats.csv";
    std::ofstream out2;
    out2.open(CSVfileName4.c_str());
    out2 << "Msg ID" << "," << "TxNode"<< "," << "RxNode" << "," << "TxPriority" << "," << "RxPriority" << "," << "Delay" << "," << "ttc" << "," << "dist" <<"," << "tx_vel" << "," << "rx_vel" << "," << "Sys_time" << std::endl;
    out2.close();
    Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                    txTime, &BsmApplication::GenerateWaveTraffic, this,
                                    recvSink, m_wavePacketSize, m_numWavePackets, waveInterPacketInterval, m_nodeId);
  }

}

void BsmApplication::StopApplication () // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);
}

void
BsmApplication::Setup (Ipv4InterfaceContainer & i,
                       int nodeId,
                       Time totalTime,
                       uint32_t wavePacketSize, // bytes
                       Time waveInterval,
                       double gpsAccuracyNs,
                       std::vector <double> rangesSq,           // m ^2
                       Ptr<WaveBsmStats> waveBsmStats,
                       Ptr<WaveBsmStats> waveBsmStats_hp,
                       std::vector<int> * nodesMoving,
                       int chAccessMode,
                       Time txMaxDelay)
{
  NS_LOG_FUNCTION (this);

  m_unirv = CreateObject<UniformRandomVariable> ();

  m_TotalSimTime = totalTime;
  m_wavePacketSize = wavePacketSize;
  m_waveInterval = waveInterval;
  m_gpsAccuracyNs = gpsAccuracyNs;
  int size = rangesSq.size ();
  m_waveBsmStats = waveBsmStats;
  m_waveBsmStats_hp = waveBsmStats_hp;
  m_nodesMoving = nodesMoving;
  m_chAccessMode = chAccessMode;
  m_txSafetyRangesSq.clear ();
  m_txSafetyRangesSq.resize (size, 0);

  for (int index = 0; index < size; index++)
    {
      // stored as square of value, for optimization
      m_txSafetyRangesSq[index] = rangesSq[index];
    }

  m_adhocTxInterfaces = &i;
  m_nodeId = nodeId;
  m_txMaxDelay = txMaxDelay;
}

void
BsmApplication::GenerateWaveTraffic (Ptr<Socket> socket, uint32_t pktSize,
		uint32_t pktCount, Time pktInterval,
		uint32_t sendingNodeId)
{
	NS_LOG_FUNCTION (this);

	if (pktCount > 0)
	{
		std::string CSVfileName4 =m_CSVfileName3+"_txmsgStats.csv";
		std::ofstream out2;
		out2.open(CSVfileName4.c_str(), std::ios::app);

		// for now, we cannot tell if each node has
		// started mobility.  so, as an optimization
		// only send if  this node is moving
		// if not, then skip
		int txNodeId = sendingNodeId;
		Ptr<Node> txNode = GetNode (txNodeId);
		Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
		NS_ASSERT (txPosition != 0);
		uint32_t temp_MsgID=0;
		double priority;

		int senderMoving = m_nodesMoving->at (txNodeId);
		if (senderMoving != 0)
		{

			priority=GetAdaptivePriorityLevel(txNodeId);

			static std::atomic<int> MsgId(0);
			uint32_t lcal = MsgId++;
			temp_MsgID=lcal;
			Ptr<Packet> pkt = Create<Packet>(pktSize);
			MyTag tag;
			tag.SetMsgId(lcal);
			tag.SetNodeId(txNodeId);
			tag.SetPrio(priority); //--> change to calculate prio   -pktInterval-m_prevTxDelay
			tag.SetEmitTime(Simulator::Now().GetMilliSeconds()-pktInterval.GetMilliSeconds()-m_prevTxDelay.GetMilliSeconds());
			pkt->AddPacketTag(tag);

			socket->Send (pkt);
			PacketInfo * Pi = new PacketInfo();
			hp_PacketList[lcal] = Pi;
			Pi->m_txNodeId= txNodeId;
			Pi->m_msgId=lcal;
			Pi->m_sysTime=Simulator::Now().GetMilliSeconds()-pktInterval.GetMilliSeconds()-m_prevTxDelay.GetMilliSeconds();
			if(priority>=5){
				m_waveBsmStats_hp->IncTxPktCount ();
				m_waveBsmStats_hp->IncTxByteCount (pktSize);
				int wavePktsSent = m_waveBsmStats_hp->GetTxPktCount ();
				if ((m_waveBsmStats_hp->GetLogging () != 0) && ((wavePktsSent % 1000) == 0))
				{
					NS_LOG_UNCOND ("Sending HP WAVE pkt # " << wavePktsSent );
				}

			}
			else{
				m_waveBsmStats->IncTxPktCount ();
				m_waveBsmStats->IncTxByteCount (pktSize);
				int wavePktsSent = m_waveBsmStats->GetTxPktCount ();
				if ((m_waveBsmStats->GetLogging () != 0) && ((wavePktsSent % 1000) == 0))
				{
					NS_LOG_UNCOND ("Sending WAVE pkt # " << wavePktsSent );
				}
			}


			// find other nodes within range that would be
			// expected to receive this broadbast
			int nRxNodes = m_adhocTxInterfaces->GetN ();
			for (int i = 0; i < nRxNodes; i++)
			{
				Ptr<Node> rxNode = GetNode (i);
				int rxNodeId = rxNode->GetId ();

				if (rxNodeId != txNodeId)
				{
					Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
					NS_ASSERT (rxPosition != 0);

					// confirm that the receiving node
					// has also started moving in the scenario
					// if it has not started moving, then
					// it is not a candidate to receive a packet
					int receiverMoving = m_nodesMoving->at (rxNodeId);
					if (receiverMoving == 1)
					{
						double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);

						double priority_rx=GetAdaptivePriorityLevel(txNodeId,rxNodeId);
						double ttc=GetTTC(txNodeId,rxNodeId);
						if(priority_rx>=5.0){
							Pi->AddNodeInRange(rxNodeId);
							Pi->AddTTCInRange(rxNodeId, ttc);
							Pi->AddDistInRange(rxNodeId, std::sqrt(distSq));
							out2 << temp_MsgID << "," << txNodeId<< "," << rxNodeId << ","<< priority << "," << distSq << Simulator::Now().GetMilliSeconds() <<std::endl;
						}


						if (distSq > 0.0)
						{
							// dest node within range?
							//todo: int base = (prio == 1) ? 0 : 10;
							int min_interval=100;
							int rangeCount = m_txSafetyRangesSq.size ();
							for (int index = 1; index <= rangeCount; index++)
							{
								if (distSq <= m_txSafetyRangesSq[index - 1])
								{
									// we should expect dest node to receive broadcast pkta
									// todo: index + base
									if(priority>=5){
										m_waveBsmStats_hp->IncExpectedRxPktCount (index);
										//pktInterval=MilliSeconds (50);
										pktInterval=MilliSeconds (min_interval);
										//pktInterval=MilliSeconds (100);//control code
									}
									else if(priority>1){
										//m_waveBsmStats_hp->IncExpectedRxPktCount (index);
										m_waveBsmStats->IncExpectedRxPktCount (index);
										//int pkt_time;
										//pkt_time=max_interval+(max_interval - min_interval)/(max_priority - min_priority) -priority* (max_interval - min_interval)/(max_priority- min_priority);
										pktInterval=MilliSeconds (100);
									}
									else{
										pktInterval=MilliSeconds (100);//control code
									}
								}
							}
						}
					}
				}
			}
		}
		out2.close();
		// every BSM must be scheduled with a tx time delay
		// of +/- (5) ms.  See comments in StartApplication().
		// we handle this as a tx delay of [0..10] ms
		// from the start of the pktInterval boundary
		uint32_t d_ns = static_cast<uint32_t> (m_txMaxDelay.GetInteger ());
		Time txDelay = NanoSeconds (m_unirv->GetInteger (0, d_ns));

		// do not want the tx delay to be cumulative, so
		// deduct the previous delay value.  thus we adjust
		// to schedule the next event at the next pktInterval,
		// plus some new [0..10] ms tx delay
		Time txTime = pktInterval - m_prevTxDelay + txDelay;
		m_prevTxDelay = txDelay;
		Simulator::ScheduleWithContext (socket->GetNode ()->GetId (),
				txTime, &BsmApplication::GeneratePriorityWaveTraffic, this,
				socket, pktSize, pktCount - 1, pktInterval,  socket->GetNode ()->GetId ());
	}
	else
	{
		std::cout << "/* Simulation Finished at time */" << (Simulator::Now ()).GetSeconds () << std::endl;
		socket->Close ();
	}
}


void
BsmApplication::GeneratePriorityWaveTraffic (Ptr<Socket> socket, uint32_t pktSize,
		uint32_t EndTime, Time pktInterval,
		uint32_t sendingNodeId)
{
	NS_LOG_FUNCTION (this);

	int min_interval=100;
	int loop_min_interval = min_interval;

#ifdef ADAPTIVE_PRIO
	int max_interval=1000;
	int max_priority=5;
	int min_priority=1;
	loop_min_interval = max_interval;
#endif

	if (EndTime > Simulator::Now().GetSeconds())
	{
		std::string CSVfileName4 =m_CSVfileName3+"_txmsgStats.csv";
		std::ofstream out2;
		out2.open(CSVfileName4.c_str(), std::ios::app);

		// for now, we cannot tell if each node has
		// started mobility.  so, as an optimization
		// only send if  this node is moving
		// if not, then skip
		int txNodeId = sendingNodeId;
		Ptr<Node> txNode = GetNode (txNodeId);
		Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
		NS_ASSERT (txPosition != 0);
		uint32_t temp_MsgID=0;
		double priority;

		int senderMoving = m_nodesMoving->at (txNodeId);
		if (senderMoving != 0)
		{

			priority=GetAdaptivePriorityLevel(txNodeId);
			static std::atomic<int> MsgId(0);
			uint32_t lcal = MsgId++;
			temp_MsgID=lcal;
			Ptr<Packet> pkt = Create<Packet>(pktSize);
			MyTag tag;
			tag.SetMsgId(lcal);
			tag.SetNodeId(txNodeId);
			tag.SetPrio(priority); //--> change to calculate prio   -pktInterval-m_prevTxDelay
			tag.SetEmitTime(Simulator::Now().GetMilliSeconds()-pktInterval.GetMilliSeconds()-m_prevTxDelay.GetMilliSeconds());
			pkt->AddPacketTag(tag);
			std::cout << "logx_sending " << Simulator::Now().GetMilliSeconds() << " txNode: " << txNodeId << " MsgId: " << temp_MsgID << std::endl;
			socket->Send (pkt);
			PacketInfo * Pi = new PacketInfo();
			hp_PacketList[lcal] = Pi;
			Pi->m_txNodeId= txNodeId;
			Pi->m_msgId=lcal;
			Pi->m_sysTime=Simulator::Now().GetMilliSeconds()-pktInterval.GetMilliSeconds()-m_prevTxDelay.GetMilliSeconds();
			if(priority>=5){
				m_waveBsmStats_hp->IncTxPktCount ();
				m_waveBsmStats_hp->IncTxByteCount (pktSize);
				int wavePktsSent = m_waveBsmStats_hp->GetTxPktCount ();
				if ((m_waveBsmStats_hp->GetLogging () != 0) && ((wavePktsSent % 1000) == 0))
				{
					NS_LOG_UNCOND ("Sending HP WAVE pkt # " << wavePktsSent );
				}

			}
			else{
				m_waveBsmStats->IncTxPktCount ();
				m_waveBsmStats->IncTxByteCount (pktSize);
				int wavePktsSent = m_waveBsmStats->GetTxPktCount ();
				if ((m_waveBsmStats->GetLogging () != 0) && ((wavePktsSent % 1000) == 0))
				{
					NS_LOG_UNCOND ("Sending WAVE pkt # " << wavePktsSent );
				}
			}


			// find other nodes within range that would be
			// expected to receive this broadbast
			int nRxNodes = m_adhocTxInterfaces->GetN ();
			for (int i = 0; i < nRxNodes; i++)
			{
				Ptr<Node> rxNode = GetNode (i);
				int rxNodeId = rxNode->GetId ();

				if (rxNodeId != txNodeId)
				{
					Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
					NS_ASSERT (rxPosition != 0);

					// confirm that the receiving node
					// has also started moving in the scenario
					// if it has not started moving, then
					// it is not a candidate to receive a packet
					int receiverMoving = m_nodesMoving->at (rxNodeId);
					if (receiverMoving == 1)
					{
						double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);

						double priority_rx=GetAdaptivePriorityLevel(txNodeId,rxNodeId);
						double ttc=GetTTC(txNodeId,rxNodeId);
						if(priority_rx>=5.0){
							Pi->AddNodeInRange(rxNodeId);
							Pi->AddTTCInRange(rxNodeId, ttc);
							Pi->AddDistInRange(rxNodeId, std::sqrt(distSq));
							out2 << temp_MsgID << "," << txNodeId<< "," << rxNodeId << ","<< priority << "," << distSq <<std::endl;
						}


						if (distSq > 0.0)
						{
							// dest node within range?
							//todo: int base = (prio == 1) ? 0 : 10;
							int pkt_time;
							int rangeCount = m_txSafetyRangesSq.size ();
							for (int index = 1; index <= rangeCount; index++)
							{
								if (distSq <= m_txSafetyRangesSq[index - 1])
								{
									// we should expect dest node to receive broadcast pkta
									// todo: index + base
									if(priority>=5){
										m_waveBsmStats_hp->IncExpectedRxPktCount (index);
										pkt_time = min_interval;
										//pktInterval=MilliSeconds (min_interval);
									}
									else if(priority>1){
										m_waveBsmStats->IncExpectedRxPktCount (index);
#ifdef ADAPTIVE_PRIO

										pkt_time=max_interval+(max_interval - min_interval)/(max_priority - min_priority) -priority* (max_interval - min_interval)/(max_priority- min_priority);
										//pktInterval=MilliSeconds (pkt_time);
#else
										//pktInterval=MilliSeconds (min_interval);
										pkt_time = min_interval;
#endif
									}
									else{
										m_waveBsmStats->IncExpectedRxPktCount (index);
#ifdef ADAPTIVE_PRIO
										//pktInterval=MilliSeconds (max_interval);
										pkt_time = max_interval;
#else
										//pktInterval=MilliSeconds (100);//control code
										pkt_time = min_interval;
#endif
									}
									loop_min_interval = (loop_min_interval < pkt_time) ? loop_min_interval: pkt_time;
								}
							} // end for

						}
					}
				}
			}
		}
		out2.close();

		pktInterval=MilliSeconds (loop_min_interval);
		//std::cout << "Packet Interval: " << pktInterval << '\n';
		// every BSM must be scheduled with a tx time delay
		// of +/- (5) ms.  See comments in StartApplication().
		// we handle this as a tx delay of [0..10] ms
		// from the start of the pktInterval boundary
		uint32_t d_ns = static_cast<uint32_t> (m_txMaxDelay.GetInteger ());
		Time txDelay = NanoSeconds (m_unirv->GetInteger (0, d_ns));

		// do not want the tx delay to be cumulative, so
		// deduct the previous delay value.  thus we adjust
		// to schedule the next event at the next pktInterval,
		// plus some new [0..10] ms tx delay
		Time txTime = pktInterval - m_prevTxDelay + txDelay;
		m_prevTxDelay = txDelay;
		Simulator::ScheduleWithContext (socket->GetNode ()->GetId (),
				txTime, &BsmApplication::GeneratePriorityWaveTraffic, this,
				socket, pktSize, EndTime , pktInterval,  socket->GetNode ()->GetId ());
	}
	else
	{
		std::cout << "/* Simulation Finished at time */" << (Simulator::Now ()).GetSeconds () << std::endl;
		socket->Close ();
	}
}

int BsmApplication::GetPriorityLevel(int sendingNodeId)
{
  int txNodeId = sendingNodeId;
  Ptr<Node> txNode = GetNode (txNodeId);
  Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
  NS_ASSERT (txPosition != 0);
  Vector vel = txPosition->GetVelocity(); // Get velocity
  double node_speed;
  node_speed = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
  int priority=1;
  int senderMoving = m_nodesMoving->at (txNodeId);
  if (senderMoving != 0)
    {
      int nRxNodes = m_adhocTxInterfaces->GetN ();
      for (int i = 0; i < nRxNodes; i++)
        {
          Ptr<Node> rxNode = GetNode (i);
          int rxNodeId = rxNode->GetId ();

          if (rxNodeId != txNodeId)
            {
              Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
              NS_ASSERT (rxPosition != 0);
              Vector vel_rx = rxPosition->GetVelocity(); // Get velocity
              double node_speed_rx;
              node_speed_rx = vel_rx.x * vel_rx.x + vel_rx.y * vel_rx.y + vel_rx.z * vel_rx.z;
              double rel_vel=(double)(std::abs(node_speed-node_speed_rx));
              double ttc;//time to collision
              double ttc_thres;
              //double ttc_thres_upper;
              ttc_thres=TTC_THRESH;
              //ttc_thres_upper=8.0;
              // confirm that the receiving node
              // has also started moving in the scenario
              // if it has not started moving, then
              // it is not a candidate to receive a packet
              int receiverMoving = m_nodesMoving->at (rxNodeId);
              if (receiverMoving == 1)
                {
                  double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);
                  double ttc_2=std::sqrt(distSq/rel_vel);
                  ttc=GetTTC(txNodeId,rxNodeId);
                  if(ttc_2!=ttc){
                    //std::cout << "PROBLEM in GPL1: " << " ttc original: "<< ttc_2<< " ttc new: "<< ttc << std::endl;
                  }
                  //ttc=(distSq/rel_vel);
                  //std::cout << "time to collision" << ttc << '\n';
                  if (distSq > 0.0)
                    {
                      int rangeCount = m_txSafetyRangesSq.size ();
                      for (int index = 1; index <= rangeCount; index++)
                        {
                          if (distSq <= m_txSafetyRangesSq[index - 1])
                            {
                              if (ttc < ttc_thres)
                              {
                                //std::cout << "priority 5 ttc: " << ttc << std::endl;
                                priority = 5;
                                return priority;
                              }
                              else{
                                priority=1;
                              }
                            }
                        }
                    }
                }
            }
        }
    }
    return priority;

}
int BsmApplication::GetPriorityLevel(int sendingNodeId, int rxNodeId)
{
  int txNodeId = sendingNodeId;
  Ptr<Node> txNode = GetNode (txNodeId);
  Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
  NS_ASSERT (txPosition != 0);
  Vector vel = txPosition->GetVelocity(); // Get velocity
  double node_speed;
  node_speed = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
  double priority=1;
  int senderMoving = m_nodesMoving->at (txNodeId);
  if (senderMoving != 0)
    {
      //int nRxNodes = m_adhocTxInterfaces->GetN ();
          Ptr<Node> rxNode = GetNode (rxNodeId);

          if (rxNodeId != txNodeId)
            {
              Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
              NS_ASSERT (rxPosition != 0);
              Vector vel_rx = rxPosition->GetVelocity(); // Get velocity
              double node_speed_rx;
              node_speed_rx = vel_rx.x * vel_rx.x + vel_rx.y * vel_rx.y + vel_rx.z * vel_rx.z;
              double rel_vel=(std::abs(node_speed-node_speed_rx));
              double ttc;//time to collision
              double ttc_thres;
              ttc_thres=TTC_THRESH;
              // confirm that the receiving node
              // has also started moving in the scenario
              // if it has not started moving, then
              // it is not a candidate to receive a packet
              int receiverMoving = m_nodesMoving->at (rxNodeId);
              if (receiverMoving == 1)
                {
                  double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);
                  double ttc_2=std::sqrt(distSq/rel_vel);
                  ttc=GetTTC(txNodeId,rxNodeId);
                  if(ttc_2!=ttc){
                    //std::cout << "PROBLEM in GPL2: " << " ttc original: "<< ttc_2<< " ttc new: "<< ttc << std::endl;
                  }
                  //ttc=(distSq/rel_vel);
                  if (distSq > 0.0)
                    {
                      int rangeCount = m_txSafetyRangesSq.size ();
                      for (int index = 1; index <= rangeCount; index++)
                        {
                          if (distSq <= m_txSafetyRangesSq[index - 1])
                            {
                              if (ttc < ttc_thres)
                              {
                                //std::cout << "ttc: " << ttc << std::endl;
                                priority = 5;
                                return priority;
                              }
                              else{
                                priority=1;
                              }
                            }
                        }
                }
            }
        }
    }
    return priority;

}
double BsmApplication::GetAdaptivePriorityLevel(int sendingNodeId)
{
	int txNodeId = sendingNodeId;
	Ptr<Node> txNode = GetNode (txNodeId);
	Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
	NS_ASSERT (txPosition != 0);
	double priority=1;
	double max_priority=5;
	double min_priority=1;
	int senderMoving = m_nodesMoving->at (txNodeId);
	if (senderMoving == 0)
		return priority;

	int nRxNodes = m_adhocTxInterfaces->GetN ();
	for (int i = 0; i < nRxNodes; i++)
	{
		Ptr<Node> rxNode = GetNode (i);
		int rxNodeId = rxNode->GetId ();

		if (rxNodeId == txNodeId) {
			continue;
		}

		Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
		NS_ASSERT (rxPosition != 0);
		double ttc;//time to collision
		double ttc_thres;
		double ttc_thres_upper;
		ttc_thres=TTC_THRESH;
		ttc_thres_upper=TTC_THRESH_UPPER;
		// confirm that the receiving node
		// has also started moving in the scenario
		// if it has not started moving, then
		// it is not a candidate to receive a packet
		int receiverMoving = m_nodesMoving->at (rxNodeId);
		if (receiverMoving != 1)
		{
			continue;
		}
		double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);

		ttc=GetTTC(txNodeId,rxNodeId);

		Ptr<ConstantVelocityMobilityModel> mob_tx = txNode-> GetObject<ConstantVelocityMobilityModel>();
		Ptr<ConstantVelocityMobilityModel> mob_rx = rxNode-> GetObject<ConstantVelocityMobilityModel>();
		NS_ASSERT(mob_tx);
		NS_ASSERT(mob_rx);
		Vector posm_rx = mob_rx->GetPosition (); // Get position
		Vector posm_tx = mob_tx->GetPosition (); // Get position

		if (distSq > 0.0)
		{
			if ((ttc < ttc_thres) && (std::abs(posm_rx.y-posm_tx.y)<m_lane_thres)){
				priority = 5;
				return priority;
			}
			else if(ttc<ttc_thres_upper && (std::abs(posm_rx.y-posm_tx.y)<m_lane_thres)){
				double c= min_priority+ttc_thres_upper*(max_priority - min_priority)/(ttc_thres_upper - ttc_thres);
				double p=  c- (max_priority - min_priority)/(ttc_thres_upper - ttc_thres)*ttc;
				if (p > priority)
					priority = p;
			}
		}
	}
	return priority;
}

double BsmApplication::GetAdaptivePriorityLevel(int sendingNodeId, int rxNodeId)
{
  int txNodeId = sendingNodeId;
  Ptr<Node> txNode = GetNode (txNodeId);
  Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
  NS_ASSERT (txPosition != 0);
  Vector vel = txPosition->GetVelocity(); // Get velocity
  double node_speed;
  node_speed = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
  double priority=1;
  double max_priority=5;
  double min_priority=1;
  int senderMoving = m_nodesMoving->at (txNodeId);
  if (senderMoving != 0)
  {
    Ptr<Node> rxNode = GetNode (rxNodeId);

    if (rxNodeId != txNodeId)
    {
      Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
      NS_ASSERT (rxPosition != 0);
      Vector vel_rx = rxPosition->GetVelocity(); // Get velocity
      double node_speed_rx;
      node_speed_rx = vel_rx.x * vel_rx.x + vel_rx.y * vel_rx.y + vel_rx.z * vel_rx.z;
      double rel_vel=(double)(std::abs(node_speed-node_speed_rx));
      double ttc;//time to collision
      double ttc_thres;
      double ttc_thres_upper;
      ttc_thres=TTC_THRESH;
      ttc_thres_upper=TTC_THRESH_UPPER;
      // confirm that the receiving node
      // has also started moving in the scenario
      // if it has not started moving, then
      // it is not a candidate to receive a packet
      int receiverMoving = m_nodesMoving->at (rxNodeId);
      if (receiverMoving == 1)
      {
        double distSq = MobilityHelper::GetDistanceSquaredBetween (txNode, rxNode);
        double ttc_2=std::sqrt(distSq/rel_vel);
        ttc=GetTTC(txNodeId,rxNodeId);
        Ptr<ConstantVelocityMobilityModel> mob_tx = txNode-> GetObject<ConstantVelocityMobilityModel>();
        Ptr<ConstantVelocityMobilityModel> mob_rx = rxNode-> GetObject<ConstantVelocityMobilityModel>();
        NS_ASSERT(mob_tx);
        NS_ASSERT(mob_rx);
        Vector posm_rx = mob_rx->GetPosition (); // Get position
        Vector posm_tx = mob_tx->GetPosition (); // Get position
        if(ttc_2!=ttc){
          //std::cout << "PROBLEM in AGPL2 " << " ttc_original: "<< ttc_2<< " ttc_new: "<< ttc << " distsq_new: "<< distSq << " relvel_new: "<< rel_vel << " txnodeID: "<< txNodeId <<" rxnodeID: "<< rxNodeId << " rxnode_pos: "<< posm_rx << " txnode_pos: "<< posm_tx << " txnode_vel: "<< vel << " rxnode_vel:"<< vel_rx << std::endl;
        }
        //ttc=(distSq/rel_vel);
        if (distSq > 0.0)
        {
          if ((ttc < ttc_thres) && (std::abs(posm_rx.y-posm_tx.y)<m_lane_thres)){
            //std::cout << "ttcHP : " << ttc << std::endl;
            //std::cout << "priority 5 ttc: " << ttc << std::endl;
            priority = 5;
            return priority;
          }
          else if(ttc<ttc_thres_upper && (std::abs(posm_rx.y-posm_tx.y)<m_lane_thres)){
            //std::cout << "mid priority ttc: " << ttc << std::endl;
            double c= min_priority+ttc_thres_upper*(max_priority - min_priority)/(ttc_thres_upper - ttc_thres);
            double p=  c- (max_priority - min_priority)/(ttc_thres_upper - ttc_thres)*ttc;
            if (p > priority)
              priority = p;
            //std::cout << "Adaptive priority "<< priority <<" c constant: " << c << " other constant" << (max_priority - min_priority)/(ttc_thres_upper - ttc_thres) << " max priority " << max_priority << " ttc_thres: "  << ttc_thres << " ttc_thres_upper: " << ttc_thres_upper << " ttc: " << ttc <<'\n';
          }
        }
      }
    }
  }
  return priority;
}


double BsmApplication::GetTTC(int sendingNodeId, int rxNodeId)
{
	int txNodeId = sendingNodeId;
	Ptr<Node> txNode = GetNode (txNodeId);
	Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
	NS_ASSERT (txPosition != 0);
	int senderMoving = m_nodesMoving->at (txNodeId);
	double ttc=1000; //time to collision
	if (rxNodeId == txNodeId || senderMoving ==0) {
		return ttc;
	}

	Vector posm_tx = txPosition->GetPosition(); // Get velocity
	Vector vel = txPosition->GetVelocity(); // Get velocity
	Ptr<Node> rxNode = GetNode (rxNodeId);

	Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
	NS_ASSERT (rxPosition != 0);
	Vector vel_rx = rxPosition->GetVelocity(); // Get velocity
	Vector posm_rx = rxPosition->GetPosition(); // Get position
	double rel_vel=(vel_rx.x-vel.x);
	if (rel_vel ==0)
		return ttc;
	int receiverMoving = m_nodesMoving->at (rxNodeId);
	if (receiverMoving == 1)
	{
		double distSq = (posm_rx.x-posm_tx.x);
		if(std::abs(posm_rx.y-posm_tx.y)<2){
			if (distSq>0){
				ttc=  distSq/rel_vel;
			}
			else{
				ttc=  -distSq/rel_vel;
			}
		}
		if (ttc < 0){
			ttc = 1000;
		}

	}

	return ttc;
}

void BsmApplication::ReceiveWavePacket (Ptr<Socket> socket)
{
	NS_LOG_FUNCTION (this);

	Ptr<Packet> packet;
	Address senderAddr;
	while ((packet = socket->RecvFrom (senderAddr)))
	{
		MyTag tag;
		packet->PeekPacketTag(tag);
		Ptr<Node> rxNode = socket->GetNode ();

		if (InetSocketAddress::IsMatchingType (senderAddr))
		{
			InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddr);
			int nodes = m_adhocTxInterfaces->GetN ();
			for (int i = 0; i < nodes; i++)
			{
				if (addr.GetIpv4 () == m_adhocTxInterfaces->GetAddress (i) )
				{
					Ptr<Node> txNode = GetNode (i);
					int txNodeId = txNode->GetId ();

					int priority_hp;
					priority_hp=GetPriorityLevel(txNodeId,rxNode->GetId ());

					int receiverMoving = m_nodesMoving->at (rxNode->GetId ());
					if (receiverMoving == 1){
						PacketInfo * pinfo = new PacketInfo();
						pinfo->SetLagTime(Simulator::Now().GetMilliSeconds());//-tag.GetEmitTime()
						pinfo->SetSendPriority(tag.GetPrio());
						pinfo->SetRecvPriority(priority_hp);
						pinfo->m_msgId=tag.GetMsgId();
						pinfo->m_txNodeId=txNodeId;
						pinfo->m_rxNodeId=rxNode->GetId ();
						PacketList.push_back(pinfo);
					}


					if(m_prioritytag){
						HandlePriorityReceivedBsmPacket (txNode, rxNode, txNodeId, rxNode->GetId (), tag.GetPrio());
					}
					else{
						HandleReceivedBsmPacket (txNode, rxNode);
					}

				}
			}
		}
	}
}


void BsmApplication::ReceiveAdaptiveWavePacket (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this);

  Ptr<Packet> packet;
  Address senderAddr;
  while ((packet = socket->RecvFrom (senderAddr)))
    {
      MyTag tag;
	  packet->PeekPacketTag(tag);
      Ptr<Node> rxNode = socket->GetNode ();

      double ttc=0;
      if (InetSocketAddress::IsMatchingType (senderAddr))
        {
          InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddr);
          int nodes = m_adhocTxInterfaces->GetN ();
          for (int i = 0; i < nodes; i++)
            {
              if (addr.GetIpv4 () == m_adhocTxInterfaces->GetAddress (i) )
                {
                  Ptr<Node> txNode = GetNode (i);
                  unsigned int txNodeId = txNode->GetId ();
                  if (txNodeId == rxNode->GetId ())
                	  continue;

                  double priority_hp;
                  priority_hp=GetAdaptivePriorityLevel(txNodeId,rxNode->GetId ());

                  int receiverMoving = m_nodesMoving->at (rxNode->GetId ());
                  if (receiverMoving == 1){
                    PacketInfo * pinfo = new PacketInfo();
                    pinfo->SetLagTime(Simulator::Now().GetMilliSeconds()-tag.GetEmitTime());//-tag.GetEmitTime()
                    pinfo->SetSendPriority(tag.GetPrio());
                    pinfo->SetRecvPriority(priority_hp);
                    pinfo->m_msgId=tag.GetMsgId();
                    pinfo->m_txNodeId=txNodeId;
                    pinfo->m_rxNodeId=rxNode->GetId ();
                    pinfo->m_sysTime=Simulator::Now().GetSeconds();

                    //ttc code
                    Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
                    NS_ASSERT (txPosition != 0);
                    Vector pos = txPosition->GetPosition(); // Get velocity
                    Vector vel = txPosition->GetVelocity(); // Get velocity

                    Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
                    NS_ASSERT (rxPosition != 0);
                    Vector pos_rx = rxPosition->GetPosition(); // Get velocity
                    Vector vel_rx = rxPosition->GetVelocity(); // Get velocity
                    ttc=GetTTC(txNodeId,rxNode->GetId ());
                    pinfo->m_txspeed=vel.x;
                    pinfo->m_rxspeed=vel_rx.x;
                    pinfo->m_ttc=ttc;
                    pinfo->m_dist=std::abs(pos.x-pos_rx.x);
                    //add in ttc
                    PacketList.push_back(pinfo);
                  }


                  if(m_prioritytag){
                    HandleAdaptivePriorityReceivedBsmPacket (txNode, rxNode, txNodeId, rxNode->GetId (), tag.GetPrio(), tag.GetMsgId());
                  }
                  else{
                    HandleAdaptivePriorityReceivedBsmPacket (txNode, rxNode, txNodeId, rxNode->GetId (), tag.GetPrio(), tag.GetMsgId());
                    //HandleReceivedBsmPacket (txNode, rxNode);
                  }

                }
            }
        }
    }
}


void BsmApplication::HandleReceivedBsmPacket (Ptr<Node> txNode,
                                              Ptr<Node> rxNode)
{
  NS_LOG_FUNCTION (this);

  m_waveBsmStats->IncRxPktCount ();

  Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
  //Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
  NS_ASSERT (rxPosition != 0);
  // confirm that the receiving node
  // has also started moving in the scenario
  // if it has not started moving, then
  // it is not a candidate to receive a packet
  int rxNodeId = rxNode->GetId ();
  int receiverMoving = m_nodesMoving->at (rxNodeId);
  if (receiverMoving == 1)
    {
      double rxDistSq = MobilityHelper::GetDistanceSquaredBetween (rxNode, txNode);
      //todo: int prio = getPrio(txNode);
      //todo: int base = (prio == 1) ? 0 : 10;
      //NS_LOG_UNCOND ("Time "<<(Simulator::Now()).GetSeconds()<<" tx velocity" << txPosition->GetVelocity());
      if (rxDistSq > 0.0)
        {
          int rangeCount = m_txSafetyRangesSq.size ();
          for (int index = 1; index <= rangeCount; index++)
            {
              if (rxDistSq <= m_txSafetyRangesSq[index - 1])
                {
                  //todo: pass index + base
                  m_waveBsmStats->IncRxPktInRangeCount (index);
                }
            }
        }
    }
}
void BsmApplication::HandlePriorityReceivedBsmPacket (Ptr<Node> txNode,
                                              Ptr<Node> rxNode, int txNodeId, int rxNodeId, double prio)
{
  NS_LOG_FUNCTION (this);
  std::cout << "in priority received bsm class(for 2 class packets)" << '\n';
  int priority;
  //int priority_hp;
  priority= prio; //=GetPriorityLevel(txNodeId);
  //priority_hp=GetPriorityLevel(txNodeId,rxNodeId);
  if(priority>=5){
    m_waveBsmStats_hp->IncRxPktCount ();
  }
  //if(priority_hp==5){
  //  m_waveBsmStats_hp->IncRxPktCount ();
  //}
  else{
    m_waveBsmStats->IncRxPktCount ();
  }

  Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();
  //Ptr<MobilityModel> txPosition = txNode->GetObject<MobilityModel> ();
  NS_ASSERT (rxPosition != 0);
  // confirm that the receiving node
  // has also started moving in the scenario
  // if it has not started moving, then
  // it is not a candidate to receive a packet
  //int rxNodeId = rxNode->GetId ();
  int receiverMoving = m_nodesMoving->at (rxNodeId);
  if (receiverMoving == 1)
    {
      double rxDistSq = MobilityHelper::GetDistanceSquaredBetween (rxNode, txNode);
      //todo: int prio = getPrio(txNode);
      //todo: int base = (prio == 1) ? 0 : 10;
      //NS_LOG_UNCOND ("Time "<<(Simulator::Now()).GetSeconds()<<" tx velocity" << txPosition->GetVelocity());
      if (rxDistSq > 0.0)
        {
          int rangeCount = m_txSafetyRangesSq.size ();
          for (int index = 1; index <= rangeCount; index++)
            {
              if (rxDistSq <= m_txSafetyRangesSq[index - 1])
                {
                  //todo: pass index + base
                  if(priority>=5){
                    m_waveBsmStats_hp->IncRxPktInRangeCount (index);
                  }
                  //if(priority_hp==5){
                  //  m_waveBsmStats_hp->IncRxPktInRangeCount (index);
                  //}
                  else{
                    m_waveBsmStats->IncRxPktInRangeCount (index);
                  }
                }
            }
        }
    }
}

bool BsmApplication::NodeInBetween (int nid1, int nid2) {
	NS_LOG_FUNCTION (this);
	Ptr<Node> node1 = GetNode (nid1);
	Ptr<Node> node2 = GetNode (nid2);

	Ptr<ConstantVelocityMobilityModel> mob_n1 = node1-> GetObject<ConstantVelocityMobilityModel>();
	Ptr<ConstantVelocityMobilityModel> mob_n2 = node2-> GetObject<ConstantVelocityMobilityModel>();
	int  n1_posx = (mob_n1->GetPosition ()).x;
	int  n1_posy = (mob_n1->GetPosition ()).y;
	int  n2_posx = (mob_n2->GetPosition ()).x;
	int x1 = n1_posx;
	int x2 = n2_posx;
	if (n1_posx > n2_posx){
		x1 = n2_posx;
		x2 = n1_posx;
	}
    int nodes = m_adhocTxInterfaces->GetN ();
    for (int i = 0; i < nodes; i++)
    {
    	if (i == nid1 || i == nid2)
    	{
    		continue;
    	}
    	Ptr<Node> cnode = GetNode (i);
    	Ptr<ConstantVelocityMobilityModel> mob_c = cnode-> GetObject<ConstantVelocityMobilityModel>();
    	int c_posx = (mob_c->GetPosition ()).x;
    	int c_posy = (mob_c->GetPosition ()).y;

    	if ((std::abs(n1_posy-c_posy)<=m_lane_thres) && (x1 <= c_posx) && (c_posx <= x2)){
    		std::cout << "nodes in between n1=" << nid1 << " c=" << i << " n2=" << nid2 << std::endl;
    		return true;
    	}
    }
	return false;
}


void BsmApplication::ReInitNodes () {
	Ptr<UniformRandomVariable> var2 = CreateObject<UniformRandomVariable> ();
	Ptr<NormalRandomVariable> var=CreateObject<NormalRandomVariable> ();
	int64_t stream = 2;
	int64_t stream2 = 2;
	var->SetStream (stream);
	var2->SetStream (stream2);
	for(unsigned int i=0;i< m_adhocTxInterfaces->GetN (); i++){
		Ptr<ConstantVelocityMobilityModel> mob = GetNode(i)-> GetObject<ConstantVelocityMobilityModel>();
		Vector posm = mob->GetPosition (); // Get position
		double temp=var2->GetValue (0.0,3.0);
		double x=var2->GetValue (0.0,ROAD_LENGTH_NUM);
		if(temp>=0.0 && temp<1.0){
			mob->SetPosition(Vector(x, 2.0, posm.z));
		}
		else if(temp>=1.0 && temp<2.0){
			mob->SetPosition(Vector(x, 6.0, posm.z));
		}
		else if(temp>=2.0 && temp<3.0){
			mob->SetPosition(Vector(x, 10.0, posm.z));
		}
		else{
			std::cout << "error in allocating y position: " << temp << '\n';
		}

		Vector node_vel=Vector(30+std::sqrt(16)*var->GetValue (), 0.0, 0.0);
		//Vector node_vel=Vector(30+0.5*var->GetValue (), 0.0, 0.0);
		mob->SetVelocity(node_vel);
	}
}



void BsmApplication::HandleAdaptivePriorityReceivedBsmPacket (Ptr<Node> txNode,
		Ptr<Node> rxNode, int txNodeId, int rxNodeId, double prio, int MsgId)
{
	NS_LOG_FUNCTION (this);
	//std::cout << "the second one is being used" << '\n';
	double priority;
	//int priority_hp;
	priority=prio; //GetAdaptivePriorityLevel(txNodeId);
	double priority_threshold=5.0;
	double priority_rxtx;
	priority_rxtx=GetAdaptivePriorityLevel(txNodeId,rxNodeId);
	if(priority>=priority_threshold){
		m_waveBsmStats_hp->IncRxPktCount ();
	}
	else{
		m_waveBsmStats->IncRxPktCount ();
	}
	Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel> ();


	Ptr<ConstantVelocityMobilityModel> mob_tx = txNode-> GetObject<ConstantVelocityMobilityModel>();
	Ptr<ConstantVelocityMobilityModel> mob_rx = rxNode-> GetObject<ConstantVelocityMobilityModel>();


	NS_ASSERT(mob_tx);
	NS_ASSERT(mob_rx);
	Vector posm_rx = mob_rx->GetPosition (); // Get position
	Vector vel_rx = mob_rx->GetVelocity (); // Get velocity
	Vector posm_tx = mob_tx->GetPosition (); // Get position
	Vector vel_tx = mob_tx->GetVelocity (); // Get velocity
	double temp_ttc=GetTTC(txNodeId,rxNodeId);
	double crash_thres=CRASH_THRESHOLD;
	static double last_crash=0;
	if(( (std::abs(posm_rx.y-posm_tx.y)<=m_lane_thres) && (posm_rx.x <= posm_tx.x) && (vel_rx.x > vel_tx.x)) ||
			( (std::abs(posm_rx.y-posm_tx.y)<=m_lane_thres) && (posm_tx.x <= posm_rx.x) && (vel_tx.x > vel_rx.x)))
	{
		std::cout << "Time: " << Simulator::Now().GetMilliSeconds () << " txNode: " << txNodeId  << "(x=" << posm_tx.x << " y=" << posm_tx.y
				<< "  vel=" << vel_tx.x << ") "
				<< " rxNode: " << rxNodeId
				<< "(x=" << posm_rx.x << " y=" << posm_rx.y  << " vel=" << vel_rx.x << ") "
				<< " priority: "<< priority_rxtx << " ttc: " << temp_ttc << " MsgId: " << MsgId << " logx_recv" << std::endl;
		//if (!NodeInBetween(txNodeId,rxNodeId)) {
		if (temp_ttc<crash_thres) {
			crashes++;
			double now_time = Simulator::Now().GetMilliSeconds();
			if( now_time - last_crash < 2000){
				std::cout << "Time: " << Simulator::Now().GetMilliSeconds ()  << " early crash " << crashes  << " txNode: " << txNodeId  << " rxNode: " << rxNodeId << " MsgId: " << MsgId << " ttc: " << temp_ttc<< " priorityrxtx: "
						  << priority_rxtx << " priority: " << priority  << " logx_recv" << std::endl ;
				mob_rx->SetVelocity(Vector(vel_tx.x,0.0,0.0));
				mob_tx->SetVelocity(Vector(vel_rx.x,0.0,0.0));
			}
			else {
				last_crash = now_time;
				std::cout << "Time: " << Simulator::Now().GetMilliSeconds () << " crash occurred" << crashes  << " txNode: " << txNodeId  << " rxNode: " << rxNodeId << " MsgId: " << MsgId << " ttc: " << temp_ttc<< " priorityrxtx: "
						 << priority_rxtx << " priority: " << priority << " logx_recv" << std::endl ;
				ReInitNodes();
			}
		}
		else {
			if (priority_rxtx >= PRIO_MANUEVER_THRESHOLD ) { //&& !NodeInBetween(txNodeId,rxNodeId)) {
				manuevers++;
				std::cout<< "Time: " << Simulator::Now().GetMilliSeconds ()  << " corrc occurred" << manuevers  << " txNode: " << txNodeId  << " rxNode: " << rxNodeId << " MsgId: " << MsgId << " ttc: " << temp_ttc<< " priorityrxtx: "
						<< priority_rxtx << " priority: " << priority << " logx_recv" << std::endl ;
				if ( (posm_rx.x <= posm_tx.x) && (vel_rx.x > vel_tx.x))
					mob_rx->SetVelocity(Vector(vel_tx.x,0.0,0.0));
				else
					mob_tx->SetVelocity(Vector(vel_rx.x,0.0,0.0));
			}
			else {
				std::cout << "Time: " << Simulator::Now().GetMilliSeconds () << " corrc skipped" << manuevers  << " txNode: " << txNodeId  << " rxNode: " << rxNodeId << " MsgId: " << MsgId << " ttc: " << temp_ttc<< " priorityrxtx: "
						<< priority_rxtx << " priority: " << priority << " logx_recv" << std::endl ;
			}
		}
		//}
	}

	NS_ASSERT (rxPosition != 0);
	// confirm that the receiving node
	// has also started moving in the scenario
	// if it has not started moving, then
	// it is not a candidate to receive a packet
	//int rxNodeId = rxNode->GetId ();
	int receiverMoving = m_nodesMoving->at (rxNodeId);
	if (receiverMoving == 1)
	{
		double rxDistSq = MobilityHelper::GetDistanceSquaredBetween (rxNode, txNode);

		if (rxDistSq > 0.0)
		{
			int rangeCount = m_txSafetyRangesSq.size ();
			//std::cout << "doing incrementing IncRxPktInRangeCount with rxDistSq="<< rxDistSq << std::endl;
			for (int index = 1; index <= rangeCount; index++)
			{
				if (rxDistSq <= m_txSafetyRangesSq[index - 1])
				{
					//todo: pass index + base
					if(priority>=priority_threshold){
						//std::cout << "incrementing high priority: " << priority << " index:" << index << std::endl;
						m_waveBsmStats_hp->IncRxPktInRangeCount (index);
					}
					else{
						m_waveBsmStats->IncRxPktInRangeCount (index);
					}
				}
			}
			//std::cout << "done incrementing IncRxPktInRangeCount" << std::endl;
		}
	}
}


int64_t
BsmApplication::AssignStreams (int64_t streamIndex)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_unirv);  // should be set by Setup() prevoiusly
  m_unirv->SetStream (streamIndex);

  return 1;
}

Ptr<Node>
BsmApplication::GetNode (int id)
{
  NS_LOG_FUNCTION (this);

  std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get (id);
  Ptr<Ipv4> pp = interface.first;
  Ptr<Node> node = pp->GetObject<Node> ();

  return node;
}

Ptr<NetDevice>
BsmApplication::GetNetDevice (int id)
{
  NS_LOG_FUNCTION (this);

  std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get (id);
  Ptr<Ipv4> pp = interface.first;
  Ptr<NetDevice> device = pp->GetObject<NetDevice> ();

  return device;
}

void printAppStats(std::string CSVfileName3){
  //std::ofstream out (CSVfileName3.c_str (), std::ios::app);
  std::ofstream out (CSVfileName3.c_str ());
  std::string CSVfileName4 =CSVfileName3+"_msgStats.csv";
  std::string CSVfileName5 =CSVfileName3+"_msgStats2.csv";
  std::string CSVfileName6 =CSVfileName3+"_msgStats3.csv";
  std::ofstream out2 (CSVfileName4.c_str ());
  std::ofstream out3 (CSVfileName5.c_str ());
  std::ofstream out4 (CSVfileName6.c_str ());
  //std::cout << "In BSM APP Stats" << std::endl;
  int hp2hp_count=0;
  int lp_count=0;
  int hp_count=0;
  double hp2hp_delay=0;
  double hp_delay=0;
  double lp_delay=0;
  int hptx_count=0;
  int lptx_count=0;
  unsigned int tempID=0;

  unsigned int tempID_msg=0;
  int hp2hp_count_msg=0;
  int hp_count_msg=0;
  int lp_count_msg=0;
//  int hptx_count_msg=0;
//  int lptx_count_msg=0;

  double hp2hp_delay_msg=0;
  double hp_delay_msg=0;
  double lp_delay_msg=0;

  double min_ttc;
  std::cout << "total crashes: " << crashes << " manuevers: " << manuevers << '\n';
  //std::map <uint32_t, PacketInfo *> hp_Vehiclelist;
  std::map <std::string, PacketInfo *> PacketListIndex;

  out << "hptx_count"<< "," << "lptx_count"<< "," << "hp2hp_count" << ","<< "hp_count" << "," << "lp_count" << "," << "hp2hp_delay" << "," << "hp_delay" << "," << "lp_delay" << "," << "crashes"<< std::endl;

  out2 << "Msg ID" << "," << "TxNode"<< "," << "RxNode" << "," << "TxPriority" << "," << "RxPriority" << "," << "Delay" << "," << "ttc" << "," << "dist" <<"," << "tx_vel" << "," << "rx_vel" << "," << "Sys_time" << std::endl;

  out4 << "Msg_ID" << "," << "TxNode"<< "," << "RxNode" << "," << "ttc" << "," << "Sys_time" << "," <<  "Msg_recv"<< std::endl;

  out3 << "Msg ID" << ","<< "TxNode" <<","<< "hp2hp_count" << ","<< "hp_count" << "," << "lp_count" << "," << "hp2hp_delay_msg" << "," << "hp_delay_msg" << "," << "lp_delay_msg" << std::endl;

  //std::cout << "dropped packets" << dropped_packets<< '\n';
  for (std::vector<PacketInfo *>::iterator it = PacketList.begin() ; it != PacketList.end(); ++it){

      //Create map going from vehicle id and message id to PacketList
      std::stringstream ss;
      ss << (*it)->m_msgId << '-' << (*it)->m_rxNodeId;
      PacketListIndex[ss.str()]= *it;


      //std::cout << "Msg ID " << it->m_msgId << " TxNode "<< it->m_txNodeId << " RxNode " << it->m_rxNodeId << " TxPriority " << it->m_txpriority << " RxPriority " << it->m_rxpriority << " Rxtime " << it->m_lag_time<<'\n';
      //std::cout << "iterator:" << it << '\n';
      //std::cout << "hp packetlist end:" << hp_PacketList.end() << '\n';
      out2 << (*it)->m_msgId << "," << (*it)->m_txNodeId<< "," << (*it)->m_rxNodeId << ","<< (*it)->m_txpriority << "," << (*it)->m_rxpriority <<"," << (*it)->m_lag_time << "," << (*it)->m_ttc <<"," << (*it)->m_dist <<"," << (*it)->m_txspeed <<"," << (*it)->m_rxspeed << "," << (*it)->m_sysTime << std::endl;

      if((*it)->m_txpriority>=5){
        if(tempID<=(*it)->m_msgId){
          tempID=(*it)->m_msgId;
          hptx_count++;
        }
        if(tempID_msg!=(*it)->m_msgId){
          tempID_msg=(*it)->m_msgId;
          hp2hp_delay_msg=hp2hp_delay_msg/((double)hp2hp_count_msg);
          hp_delay_msg=hp_delay_msg/((double)hp_count_msg);
          lp_delay_msg=lp_delay_msg/((double)lp_count_msg);

          if(std::isnan(hp2hp_delay_msg)){
            hp2hp_delay_msg=-1;
          }
          if(std::isnan(hp_delay_msg)){
            hp_delay_msg=-1;
          }
          if(std::isnan(lp_delay_msg)){
            lp_delay_msg=-1;
          }
          out3 << (*it)->m_msgId << "," << (*it)->m_txNodeId <<","<< hp2hp_count_msg << ","<< hp_count_msg << "," << lp_count_msg << "," << hp2hp_delay_msg << "," << hp_delay_msg << ","<<lp_delay_msg << std::endl;

          hp2hp_count_msg=0;
          hp_count_msg=0;
          lp_count_msg=0;
//          hptx_count_msg=0;
 //         lptx_count_msg=0;
          hp2hp_delay_msg=0;
          hp_delay_msg=0;
          lp_delay_msg=0;
          min_ttc=0;
        }

        if((*it)->m_rxpriority==(*it)->m_txpriority){
          hp2hp_count++;
          hp2hp_count_msg++;
          hp2hp_delay=hp2hp_delay+(*it)->m_lag_time;
          hp2hp_delay_msg=hp2hp_delay_msg+(*it)->m_lag_time;
          if(min_ttc>(*it)->m_ttc){
              min_ttc=(*it)->m_ttc;
          }
          if(min_ttc==0){
            min_ttc=(*it)->m_ttc;
          }


        }
        else{
          hp_count++;
          hp_count_msg++;
          hp_delay=hp_delay+(*it)->m_lag_time;
          hp_delay_msg=hp_delay_msg+(*it)->m_lag_time;
          if(min_ttc>(*it)->m_ttc){
              min_ttc=(*it)->m_ttc;
          }
          if(min_ttc==0){
            min_ttc=(*it)->m_ttc;
          }
        }
      }
      else{
        if(tempID<=(*it)->m_msgId){
          tempID=(*it)->m_msgId;
          lptx_count++;
        }
        if(tempID_msg!=(*it)->m_msgId){
          tempID_msg=(*it)->m_msgId;
          hp2hp_delay_msg=hp2hp_delay_msg/((double)hp2hp_count_msg);
          hp_delay_msg=hp_delay_msg/((double)hp_count_msg);
          lp_delay_msg=lp_delay_msg/((double)lp_count_msg);

          if(std::isnan(hp2hp_delay_msg)){
            hp2hp_delay_msg=-1;
          }
          if(std::isnan(hp_delay_msg)){
            hp_delay_msg=-1;
          }
          if(std::isnan(lp_delay_msg)){
            lp_delay_msg=-1;
          }
          out3 << (*it)->m_msgId << "," << (*it)->m_txNodeId <<","<< hp2hp_count_msg << ","<< hp_count_msg << "," << lp_count_msg << "," << hp2hp_delay_msg << "," << hp_delay_msg << ","<<lp_delay_msg << std::endl;

          hp2hp_count_msg=0;
          hp_count_msg=0;
          lp_count_msg=0;
//          hptx_count_msg=0;
//          lptx_count_msg=0;
          hp2hp_delay_msg=0;
          hp_delay_msg=0;
          lp_delay_msg=0;
          min_ttc=0;
        }
        lp_count++;
        lp_count_msg++;
        lp_delay=lp_delay+(*it)->m_lag_time;
        lp_delay_msg=lp_delay_msg+(*it)->m_lag_time;
        if(min_ttc>(*it)->m_ttc){
            min_ttc=(*it)->m_ttc;
        }
        if(min_ttc==0){
          min_ttc=(*it)->m_ttc;
        }
      }

  }
  hp2hp_delay=hp2hp_delay/((double)hp2hp_count);
  hp_delay=hp_delay/((double)hp_count);
  lp_delay=lp_delay/((double)lp_count);
  out << hptx_count<< "," << lptx_count<< "," << hp2hp_count << ","<< hp_count << "," << lp_count << "," << hp2hp_delay << "," << hp_delay << ","<<lp_delay << ","<< crashes << std::endl;
  out.close ();
  out2.close ();
  out3.close ();


//  int hp_counti=0;
  int dropped_packets=0;
  std::map<uint32_t, PacketInfo *> Nodes_dropped;//=it_hp->second->NodesInRange;
  for (std::map< uint32_t, PacketInfo *>::iterator it_hp = hp_PacketList.begin() ; it_hp != hp_PacketList.end(); ++it_hp){
//    hp_counti=it_hp->second->m_msgId;
    //int sz = hp_PacketList[hp_counti]->NodesInRange.size();
    int sz = it_hp->second->NodesInRange.size();
    if(sz>0){
      std::map<uint32_t, uint32_t> Nodes_recv;//=it_hp->second->NodesInRange;

      for(std::vector<uint32_t>::iterator it_hp2 = it_hp->second->NodesInRange.begin() ; it_hp2 != it_hp->second->NodesInRange.end(); ++it_hp2){
        Nodes_recv[*it_hp2]=0;
        std::stringstream ss;

        ss << it_hp->second->m_msgId << '-' << *it_hp2;
        if(PacketListIndex.find(ss.str()) != PacketListIndex.end()){
          std::cout << "received message ID: " << ss.str() << '\n';
          Nodes_recv[*it_hp2]=1;
          out4 << it_hp->second->m_msgId << "," << it_hp->second->m_txNodeId<< "," << *it_hp2 << ","<< it_hp->second->TTCInRange[*it_hp2] << ","<< it_hp->second->m_sysTime <<","<< Nodes_recv[*it_hp2] << "," << it_hp->second->DistInRange[*it_hp2] << std::endl;
        }
        else{
          out4 << it_hp->second->m_msgId << "," << it_hp->second->m_txNodeId<< "," << *it_hp2 << ","<< it_hp->second->TTCInRange[*it_hp2] << ","<< it_hp->second->m_sysTime <<","<< Nodes_recv[*it_hp2] << "," << it_hp->second->DistInRange[*it_hp2] << std::endl;
        }
      }
    }
    if(sz!=0){
      std::cout << "size " << sz << '\n';
      dropped_packets++;
    }
  }

  out4.close ();

#ifdef ADAPTIVE_PRIO
  std::cout << "end of ADAPTIVE_PRIO crash run" << std::endl;
#else
  std::cout << "end of control crash run" << std::endl;
#endif
  //std::cout << "hptx count" << hptx_count << '\n';
  //std::cout << "Hp2HP count" << hp2hp_count << '\n';
  //std::cout << "HP count" << hp_count << '\n';
  //std::cout << "lp count" << lp_count << '\n';
  //std::cout << "LPtx count" << lptx_count << '\n';
}

void SetupLogFile(std::string CSVfileName3){
  m_CSVfileName3=CSVfileName3;

}
} // namespace ns3
