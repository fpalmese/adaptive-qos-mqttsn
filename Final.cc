/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Foundation, Inc., 59 Temple Place, Suite 1600, Boston, MA  02111-1307  USA
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/nist-error-rate-model.h"
#include "ns3/error-model.h"
#include "ns3/qos-header.h"
#include "ns3/uid-header.h"
#include "ns3/counter-header.h"
#include "ns3/snr-tag.h"
#include <string>
#include <random>

// Default Network Topology
//
//   Wifi 10.1.2.0
//                 AP
//  *    *    *    *
//  |    |    |    |    10.1.1.0
// n5   n6   n7   n0 -------------- n1  
//                            point-to-point 
//                                 

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

//defining the wifi mode that is going to be used
std::string phyMode ("DsssRate2Mbps");
//std::string phyMode ("ErpOfdmRate48Mbps");

//struct for packets with QoS = 0
struct Memory0{ uint64_t uid; uint64_t startingTime; uint64_t arrivalTime; uint16_t sent; uint16_t startQoS; Ipv4Address addr; uint16_t marked;};
//struct for packets with QoS = 1
struct Memory1{ uint64_t uid; uint16_t ack1; uint16_t send1; uint16_t ack2; uint32_t send2; uint64_t latency1; uint64_t latency2; uint64_t startingTime; uint64_t arrivalTime; uint64_t subRecTime; uint16_t sent; uint16_t startQoS; uint16_t retry; Ipv4Address addr; uint16_t marked;};
//struct for packets with QoS = 2
struct Memory2{ uint64_t uid; uint16_t ack1; uint16_t send1; uint32_t ack3; uint32_t send3; uint64_t latency1; uint64_t latency2; uint64_t latency3; uint64_t latency4; uint64_t startingTime; uint64_t arrivalTime; uint64_t subRecTime; uint16_t sent; uint16_t startQoS; uint16_t retry; uint16_t marked; Ipv4Address addr;};
/struct for Subscribers
struct Sub{uint64_t chosenQoS; Ipv4Address addr; float hitrate; float latency; float hitrateTemp; uint16_t lastChange; uint16_t loop1; uint16_t loop2;};
//struct to compute the correct QoS 1 hitrate
struct Help{ uint64_t startingTime; Ipv4Address addr; uint16_t bol; uint16_t marked; uint64_t arrivalTime;};

//struct arrays for the various categories
Memory0 memory0Pub[100];
Memory1 memory1Pub[100];
Memory2 memory2Pub[1000];
Memory0 memory0Sub[10000];
Memory1 memory1Sub[8000];
Memory2 memory2Sub[8000];
Sub subs[10];

Help help0[6400] = {1, subs[0].addr, 0, 0, 0};
Help help1[6400] = {1, subs[0].addr, 0, 0, 0};

//Universal variables definitions, useful for statistics mainly
float hitratesCont[sizeof(subs)/sizeof(subs[0])] = {0};
float latCont[sizeof(subs)/sizeof(subs[0])] = {0};
float stdLatCont[sizeof(subs)/sizeof(subs[0])] = {0};
uint16_t QoS0[sizeof(subs)/sizeof(subs[0])] = {0};
uint16_t QoS1[sizeof(subs)/sizeof(subs[0])] = {0};
uint16_t QoS2[sizeof(subs)/sizeof(subs[0])] = {0};


uint32_t launchedPacketsPubQoS0 = 0;
uint32_t arrivedPacketsPubQoS0 = 0;
uint32_t launchedPacketsPubQoS1 = 0;
uint32_t halfPacketsPubQoS1 = 0;
uint32_t arrivedPacketsPubQoS1 = 0;
uint32_t launchedPacketsPubQoS2 = 0;
uint32_t launchedPacketsSubQoS0 = 0;
uint32_t arrivedPacketsSubQoS0PubQoS0 = 0;
uint32_t arrivedPacketsSubQoS0PubQoS1 = 0;
uint32_t arrivedPacketsSubQoS0PubQoS2 = 0;
uint32_t launchedPacketsSubQoS1 = 0;
uint32_t halfPacketsSubQoS1 = 0;
uint32_t arrivedPacketsSubQoS1PubQoS0 = 0;
uint32_t arrivedPacketsSubQoS1PubQoS1 = 0;
uint32_t arrivedPacketsSubQoS1PubQoS2 = 0;
uint32_t launchedPacketsSubQoS2 = 0;
uint32_t halfPacketsSubQoS2 = 0; 
uint32_t arrivedPacketsPubQoS2 = 0;
uint32_t arrivedPacketsSubQoS2PubQoS0 = 0;
uint32_t arrivedPacketsSubQoS2PubQoS1 = 0;
uint32_t arrivedPacketsSubQoS2PubQoS2 = 0;
uint32_t retryQoS1 = 0;
uint32_t retryQoS2 = 0;
uint16_t interfCounter = 0;

//QoS of publishers and subscribers
uint64_t QoS = 2;
uint64_t pubQoS = 2;

//Useful global counters used by all the functions
uint16_t counterControllerSub = 0;

uint32_t k0Pub = 0;
uint32_t k1Pub = 0;
uint32_t k2Pub = 0;
uint32_t k0Sub = 0;
uint32_t k1Sub = 0;
uint32_t testing = 0;
uint32_t k2Sub = k0Sub;
uint64_t overIndex = 1;

uint16_t index0Pub = 0;
uint16_t index1Pub = 0;
uint16_t index2Pub = 0;
uint16_t index0Sub = 0;
uint16_t index1Sub = 0;
uint16_t index2Sub = 0;

uint16_t subReverse = 0;

//Controller inputs
uint16_t inputControllerUser = 1;
float inputHitrate = 90;
float inputLatency = 3100000;

//Packet size declaration
uint32_t packetSize = 1400; // bytes
uint32_t packetSizeAck = 1; // bytes

//Output file declaration, for hitrate, latency and QoS flows
std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;

Ptr<ErrorModel> rem = CreateObject<RateErrorModel> ();

//Packet Error Rate definitions
std::random_device rd1{}; // use to seed the rng 
std::mt19937 rng1{rd1()}; // rng
double p1 = 1; // probability
std::bernoulli_distribution d1(p1);


std::random_device rd2{}; // use to seed the rng 
std::mt19937 rng2{rd2()}; // rng
double p2 = 1; // probability
std::bernoulli_distribution d2(p2);


std::random_device rd3{}; // use to seed the rng 
std::mt19937 rng3{rd3()}; // rng
double p3 = 1; // probability
std::bernoulli_distribution d3(p3);


std::random_device rd4{}; // use to seed the rng 
std::mt19937 rng4{rd4()}; // rng
double p4 = 0.35; // probability
std::bernoulli_distribution d4(p4);


std::random_device rd5{}; // use to seed the rng 
std::mt19937 rng5{rd5()}; // rng
double p5 = 1; // probability
std::bernoulli_distribution d5(p5);


std::random_device rd6{}; // use to seed the rng 
std::mt19937 rng6{rd6()}; // rng
double p6 = 1; // probability
std::bernoulli_distribution d6(p6);


std::random_device rd7{}; // use to seed the rng 
std::mt19937 rng7{rd7()}; // rng
double p7 = 1; // probability
std::bernoulli_distribution d7(p7);


std::random_device rd8{}; // use to seed the rng 
std::mt19937 rng8{rd8()}; // rng
double p8 = 1; // probability
std::bernoulli_distribution d8(p8);


std::random_device rd9{}; // use to seed the rng 
std::mt19937 rng9{rd9()}; // rng
double p9 = 1; // probability
std::bernoulli_distribution d9(p9);


std::random_device rd10{}; // use to seed the rng 
std::mt19937 rng10{rd10()}; // rng
double p10 = 1; // probability
std::bernoulli_distribution d10(p10);


/*std::random_device rd11{}; // use to seed the rng 
std::mt19937 rng11{rd11()}; // rng
double p11 = 0.8546600497762231; // probability
std::bernoulli_distribution d11(p11);


std::random_device rd12{}; // use to seed the rng 
std::mt19937 rng12{rd12()}; // rng
double p12 = 0.8969446873910927; // probability
std::bernoulli_distribution d12(p12);


std::random_device rd13{}; // use to seed the rng 
std::mt19937 rng13{rd13()}; // rng
double p13 = 0.9068536916240811; // probability
std::bernoulli_distribution d13(p13);


std::random_device rd14{}; // use to seed the rng 
std::mt19937 rng14{rd14()}; // rng
double p14 = 0.8154586200925154; // probability
std::bernoulli_distribution d14(p14);


std::random_device rd15{}; // use to seed the rng 
std::mt19937 rng15{rd15()}; // rng
double p15 = 0.483532897026062; // probability
std::bernoulli_distribution d15(p15);


std::random_device rd16{}; // use to seed the rng 
std::mt19937 rng16{rd16()}; // rng
double p16 = 0.821282506124847; // probability
std::bernoulli_distribution d16(p16);


std::random_device rd17{}; // use to seed the rng 
std::mt19937 rng17{rd17()}; // rng
double p17 = 0.46373213913660755; // probability
std::bernoulli_distribution d17(p17);


std::random_device rd18{}; // use to seed the rng 
std::mt19937 rng18{rd18()}; // rng
double p18 = 0.9069949004802726; // probability
std::bernoulli_distribution d18(p18);


std::random_device rd19{}; // use to seed the rng 
std::mt19937 rng19{rd19()}; // rng
double p19 = 0.9377655753517249; // probability
std::bernoulli_distribution d19(p19);


std::random_device rd20{}; // use to seed the rng 
std::mt19937 rng20{rd20()}; // rng
double p20 = 0.8985315819161976; // probability
std::bernoulli_distribution d20(p20);

*/

//Functions which are going to be defined later

static void GenerateTraffic(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                            uint64_t counterInt, uint64_t inputQos);

static void Retransmission1(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

static void Retransmission21(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

static void Retransmission23(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

static void Retransmission1Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

static void Retransmission21Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

static void Retransmission23Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid);

//Standard deviation function for the 10 subscribers case

float calculateSD(Sub subFunc[10]) 
{
  float sum = 0.0, mean, standardDeviation = 0.0;
  int i;

  for(i = 0; i < 10; ++i) {
    sum += subFunc[i].latency;
  }

  mean = sum / 10;

  for(i = 0; i < 10; ++i) {
    standardDeviation += pow(subFunc[i].latency - mean, 2);
  }

  return sqrt(standardDeviation / 10);
}

//Old controller, can be discarded

static uint64_t ControllerPub(uint64_t maintain)
{
  counterControllerSub = 0;
  uint64_t out = 0;
  float average = 0;
  for(int i = index0Pub; i < sizeof(memory0Sub)/sizeof(memory0Sub[0]); i++)
  {
    if(memory0Sub[i].arrivalTime != 0 && memory0Sub[i].uid != 0)
    {
      average = average + memory0Sub[i].arrivalTime - memory0Sub[i].startingTime;
      index0Pub++;
    }
  }
  for(int i = index1Pub; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
  {
    if(memory1Sub[i].subRecTime != 0 && memory1Sub[i].uid != 0)
    {
      average = average + memory1Sub[i].subRecTime - memory1Sub[i].startingTime;
      index1Pub++;
    }
  }
  for(int i = index2Pub; i < sizeof(memory0Sub)/sizeof(memory0Sub[0]); i++)
  {
    if(memory2Sub[i].subRecTime != 0 && memory2Sub[i].uid != 0)
    {
      average = average + memory2Sub[i].subRecTime - memory2Sub[i].startingTime;
      index2Pub++;
    }
  }
  average = average/(index0Pub + index1Pub + index2Pub);
  float perc = 100;
  if(pubQoS == 0)
  {
    perc = (float)arrivedPacketsPubQoS0*100/launchedPacketsPubQoS0;
    
  }
  if(pubQoS == 1)
  {
    perc = (float)arrivedPacketsPubQoS1/launchedPacketsPubQoS1;
  }
  if(perc < 90)
  {
    out = 1;
  }
  if(average > 1000000)
  {
    out = 0;
  }
  if(perc > 90 && average < 1000000)
  {
    out = maintain;
  }
  return out;
}

//Working controller

static uint64_t ControllerSub(uint64_t maintain, uint64_t qosPub)
{
  NS_LOG_UNCOND("Entering shadow realm");

//Initializing subscriber variables for the evaluation
  for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
  {
    subs[j].hitrate = 0;
    subs[j].latency = 0;
    subs[j].hitrateTemp = 0;
    if(subs[j].chosenQoS == 2)
    {
      subs[j].hitrateTemp = 30;
    }
  }

  float num = 0;
  float den = 10;
  uint64_t out = 0;
  float average = 0;
  index0Sub = 0;
  index1Sub = 0;
  uint16_t changed2 = 0;
  uint16_t changed1 = 0;
  uint16_t faulty = 0;
  uint16_t notFaulty = 0;

//Evaluating the packet with QoS = 0 and publish QoS = 0 or 2

  for(int i = 0; i < sizeof(memory0Sub)/sizeof(memory0Sub[0]); i++)
  {
    if(memory0Sub[i].arrivalTime != 0 && memory0Sub[i].uid != 0 && memory0Sub[i].marked == 0)
    {
      if(memory0Sub[i].startQoS == 0 || memory0Sub[i].startQoS == 2)
      {
        average = average + memory0Sub[i].arrivalTime - memory0Sub[i].startingTime; //Useless
        memory0Sub[i].marked = 1; //Marking a packet so it will not be counted twice
        index0Sub++; //Incrementing index of array
        num++; //Useless
        for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
        {
          if(memory0Sub[i].addr == subs[j].addr)
          {
	    subs[j].hitrate++; //Incrementing packet count
            subs[j].hitrateTemp++; //Incrementing alternative packet count (introduced to skew statistic with QoS = 2)
            subs[j].latency = subs[j].latency + memory0Sub[i].arrivalTime - memory0Sub[i].startingTime; //Sum of the latencies of the packets involved
          }
        }
        if(memory0Sub[i].startingTime < Simulator::Now().GetMicroSeconds() - 1000000)
        {
          den++; //Useless part
        }
      }
    }
  }

//Computing the packets with QoS of the subscriber equal to 0 and QoS of the publisher equal to 1

  for(int i = 0; i < sizeof(help0)/sizeof(help0[0]); i++)
  {
    if(help0[i].bol == 1 && help0[i].marked == 0)
    {
      average = average + help0[i].arrivalTime - help0[i].startingTime;
      index0Sub++; //Incrementing denominator
      help0[i].marked = 1; //Marking a packet so it will not be counted twice
      num++; //Useless
      for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
      {
        if(help0[i].addr == subs[j].addr)
        {
	  subs[j].hitrate++; //Incrementing packet count
          subs[j].latency = subs[j].latency + help0[i].arrivalTime - help0[i].startingTime; //Sum of the latencies of the packets involved
        }
      }
      if(help0[i].startingTime < Simulator::Now().GetMicroSeconds() - 1000000)
      {
        den++; //Useless part
      }
    }
  }

//Computing the packets with QoS of the subscriber equal to 1 and QoS of the publisher equal to 1

  for(int i = 0; i < sizeof(help1)/sizeof(help1[0]); i++)
  {
    if(help1[i].bol == 1 && help1[i].marked == 0)
    {
      average = average + help1[i].arrivalTime - help1[i].startingTime;
      index1Sub++; //Incrementing denominator
      help1[i].marked = 1; //Marking a packet so it will not be counted twice
      num++;
      for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
      {
        if(help1[i].addr == subs[j].addr)
        {
	  subs[j].hitrate++; //Incrementing packet count
          subs[j].latency = subs[j].latency + help1[i].arrivalTime - help1[i].startingTime; //Sum of the latencies of the packets involved
        }
      }
      if(help1[i].startingTime < Simulator::Now().GetMicroSeconds() - 1000000)
      {
        den++; //Useless
      }
    }
  }

//Computing the packets with QoS of the subscriber equal to 1 and QoS of the publisher equal to 2

  for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
  {
    if(memory1Sub[i].subRecTime != 0 && memory1Sub[i].uid != 0 && memory1Sub[i].marked == 0)
    {
      if(memory1Sub[i].startQoS == 2)
      {
        average = average + memory1Sub[i].subRecTime - memory1Sub[i].startingTime;
        index1Sub++; //Incrementing denominator
        memory1Sub[i].marked = 1; //Marking a packet so it will not be counted twice
        num++;
        for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
        {
          if(memory1Sub[i].addr == subs[j].addr)
          {
	    subs[j].hitrate++; //Incrementing packet count
            subs[j].hitrateTemp++; //Incrementing alternative packet count (introduced to skew statistic with QoS = 2)
            subs[j].latency = subs[j].latency + memory1Sub[i].subRecTime - memory1Sub[i].startingTime; //Sum of the latencies of the packets involved
          }
        }
        if(memory1Sub[i].startingTime < Simulator::Now().GetMicroSeconds() - 1000000)
        {
          den++; //Useless
        }
      }
    }
  }

//Computing the packets with QoS of the subscriber equal to 2 and QoS of the publisher equal to 2

  for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
  {
    if(memory2Sub[i].subRecTime != 0 && memory2Sub[i].uid != 0 && memory2Sub[i].marked == 0 && memory2Sub[i].startingTime >= Simulator::Now().GetMicroSeconds() - 30000000)
    {
      average = average + memory2Sub[i].subRecTime - memory2Sub[i].startingTime;
      memory2Sub[i].marked = 1; //Marking a packet so it will not be counted twice
      num++;
      for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
      {
        if(memory2Sub[i].addr == subs[j].addr)
        {
	  subs[j].hitrate++; //Incrementing packet count
          subs[j].hitrateTemp = 30; //Skewing statistics with sub QoS = 2
          subs[j].latency = subs[j].latency + memory2Sub[i].subRecTime - memory2Sub[i].startingTime; //Sum of the latencies of the packets involved
        }
      }
      if(memory2Sub[i].startingTime < Simulator::Now().GetMicroSeconds() - 1000000)
      {
        den++; //Useless
      }
    }
  }

//Mechanism to evaluate the worst two subscriber nodes regarding the hitrate
  
  Sub worstSubsHitrate[2] = {1, subs[0].addr, 3200, 0, 0, 0}; //Creation of array of size 2
  NS_LOG_UNCOND("Testing: " << worstSubsHitrate[0].hitrate);
  for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
  {
    if(subs[i].hitrateTemp <= worstSubsHitrate[0].hitrate)
    {
      worstSubsHitrate[1] = worstSubsHitrate[0]; //If new worse than both the previous, then worst 1 goes in position 2, worst 2 is eliminated
      worstSubsHitrate[0] = subs[i]; // and new worst goes in position 1
      NS_LOG_UNCOND("Worst hitrates: " << worstSubsHitrate[0].addr << "-" << worstSubsHitrate[1].addr); // Debug
    }
    if(subs[i].hitrateTemp <= worstSubsHitrate[1].hitrateTemp && subs[i].hitrateTemp > worstSubsHitrate[0].hitrateTemp)
    {
      worstSubsHitrate[1] = subs[i]; // If new sub is worse than worst 2 but better than worst 1, it takes worst 2 place
      NS_LOG_UNCOND("Worst hitrates: " << worstSubsHitrate[0].addr << "-" << worstSubsHitrate[1].addr); // Debug
    }
  }
  
  for(int i = 0; i < sizeof(worstSubsHitrate)/sizeof(worstSubsHitrate[0]); i++)
  {
    NS_LOG_UNCOND(worstSubsHitrate[i].addr << "-" << worstSubsHitrate[i].hitrateTemp << "-" << worstSubsHitrate[i].latency); // Additional debug
  }

//Same mechanism as the previous clause, only evaluating the latencies instead of the hitrate

  Sub worstSubsLatency[2] = {1, subs[0].addr, 10, 0, 0, 0};
  NS_LOG_UNCOND("Testing: " << worstSubsLatency[0].latency);
  for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
  {
    if((float)subs[i].latency/subs[i].hitrate >= (float)worstSubsLatency[0].latency/worstSubsLatency[0].hitrate)
    {
      worstSubsLatency[1] = worstSubsLatency[0];
      worstSubsLatency[0] = subs[i];
      NS_LOG_UNCOND("Worst latency: " << worstSubsLatency[0].addr << "-" << worstSubsLatency[1].addr);
    }
    if((float)subs[i].latency/subs[i].hitrate >= (float)worstSubsLatency[1].latency/worstSubsLatency[1].hitrate && (float)subs[i].latency/subs[i].hitrate < (float)worstSubsLatency[0].latency/worstSubsLatency[0].hitrate)
    {
      worstSubsLatency[1] = subs[i];
      NS_LOG_UNCOND("Worst latency: " << worstSubsLatency[0].addr << "-" << worstSubsLatency[1].addr);
    }
  }
  
  for(int i = 0; i < sizeof(worstSubsLatency)/sizeof(worstSubsLatency[0]); i++)
  {
    NS_LOG_UNCOND(worstSubsLatency[i].addr << "-" << worstSubsLatency[i].hitrateTemp << "-" << worstSubsLatency[i].latency); // More debug
  }
  
//Writing the simulator time in the csv files in output

  myfile1 << "\n";
  myfile1 << Simulator::Now().GetSeconds() - 1 << ";";
  myfile2 << "\n";
  myfile2 << Simulator::Now().GetSeconds() - 1 << ";";
  myfile3 << "\n";
  myfile3 << Simulator::Now().GetSeconds() - 1 << ";";

//Controller actions

  for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
  {
    subs[i].lastChange++; //Incrementing variable responsible of keeping track of last change of the subscriber
    if(1)
    {
      if(pubQoS == 0) //Not used, since the publisher QoS has always been equal to 2, future work can start from here
      {
        if((float)subs[i].hitrateTemp*100/30 < 70)
        {
          faulty++; //Important variable for majority voting regarding all the subscribers
        }      
      }
      if(pubQoS == 1) //Not used, since the publisher QoS has always been equal to 2, future work can start from here
      {
        changed1 = 0;
        NS_LOG_UNCOND("Hitrates: " << subs[i].hitrate*100/30 << " Latency: " << subs[i].latency/30);
        if(subs[i].chosenQoS == 0 && changed1 == 0)
        {
          if(subs[i].hitrate*100/(float)30 < 80) //Hitrate evaluation, not theoretically accurate
          {
            NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1");
            subs[i].chosenQoS = 1;
            changed1 = 1;
            faulty++; //Important variable for majority voting regarding all the subscribers
          }
        }
        if(subs[i].chosenQoS == 1 && changed1 == 0)
        {
          if(subs[i].latency/30 > 600000) //Latency evaluation, not theoretically accurate
          {
            NS_LOG_UNCOND("Changing2: from QoS 1 to QoS 0");
            subs[i].chosenQoS = 0;
            changed1 = 1;
            notFaulty++; //Important variable for majority voting regarding all the subscribers
          }
        }
      }
      if(pubQoS == 2) //Fun part, real controller implementation
      {
        changed2 = 0; //Initialization of variable necessary to avoid multiple changes inside controller
        hitratesCont[index2Sub] = hitratesCont[index2Sub] + ((float)subs[i].hitrateTemp*(float)100/(float)30); //Average subscribers hitrates 
        latCont[index2Sub] = latCont[index2Sub] + (float)subs[i].latency/(float)subs[i].hitrate; //Average subscribers latency
        NS_LOG_UNCOND("Hitrates: " << subs[i].hitrateTemp*100/30 << " Latency: " << subs[i].latency/subs[i].hitrate << " Addr: " << subs[i].addr << " QoS: " << subs[i].chosenQoS);
        myfile1 << subs[i].hitrateTemp*100/30; //Writing hitrate in csv
        myfile2 << subs[i].latency/subs[i].hitrate; //Writing latency in csv
        myfile3 << subs[i].chosenQoS; //Writing QoS in csv

        if(i != 9) //Number separators in the csv files
        {
          myfile1 << ";";
          myfile2 << ";";
          myfile3 << ";";
        }
        if(inputControllerUser == 0) //First controller mode, general evaluation
        {

//Subscriber selection and controller time check

          if((subs[i].addr == worstSubsHitrate[0].addr || subs[i].addr == worstSubsHitrate[1].addr || subs[i].addr == worstSubsLatency[0].addr || subs[i].addr == worstSubsLatency[1].addr) && subs[i].lastChange > 2 && Simulator::Now().GetSeconds() > 92) 
          {
            if(subs[i].chosenQoS == 0 && changed2 == 0) //Subscriber QoS check
            {
              if(subs[i].hitrateTemp*100/30 < 80) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1"); // Debug
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0; // last change of subscriber goes to 0, since it just happened. It is equal in every successive if clause
              }
              if(subs[i].hitrateTemp*100/30 < 45) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing5: from QoS 0 to QoS 2");
                subs[i].chosenQoS = 2; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
            }
            if(subs[i].chosenQoS == 1 && changed2 == 0) //Subscriber QoS check
            {
              if(subs[i].hitrateTemp*100/30 < 95) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing2: from QoS 1 to QoS 2");
                subs[i].chosenQoS = 2; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
              if((float)subs[i].latency/subs[i].hitrateTemp > 2500000) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing3: from QoS 1 to QoS 0");
                subs[i].chosenQoS = 0; //New QoS value
                changed2 = 1;
                notFaulty++;
                subs[i].lastChange = 0;
              }
              if((float)subs[i].latency/subs[i].hitrateTemp > 2500000 && subs[i].hitrateTemp*100/30 < 95) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Put me out of my misery: from QoS 1 to QoS 1");
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                notFaulty++;
                subs[i].lastChange = 0;
              }
            }
            if(subs[i].chosenQoS == 2 && changed2 == 0) //Subscriber QoS check
            {
              if((float)subs[i].latency/subs[i].hitrate> 5000000) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing4: from QoS 2 to QoS 1");
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                notFaulty++;
                subs[i].lastChange = 0;
              }
            }
          }
        }
        if(inputControllerUser == 1) //Second controller mode, hitrate threshold
        {

//Subscriber selection and controller time check

          if((subs[i].addr == worstSubsHitrate[0].addr || subs[i].addr == worstSubsHitrate[1].addr || subs[i].addr == worstSubsLatency[0].addr || subs[i].addr == worstSubsLatency[1].addr) && subs[i].lastChange > 2 && Simulator::Now().GetSeconds() > 92)
          {
            if(subs[i].chosenQoS == 0 && changed2 == 0) //Subscriber QoS check
            {
              if(subs[i].hitrateTemp*100/30 < inputHitrate - 10) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1");
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
            }
            if(subs[i].chosenQoS == 1 && changed2 == 0) //Subscriber QoS check
            {
              if(subs[i].hitrateTemp*100/30 < inputHitrate - 5)// && subs[i].loop2 < 2) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing1: from QoS 1 to QoS 2");
                subs[i].chosenQoS = 2; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
              if((float)subs[i].latency/subs[i].hitrate > 2300000)// && subs[i].loop1 < 2) //Controller finite state machine arch
              {
                if(subs[i].hitrateTemp*100/30 > std::min(inputHitrate + 20, (float)99.9)) //Controller finite state machine arch
                {
                  NS_LOG_UNCOND("Changing4: from QoS 1 to QoS 0");
                  subs[i].chosenQoS = 0; //New QoS value
                  changed2 = 1;
                  notFaulty++;
                  subs[i].lastChange = 0;
                  subs[i].loop1++;
                }
              }
            }
            if(subs[i].chosenQoS == 2 && changed2 == 0) //Subscriber QoS check
            {
              if((float)subs[i].latency/subs[i].hitrate > 1500000 && (float)subs[i].latency/subs[i].hitrate < 10000000) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing4: from QoS 2 to QoS 1");
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                notFaulty++;
                subs[i].lastChange = 0;
                subs[i].loop2++;
              }
            }            
          }         
        }
        if(inputControllerUser == 2) Third controller mode, latency threshold
        {

//Subscriber selection and controller time check

          if((subs[i].addr == worstSubsHitrate[0].addr || subs[i].addr == worstSubsHitrate[1].addr || subs[i].addr == worstSubsLatency[0].addr || subs[i].addr == worstSubsLatency[1].addr) && subs[i].lastChange > 2 && Simulator::Now().GetSeconds() > 92)
          {
            if(subs[i].chosenQoS == 2 && changed2 == 0) //Subscriber QoS check
            {
              if((float)subs[i].latency/subs[i].hitrate > inputLatency + 1000000) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing1: from QoS 2 to QoS 1");
                subs[i].chosenQoS = 1; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
              if((float)subs[i].latency/subs[i].hitrate > inputLatency + 2000000) //Controller finite state machine arch
              {
                if(inputLatency < 500000)
                {
                  NS_LOG_UNCOND("Changing1: from QoS 2 to QoS 0");
                  subs[i].chosenQoS = 0; //New QoS value
                  changed2 = 1;
                  subs[i].lastChange = 0;
                }
                if(inputLatency >= 500000)
                {
                  NS_LOG_UNCOND("Changing1: from QoS 2 to QoS 1");
                  subs[i].chosenQoS = 1; //New QoS value
                  changed2 = 1;
                  subs[i].lastChange = 0;
                }
              }                
            }
            if(subs[i].chosenQoS == 1 && changed2 == 0) //Subscriber QoS check
            {
              if((float)subs[i].latency/subs[i].hitrate > inputLatency + 1000000) //Controller finite state machine arch
              {
                NS_LOG_UNCOND("Changing1: from QoS 1 to QoS 0");
                subs[i].chosenQoS = 0; //New QoS value
                changed2 = 1;
                subs[i].lastChange = 0;
              }
              if(subs[i].hitrateTemp*100/30 < 99.9) //Controller finite state machine arch
              {
                if((float)subs[i].latency/subs[i].hitrate < (inputLatency - 1000000) && inputLatency < 3500000) //Controller finite state machine arch
                {
                  NS_LOG_UNCOND("Changing4: from QoS 1 to QoS 2");
                  subs[i].chosenQoS = 2; //New QoS value
                  changed2 = 1;
                  notFaulty++;
                  subs[i].lastChange = 0;
                }
              }
            }
            if(subs[i].chosenQoS == 0 && changed2 == 0) //Subscriber QoS check
            {
              if(inputLatency < 1000000) //Controller finite state machine arch
              {
                if(subs[i].hitrateTemp*100/30 > 80 && subs[i].hitrateTemp*100/30 < 95) //Controller finite state machine arch
                {
                  NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1");
                  subs[i].chosenQoS = 1; //New QoS value
                  changed2 = 1;  
                  subs[i].lastChange = 0;
                }
              }
              if(inputLatency > 1300000) //Controller finite state machine arch
              {
                if(subs[i].hitrateTemp*100/30 > 75) //Controller finite state machine arch
                {
                  NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1");
                  subs[i].chosenQoS = 1; //New QoS value
                  changed2 = 1;  
                  subs[i].lastChange = 0;
                }
              }
              if(inputLatency > 4500000) //Controller finite state machine arch
              {
                if(subs[i].hitrateTemp*100/30 > 55) //Controller finite state machine arch
                {
                  NS_LOG_UNCOND("Changing1: from QoS 0 to QoS 1");
                  subs[i].chosenQoS = 1; //New QoS value
                  changed2 = 1;  
                  subs[i].lastChange = 0;
                }
              }
            }            
          }         
        }
      }
    }
  }

//Computing number of subscribers with a certain QoS

  for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
  {
    if(subs[i].chosenQoS == 0)
    {
      QoS0[index2Sub]++;
    }
    if(subs[i].chosenQoS == 1)
    {
      QoS1[index2Sub]++;
    }
    if(subs[i].chosenQoS == 2)
    {
      QoS2[index2Sub]++;
    }
  }
  hitratesCont[index2Sub] /= 10; //Computing average hitrate
  latCont[index2Sub] /= 10; //Computing average latency
  stdLatCont[index2Sub] = calculateSD(subs); //Computing standard deviation of latency
  NS_LOG_UNCOND(hitratesCont[index2Sub] << " | " << latCont[index2Sub] << " | " << stdLatCont[index2Sub] << " | " << index2Sub);
  index2Sub++;
  /*if(faulty > 5 && pubQoS == 1)
  {
    NS_LOG_UNCOND("Changing pub QoS from 1 to 2");
    pubQoS++;
    for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
    {
      subs[i].chosenQoS = 1;
    }
  }
  
  if(faulty > 5 && pubQoS == 0)
  {
    NS_LOG_UNCOND("Changing pub QoS from 0 to 1");
    pubQoS++;
    for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
    {
      subs[i].chosenQoS = 0;
    }
  }
  if(notFaulty > 5 && pubQoS == 1)
  {
    NS_LOG_UNCOND("Changing pub QoS from 1 to 0");
    pubQoS--;
    for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
    {
      subs[i].chosenQoS = 0;
    }
  }
  
  if(notFaulty > 5 && pubQoS == 2)
  {
    NS_LOG_UNCOND("Changing pub QoS from 2 to 1");
    pubQoS--;
    for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
    {
      subs[i].chosenQoS = 1;
    }
  }*/


  NS_LOG_UNCOND(num);
  return out;
}


//This part is dedicated to the ack check and retransmission

static void CheckAck1(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory1Pub)/sizeof(memory1Pub[0]); i++) //Pub with QoS 1 scan
  {
    if(memory1Pub[i].uid == uid) //Check packet uid
    {
      if(memory1Pub[i].ack1 == 0) //Check if ack has arrived or not
      {
        if(memory1Pub[i].send1 < 4) //If not, control the number or retransmissions
        {
          NS_LOG_UNCOND ("PUBLISH: Packet " << uid << " not received in time, sending another");
          memory1Pub[i].send1++; //Increment retransmission number to detect late messages
          float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
          //Issue another transmission
          Simulator::Schedule(Seconds(memory1Pub[i].send1 + r), &Retransmission1, socket, dst, pktSize, counterInt, inputQos, uid);
        } else {
           NS_LOG_UNCOND ("Dropping packet " << uid << ", sent too many times"); 
           memory1Pub[i].send1++; //Dropping packet
        }
      }
    }
  }
}

static void Retransmission1(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    retryQoS1++; //Incrementnig stat measure
    Ptr<Packet> packet = Create<Packet>(packetSize); //New packet creation
    QoSHeader qos; //QoS header creation
    qos.SetData(inputQos); //QoS value insertion
    packet->AddHeader(qos); //Adding header to packet
    UidHeader id; //Same procedure for uid of packet
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for counter
    counterInt = counterInt + 1; //Incrementing counter to match send1 variable
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    QoSHeader pubOrSub; //Header procedure for pubOrSub
    pubOrSub.SetData(1);
    packet->AddHeader(pubOrSub);
    socket->SendTo (packet, 0, dst); //Sending packet 
    for(int i = 0; i < sizeof(memory1Pub)/sizeof(memory1Pub[0]); i++)
    {
      if(memory1Pub[i].uid == uid)
      {
        memory1Pub[i].latency1 = Simulator::Now().GetMicroSeconds(); //Insert packet stats
        uint64_t startTime = memory1Pub[i].startingTime;
      }
    }
    NS_LOG_UNCOND("PUBLISH " << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck1, uid, socket, dst, pktSize, counterInt, inputQos); //Issue ack control
    }
}


static void CheckAck1Sub(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++) //Subs with QoS 1 scan
  {
    if(memory1Sub[i].uid == uid) //Uid check
    {
      if(memory1Sub[i].ack1 == 0) //Ack check
      {
        if(memory1Sub[i].send1 < 4) //Number of retransmission check
        {
          NS_LOG_UNCOND ("SUB 1 QoS 1: Packet " << uid << " not received in time, sending another");
          memory1Sub[i].send1++; //Incrementing variable to detect late arrivals
          float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
          //Issuing the retransmission
          Simulator::Schedule(Seconds(memory1Sub[i].send1 + r), &Retransmission1Sub, socket, dst, pktSize, counterInt, inputQos, uid);
        } else {
           NS_LOG_UNCOND ("Dropping packet " << uid << ", sent too many times");
           memory1Sub[i].send1++; //Dropping the packet
        }
      }
    }
  }
}


static void Retransmission1Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    Ptr<Packet> packet = Create<Packet>(packetSize); //Creating the packet
    QoSHeader qos; //Creating the QoS header
    qos.SetData(inputQos); //Adding the value to the header
    packet->AddHeader(qos); //Adding the header to the packet
    UidHeader id; //Same procedure for uid header
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for counter header
    counterInt = counterInt + 1; //Increment value to match send1 variable
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    socket->SendTo (packet, 0, dst); //Sending the packet
    for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
    {
      if(memory1Sub[i].uid == uid)
      {
        memory1Sub[i].latency1 = Simulator::Now().GetMicroSeconds(); //Adding stats measures
      }
    }
    retryQoS1++; //Incrementing retransmission QoS 1 number
    NS_LOG_UNCOND("SUB 1 QoS 1:" << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck1Sub, uid, socket, dst, pktSize, counterInt, inputQos); //Issue check for ack
    }
}


static void CheckAck21(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
  {
    if(memory2Pub[i].uid == uid) //Check for uid
    {
      if(memory2Pub[i].ack1 == 0) //Check for first ack (PUBREC)
      {
        NS_LOG_UNCOND ("PUBLISH: Packet " << uid << " not received in time, sending another");
        memory2Pub[i].send1++; //Increment value to detect late reception
        float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
        //Issue retransmission for the packet
        Simulator::Schedule(Seconds(memory2Pub[i].send1 + r), &Retransmission21, socket, dst, pktSize, counterInt, inputQos, uid);
      }
    }
  }
}


static void CheckAck23(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
  {
    if(memory2Pub[i].uid == uid) //Check for uid
    {
      if(memory2Pub[i].ack3 == 0) //Check for second ack (PUBCOMP)
      {
        NS_LOG_UNCOND ("PUBREL: Packet " << uid << " not received in time, sending another");
        memory2Pub[i].send3++; //Increment value to detect late reception
        float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
	//Issue retransmission for the packet
        Simulator::Schedule(Seconds(memory2Pub[i].send3 + r), &Retransmission23, socket, dst, pktSize, counterInt, inputQos, uid);
      }
    }
  }
}


static void Retransmission21(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    Ptr<Packet> packet = Create<Packet>(packetSize); //Create packet 
    QoSHeader exchange; //Create exchange header (only for QoS 2)
    exchange.SetData(0); //Input the correct exchange value
    packet->AddHeader(exchange); //Add the header to the packet
    QoSHeader qos; //Same procedure for the QoS header 
    qos.SetData(inputQos);
    packet->AddHeader(qos);
    UidHeader id; //Same procedure for the uid Header
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for the counter header
    counterInt = counterInt + 1; //Incrementing value to match send1 variable
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    QoSHeader pubOrSub; //Same procedure for header pub or sub (distinction between packet from subscribers or from publishers)
    pubOrSub.SetData(1);
    packet->AddHeader(pubOrSub);
    socket->SendTo (packet, 0, dst); //Sending the packet
    retryQoS2++; //Incrementing the QoS 2 retransmission number
    for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
    {
      if(memory2Pub[i].uid == uid)
      {
        memory2Pub[i].latency1 = Simulator::Now().GetMicroSeconds(); //Adding stats
      }
    }
    NS_LOG_UNCOND("PUBLISH " << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck21, uid, socket, dst, pktSize, counterInt, inputQos); //Issuing first ack control
    }
}


static void Retransmission23(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    Ptr<Packet> packet = Create<Packet>(packetSizeAck); //Create packet 
    QoSHeader exchange; //Create exchange header (only for QoS 2)
    exchange.SetData(1); //Input the correct exchange value
    packet->AddHeader(exchange); //Add the header to the packet  
    QoSHeader qos; //Same procedure for QoS header
    qos.SetData(inputQos);
    packet->AddHeader(qos);
    UidHeader id; //Same procedure for uid header
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for counter header
    counterInt = counterInt + 1;
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    QoSHeader pubOrSub; //Same procedure for pub or sub header
    pubOrSub.SetData(1);
    packet->AddHeader(pubOrSub); 
    socket->SendTo (packet, 0, dst); //Sending the packet
    retryQoS2++; //Increment QoS 2 retransmission
    for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
    {
      if(memory2Pub[i].uid == uid)
      {
        memory2Pub[i].latency3 = Simulator::Now().GetMicroSeconds(); //Adding statistics
      }
    }
    NS_LOG_UNCOND("PUBREL " << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck23, uid, socket, dst, pktSize, counterInt, inputQos); //Issue second ack check 
    }
}


static void CheckAck21Sub(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
  {
    if(memory2Sub[i].uid == uid) //Uid check
    {
      if(memory2Sub[i].ack1 == 0) //First ack check (PUBREC)
      {
        NS_LOG_UNCOND ("SUB 1 QoS 2: Packet " << uid << " not received in time, sending another");
        memory2Sub[i].send1++; //Increment to avoid late reception
        float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
	//Issuing retransmission
        Simulator::Schedule(Seconds(memory2Sub[i].send1 + r), &Retransmission21Sub, socket, dst, pktSize, counterInt, inputQos, uid);
      }
    }
  }
}


static void CheckAck23Sub(uint64_t uid, Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos)
{
  for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
  {
    if(memory2Sub[i].uid == uid) //Uid check
    {
      if(memory2Sub[i].ack3 == 0) //Second ack check (PUBCOMP)
      {
        NS_LOG_UNCOND ("SUB 3 QoS 2: Packet " << uid << " not received in time, sending another");
        memory2Sub[i].send3++; //Increment to avoid late reception
        float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.3));
	//Issuing retransmission
        Simulator::Schedule(Seconds(memory2Sub[i].send3 + r), &Retransmission23Sub, socket, dst, pktSize, counterInt, inputQos, uid);
      }
    }
  }
}



static void Retransmission21Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    Ptr<Packet> packet = Create<Packet>(packetSize); //Create packet
    QoSHeader exchange; //Create header exchange (only for QoS 2)
    exchange.SetData(0); //Input the correct exchange value
    packet->AddHeader(exchange); //Add the header to the packet
    QoSHeader qos; //Same procedure QoS header
    qos.SetData(inputQos);
    packet->AddHeader(qos);
    UidHeader id; //Same procedure for uid header
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for counter header 
    counterInt = counterInt + 1; //Increment value to match with send1 variable
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    socket->SendTo (packet, 0, dst); //Send the packet
    retryQoS2++; //Increment QoS 2 retransmissions
    for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
    {
      if(memory2Sub[i].uid == uid)
      {
        memory2Sub[i].latency1 = Simulator::Now().GetMicroSeconds(); //Add statistics
      }
    }
    NS_LOG_UNCOND("SUB 1 QoS 2: " << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck21Sub, uid, socket, dst, pktSize, counterInt, inputQos); //Issue first ack control 
    }
}

static void Retransmission23Sub(Ptr<Socket> socket,InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos, uint64_t uid)
{
    Ptr<Packet> packet = Create<Packet>(packetSizeAck); //Create packet 
    QoSHeader exchange; //Create header exchange (only for QoS 2)
    exchange.SetData(1); //Input the correct exchange value
    packet->AddHeader(exchange); //Add the header to the packet
    QoSHeader qos; //Same procedure for QoS header
    qos.SetData(inputQos);
    packet->AddHeader(qos);
    UidHeader id; //Same procedure for uid header
    id.SetData(uid);
    packet->AddHeader(id);
    CounterHeader counter; //Same procedure for counter header 
    counterInt = counterInt + 1; //Increment value to match with send1 variable
    counter.SetData(counterInt);
    packet->AddHeader(counter);
    socket->SendTo (packet, 0, dst); //Send the packet
    retryQoS2++; //Increment QoS 2 retransmission
    for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
    {
      if(memory2Sub[i].uid == uid)
      {
        memory2Sub[i].latency3 = Simulator::Now().GetMicroSeconds(); //Add statistics
      }
    }
    NS_LOG_UNCOND("SUB 3 QoS 2: " << uid << ": Retransmitting at time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos > 0)
    {
      Simulator::Schedule(Seconds(1), &CheckAck23Sub, uid, socket, dst, pktSize, counterInt, inputQos); //Issue second ack control
    }
}

void ReceivePacket (Ptr<Socket> socket) //Broker function
{
  Address from; //Address of receiver
  Ptr<Packet> packet = socket->RecvFrom(from); //Received packet
  QoSHeader pubOrSub; //Creating header to remove
  packet->RemoveHeader(pubOrSub); //Removing header from packet
  uint16_t corrupt = 0; //PER variable
  InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (from); //Address conversion
  //Setting up the PER
  if(iaddr.GetIpv4() == subs[0].addr)
  {
    corrupt = !d1(rng1);
  }
  if(iaddr.GetIpv4() == subs[1].addr)
  {
    corrupt = !d2(rng2);
  }
  if(iaddr.GetIpv4() == subs[2].addr)
  {
    corrupt = !d3(rng3);
  }
  if(iaddr.GetIpv4() == subs[3].addr)
  {
    if(Simulator::Now().GetSeconds() < 205)
    {
      corrupt = !d4(rng4);
    }
    else
    {
      corrupt = !d4(rng4);
    }
  }
  if(iaddr.GetIpv4() == subs[4].addr)
  {
    corrupt = !d5(rng5);
  }
  if(iaddr.GetIpv4() == subs[5].addr)
  {
    corrupt = !d6(rng6);
  }
  if(iaddr.GetIpv4() == subs[6].addr)
  {
    corrupt = !d7(rng7);
  }
  if(iaddr.GetIpv4() == subs[7].addr)
  {
    corrupt = !d8(rng8);
  }
  if(iaddr.GetIpv4() == subs[8].addr)
  {
    corrupt = !d9(rng9);
  }
  if(iaddr.GetIpv4() == subs[9].addr)
  {
    corrupt = !d10(rng10);
  }
  /*if(iaddr.GetIpv4() == subs[10].addr)
  {
    corrupt = !d11(rng11);
  }
  if(iaddr.GetIpv4() == subs[11].addr)
  {
    corrupt = !d12(rng12);
  }
  if(iaddr.GetIpv4() == subs[12].addr)
  {
    corrupt = !d13(rng13);
  }
  if(iaddr.GetIpv4() == subs[13].addr)
  {
    corrupt = !d14(rng14);
  }
  if(iaddr.GetIpv4() == subs[14].addr)
  {
    corrupt = !d15(rng15);
  }
  if(iaddr.GetIpv4() == subs[15].addr)
  {
    corrupt = !d16(rng16);
  }
  if(iaddr.GetIpv4() == subs[16].addr)
  {
    corrupt = !d17(rng17);
  }
  if(iaddr.GetIpv4() == subs[17].addr)
  {
    corrupt = !d18(rng18);
  }
  if(iaddr.GetIpv4() == subs[18].addr)
  {
    corrupt = !d19(rng19);
  }
  if(iaddr.GetIpv4() == subs[19].addr)
  {
    corrupt = !d20(rng20);
  }*/

  if(!corrupt) //Correct reception
  {
  if(pubOrSub.GetData() == 1) //First header check, packet from publisher
  {
    CounterHeader counterHeader; //Removing counter header procedure
    packet->RemoveHeader(counterHeader);
    UidHeader idHeader;
    packet->RemoveHeader(idHeader); //Removing id header procedure
    QoSHeader qos;
    packet->RemoveHeader(qos); //Removing QoS header procedure
    if(qos.GetData() == 0)
    {
      /*counterControllerSub++;
      if(counterControllerSub == 60)
      {
        uint64_t subs = ControllerSub(QoS, qos.GetData());
      }*/
      NS_LOG_UNCOND ("PUBLISH " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
      arrivedPacketsPubQoS0++; //Update statistics
      for(int i = 0; i < sizeof(memory0Pub)/sizeof(memory0Pub[0]); i++)
      {
        if(idHeader.GetData() == memory0Pub[i].uid) //Scanning for the right packet
        {
          memory0Pub[i].arrivalTime = Simulator::Now().GetMicroSeconds(); //Update statistics
        }
      }
      for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++) //Sending a packet to each sub
      {
        Ptr<Packet> sub = Create<Packet> (packetSize); //Packet creation
        QoSHeader subQoS; //QoS header procedure
        subQoS.SetData(subs[j].chosenQoS);
        InetSocketAddress subTarget = InetSocketAddress (subs[j].addr, 10); //Target address initialization
        if(subs[j].chosenQoS == 0) //Publisher with QoS = 0
        {
          sub->AddHeader(subQoS);      
          UidHeader uidHeaderSub; //Uid header procedure
          uidHeaderSub.SetData(overIndex);
          sub->AddHeader(uidHeaderSub);
          CounterHeader counterHeaderSub; //Counter header procedure
          counterHeaderSub.SetData(0);
          sub->AddHeader(counterHeaderSub);
          socket->SendTo (sub, 0, subTarget); //Sending packet
          NS_LOG_UNCOND ("SUB 1 QoS 0: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
          << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
          launchedPacketsSubQoS0++;
          memory0Sub[k0Sub].uid = overIndex; //Inserting values into packet array for sub QoS 0
          memory0Sub[k0Sub].startQoS = 0;
          memory0Sub[k0Sub].addr = subs[j].addr;
          overIndex++; //Incrementing overall packet index
          for(int k = 0; k < sizeof(memory0Pub)/sizeof(memory0Pub[0]); k++)
          {
            if(memory0Pub[k].uid == idHeader.GetData())
            {
              memory0Sub[k0Sub].startingTime = memory0Pub[k].startingTime; //Adding statistics
            }
          }
          k0Sub++; //Incrementing array index
        }
        if(subs[j].chosenQoS == 1)
        {
          memory1Sub[k1Sub].uid = overIndex; //Inserting values into packet array for sub QoS 1
          memory1Sub[k1Sub].send1 = 0;
          memory1Sub[k1Sub].startQoS = 0;
          memory1Sub[k1Sub].addr = subs[j].addr;
          memory1Sub[k1Sub].latency1 = Simulator::Now().GetMicroSeconds();
          for(int k = 0; k < sizeof(memory0Pub)/sizeof(memory0Pub[0]); k++)
          {
            if(memory0Pub[k].uid == idHeader.GetData())
            {
              memory1Sub[k1Sub].startingTime = memory0Pub[k].startingTime; //Adding stats
            }
          }
          k1Sub++; //Incrementing array index
          sub->AddHeader(subQoS); //QoS header procedure
          UidHeader uidHeaderSub;
          uidHeaderSub.SetData(overIndex); //Uid header procedure
          sub->AddHeader(uidHeaderSub);
          CounterHeader counterHeaderSub; //Counter header procedure
          counterHeaderSub.SetData(0);
          sub->AddHeader(counterHeaderSub);
          socket->SendTo (sub, 0, subTarget); //Sending packet
          launchedPacketsSubQoS1++; //Update stats
          NS_LOG_UNCOND ("SUB 1 QoS 1: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
          << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
          Simulator::Schedule(Seconds(1), &CheckAck1Sub, overIndex, socket, subTarget, 1024, 0, subs[j].chosenQoS); //Scheduling ack check
          overIndex++; //Incrementing overall packet index
         }
        if(subs[j].chosenQoS == 2)
        {
          memory2Sub[k2Sub].uid = overIndex; //Inserting values into packet array for sub QoS 2
          memory2Sub[k2Sub].send1 = 0;
          memory2Sub[k2Sub].latency1 = Simulator::Now().GetMicroSeconds();
          memory2Sub[k2Sub].startQoS = 0;
          memory2Sub[k2Sub].addr = subs[j].addr;
          for(int k = 0; k < sizeof(memory0Pub)/sizeof(memory0Pub[0]); k++)
          {
            if(memory0Pub[k].uid == idHeader.GetData())
            {
              memory2Sub[k2Sub].startingTime = memory0Pub[k].startingTime; //Update stats
            }
          }
          k2Sub++; //Incrementing array index
          QoSHeader exchangeHeaderSub; //QoS header procedure
          exchangeHeaderSub.SetData(0); //Exchange header procedure
          sub->AddHeader(exchangeHeaderSub);
          sub->AddHeader(subQoS);
          UidHeader uidHeaderSub; //Uid header procedure
          uidHeaderSub.SetData(overIndex);
          sub->AddHeader(uidHeaderSub);
          CounterHeader counterHeaderSub; //Counter header procedure
          counterHeaderSub.SetData(0);
          sub->AddHeader(counterHeaderSub);
          socket->SendTo(sub, 0, subTarget); //Sending packet
	  launchedPacketsSubQoS2++; //Update stats
          NS_LOG_UNCOND ("SUB 1 QoS 2: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
          << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort()); 
          Simulator::Schedule(Seconds(1), &CheckAck21Sub, overIndex, socket, subTarget, 1024, 0, subs[j].chosenQoS); //Scheduling ack check
          overIndex++; //Incrementing overall index
        }
      }
    }
    if(qos.GetData() == 1) //Publisher with QoS = 1
    {
      NS_LOG_UNCOND ("PUBLISH " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
      for(int i = 0; i < sizeof(memory1Pub)/sizeof(memory1Pub[0]); i++)
      {
        if(memory1Pub[i].uid == idHeader.GetData()) //Uid check
        {
          memory1Pub[i].latency1 = Simulator::Now().GetMicroSeconds() - memory1Pub[i].latency1; //Stats updates
          memory1Pub[i].latency2 = Simulator::Now().GetMicroSeconds();
          halfPacketsPubQoS1++;
          Ptr<Packet> ack = Create<Packet> (packetSizeAck); //Ack creation
          QoSHeader qosHeader; //QoS header procedure
          qosHeader.SetData(qos.GetData());
          ack->AddHeader(qosHeader);
          UidHeader id; //Uid header procedure
          id.SetData(idHeader.GetData());
          ack->AddHeader(idHeader);
          CounterHeader counter; //Counter header procedure
          counter.SetData(counterHeader.GetData());
          ack->AddHeader(counter);
          socket->SendTo (ack, 0, from); //Sending ack back
          NS_LOG_UNCOND ("PUBACK " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
          << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
            for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++) //Sending packets to subscribers, same procedure as before
            {
              Ptr<Packet> sub = Create<Packet> (packetSize);
              QoSHeader subQoS;
              subQoS.SetData(subs[j].chosenQoS);
              sub->AddHeader(subQoS);
              InetSocketAddress subTarget = InetSocketAddress (subs[j].addr, 10);
              if(subs[j].chosenQoS == 0)
              {
                UidHeader uidHeaderSub;
                uidHeaderSub.SetData(overIndex);
                sub->AddHeader(uidHeaderSub);
                CounterHeader counterHeaderSub;
                counterHeaderSub.SetData(0);
                sub->AddHeader(counterHeaderSub);
                socket->SendTo (sub, 0, subTarget);
                NS_LOG_UNCOND ("SUB 1 QoS 0: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                launchedPacketsSubQoS0++;
                memory0Sub[k0Sub].uid = overIndex;
                memory0Sub[k0Sub].addr = subs[j].addr;
                overIndex++;
                memory0Sub[k0Sub].startingTime = memory1Pub[i].startingTime;
                memory0Sub[k0Sub].startQoS = 1;
		if(memory1Pub[i].sent == 0)
                {
                  memory0Sub[k0Sub].sent = 1;
                }
                k0Sub++;
              }
              if(subs[j].chosenQoS == 1)
              {
                memory1Sub[k1Sub].uid = overIndex;
                memory1Sub[k1Sub].send1 = 0;
                memory1Sub[k1Sub].latency1 = Simulator::Now().GetMicroSeconds();
                memory1Sub[k1Sub].startingTime = memory1Pub[i].startingTime;
                memory1Sub[k1Sub].startQoS = 1;
                memory1Sub[k1Sub].addr = subs[j].addr;
                if(memory1Pub[i].sent == 0)
		{
		  memory1Sub[k1Sub].sent = 1;
		}
                k1Sub++;
                UidHeader uidHeaderSub;
                uidHeaderSub.SetData(overIndex);
                sub->AddHeader(uidHeaderSub);
                CounterHeader counterHeaderSub;
                counterHeaderSub.SetData(0);
                sub->AddHeader(counterHeaderSub);
                socket->SendTo (sub, 0, subTarget);
                launchedPacketsSubQoS1++;
                NS_LOG_UNCOND ("SUB 1 QoS 1: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                Simulator::Schedule(Seconds(1), &CheckAck1Sub, overIndex, socket, subTarget, 1024, 0, subs[j].chosenQoS);
                overIndex++;
              }
              if(subs[j].chosenQoS == 2)
              {
                memory2Sub[k2Sub].uid = overIndex;
                memory2Sub[k2Sub].send1 = 0;
                memory2Sub[k2Sub].addr = subs[j].addr;
                memory2Sub[k2Sub].latency1 = Simulator::Now().GetMicroSeconds();
                memory2Sub[k2Sub].startingTime = memory1Pub[i].startingTime;
                memory2Sub[k2Sub].startQoS = 1;
                if(memory1Pub[i].sent == 0)
                {
                  memory2Sub[k2Sub].sent = 1;
                }
                k2Sub++;
                QoSHeader exchangeHeaderSub;
                exchangeHeaderSub.SetData(0);
                sub->AddHeader(exchangeHeaderSub);
                sub->AddHeader(subQoS);
                UidHeader uidHeaderSub;
                uidHeaderSub.SetData(overIndex);
                sub->AddHeader(uidHeaderSub);
                CounterHeader counterHeaderSub;
                counterHeaderSub.SetData(0);
                sub->AddHeader(counterHeaderSub);
                socket->SendTo(sub, 0, subTarget);
		launchedPacketsSubQoS2++;
                NS_LOG_UNCOND ("SUB 1 QoS 2: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                Simulator::Schedule(Seconds(1), &CheckAck21Sub, overIndex, socket, subTarget, 1024, 0, subs[j].chosenQoS);
                overIndex++;
              }
            }
            memory1Pub[i].sent = 1;
            uint64_t counterInt = counterHeader.GetData();
            uint64_t idInt = idHeader.GetData();
            InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());          }
        }
    }
    if(qos.GetData() == 2) //Publisher with QoS = 2
    {
      QoSHeader exchange; //Peeking exchange header
      packet->PeekHeader(exchange);
      if(exchange.GetData() == 0) //First handshake scenario
      {
        counterControllerSub++; //Not used
        NS_LOG_UNCOND ("PUBLISH " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
        << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
        for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
        {
          if(memory2Pub[i].uid == idHeader.GetData()) //Checking for the right packet
          {
              memory2Pub[i].latency1 = Simulator::Now().GetMicroSeconds() - memory2Pub[i].latency1; //Update stats
              memory2Pub[i].latency2 = Simulator::Now().GetMicroSeconds();
              Ptr<Packet> ack = Create<Packet> (packetSizeAck); //Creating ack (PUBREC)
              QoSHeader exchangeSend; //Exchange header procedure
              exchangeSend.SetData(0);
              ack->AddHeader(exchangeSend);
              QoSHeader qosHeader; //QoS header procedure
              qosHeader.SetData(qos.GetData());
              ack->AddHeader(qosHeader);
              UidHeader id; //Uid header procedure
              id.SetData(packet->GetUid());
              ack->AddHeader(idHeader);
              CounterHeader counter; //Counter header procedure
              counter.SetData(counterHeader.GetData());
              ack->AddHeader(counter);
              socket->SendTo (ack, 0, from); //Sending the ack
              NS_LOG_UNCOND ("PUBREC " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
              << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
              uint64_t idInt = idHeader.GetData();
              uint64_t counterInt = counter.GetData();
              InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());
	      if(memory2Pub[i].sent == 0) //Checking if message has already been sent
	      {
                memory2Pub[i].sent = 1; //If not setting variable up and then initiate send procedure
	        for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++) //Sending packets to subscribers, same procedure as before
                {
                  int temp = j;
                  if((subReverse % 2) == 1)
                  {
                    temp = sizeof(subs)/sizeof(subs[0]) - 1 - j;
                  }
                  Ptr<Packet> sub = Create<Packet> (packetSize);
                  QoSHeader subQoS;
                  subQoS.SetData(subs[temp].chosenQoS);
                  sub->AddHeader(subQoS);
                  InetSocketAddress subTarget = InetSocketAddress (subs[temp].addr, 10);
                  if(subs[temp].chosenQoS == 0)
                  {
                    UidHeader uidHeaderSub;
                    uidHeaderSub.SetData(overIndex);
                    sub->AddHeader(uidHeaderSub);
                    CounterHeader counterHeaderSub;
                    counterHeaderSub.SetData(0);
                    sub->AddHeader(counterHeaderSub);
                    socket->SendTo (sub, 0, subTarget);
                    NS_LOG_UNCOND ("SUB 1 QoS 0:  " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                    << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                    launchedPacketsSubQoS0++;
                    memory0Sub[k0Sub].uid = overIndex;
                    memory0Sub[k0Sub].startQoS = 2;
                    memory0Sub[k0Sub].addr = subs[temp].addr;
                    overIndex++;
                    memory0Sub[k0Sub].startingTime = memory2Pub[i].startingTime;
                    k0Sub++;
                  }
                  if(subs[temp].chosenQoS == 1)
                  {
                    memory1Sub[k1Sub].uid = overIndex;
                    memory1Sub[k1Sub].send1 = 0;
                    memory1Sub[k1Sub].latency1 = Simulator::Now().GetMicroSeconds();
                    memory1Sub[k1Sub].startingTime = memory2Pub[i].startingTime;
                    memory1Sub[k1Sub].addr = subs[temp].addr;
                    memory1Sub[k1Sub].startQoS = 2;
                    k1Sub++;
                    UidHeader uidHeaderSub;
                    uidHeaderSub.SetData(overIndex);
                    sub->AddHeader(uidHeaderSub);
                    CounterHeader counterHeaderSub;
                    counterHeaderSub.SetData(0);
                    sub->AddHeader(counterHeaderSub);
                    socket->SendTo (sub, 0, subTarget);
                    launchedPacketsSubQoS1++;
                    NS_LOG_UNCOND ("SUB 1 QoS 1: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                    << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                    Simulator::Schedule(Seconds(1), &CheckAck1Sub, overIndex, socket, subTarget, 1024, 0, subs[temp].chosenQoS);
                    overIndex++;
                  }
                  if(subs[temp].chosenQoS == 2)
                  {
                    memory2Sub[k2Sub].uid = overIndex;
                    memory2Sub[k2Sub].send1 = 0;
                    memory2Sub[k2Sub].latency1 = Simulator::Now().GetMicroSeconds();
                    memory2Sub[k2Sub].startingTime = memory2Pub[i].startingTime;
                    memory2Sub[k2Sub].addr = subs[temp].addr;
                    memory2Sub[k2Sub].startQoS = 2;
                    k2Sub++;
		    launchedPacketsSubQoS2++;
                    QoSHeader exchangeHeaderSub;
                    exchangeHeaderSub.SetData(0);
                    sub->AddHeader(exchangeHeaderSub);
                    sub->AddHeader(subQoS);
                    UidHeader uidHeaderSub;
                    uidHeaderSub.SetData(overIndex);
                    sub->AddHeader(uidHeaderSub);
                    CounterHeader counterHeaderSub;
                    counterHeaderSub.SetData(0);
                    sub->AddHeader(counterHeaderSub);
                    socket->SendTo(sub, 0, subTarget);
                    NS_LOG_UNCOND ("SUB 1 QoS 2: " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
                    << InetSocketAddress::ConvertFrom(subTarget).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(subTarget).GetPort());
                    Simulator::Schedule(Seconds(1), &CheckAck21Sub, overIndex, socket, subTarget, 1024, 0, subs[temp].chosenQoS);
                    overIndex++;
                  }
                }
	      }
            }
        }
        subReverse++;
      }
      if(exchange.GetData() == 1) //Second handshake scenario
      {
        NS_LOG_UNCOND ("PUBREL " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
        << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
        for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
        {
          if(memory2Pub[i].uid == idHeader.GetData())
          {
              memory2Pub[i].latency3 = Simulator::Now().GetMicroSeconds() - memory2Pub[i].latency3; //Update stats
              memory2Pub[i].latency4 = Simulator::Now().GetMicroSeconds();
              Ptr<Packet> ack = Create<Packet> (packetSizeAck); //Create ack (PUBCOMP)
              QoSHeader exchangeSend; //Exchange header procedure
              exchangeSend.SetData(1);
              ack->AddHeader(exchangeSend);
              QoSHeader qosHeader; //QoS header procedure
              qosHeader.SetData(qos.GetData());
              ack->AddHeader(qosHeader);
              ack->AddHeader(idHeader); //Id header procedure
              CounterHeader counter; //Counter header procedure
              counter.SetData(counterHeader.GetData());
              ack->AddHeader(counter);
              socket->SendTo (ack, 0, from); //Sending ack
              NS_LOG_UNCOND ("PUBCOMP " << idHeader.GetData() << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
              << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
              uint64_t idInt = idHeader.GetData(); //Useless
              uint64_t counterInt = counter.GetData(); //Useless
            }         
        }     
      }
    }
  }
  if(pubOrSub.GetData() == 2) //Packet received from subscribers
  {
   UidHeader uidHeaderSub; //Uid header procedure
   packet->RemoveHeader(uidHeaderSub);
   CounterHeader counterHeaderSub; //Counter header procedure
   packet->RemoveHeader(counterHeaderSub);
   QoSHeader qosHeaderSub; //QoS header procedure
   packet->RemoveHeader(qosHeaderSub);
   if(qosHeaderSub.GetData() == 1) //Checking QoS 1 ack
   {
     NS_LOG_UNCOND ("SUB 2 QoS 1: "<< uidHeaderSub.GetData() <<" : At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
     << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
     for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
     {
       if(memory1Sub[i].uid == uidHeaderSub.GetData()) //Searching the right packet
       {
         if(memory1Sub[i].send1 == counterHeaderSub.GetData()) //Checking if counter correspond to variable send1
         { 
           memory1Sub[i].ack1 = 1; //Setting the ack as received
           memory1Sub[i].latency2 = Simulator::Now().GetMicroSeconds() - memory1Sub[i].latency2; //Update stats
           memory1Sub[i].arrivalTime = Simulator::Now().GetMicroSeconds(); 
         }
       }
     }
   }
   if(qosHeaderSub.GetData() == 2) //Checking QoS 2 ack
   {
     QoSHeader exchangeHeaderSub; //Exchange header procedure
     packet->RemoveHeader(exchangeHeaderSub);
     if(exchangeHeaderSub.GetData() == 0) //First exchange QoS 2
     {
     NS_LOG_UNCOND ("SUB 2 QoS 2: "<< uidHeaderSub.GetData() <<" : At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
     << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
       for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
       {
         if(memory2Sub[i].uid == uidHeaderSub.GetData()) //Searching the right packet
         {
           if(memory2Sub[i].send1 == counterHeaderSub.GetData()) //Checking if counter correspond to variable send1
           {
             memory2Sub[i].ack1 = 1; //Setting the ack1 (PUBREC) as received
             memory2Sub[i].send3 = 0; //Initializing the various fields
             memory2Sub[i].ack3 = 0;
             memory2Sub[i].latency2 = Simulator::Now().GetMicroSeconds() - memory2Sub[i].latency2; //Update stats
             memory2Sub[i].latency3 = Simulator::Now().GetMicroSeconds();
             Ptr<Packet> ack = Create<Packet> (packetSizeAck); //Create ack
             QoSHeader exchangeHeaderSubSend; //Exchange header procedure
             exchangeHeaderSubSend.SetData(1);
             ack->AddHeader(exchangeHeaderSubSend);
             ack->AddHeader(qosHeaderSub); //QoS header procedure
             ack->AddHeader(uidHeaderSub); //Uid header procedure
             counterHeaderSub.SetData(0);
             ack->AddHeader(counterHeaderSub);
             socket->SendTo (ack, 0, from); //Sending packet
             NS_LOG_UNCOND ("SUB 3 QoS 2: " << memory2Sub[i].uid << ": At time " << Simulator::Now().As (Time::S) << " server sent 1024 bytes to "
             << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
             uint64_t idInt = uidHeaderSub.GetData();
             uint64_t counterInt = counterHeaderSub.GetData();
             InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());
             Simulator::Schedule(Seconds(1), &CheckAck23Sub, idInt, socket, addr, 1024, 0, qosHeaderSub.GetData()); //Issue ack3 check
           }
         }
       }
     }
     if(exchangeHeaderSub.GetData() == 1) //Second exchange QoS 2
     {
     NS_LOG_UNCOND ("SUB 4 QoS 2: "<< uidHeaderSub.GetData() <<" : At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
     << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
       for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
       {
         if(memory2Sub[i].uid == uidHeaderSub.GetData()) //Searching the right packet
         {
           if(memory2Sub[i].send3 == counterHeaderSub.GetData()) //Checking if counter correspond to variable send3
           { 
             memory2Sub[i].ack3 = 1; //Setting the ack3 (PUBCOMP) as received
             memory2Sub[i].latency4 = Simulator::Now().GetMicroSeconds() - memory2Sub[i].latency4; //Update stats
             memory2Sub[i].arrivalTime = Simulator::Now().GetMicroSeconds();
           }
         }
        }
      }
    }
  }
  if(pubOrSub.GetData() == 3) //Interference packet
  {
    NS_LOG_UNCOND ("INTERFERENCE: At time " << Simulator::Now().As (Time::S) << " server received 1024 bytes from "
    << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
    interfCounter++; //Update interference stats
  } 
}
} 
void ReceivePacketClient (Ptr<Socket> socket) //Publisher function
{
  Address from; //Sender address
  Ptr<Packet> packet = socket->RecvFrom(from); //Received packet
  CounterHeader counter; //Counter header procedure
  packet->RemoveHeader(counter);
  UidHeader id; //Uid header procedure
  packet->RemoveHeader(id); 
  QoSHeader qos; //QoS header procedure
  packet->RemoveHeader(qos);
  if(qos.GetData() == 1) //Ack for QoS 1
  {
    NS_LOG_UNCOND ("PUBACK " << id.GetData() << ": At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
    << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
    for(int i = 0; i < sizeof(memory1Pub)/sizeof(memory1Pub[0]); i++)
    {
      if(memory1Pub[i].uid == id.GetData()) //Searching right packet
      {
        if(memory1Pub[i].send1 == counter.GetData()) //Checking retransmission counter
        {
          memory1Pub[i].ack1 = 1; //Setting ack
          arrivedPacketsPubQoS1++; //Update stats
          memory1Pub[i].latency2 = Simulator::Now().GetMicroSeconds() - memory1Pub[i].latency2;
        }
      }
    }
  }
  if(qos.GetData() == 2) //Ack for QoS 2
  {
    QoSHeader exchange; //Exchange header procedure
    packet->PeekHeader(exchange);
    if(exchange.GetData() == 0) //First exchange QoS 2
    {
      NS_LOG_UNCOND ("PUBREC " << id.GetData() << ": At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
      for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
      {
        if(memory2Pub[i].uid == id.GetData()) //Searching the right packet
        {
          if(memory2Pub[i].send1 == counter.GetData()) //Checking retransmission counter
          {
            memory2Pub[i].ack1 = 1; //Setting ack1 (PUBREC)
            memory2Pub[i].ack3 = 0; //Initializing values
	    memory2Pub[i].send3 = 0;
            memory2Pub[i].latency2 = Simulator::Now().GetMicroSeconds() - memory2Pub[i].latency2;
            memory2Pub[i].latency3 = Simulator::Now().GetMicroSeconds();
            Ptr<Packet> ack = Create<Packet> (packetSizeAck); //Create ack (PUBREL) 
            QoSHeader exchangeSend; //Exchange header procedure
            exchangeSend.SetData(1);
            ack->AddHeader(exchangeSend);
            QoSHeader qosHeader; //QoS header procedure
            qosHeader.SetData(qos.GetData());
            ack->AddHeader(qosHeader);
            UidHeader idHeader; //Uid header procedure
            idHeader.SetData(memory2Pub[i].uid);
            ack->AddHeader(idHeader);
            CounterHeader counterHeader; //Counter header procedure
            counterHeader.SetData(0);
            ack->AddHeader(counterHeader);
            QoSHeader pubOrSub; //PubOrSub header procedure
            pubOrSub.SetData(1);
            ack->AddHeader(pubOrSub);
            socket->SendTo (ack, 0, from); //Sending ack
            NS_LOG_UNCOND ("PUBREL " << memory2Pub[i].uid << ": At time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
            << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
            uint64_t idInt = idHeader.GetData();
            uint64_t counterInt = counterHeader.GetData();
            InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());
            Simulator::Schedule(Seconds(1), &CheckAck23, idInt, socket, addr, 1024, 0, qos.GetData()); //Issuing ack3 check
          }
        }
      }
    }
    if(exchange.GetData() == 1) //Second exchange QoS 2
    {
      NS_LOG_UNCOND ("PUBCOMP " << id.GetData() << ": At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());      
      for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
      {
        if(memory2Pub[i].uid == id.GetData()) //Right packet
        {
          if(memory2Pub[i].send3 == counter.GetData()) //Checking retransmission counter
          {
            memory2Pub[i].ack3 = 1; //Setting ack3 (PUBCOMP)
            memory2Pub[i].latency4 = Simulator::Now().GetMicroSeconds() - memory2Pub[i].latency4; //Update stats
	    arrivedPacketsPubQoS2++;
          }
        }
      }
    }
  }
}

void ReceivePacketSubscription (Ptr<Socket> socket) //Subscriber function
{
  Address from; //Receiver address 
  Ptr<Packet> packet = socket->RecvFrom(from); //Received packet
  CounterHeader counterHeaderSub; //Counter header procedure
  packet->RemoveHeader(counterHeaderSub);
  UidHeader uidHeaderSub; //Uid header procedure
  packet->RemoveHeader(uidHeaderSub);
  QoSHeader subQoS; //QoS header procedure
  packet->RemoveHeader(subQoS);
  uint16_t corrupt = 0;
  Address addr2; //Initializing an address and putting the address of the socket inside it
  socket->GetSockName (addr2); 
  //Packet Error Rate section
  InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr2);
  
  if(iaddr.GetIpv4() == subs[0].addr)
  {
    corrupt = !d1(rng1);
  }
  if(iaddr.GetIpv4() == subs[1].addr)
  {
    corrupt = !d2(rng2);
  }
  if(iaddr.GetIpv4() == subs[2].addr)
  {
    corrupt = !d3(rng3);
  }
  if(iaddr.GetIpv4() == subs[3].addr)
  {
    if(Simulator::Now().GetSeconds() < 205)
    {
      corrupt = !d4(rng4);
    }
    else
    {
      corrupt = !d4(rng4);
    }
  }
  if(iaddr.GetIpv4() == subs[4].addr)
  {
    corrupt = !d5(rng5);
  }
  if(iaddr.GetIpv4() == subs[5].addr)
  {
    corrupt = !d6(rng6);
  }
  if(iaddr.GetIpv4() == subs[6].addr)
  {
    corrupt = !d7(rng7);
  }
  if(iaddr.GetIpv4() == subs[7].addr)
  {
    corrupt = !d8(rng8);
  }
  if(iaddr.GetIpv4() == subs[8].addr)
  {
    corrupt = !d9(rng9);
  }
  if(iaddr.GetIpv4() == subs[9].addr)
  {
    corrupt = !d10(rng10);
  }
  /*if(iaddr.GetIpv4() == subs[10].addr)
  {
    corrupt = !d11(rng11);
  }
  if(iaddr.GetIpv4() == subs[11].addr)
  {
    corrupt = !d12(rng12);
  }
  if(iaddr.GetIpv4() == subs[12].addr)
  {
    corrupt = !d13(rng13);
  }
  if(iaddr.GetIpv4() == subs[13].addr)
  {
    corrupt = !d14(rng14);
  }
  if(iaddr.GetIpv4() == subs[14].addr)
  {
    corrupt = !d15(rng15);
  }
  if(iaddr.GetIpv4() == subs[15].addr)
  {
    corrupt = !d16(rng16);
  }
  if(iaddr.GetIpv4() == subs[16].addr)
  {
    corrupt = !d17(rng17);
  }
  if(iaddr.GetIpv4() == subs[17].addr)
  {
    corrupt = !d18(rng18);
  }
  if(iaddr.GetIpv4() == subs[18].addr)
  {
    corrupt = !d19(rng19);
  }
  if(iaddr.GetIpv4() == subs[19].addr)
  {
    corrupt = !d20(rng20);
  }*/
  
  if(!corrupt)
  {
  if(subQoS.GetData() == 0) //QoS = 0
  {
    NS_LOG_UNCOND ("SUB 1 QoS 0: "<< uidHeaderSub.GetData() <<" : At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
    << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
    for(int i = 0; i < sizeof(memory0Sub)/sizeof(memory0Sub[0]); i++)
    {
      if(uidHeaderSub.GetData() == memory0Sub[i].uid) //Searching right packet
      {
        memory0Sub[i].arrivalTime = Simulator::Now().GetMicroSeconds(); //Update stats
        //Pub qos 0
        if(memory0Sub[i].startQoS == 0)
        { 
          arrivedPacketsSubQoS0PubQoS0++; //Update stats
        }
        //Pub qos 2
        if(memory0Sub[i].startQoS == 2)
        { 
          arrivedPacketsSubQoS0PubQoS2++; //Update stats
        }
        //Pub qos 1
	if(memory0Sub[i].startQoS == 1)
        {
          for(int j = 0; j < sizeof(help0)/sizeof(help0[0]); j++)
          {
            if(memory0Sub[i].startingTime == help0[j].startingTime && memory0Sub[i].addr == help0[j].addr)
            {
              if(help0[j].bol == 0)
              {
                help0[j].arrivalTime = Simulator::Now().GetMicroSeconds(); //Update stats using auxiliary memory for pub with QoS 1
              }
              help0[j].bol = 1;
	    }
          }
        }
      } 
    }
  }
  if(subQoS.GetData() == 1) //QoS = 1
  {
    NS_LOG_UNCOND ("SUB 1 QoS 1: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
    << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
    Ptr<Node> node = socket->GetNode();
    Ptr<Ipv4> nodeAddrRaw = node->GetObject<Ipv4>(); //Address of the node
    for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
    {
      if(memory1Sub[i].uid == uidHeaderSub.GetData())
      {
          memory1Sub[i].latency1 = Simulator::Now().GetMicroSeconds() - memory1Sub[i].latency1; //Update stats
          memory1Sub[i].latency2 = Simulator::Now().GetMicroSeconds(); 
	  memory1Sub[i].subRecTime = Simulator::Now().GetMicroSeconds();
          halfPacketsSubQoS1++;
          //Pub qos 0
          if(memory1Sub[i].startQoS == 0 && memory1Sub[i].retry == 0)
          { 
            arrivedPacketsSubQoS1PubQoS0++; //Update stats
          }
          //Pub qos 2
          if(memory1Sub[i].startQoS == 2 && memory1Sub[i].retry == 0)
          { 
            memory1Sub[i].retry = 1;
            arrivedPacketsSubQoS1PubQoS2++; //Update stats
          }
          //Pub qos 1
          if(memory1Sub[i].startQoS == 1 && memory1Sub[i].retry == 0)
          {
            memory1Sub[i].retry = 1;
            for(int j = 0; j < sizeof(help1)/sizeof(help1[0]); j++)
            {  
              if(memory1Sub[i].startingTime == help1[j].startingTime && memory1Sub[i].addr == help1[j].addr)
              { 
                if(help1[j].bol == 0)
                {
                  help1[j].arrivalTime = Simulator::Now().GetMicroSeconds(); //Update stats using auxiliary memory for pub with QoS 1
                }
                help1[j].bol = 1;
              }
            }
          }
          Ptr<Packet> ack = Create<Packet>(packetSizeAck); //Create ack
          QoSHeader pubOrSub; //PubOrSub header procedure
          pubOrSub.SetData(2);
          ack->AddHeader(subQoS); //QoS header procedure 
          ack->AddHeader(counterHeaderSub); //Counter header procedure
          ack->AddHeader(uidHeaderSub); //Uid header procedure
          ack->AddHeader(pubOrSub);
          socket->SendTo (ack, 0, from); //Sending ack
          NS_LOG_UNCOND("SUB 2 QoS 1: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
          << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
          InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());
      }
    } 
  }
  if(subQoS.GetData() == 2) //QoS 2
  {
    QoSHeader exchangeHeaderSub; //Exchange header procedure
    packet->PeekHeader(exchangeHeaderSub);
    if(exchangeHeaderSub.GetData() == 0) //First exchange QoS 2
    {
      NS_LOG_UNCOND ("SUB 1 QoS 2: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
      for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
      {
        if(memory2Sub[i].uid == uidHeaderSub.GetData()) //Searching right packet
        {
            memory2Sub[i].latency1 = Simulator::Now().GetMicroSeconds() - memory2Sub[i].latency1; //Update stats
            memory2Sub[i].latency2 = Simulator::Now().GetMicroSeconds();
	    memory2Sub[i].subRecTime = Simulator::Now().GetMicroSeconds();
            if(memory2Sub[i].startQoS == 0 && memory2Sub[i].retry == 0)
            { 
	      memory2Sub[i].retry = 1; //Unused
              arrivedPacketsSubQoS2PubQoS0++; //Update stats
            }
            if(memory2Sub[i].startQoS == 1 && memory2Sub[i].retry == 0)
            { 
	      memory2Sub[i].retry = 1; //Unused
              arrivedPacketsSubQoS2PubQoS1++; //Update stats
            }
            if(memory2Sub[i].startQoS == 2 && memory2Sub[i].retry == 0)
            { 
	      memory2Sub[i].retry = 1; //Unused
              arrivedPacketsSubQoS2PubQoS2++; //Update stats
            }
            Ptr<Packet> ack = Create<Packet>(packetSizeAck); //Ack creation (PUBREC)
            QoSHeader pubOrSub; //pubOrSub header procedure
            pubOrSub.SetData(2);
            ack->AddHeader(exchangeHeaderSub); //Exchange header procedure
            ack->AddHeader(subQoS); //QoS header procedure
            ack->AddHeader(counterHeaderSub); //Counter header procedure
            ack->AddHeader(uidHeaderSub); //Uid header procedure
            ack->AddHeader(pubOrSub); //PubOrSub header procedure
            socket->SendTo (ack, 0, from); //Sending the packet
            NS_LOG_UNCOND("SUB 2 QoS 2: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
            << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
            InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort());
          } 
        }
      }
    if(exchangeHeaderSub.GetData() == 1) //Second exchange QoS 2
    {
      NS_LOG_UNCOND ("SUB 3 QoS 2: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client received 1024 bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
      for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
      {
        if(memory2Sub[i].uid == uidHeaderSub.GetData()) //Searching right packet
        {
            memory2Sub[i].latency3 = Simulator::Now().GetMicroSeconds() - memory2Sub[i].latency3; //Update stats
            memory2Sub[i].latency4 = Simulator::Now().GetMicroSeconds();
            Ptr<Packet> ack = Create<Packet>(packetSizeAck); //Create ack3 (PUBCOMP)
            QoSHeader pubOrSub; //PubOrSub header procedure 
            pubOrSub.SetData(2);
            ack->AddHeader(exchangeHeaderSub); //Exchange header procedure
            ack->AddHeader(subQoS); //QoS header procedure
            ack->AddHeader(counterHeaderSub); //Counter header procedure 
            ack->AddHeader(uidHeaderSub); //Uid header procedure 
            ack->AddHeader(pubOrSub);
            socket->SendTo (ack, 0, from); //Sending the ack
            NS_LOG_UNCOND("SUB 4 QoS 2: "<< uidHeaderSub.GetData() << " : At time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
            << InetSocketAddress::ConvertFrom(from).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(from).GetPort());
            InetSocketAddress addr = InetSocketAddress (InetSocketAddress::ConvertFrom(from).GetIpv4(), InetSocketAddress::ConvertFrom(from).GetPort()); 
        }
      }      
    }
  }
}
}

static void GenerateTraffic (Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize,
                             uint64_t counterInt, uint64_t inputQos) //Function to generate a new publication
{
    Ptr<Packet> packet = Create<Packet>(packetSize); //Create packet
    if(inputQos == 2) 
    {
      QoSHeader exchange; //If QoS is 2, add exchange header
      exchange.SetData(0);
      packet->AddHeader(exchange);
    }
    double p4 = 0.65; // probability
std::bernoulli_distribution d4(p4); //Useless part
    if(Simulator::Now().GetSeconds() == 180)
    {
      NS_LOG_UNCOND("Changing probability");
      p4 = 1; // probability
      std::bernoulli_distribution d4(p4);
      for(int i = 0; i < 40; i++)
      {
        uint16_t f = d4(rng4);
        NS_LOG_UNCOND(f << " " << !f);
      }
    }
    uint64_t qosControlled = 0; //Useless
    uint16_t set = 0; //Useless
    /*for(int step = 10; step < 61; step=step+10)
    {
      if(step == Simulator::Now().GetSeconds())
      {
        qosControlled = ControllerPub(inputQos);
        set = 1;
      }
    }
    if(set == 1)
    {
      pubQoS = qosControlled;
    }
    inputQos = pubQoS;*/
    /*for(int step = 31; step < 920; step=step+30) //Controller steps
    {
      if(step == Simulator::Now().GetSeconds())
      {
        NS_LOG_UNCOND("Hello there");
        qosControlled = ControllerSub(QoS, inputQos); //Controller activated
        set = 1;
      }
    }*/
    inputQos = pubQoS; //QoS insertion
    QoSHeader qos; //QoS header procedure
    qos.SetData(inputQos);
    packet->AddHeader(qos);
    UidHeader uid; //Uid header procedure
    uid.SetData(overIndex);
    packet->AddHeader(uid);
    CounterHeader counter; //Counter header procedure
    counter.SetData(0);
    packet->AddHeader(counter);
    QoSHeader pubOrSub; //PubOrSub header procedure
    pubOrSub.SetData(1);
    packet->AddHeader(pubOrSub);
    socket->SendTo (packet, 0, dst); //Sending packet
    Address addr2;
    socket->GetSockName (addr2);
    InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr2);
    NS_LOG_UNCOND("From: " << InetSocketAddress::ConvertFrom(iaddr).GetIpv4());
    NS_LOG_UNCOND("PUBLISH " << overIndex << ": At time " << Simulator::Now().As (Time::S) << " client sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
    if(inputQos == 0) //QoS 0 case
    {
      launchedPacketsPubQoS0++; //Update stats
      memory0Pub[k0Pub].uid = overIndex; 
      overIndex++; //Update overall packet index
      memory0Pub[k0Pub].startingTime = Simulator::Now().GetMicroSeconds();
      k0Pub++; //Update QoS 0 array index
    } 
    if(inputQos == 1) //QoS 1 case
    {
      memory1Pub[k1Pub].uid = overIndex; //Update stats
      memory1Pub[k1Pub].send1 = 0;
      memory1Pub[k1Pub].latency1 = Simulator::Now().GetMicroSeconds();
      memory1Pub[k1Pub].startingTime = Simulator::Now().GetMicroSeconds();
      k1Pub++;
      launchedPacketsPubQoS1++;
      Simulator::Schedule(Seconds(1), &CheckAck1, overIndex, socket, dst, pktSize, 0, inputQos); //Issuing ack check
      overIndex++; //Update overall packet index
    }
    if(inputQos == 2) //QoS 2 case, works like the QoS 1 case in this part
    {
      memory2Pub[k2Pub].uid = overIndex;
      memory2Pub[k2Pub].send1 = 0;
      memory2Pub[k2Pub].latency1 = Simulator::Now().GetMicroSeconds();
      memory2Pub[k2Pub].startingTime = Simulator::Now().GetMicroSeconds();
      k2Pub++;
      launchedPacketsPubQoS2++;
      Simulator::Schedule(Seconds(1), &CheckAck21, overIndex, socket, dst, pktSize, 0, inputQos);
      overIndex++;
    }
}

static void GenerateInterference (Ptr<Socket> socket, InetSocketAddress dst, uint32_t pktSize) //Function to generate interference packets
{
    Ptr<Packet> packet = Create<Packet>(packetSize); //Create packet
    QoSHeader pubOrSub; //Just one header needed
    pubOrSub.SetData(3);
    packet->AddHeader(pubOrSub);
    socket->SendTo (packet, 0, dst); //Sending packet
    NS_LOG_UNCOND("INTERFERENCE: At time " << Simulator::Now().As (Time::S) << " interferer sent 1024 bytes to "
    << InetSocketAddress::ConvertFrom(dst).GetIpv4() <<" port " << InetSocketAddress::ConvertFrom(dst).GetPort());
}

int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nWifi = 120; //Number of wifi nodes
  double interval = 1.0; // seconds
  Time interPacketInterval = Seconds (interval);
  uint32_t interf = 0;
  float ber = 0; //Not the real bar, useful just for the output file
  //printf("Insert QoS: \n");
  //scanf("%i", &QoS);
  std::string errorModelType = "ns3::NistErrorRateModel";
  bool tracing = true;
  //Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("1500"));
  myfile1.open ("/home/antfabio/federico/outputs/3dControlled/subsHitrate.csv"); //Various outputs files
  myfile2.open ("/home/antfabio/federico/outputs/3dControlled/subsLatency.csv");
  myfile3.open ("/home/antfabio/federico/outputs/3dControlled/subsQoS.csv");
  myfile1 << "Seconds;Sub1;Sub2;Sub3;Sub4;Sub5;Sub6;Sub7;Sub8;Sub9;Sub10\n";//;Sub11;Sub12;Sub13;Sub14;Sub15;Sub16;Sub17;Sub18;Sub19;Sub20\n";
  myfile1 << "0;" << p1 << ";" << p2 << ";" << p3 << ";" << p4 << ";" << p5 << ";" << p6 << ";" << p7 << ";" << p8 << ";" << p9 << ";" << p10;// << ";" << p11 << ";" << p12 << ";" << p13 << ";" << p14 << ";" << p15 << ";" << p16 << ";" << p17 << ";" << p18 << ";" << p19 << ";" << p20;
  myfile2 << "Seconds;Sub1;Sub2;Sub3;Sub4;Sub5;Sub6;Sub7;Sub8;Sub9;Sub10\n";//;Sub11;Sub12;Sub13;Sub14;Sub15;Sub16;Sub17;Sub18;Sub19;Sub20\n";
  myfile2 << "0;" << p1 << ";" << p2 << ";" << p3 << ";" << p4 << ";" << p5 << ";" << p6 << ";" << p7 << ";" << p8 << ";" << p9 << ";" << p10;// << ";" << p11 << ";" << p12 << ";" << p13 << ";" << p14 << ";" << p15 << ";" << p16 << ";" << p17 << ";" << p18 << ";" << p19 << ";" << p20; 
  myfile3 << "Seconds;Sub1;Sub2;Sub3;Sub4;Sub5;Sub6;Sub7;Sub8;Sub9;Sub10\n";//;Sub11;Sub12;Sub13;Sub14;Sub15;Sub16;Sub17;Sub18;Sub19;Sub20\n";
  myfile3 << "0;" << p1 << ";" << p2 << ";" << p3 << ";" << p4 << ";" << p5 << ";" << p6 << ";" << p7 << ";" << p8 << ";" << p9 << ";" << p10;// << ";" << p11 << ";" << p12 << ";" << p13 << ";" << p14 << ";" << p15 << ";" << p16 << ";" << p17 << ";" << p18 << ";" << p19 << ";" << p20;   
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  rem->SetAttribute("RanVar", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]")); 
  rem->SetAttribute("ErrorRate", DoubleValue(0.00128347)); 
 
  CommandLine cmd (__FILE__); //Values taken from command line
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);
  cmd.AddValue("pubQoS", "QoS of the publishers", pubQoS);
  cmd.AddValue("QoS","QoS of the subscribers", QoS);
  cmd.AddValue("interf","Nr of interferers", interf);
  cmd.AddValue("ber","Ber value for file", ber);
  cmd.AddValue("inputControllerUser","Controller behavior", inputControllerUser);
  cmd.Parse (argc,argv);

  if(inputControllerUser < 0 || inputControllerUser > 2)
  {
    NS_LOG_UNCOND("Unidentified behavior");
    return 0;
  }

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  //Creating p2p network
  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("11Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (p2pNodes);

  //Creating 802.11b network
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);
  NodeContainer wifiApNode = p2pNodes.Get (0);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel", "Speed", DoubleValue(2.99792e+08)); 
  // costante dispersione del segnale nel tempo

  //channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
  //channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(3.0),"ReferenceLoss", DoubleValue (0.2), "ReferenceDistance", DoubleValue (450.0));
  //channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(40.0));

  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create());
  //phy.Set ("TxPowerLevels", UintegerValue(1));
  //phy.SetErrorRateModel (errorModelType);

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  //wifi.SetStandard(WIFI_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue (phyMode),
                                "ControlMode", StringValue (phyMode)); 

// data mode: mode to use for every packet tramission
// control mode: trasmission mode to use every RTS packet trasmission

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));

// Active probing indicates if there is a control for the correct behaviour of the network

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer apDevices;
  apDevices = wifi.Install (phy, mac, wifiApNode);

  //Assigning the positions in the grid
  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (5.0),
                                 "GridWidth", UintegerValue (7),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNode);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiStaNodes);

  //Distance is in meters inside the graph

  InternetStackHelper stack;
  stack.Install (wifiStaNodes);
  stack.Install(wifiApNode);
  stack.Install(p2pNodes.Get(1));

  //Assigning ip addresses
  Ipv4AddressHelper address;

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces;
  p2pInterfaces = address.Assign (p2pDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  address.Assign (apDevices);
  address.Assign (staDevices);


  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  if (tracing)
    {
      phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      //pointToPoint.EnablePcapAll ("mythird");
      phy.EnablePcap ("mythird", apDevices.Get (0));
    }

  Ptr<Ipv4> dstAddrRaw = p2pNodes.Get(1)->GetObject<Ipv4>();
  
  /*Ptr<Ipv4> interference1AddrRaw = wifiStaNodes.Get(5)->GetObject<Ipv4>();
  Ptr<Ipv4> interference2AddrRaw = wifiStaNodes.Get(23)->GetObject<Ipv4>();
  Ptr<Ipv4> interference3AddrRaw = wifiStaNodes.Get(24)->GetObject<Ipv4>();
  Ptr<Ipv4> interference4AddrRaw = wifiStaNodes.Get(25)->GetObject<Ipv4>();
  Ptr<Ipv4> interference5AddrRaw = wifiStaNodes.Get(26)->GetObject<Ipv4>();
  Ptr<Ipv4> interference6AddrRaw = wifiStaNodes.Get(27)->GetObject<Ipv4>();
  Ptr<Ipv4> interference7AddrRaw = wifiStaNodes.Get(28)->GetObject<Ipv4>();
  Ptr<Ipv4> interference8AddrRaw = wifiStaNodes.Get(29)->GetObject<Ipv4>();
  Ptr<Ipv4> interference9AddrRaw = wifiStaNodes.Get(30)->GetObject<Ipv4>();
  Ptr<Ipv4> interference10AddrRaw = wifiStaNodes.Get(31)->GetObject<Ipv4>();
  Ptr<Ipv4> interference11AddrRaw = wifiStaNodes.Get(32)->GetObject<Ipv4>();
  Ptr<Ipv4> interference12AddrRaw = wifiStaNodes.Get(33)->GetObject<Ipv4>();
  Ptr<Ipv4> interference13AddrRaw = wifiStaNodes.Get(34)->GetObject<Ipv4>();
  Ptr<Ipv4> interference14AddrRaw = wifiStaNodes.Get(35)->GetObject<Ipv4>();
  Ptr<Ipv4> interference15AddrRaw = wifiStaNodes.Get(36)->GetObject<Ipv4>();
  Ptr<Ipv4> interference16AddrRaw = wifiStaNodes.Get(37)->GetObject<Ipv4>();
  Ptr<Ipv4> interference17AddrRaw = wifiStaNodes.Get(38)->GetObject<Ipv4>();
  Ptr<Ipv4> interference18AddrRaw = wifiStaNodes.Get(39)->GetObject<Ipv4>();
  Ptr<Ipv4> interference19AddrRaw = wifiStaNodes.Get(40)->GetObject<Ipv4>();
  Ptr<Ipv4> interference20AddrRaw = wifiStaNodes.Get(41)->GetObject<Ipv4>();*/  

  Ptr<Ipv4> sub1AddrRaw = wifiStaNodes.Get(1)->GetObject<Ipv4>();
  Ptr<Ipv4> sub2AddrRaw = wifiStaNodes.Get(2)->GetObject<Ipv4>();
  Ptr<Ipv4> sub3AddrRaw = wifiStaNodes.Get(22)->GetObject<Ipv4>();
  Ptr<Ipv4> sub4AddrRaw = wifiStaNodes.Get(6)->GetObject<Ipv4>();
  Ptr<Ipv4> sub5AddrRaw = wifiStaNodes.Get(52)->GetObject<Ipv4>();
  Ptr<Ipv4> sub6AddrRaw = wifiStaNodes.Get(17)->GetObject<Ipv4>();
  Ptr<Ipv4> sub7AddrRaw = wifiStaNodes.Get(21)->GetObject<Ipv4>();
  Ptr<Ipv4> sub8AddrRaw = wifiStaNodes.Get(13)->GetObject<Ipv4>();
  Ptr<Ipv4> sub9AddrRaw = wifiStaNodes.Get(14)->GetObject<Ipv4>();
  Ptr<Ipv4> sub10AddrRaw = wifiStaNodes.Get(20)->GetObject<Ipv4>();
  Ptr<Ipv4> sub11AddrRaw = wifiStaNodes.Get(23)->GetObject<Ipv4>();
  Ptr<Ipv4> sub12AddrRaw = wifiStaNodes.Get(24)->GetObject<Ipv4>();
  Ptr<Ipv4> sub13AddrRaw = wifiStaNodes.Get(25)->GetObject<Ipv4>();
  Ptr<Ipv4> sub14AddrRaw = wifiStaNodes.Get(26)->GetObject<Ipv4>();
  Ptr<Ipv4> sub15AddrRaw = wifiStaNodes.Get(27)->GetObject<Ipv4>();
  Ptr<Ipv4> sub16AddrRaw = wifiStaNodes.Get(28)->GetObject<Ipv4>();
  Ptr<Ipv4> sub17AddrRaw = wifiStaNodes.Get(29)->GetObject<Ipv4>();
  Ptr<Ipv4> sub18AddrRaw = wifiStaNodes.Get(30)->GetObject<Ipv4>();
  Ptr<Ipv4> sub19AddrRaw = wifiStaNodes.Get(31)->GetObject<Ipv4>();
  Ptr<Ipv4> sub20AddrRaw = wifiStaNodes.Get(32)->GetObject<Ipv4>();
  
  Ptr<Ipv4> pub1AddrRaw = wifiStaNodes.Get(3)->GetObject<Ipv4>();
  Ptr<Ipv4> pub2AddrRaw = wifiStaNodes.Get(4)->GetObject<Ipv4>();
  Ptr<Ipv4> pub3AddrRaw = wifiStaNodes.Get(7)->GetObject<Ipv4>();
  Ptr<Ipv4> pub4AddrRaw = wifiStaNodes.Get(8)->GetObject<Ipv4>();
  Ptr<Ipv4> pub5AddrRaw = wifiStaNodes.Get(12)->GetObject<Ipv4>();
  Ptr<Ipv4> pub6AddrRaw = wifiStaNodes.Get(11)->GetObject<Ipv4>();
  Ptr<Ipv4> pub7AddrRaw = wifiStaNodes.Get(10)->GetObject<Ipv4>();
  Ptr<Ipv4> pub8AddrRaw = wifiStaNodes.Get(15)->GetObject<Ipv4>();
  Ptr<Ipv4> pub9AddrRaw = wifiStaNodes.Get(18)->GetObject<Ipv4>();
  Ptr<Ipv4> pub10AddrRaw = wifiStaNodes.Get(19)->GetObject<Ipv4>();

  Ipv4Address dstaddr = dstAddrRaw->GetAddress(1,0).GetLocal();

  /*Ipv4Address interference1addr = interference1AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference2addr = interference2AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference3addr = interference3AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference4addr = interference4AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference5addr = interference5AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference6addr = interference6AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference7addr = interference7AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference8addr = interference8AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference9addr = interference9AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference10addr = interference10AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference11addr = interference11AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference12addr = interference12AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference13addr = interference13AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference14addr = interference14AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference15addr = interference15AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference16addr = interference16AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference17addr = interference17AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference18addr = interference18AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference19addr = interference19AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address interference20addr = interference20AddrRaw->GetAddress(1,0).GetLocal();*/


  Ipv4Address sub1addr = sub1AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub2addr = sub2AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub3addr = sub3AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub4addr = sub4AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub5addr = sub5AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub6addr = sub6AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub7addr = sub7AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub8addr = sub8AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub9addr = sub9AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub10addr = sub10AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub11addr = sub11AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub12addr = sub12AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub13addr = sub13AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub14addr = sub14AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub15addr = sub15AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub16addr = sub16AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub17addr = sub17AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub18addr = sub18AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub19addr = sub19AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address sub20addr = sub20AddrRaw->GetAddress(1,0).GetLocal();

  Ipv4Address pub1addr = pub1AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub2addr = pub2AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub3addr = pub3AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub4addr = pub4AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub5addr = pub5AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub6addr = pub6AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub7addr = pub7AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub8addr = pub8AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub9addr = pub9AddrRaw->GetAddress(1,0).GetLocal();
  Ipv4Address pub10addr = pub10AddrRaw->GetAddress(1,0).GetLocal();

  //Socket initialization
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (p2pNodes.Get (1), tid);
  InetSocketAddress local = InetSocketAddress (dstaddr, 10);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  //Interferences
  /*Ptr<Socket> interference1 = Socket::CreateSocket (wifiStaNodes.Get (5), tid);
  InetSocketAddress interference1start = InetSocketAddress (interference1addr, 10);
  interference1->Bind(interference1start);

  Ptr<Socket> interference2 = Socket::CreateSocket (wifiStaNodes.Get (23), tid);
  InetSocketAddress interference2start = InetSocketAddress (interference2addr, 10);
  interference2->Bind(interference2start);

  Ptr<Socket> interference3 = Socket::CreateSocket (wifiStaNodes.Get (24), tid);
  InetSocketAddress interference3start = InetSocketAddress (interference3addr, 10);
  interference3->Bind(interference3start);

  Ptr<Socket> interference4 = Socket::CreateSocket (wifiStaNodes.Get (25), tid);
  InetSocketAddress interference4start = InetSocketAddress (interference4addr, 10);
  interference4->Bind(interference4start);

  Ptr<Socket> interference5 = Socket::CreateSocket (wifiStaNodes.Get (26), tid);
  InetSocketAddress interference5start = InetSocketAddress (interference5addr, 10);
  interference5->Bind(interference5start);

  Ptr<Socket> interference6 = Socket::CreateSocket (wifiStaNodes.Get (27), tid);
  InetSocketAddress interference6start = InetSocketAddress (interference6addr, 10);
  interference6->Bind(interference6start);

  Ptr<Socket> interference7 = Socket::CreateSocket (wifiStaNodes.Get (28), tid);
  InetSocketAddress interference7start = InetSocketAddress (interference7addr, 10);
  interference7->Bind(interference7start);
  
  Ptr<Socket> interference8 = Socket::CreateSocket (wifiStaNodes.Get (29), tid);
  InetSocketAddress interference8start = InetSocketAddress (interference8addr, 10);
  interference8->Bind(interference8start);

  Ptr<Socket> interference9 = Socket::CreateSocket (wifiStaNodes.Get (30), tid);
  InetSocketAddress interference9start = InetSocketAddress (interference9addr, 10);
  interference9->Bind(interference9start);

  Ptr<Socket> interference10 = Socket::CreateSocket (wifiStaNodes.Get (31), tid);
  InetSocketAddress interference10start = InetSocketAddress (interference10addr, 10);
  interference10->Bind(interference10start);

  Ptr<Socket> interference11 = Socket::CreateSocket (wifiStaNodes.Get (32), tid);
  InetSocketAddress interference11start = InetSocketAddress (interference11addr, 10);
  interference11->Bind(interference11start);

  Ptr<Socket> interference12 = Socket::CreateSocket (wifiStaNodes.Get (33), tid);
  InetSocketAddress interference12start = InetSocketAddress (interference12addr, 10);
  interference12->Bind(interference12start);

  Ptr<Socket> interference13 = Socket::CreateSocket (wifiStaNodes.Get (34), tid);
  InetSocketAddress interference13start = InetSocketAddress (interference13addr, 10);
  interference13->Bind(interference13start);

  Ptr<Socket> interference14 = Socket::CreateSocket (wifiStaNodes.Get (35), tid);
  InetSocketAddress interference14start = InetSocketAddress (interference14addr, 10);
  interference14->Bind(interference14start);

  Ptr<Socket> interference15 = Socket::CreateSocket (wifiStaNodes.Get (36), tid);
  InetSocketAddress interference15start = InetSocketAddress (interference15addr, 10);
  interference15->Bind(interference15start);

  Ptr<Socket> interference16 = Socket::CreateSocket (wifiStaNodes.Get (37), tid);
  InetSocketAddress interference16start = InetSocketAddress (interference16addr, 10);
  interference16->Bind(interference16start);

  Ptr<Socket> interference17 = Socket::CreateSocket (wifiStaNodes.Get (38), tid);
  InetSocketAddress interference17start = InetSocketAddress (interference17addr, 10);
  interference17->Bind(interference17start);
  
  Ptr<Socket> interference18 = Socket::CreateSocket (wifiStaNodes.Get (39), tid);
  InetSocketAddress interference18start = InetSocketAddress (interference18addr, 10);
  interference18->Bind(interference18start);

  Ptr<Socket> interference19 = Socket::CreateSocket (wifiStaNodes.Get (40), tid);
  InetSocketAddress interference19start = InetSocketAddress (interference19addr, 10);
  interference19->Bind(interference19start);

  Ptr<Socket> interference20 = Socket::CreateSocket (wifiStaNodes.Get (41), tid);
  InetSocketAddress interference20start = InetSocketAddress (interference20addr, 10);
  interference20->Bind(interference20start);*/

  //Subscribers
  Ptr<Socket> sub1 = Socket::CreateSocket (wifiStaNodes.Get (1), tid);
  InetSocketAddress sub1target = InetSocketAddress (sub1addr, 10);
  sub1->Bind(sub1target);
  sub1->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[0].chosenQoS = QoS;
  subs[0].addr = InetSocketAddress::ConvertFrom(sub1target).GetIpv4();
  subs[0].lastChange = 3;
  subs[0].loop1 = 0;
  subs[0].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[0].addr << ", qos: " << subs[0].chosenQoS);

  Ptr<Socket> sub2 = Socket::CreateSocket (wifiStaNodes.Get (2), tid);
  InetSocketAddress sub2target = InetSocketAddress (sub2addr, 10);
  sub2->Bind(sub2target);
  sub2->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[1].chosenQoS = QoS;
  subs[1].addr = InetSocketAddress::ConvertFrom(sub2target).GetIpv4();
  subs[1].lastChange = 3;
  subs[1].loop1 = 0;
  subs[1].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[1].addr << ", qos: " << subs[1].chosenQoS);

  Ptr<Socket> sub3 = Socket::CreateSocket (wifiStaNodes.Get (22), tid);
  InetSocketAddress sub3target = InetSocketAddress (sub3addr, 10);
  sub3->Bind(sub3target);
  sub3->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[2].chosenQoS = QoS;
  subs[2].addr = InetSocketAddress::ConvertFrom(sub3target).GetIpv4();
  subs[2].lastChange = 3;
  subs[2].loop1 = 0;
  subs[2].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[2].addr << ", qos: " << subs[2].chosenQoS);
  
  Ptr<Socket> sub4 = Socket::CreateSocket (wifiStaNodes.Get (6), tid);
  InetSocketAddress sub4target = InetSocketAddress (sub4addr, 10);
  sub4->Bind(sub4target);
  sub4->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[3].chosenQoS = QoS;
  subs[3].addr = InetSocketAddress::ConvertFrom(sub4target).GetIpv4();
  subs[3].lastChange = 3;
  subs[3].loop1 = 0;
  subs[3].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[3].addr << ", qos: " << subs[3].chosenQoS);

  Ptr<Socket> sub5 = Socket::CreateSocket (wifiStaNodes.Get (52), tid);
  InetSocketAddress sub5target = InetSocketAddress (sub5addr, 10);
  sub5->Bind(sub5target);
  sub5->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[4].chosenQoS = QoS;
  subs[4].addr = InetSocketAddress::ConvertFrom(sub5target).GetIpv4();
  subs[4].lastChange = 3;
  subs[4].loop1 = 0;
  subs[4].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[4].addr << ", qos: " << subs[4].chosenQoS);

  Ptr<Socket> sub6 = Socket::CreateSocket (wifiStaNodes.Get (17), tid);
  InetSocketAddress sub6target = InetSocketAddress (sub6addr, 10);
  sub6->Bind(sub6target);
  sub6->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[5].chosenQoS = QoS;
  subs[5].addr = InetSocketAddress::ConvertFrom(sub6target).GetIpv4();
  subs[5].lastChange = 3;
  subs[5].loop1 = 0;
  subs[5].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[5].addr << ", qos: " << subs[5].chosenQoS);

  Ptr<Socket> sub7 = Socket::CreateSocket (wifiStaNodes.Get (21), tid);
  InetSocketAddress sub7target = InetSocketAddress (sub7addr, 10);
  sub7->Bind(sub7target);
  sub7->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[6].chosenQoS = QoS;
  subs[6].addr = InetSocketAddress::ConvertFrom(sub7target).GetIpv4();
  subs[6].lastChange = 3;
  subs[6].loop1 = 0;
  subs[6].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[6].addr << ", qos: " << subs[6].chosenQoS);

  Ptr<Socket> sub8 = Socket::CreateSocket (wifiStaNodes.Get (13), tid);
  InetSocketAddress sub8target = InetSocketAddress (sub8addr, 10);
  sub8->Bind(sub8target);
  sub8->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[7].chosenQoS = QoS;
  subs[7].addr = InetSocketAddress::ConvertFrom(sub8target).GetIpv4();
  subs[7].lastChange = 3;
  subs[7].loop1 = 0;
  subs[7].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[7].addr << ", qos: " << subs[7].chosenQoS);

  Ptr<Socket> sub9 = Socket::CreateSocket (wifiStaNodes.Get (14), tid);
  InetSocketAddress sub9target = InetSocketAddress (sub9addr, 10);
  sub9->Bind(sub9target);
  sub9->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[8].chosenQoS = QoS;
  subs[8].addr = InetSocketAddress::ConvertFrom(sub9target).GetIpv4();
  subs[8].lastChange = 3;
  subs[8].loop1 = 0;
  subs[8].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[8].addr << ", qos: " << subs[8].chosenQoS);

  Ptr<Socket> sub10 = Socket::CreateSocket (wifiStaNodes.Get (20), tid);
  InetSocketAddress sub10target = InetSocketAddress (sub10addr, 10);
  sub10->Bind(sub10target);
  sub10->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[9].chosenQoS = QoS;
  subs[9].addr = InetSocketAddress::ConvertFrom(sub10target).GetIpv4();
  subs[9].lastChange = 3;
  subs[9].loop1 = 0;
  subs[9].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[9].addr << ", qos: " << subs[9].chosenQoS);

  /*Ptr<Socket> sub11 = Socket::CreateSocket (wifiStaNodes.Get (23), tid);
  InetSocketAddress sub11target = InetSocketAddress (sub11addr, 10);
  sub11->Bind(sub11target);
  sub11->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[10].chosenQoS = QoS;
  subs[10].addr = InetSocketAddress::ConvertFrom(sub11target).GetIpv4();
  subs[10].lastChange = 3;
  subs[10].loop1 = 0;
  subs[10].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[10].addr << ", qos: " << subs[10].chosenQoS);

  Ptr<Socket> sub12 = Socket::CreateSocket (wifiStaNodes.Get (24), tid);
  InetSocketAddress sub12target = InetSocketAddress (sub12addr, 10);
  sub12->Bind(sub12target);
  sub12->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[11].chosenQoS = QoS;
  subs[11].addr = InetSocketAddress::ConvertFrom(sub12target).GetIpv4();
  subs[11].lastChange = 3;
  subs[11].loop1 = 0;
  subs[11].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[11].addr << ", qos: " << subs[11].chosenQoS);

  Ptr<Socket> sub13 = Socket::CreateSocket (wifiStaNodes.Get (25), tid);
  InetSocketAddress sub13target = InetSocketAddress (sub13addr, 10);
  sub13->Bind(sub13target);
  sub13->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[12].chosenQoS = QoS;
  subs[12].addr = InetSocketAddress::ConvertFrom(sub13target).GetIpv4();
  subs[12].lastChange = 3;
  subs[12].loop1 = 0;
  subs[12].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[12].addr << ", qos: " << subs[12].chosenQoS);
  
  Ptr<Socket> sub14 = Socket::CreateSocket (wifiStaNodes.Get (26), tid);
  InetSocketAddress sub14target = InetSocketAddress (sub14addr, 10);
  sub14->Bind(sub14target);
  sub14->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[13].chosenQoS = QoS;
  subs[13].addr = InetSocketAddress::ConvertFrom(sub14target).GetIpv4();
  subs[13].lastChange = 3;
  subs[13].loop1 = 0;
  subs[13].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[13].addr << ", qos: " << subs[13].chosenQoS);

  Ptr<Socket> sub15 = Socket::CreateSocket (wifiStaNodes.Get (27), tid);
  InetSocketAddress sub15target = InetSocketAddress (sub15addr, 10);
  sub15->Bind(sub15target);
  sub15->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[14].chosenQoS = QoS;
  subs[14].addr = InetSocketAddress::ConvertFrom(sub15target).GetIpv4();
  subs[14].lastChange = 3;
  subs[14].loop1 = 0;
  subs[14].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[14].addr << ", qos: " << subs[14].chosenQoS);

  Ptr<Socket> sub16 = Socket::CreateSocket (wifiStaNodes.Get (28), tid);
  InetSocketAddress sub16target = InetSocketAddress (sub16addr, 10);
  sub16->Bind(sub16target);
  sub16->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[15].chosenQoS = QoS;
  subs[15].addr = InetSocketAddress::ConvertFrom(sub16target).GetIpv4();
  subs[15].lastChange = 3;
  subs[15].loop1 = 0;
  subs[15].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[15].addr << ", qos: " << subs[15].chosenQoS);

  Ptr<Socket> sub17 = Socket::CreateSocket (wifiStaNodes.Get (29), tid);
  InetSocketAddress sub17target = InetSocketAddress (sub17addr, 10);
  sub17->Bind(sub17target);
  sub17->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[16].chosenQoS = QoS;
  subs[16].addr = InetSocketAddress::ConvertFrom(sub17target).GetIpv4();
  subs[16].lastChange = 3;
  subs[16].loop1 = 0;
  subs[16].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[16].addr << ", qos: " << subs[16].chosenQoS);

  Ptr<Socket> sub18 = Socket::CreateSocket (wifiStaNodes.Get (30), tid);
  InetSocketAddress sub18target = InetSocketAddress (sub18addr, 10);
  sub18->Bind(sub18target);
  sub18->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[17].chosenQoS = QoS;
  subs[17].addr = InetSocketAddress::ConvertFrom(sub18target).GetIpv4();
  subs[17].lastChange = 3;
  subs[17].loop1 = 0;
  subs[17].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[17].addr << ", qos: " << subs[17].chosenQoS);

  Ptr<Socket> sub19 = Socket::CreateSocket (wifiStaNodes.Get (31), tid);
  InetSocketAddress sub19target = InetSocketAddress (sub19addr, 10);
  sub19->Bind(sub19target);
  sub19->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[18].chosenQoS = QoS;
  subs[18].addr = InetSocketAddress::ConvertFrom(sub19target).GetIpv4();
  subs[18].lastChange = 3;
  subs[18].loop1 = 0;
  subs[18].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[18].addr << ", qos: " << subs[18].chosenQoS);

  Ptr<Socket> sub20 = Socket::CreateSocket (wifiStaNodes.Get (32), tid);
  InetSocketAddress sub20target = InetSocketAddress (sub20addr, 10);
  sub20->Bind(sub20target);
  sub20->SetRecvCallback(MakeCallback (&ReceivePacketSubscription));
  subs[19].chosenQoS = QoS;
  subs[19].addr = InetSocketAddress::ConvertFrom(sub20target).GetIpv4();
  subs[19].lastChange = 3;
  subs[19].loop1 = 0;
  subs[19].loop2 = 0;
  NS_LOG_UNCOND("Address of src is: " << subs[19].addr << ", qos: " << subs[19].chosenQoS);*/

  //Publishers
  Ptr<Socket> pub1 = Socket::CreateSocket (wifiStaNodes.Get (3), tid);
  InetSocketAddress pub1start = InetSocketAddress (pub1addr, 10);
  pub1->Bind(pub1start);
  pub1->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub2 = Socket::CreateSocket (wifiStaNodes.Get (4), tid);
  InetSocketAddress pub2start = InetSocketAddress (pub2addr, 10);
  pub2->Bind(pub2start);
  pub2->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub3 = Socket::CreateSocket (wifiStaNodes.Get (7), tid);
  InetSocketAddress pub3start = InetSocketAddress (pub3addr, 10);
  pub3->Bind(pub3start);
  pub3->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub4 = Socket::CreateSocket (wifiStaNodes.Get (8), tid);
  InetSocketAddress pub4start = InetSocketAddress (pub4addr, 10);
  pub4->Bind(pub4start);
  pub4->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub5 = Socket::CreateSocket (wifiStaNodes.Get (12), tid);
  InetSocketAddress pub5start = InetSocketAddress (pub5addr, 10);
  pub5->Bind(pub5start);
  pub5->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub6 = Socket::CreateSocket (wifiStaNodes.Get (11), tid);
  InetSocketAddress pub6start = InetSocketAddress (pub6addr, 10);
  pub6->Bind(pub6start);
  pub6->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub7 = Socket::CreateSocket (wifiStaNodes.Get (10), tid);
  InetSocketAddress pub7start = InetSocketAddress (pub7addr, 10);
  pub7->Bind(pub7start);
  pub7->SetRecvCallback(MakeCallback (&ReceivePacketClient));
  
  Ptr<Socket> pub8 = Socket::CreateSocket (wifiStaNodes.Get (15), tid);
  InetSocketAddress pub8start = InetSocketAddress (pub8addr, 10);
  pub8->Bind(pub8start);
  pub8->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub9 = Socket::CreateSocket (wifiStaNodes.Get (18), tid);
  InetSocketAddress pub9start = InetSocketAddress (pub9addr, 10);
  pub9->Bind(pub9start);
  pub9->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<Socket> pub10 = Socket::CreateSocket (wifiStaNodes.Get (19), tid);
  InetSocketAddress pub10start = InetSocketAddress (pub10addr, 10);
  pub10->Bind(pub10start);
  pub10->SetRecvCallback(MakeCallback (&ReceivePacketClient));

  Ptr<MobilityModel> modelAp = p2pNodes.Get(0)->GetObject<MobilityModel>();
  
  //Printing position in the grid of the various nodes
  for(NodeContainer::Iterator j = wifiStaNodes.Begin(); j != wifiStaNodes.End(); ++j)
  {
    Ptr<Node> object = *j;
    Ptr<Ipv4> addrRaw = object->GetObject<Ipv4>();
    Ipv4Address addr = addrRaw->GetAddress(1,0).GetLocal();
    Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
    NS_ASSERT(position != 0);
    Vector pos = position->GetPosition();
    double distance = position->GetDistanceFrom(modelAp);
    NS_LOG_UNCOND("x = " << pos.x << ", y = " << pos.y << ", z = " << pos.z << ", addr = " << addr);
    NS_LOG_UNCOND("Distance from Ap = " << distance);
  }

 for(NodeContainer::Iterator j = wifiApNode.Begin(); j != wifiApNode.End(); ++j)
  {
    Ptr<Node> object = *j;
    Ptr<Ipv4> addrRaw = object->GetObject<Ipv4>();
    Ipv4Address addr = addrRaw->GetAddress(1,0).GetLocal();
    Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
    NS_ASSERT(position != 0);
    Vector pos = position->GetPosition();
    NS_LOG_UNCOND("x = " << pos.x << ", y = " << pos.y << ", z = " << pos.z << ", addr = " << addr);
  }

  //Initializing auxiliary memory for pub QoS 1
  int k = 0;
  for(int step = 1000000; step <= 310000000; step = step + 1000000)
  {
    for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
    { 
      help0[k].addr = subs[j].addr;
      help0[k].startingTime = step;
      help0[k].bol = 0;
      k++;
    }
  }

  int i = 0;
  for(int step = 1000000; step <= 610000000; step = step + 1000000)
  {
    for(int j = 0; j < sizeof(subs)/sizeof(subs[0]); j++)
    {
      help1[i].addr = subs[j].addr;
      help1[i].startingTime = step;
      help1[i].bol = 0;
      i++;
    }
  }
 
  //Latency threshold subscribers QoS initialization
  for(int i = 0; i < sizeof(subs)/sizeof(subs[0]); i++)
  {
    if(inputControllerUser == 2)
    {
      subs[i].chosenQoS = 2;
    }
  }
  
  //Simulator starts
  Simulator::Stop(Seconds(950));

// NON SERVE DECOMMENTARE PERCHE SOLO DI LOG
/*Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
Seconds (11), &GenerateInterference,
interference1, local, packetSize);

Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
Seconds (11.2), &GenerateInterference,
interference2, local, packetSize);

Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
Seconds (11.4), &GenerateInterference,
interference3, local, packetSize);

Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
Seconds (11.6), &GenerateInterference,
interference4, local, packetSize);

Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
Seconds (11.8), &GenerateInterference,
interference5, local, packetSize);

Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
Seconds (12), &GenerateInterference,
interference6, local, packetSize);

Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
Seconds (12.2), &GenerateInterference,
interference7, local, packetSize);

Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
Seconds (12.4), &GenerateInterference,
interference8, local, packetSize);

Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
Seconds (12.6), &GenerateInterference,
interference9, local, packetSize);

Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
Seconds (12.8), &GenerateInterference,
interference10, local, packetSize);

Simulator::ScheduleWithContext (interference11->GetNode ()->GetId (),
Seconds (12.9), &GenerateInterference,
interference11, local, packetSize);

Simulator::ScheduleWithContext (interference12->GetNode ()->GetId (),
Seconds (13.2), &GenerateInterference,
interference12, local, packetSize);

Simulator::ScheduleWithContext (interference13->GetNode ()->GetId (),
Seconds (13.4), &GenerateInterference,
interference13, local, packetSize);

Simulator::ScheduleWithContext (interference14->GetNode ()->GetId (),
Seconds (13.6), &GenerateInterference,
interference14, local, packetSize);

Simulator::ScheduleWithContext (interference15->GetNode ()->GetId (),
Seconds (13.8), &GenerateInterference,
interference15, local, packetSize);

Simulator::ScheduleWithContext (interference16->GetNode ()->GetId (),
Seconds (14), &GenerateInterference,
interference16, local, packetSize);

Simulator::ScheduleWithContext (interference17->GetNode ()->GetId (),
Seconds (14.2), &GenerateInterference,
interference17, local, packetSize);

Simulator::ScheduleWithContext (interference18->GetNode ()->GetId (),
Seconds (14.4), &GenerateInterference,
interference18, local, packetSize);

Simulator::ScheduleWithContext (interference19->GetNode ()->GetId (),
Seconds (14.6), &GenerateInterference,
interference19, local, packetSize);

Simulator::ScheduleWithContext (interference20->GetNode ()->GetId (),
Seconds (14.8), &GenerateInterference,
interference20, local, packetSize);*/

/*if(interf == 1)
{
  for(int i = 1; i < 900; i=i+1)
  { 
    Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
    Seconds (i), &GenerateInterference,
    interference1,local, packetSize);
  }
}

if(interf == 2)
{
  for(int i = 1; i < 900; i=i+1)
  { 
    Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
    Seconds (i), &GenerateInterference,
    interference1,local, packetSize);
    Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
    Seconds (i+0.5), &GenerateInterference,
    interference2,local, packetSize);
  }
}

if(interf == 3)
{
  for(int i = 1; i < 900; i=i+1)
  { 
    Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
    Seconds (i), &GenerateInterference,
    interference1,local, packetSize);
    Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
    Seconds (i+0.33), &GenerateInterference,
    interference2,local, packetSize);
    Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
    Seconds (i+0.66), &GenerateInterference,
    interference3,local, packetSize);
  }
}

if(interf == 4)
{
  for(int i = 1; i < 900; i=i+1)
  {
    Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
    Seconds (i), &GenerateInterference,
    interference1,local, packetSize);
    Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
    Seconds (i+0.25), &GenerateInterference,
    interference2,local, packetSize);
    Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
    Seconds (i+0.5), &GenerateInterference,
    interference3,local, packetSize);
    Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
    Seconds (i+0.75), &GenerateInterference,
    interference4,local, packetSize);
  }
}

if(interf == 5)
{
  for(int i = 1; i < 900; i=i+1)
  {
    Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
    Seconds (i), &GenerateInterference,
    interference1,local, packetSize);
    Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
    Seconds (i+0.2), &GenerateInterference,
    interference2,local, packetSize);
    Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
    Seconds (i+0.4), &GenerateInterference,
    interference3,local, packetSize);
    Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
    Seconds (i+0.6), &GenerateInterference,
    interference4,local, packetSize);
    Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
    Seconds (i+0.8), &GenerateInterference,
    interference5,local, packetSize);
  }
}

if(interf == 10)
{
  for(int i = 1; i < 900; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.5)
    { 
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.1), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.2), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.3), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.4), &GenerateInterference,
      interference5,local, packetSize);
    }
  }
}

if(interf == 15)
{
  for(int i = 1; i < 900; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.35)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.07), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.14), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.21), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.28), &GenerateInterference,
      interference5,local, packetSize);
    }
  }
}
  

if(interf == 20)
{
  for(int i = 1; i < 900; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.5)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.05), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.1), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.15), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.2), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.25), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.3), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.35), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.4), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.45), &GenerateInterference,
      interference10,local, packetSize);
    }
  }
}
  

if(interf == 25)
{
  for(int i = 1; i < 900; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.4)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.04), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.08), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.12), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.16), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.2), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.24), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.28), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.32), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.36), &GenerateInterference,
      interference10,local, packetSize);
    }
    for(float j = 0.8; j < 1; j=j+0.3)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.04), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.08), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.12), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.16), &GenerateInterference,
      interference5,local, packetSize);
    }
  }
}

if(interf == 30)
{
  for(int i = 1; i < 1600; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.34)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.033), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.066), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.099), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.132), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.165), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.198), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.231), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.264), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.297), &GenerateInterference,
      interference10,local, packetSize);
    }
  }
}

if(interf == 60)
{
  for(int i = 1; i < 1600; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.17)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.017), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.034), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.051), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.068), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.085), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.102), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.119), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.136), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.153), &GenerateInterference,
      interference10,local, packetSize);
    }
  }
}

if(interf == 90)
{
  for(int i = 1; i < 1600; i=i+1)
  {
    for(float j = 0; j < 0.98; j=j+0.165)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.011), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.022), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.033), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.044), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.055), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.066), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.077), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.088), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.099), &GenerateInterference,
      interference10,local, packetSize);
      Simulator::ScheduleWithContext (interference11->GetNode ()->GetId (),
      Seconds (i+j+0.11), &GenerateInterference,
      interference11,local, packetSize);
      Simulator::ScheduleWithContext (interference12->GetNode ()->GetId (),
      Seconds (i+j+0.121), &GenerateInterference,
      interference12,local, packetSize);
      Simulator::ScheduleWithContext (interference13->GetNode ()->GetId (),
      Seconds (i+j+0.132), &GenerateInterference,
      interference13,local, packetSize);
      Simulator::ScheduleWithContext (interference14->GetNode ()->GetId (),
      Seconds (i+j+0.143), &GenerateInterference,
      interference14,local, packetSize);
      Simulator::ScheduleWithContext (interference15->GetNode ()->GetId (),
      Seconds (i+j+0.154), &GenerateInterference,
      interference15,local, packetSize);
    }
  }
}

if(interf == 120)
{
  for(int i = 1; i < 1600; i=i+1)
  {
    for(float j = 0; j < 1; j=j+0.167)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.0083), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.0166), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.0249), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.0332), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.0415), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.0498), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.0581), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.0664), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.0747), &GenerateInterference,
      interference10,local, packetSize);
      Simulator::ScheduleWithContext (interference11->GetNode ()->GetId (),
      Seconds (i+j+0.83), &GenerateInterference,
      interference11,local, packetSize);
      Simulator::ScheduleWithContext (interference12->GetNode ()->GetId (),
      Seconds (i+j+0.0913), &GenerateInterference,
      interference12,local, packetSize);
      Simulator::ScheduleWithContext (interference13->GetNode ()->GetId (),
      Seconds (i+j+0.0996), &GenerateInterference,
      interference13,local, packetSize);
      Simulator::ScheduleWithContext (interference14->GetNode ()->GetId (),
      Seconds (i+j+0.1079), &GenerateInterference,
      interference14,local, packetSize);
      Simulator::ScheduleWithContext (interference15->GetNode ()->GetId (),
      Seconds (i+j+0.1162), &GenerateInterference,
      interference15,local, packetSize);
      Simulator::ScheduleWithContext (interference16->GetNode ()->GetId (),
      Seconds (i+j+0.1245), &GenerateInterference,
      interference16,local, packetSize);
      Simulator::ScheduleWithContext (interference17->GetNode ()->GetId (),
      Seconds (i+j+0.1328), &GenerateInterference,
      interference17,local, packetSize);
      Simulator::ScheduleWithContext (interference18->GetNode ()->GetId (),
      Seconds (i+j+0.1411), &GenerateInterference,
      interference18,local, packetSize);
      Simulator::ScheduleWithContext (interference19->GetNode ()->GetId (),
      Seconds (i+j+0.1494), &GenerateInterference,
      interference19,local, packetSize);
      Simulator::ScheduleWithContext (interference20->GetNode ()->GetId (),
      Seconds (i+j+0.1577), &GenerateInterference,
      interference20,local, packetSize);
    }
  }
}

if(interf == 150)
{
  for(int i = 1; i < 10; i=i+1)
  {
    for(float j = 0; j < 0.93; j=j+0.134)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.0067), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.0134), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.0201), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.0268), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.0335), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.0402), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.0469), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.0536), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.0603), &GenerateInterference,
      interference10,local, packetSize);
      Simulator::ScheduleWithContext (interference11->GetNode ()->GetId (),
      Seconds (i+j+0.067), &GenerateInterference,
      interference11,local, packetSize);
      Simulator::ScheduleWithContext (interference12->GetNode ()->GetId (),
      Seconds (i+j+0.0737), &GenerateInterference,
      interference12,local, packetSize);
      Simulator::ScheduleWithContext (interference13->GetNode ()->GetId (),
      Seconds (i+j+0.0804), &GenerateInterference,
      interference13,local, packetSize);
      Simulator::ScheduleWithContext (interference14->GetNode ()->GetId (),
      Seconds (i+j+0.0871), &GenerateInterference,
      interference14,local, packetSize);
      Simulator::ScheduleWithContext (interference15->GetNode ()->GetId (),
      Seconds (i+j+0.0938), &GenerateInterference,
      interference15,local, packetSize);
      Simulator::ScheduleWithContext (interference16->GetNode ()->GetId (),
      Seconds (i+j+0.1005), &GenerateInterference,
      interference16,local, packetSize);
      Simulator::ScheduleWithContext (interference17->GetNode ()->GetId (),
      Seconds (i+j+0.1072), &GenerateInterference,
      interference17,local, packetSize);
      Simulator::ScheduleWithContext (interference18->GetNode ()->GetId (),
      Seconds (i+j+0.1139), &GenerateInterference,
      interference18,local, packetSize);
      Simulator::ScheduleWithContext (interference19->GetNode ()->GetId (),
      Seconds (i+j+0.1206), &GenerateInterference,
      interference19,local, packetSize);
      Simulator::ScheduleWithContext (interference20->GetNode ()->GetId (),
      Seconds (i+j+0.1273), &GenerateInterference,
      interference20,local, packetSize);
    }
    for(float j = 0.938; j < 1; j=j+1)
    {
      Simulator::ScheduleWithContext (interference1->GetNode ()->GetId (),
      Seconds (i+j), &GenerateInterference,
      interference1,local, packetSize);
      Simulator::ScheduleWithContext (interference2->GetNode ()->GetId (),
      Seconds (i+j+0.0067), &GenerateInterference,
      interference2,local, packetSize);
      Simulator::ScheduleWithContext (interference3->GetNode ()->GetId (),
      Seconds (i+j+0.0134), &GenerateInterference,
      interference3,local, packetSize);
      Simulator::ScheduleWithContext (interference4->GetNode ()->GetId (),
      Seconds (i+j+0.0201), &GenerateInterference,
      interference4,local, packetSize);
      Simulator::ScheduleWithContext (interference5->GetNode ()->GetId (),
      Seconds (i+j+0.0268), &GenerateInterference,
      interference5,local, packetSize);
      Simulator::ScheduleWithContext (interference6->GetNode ()->GetId (),
      Seconds (i+j+0.0335), &GenerateInterference,
      interference6,local, packetSize);
      Simulator::ScheduleWithContext (interference7->GetNode ()->GetId (),
      Seconds (i+j+0.0402), &GenerateInterference,
      interference7,local, packetSize);
      Simulator::ScheduleWithContext (interference8->GetNode ()->GetId (),
      Seconds (i+j+0.0469), &GenerateInterference,
      interference8,local, packetSize);
      Simulator::ScheduleWithContext (interference9->GetNode ()->GetId (),
      Seconds (i+j+0.0536), &GenerateInterference,
      interference9,local, packetSize);
      Simulator::ScheduleWithContext (interference10->GetNode ()->GetId (),
      Seconds (i+j+0.0603), &GenerateInterference,
      interference10,local, packetSize);
    }
  }
}
*/


for(int i = 1; i < 307; i=i+10)
{ 
  Simulator::ScheduleWithContext (pub1->GetNode ()->GetId (),
  Seconds (i), &GenerateTraffic,
  pub1, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub2->GetNode ()->GetId (),
  Seconds (i+1), &GenerateTraffic,
  pub2, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub3->GetNode ()->GetId (),
  Seconds (i+2), &GenerateTraffic,
  pub3, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub4->GetNode ()->GetId (),
  Seconds (i+3), &GenerateTraffic,
  pub4, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub5->GetNode ()->GetId (),
  Seconds (i+4), &GenerateTraffic,
  pub5, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub6->GetNode ()->GetId (),
  Seconds (i+5), &GenerateTraffic,
  pub6, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub7->GetNode ()->GetId (),
  Seconds (i+6), &GenerateTraffic,
  pub7, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub8->GetNode ()->GetId (),
  Seconds (i+7), &GenerateTraffic,
  pub8, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub9->GetNode ()->GetId (),
  Seconds (i+8), &GenerateTraffic,
  pub9, local, packetSize, 0, pubQoS);
  Simulator::ScheduleWithContext (pub10->GetNode ()->GetId (),
  Seconds (i+9), &GenerateTraffic,
  pub10, local, packetSize, 0, pubQoS);
}


  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowMonitorHelper;
  flowMonitor = flowMonitorHelper.InstallAll();

  Simulator::Run ();
  flowMonitor->SerializeToXmlFile("test.xml", true, true);
  Simulator::Destroy ();

  //Printing the various statistics
  NS_LOG_UNCOND("Pub QoS 0");
  for(int i = 0; i < sizeof(memory0Pub)/sizeof(memory0Pub[0]); i++)
  {
    if(memory0Pub[i].uid != 0)
    {
      NS_LOG_UNCOND("Packet : "<< memory0Pub[i].uid << " Start time = " << memory0Pub[i].startingTime << "; Arrival time = " << memory0Pub[i].arrivalTime << ";");
    }
  }
  NS_LOG_UNCOND("Pub QoS 1");
  for(int i = 0; i < sizeof(memory1Pub)/sizeof(memory1Pub[0]); i++)
  {
    if(memory1Pub[i].uid != 0)
    {
      NS_LOG_UNCOND("Packet : "<< memory1Pub[i].uid << " First latency = " << memory1Pub[i].latency1 << "; Second latency = " << memory1Pub[i].latency2 << ";");
    }
  }
  NS_LOG_UNCOND("Pub QoS 2");
  for(int i = 0; i < sizeof(memory2Pub)/sizeof(memory2Pub[0]); i++)
  {
    if(memory2Pub[i].uid != 0)
    {
      NS_LOG_UNCOND("Packet : "<< memory2Pub[i].uid << " First latency = " << memory2Pub[i].latency1 << "; Second latency = " << memory2Pub[i].latency2 << "; Third latency = " << memory2Pub[i].latency3 << "; Fourth latency = " << memory2Pub[i].latency4 << ";");
    }
  }

  
  std::ofstream ofile;
  std::ofstream ofile2;
  std::string latFile= "/home/antfabio/federico/outputs/3dControlled/PostProcess.txt";
  ofile2.open(latFile, std::ios::app);
  NS_LOG_UNCOND("Sub QoS 0");
  if(pubQoS == 0 || pubQoS == 2)
  {
    for(int i = 0; i < sizeof(memory0Sub)/sizeof(memory0Sub[0]); i++)
    {
      if(memory0Sub[i].uid != 0)
      {
        NS_LOG_UNCOND(memory0Sub[i].startingTime << ";" << memory0Sub[i].arrivalTime << ";" << memory0Sub[i].addr);
        ofile2 << memory0Sub[i].startingTime << ";" << memory0Sub[i].arrivalTime << ";" << memory0Sub[i].addr << std::endl;
      }
    }
  }
  if(pubQoS == 1)
  {
    for(int i = 0; i < sizeof(help0)/sizeof(help0[0]); i++)
    {
      if(help0[i].arrivalTime != 0)
      {
        //ofile2 << help0[i].startingTime << ";" help0.arrivalTime << ";" << help0[i].addr << std::endl;
        NS_LOG_UNCOND(help0[i].startingTime << ";" << help0[i].arrivalTime);
      }  
    }
  }
  NS_LOG_UNCOND("Sub QoS 1");
  if(pubQoS == 1)
  {
    for(int i = 0; i < sizeof(help1)/sizeof(help1[0]); i++)
    {
      if(help1[i].arrivalTime != 0)
      {
        //ofile2 << help1[i].startingTime << ";" help1.arrivalTime << ";" << help1[i].addr << std::endl;
        NS_LOG_UNCOND(help1[i].startingTime << ";" << help1[i].arrivalTime);
      }  
    }
  }
  if(pubQoS == 2)
  {
    for(int i = 0; i < sizeof(memory1Sub)/sizeof(memory1Sub[0]); i++)
    {
      if(memory1Sub[i].uid != 0)
      {
        ofile2 << memory1Sub[i].startingTime << ";" << memory1Sub[i].subRecTime << ";" << memory1Sub[i].addr << std::endl;
        NS_LOG_UNCOND(memory1Sub[i].startingTime << ";" << memory1Sub[i].subRecTime << ";" << memory1Sub[i].arrivalTime);
      }
    }
  }
  NS_LOG_UNCOND("Sub QoS 2");
  for(int i = 0; i < sizeof(memory2Sub)/sizeof(memory2Sub[0]); i++)
  {
    if(memory2Sub[i].uid != 0)
    {
      ofile2 << memory2Sub[i].startingTime << ";" << memory2Sub[i].subRecTime << ";" << memory2Sub[i].addr << std::endl;    
      NS_LOG_UNCOND("Packet : "<< memory2Sub[i].uid << " Start time = " << memory2Sub[i].startingTime << "; Arrival time = " << memory2Sub[i].arrivalTime << ";");
      NS_LOG_UNCOND("Packet : "<< memory2Sub[i].uid << " First latency = " << memory2Sub[i].latency1 << "; Second latency = " << memory2Sub[i].latency2 << "; Third latency = " << memory2Sub[i].latency3 << "; Fourth latency = " << memory2Sub[i].latency4 << ";");
    }
  }
 
  for(int i = 0; i < sizeof(help0)/sizeof(help0[0]); i++)
  {
    if(help0[i].bol == 1)
    {
      arrivedPacketsSubQoS0PubQoS1++;
    }
  }

  for(int i = 0; i < sizeof(help1)/sizeof(help1[0]); i++)
  {
    if(help1[i].bol == 1)
    {
      arrivedPacketsSubQoS1PubQoS1++;
    }
  }

  NS_LOG_UNCOND("QoS 0 Pub packets sent: " << launchedPacketsPubQoS0 << "; QoS 0 Packets arrived: "<< arrivedPacketsPubQoS0 << ";");
  NS_LOG_UNCOND("QoS 0 Sub packets sent: " << launchedPacketsSubQoS0 << "; QoS 0 Packets arrived: "<< arrivedPacketsSubQoS0PubQoS0 + arrivedPacketsSubQoS0PubQoS1 + arrivedPacketsSubQoS0PubQoS2 << ";");
  NS_LOG_UNCOND("QoS 1 Pub packets sent: " << launchedPacketsPubQoS1 << "; QoS 1 Half arrived: " << halfPacketsPubQoS1<<"; QoS 1 Packets arrived: "<< arrivedPacketsPubQoS1 << "; QoS 1 Retry: " << retryQoS1);
  NS_LOG_UNCOND("QoS 1 Sub packets sent: " << launchedPacketsSubQoS1 << "; QoS 1 Half arrived: " << halfPacketsSubQoS1<<"; QoS 1 Packets arrived: "<< arrivedPacketsSubQoS1PubQoS0 + arrivedPacketsSubQoS1PubQoS1 + arrivedPacketsSubQoS1PubQoS2<< "; QoS 1 Retry: " << retryQoS1);
  NS_LOG_UNCOND("QoS 2 retries: " << retryQoS2 << "; QoS 2 pub packets arrived = " << arrivedPacketsPubQoS2 << "; QoS 2 sub packets arrived = " << arrivedPacketsSubQoS2PubQoS0 + arrivedPacketsSubQoS2PubQoS1 + arrivedPacketsSubQoS2PubQoS2 << ";");
  NS_LOG_UNCOND("Interferences arrived: " << interfCounter);

  float pubPercQoS2 = (float)arrivedPacketsPubQoS2*100/launchedPacketsPubQoS2;
  //uint32_t pubHalfPerc = halfPacketsPubQoS1*100/launchedPacketsPubQoS1;
  float subPercQoS2 = (arrivedPacketsSubQoS0PubQoS2 + arrivedPacketsSubQoS1PubQoS2 + arrivedPacketsSubQoS2PubQoS2)*100/(float)(launchedPacketsPubQoS2*10);
  //uint32_t subHalfPerc = halfPacketsSubQoS0*100/launchedPacketsSubQoS0;
  float pubPercQoS1 = (float)arrivedPacketsPubQoS1*100/launchedPacketsPubQoS1;
  float subPercQoS1 = (arrivedPacketsSubQoS0PubQoS1 + arrivedPacketsSubQoS1PubQoS1)*100/(float)(launchedPacketsPubQoS1*10);
  float pubPercQoS0 = (float)arrivedPacketsPubQoS0*100/launchedPacketsPubQoS0;
  float subPercQoS0 = (float)arrivedPacketsSubQoS0PubQoS0*100/(launchedPacketsPubQoS0*10);
  NS_LOG_UNCOND(pubPercQoS0 << ";" << subPercQoS0);
  NS_LOG_UNCOND(pubPercQoS1 << ";" << subPercQoS1);
  NS_LOG_UNCOND(pubPercQoS2 << ";" << subPercQoS2);
  NS_LOG_UNCOND("Tot perc: " << (float)(arrivedPacketsSubQoS0PubQoS2 + arrivedPacketsSubQoS1PubQoS2 + arrivedPacketsSubQoS2PubQoS2 + arrivedPacketsSubQoS0PubQoS1 + arrivedPacketsSubQoS1PubQoS1 + arrivedPacketsSubQoS0PubQoS0)*100/6200);
  std::string controlFile= "/home/antfabio/federico/outputs/3dControlled/Int" + std::to_string(interf) + "Ber" + std::to_string(ber) + ".txt";
  ofile.open(controlFile, std::ios::app);
  ofile << "Seconds;Hitrate;LatencyAv;LatencyStd;QoS0;QoS1;QoS2" << std::endl;
  uint16_t timestamp = 30;
  for(int i = 0; i < 10; i++)
  {
    NS_LOG_UNCOND(timestamp << ";" << hitratesCont[i] << ";" << latCont[i] << ";" << stdLatCont[i]);
    ofile << timestamp << ";" << hitratesCont[i] << ";" << latCont[i] << ";" << stdLatCont[i] << ";" << QoS0[i] << ";" << QoS1[i] << ";" << QoS2[i] << std::endl;
    timestamp += 30;    
  }
  //ofile << ber << ";" << interf << ";"  << pubPerc << ";" << subPerc << ";" << retryQoS1 << std::endl;
  //ofile2.close();
  myfile1.close();
  myfile2.close();
  myfile3.close();
  ofile.close();
  flowMonitor->SerializeToXmlFile("Test.xml", true, true);
  return 0;
}
