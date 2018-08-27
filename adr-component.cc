/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 University of Padova
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
 * Author: Matteo Perin <matteo.perin.2@studenti.unipd.it
 */

#include "ns3/adr-component.h"

namespace ns3 {

  ////////////////////////////////////////
  // LinkAdrRequest commands management //
  ////////////////////////////////////////

  NS_LOG_COMPONENT_DEFINE ("AdrComponent");

  NS_OBJECT_ENSURE_REGISTERED (AdrComponent);

  TypeId AdrComponent::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::AdrComponent")
    .SetGroupName ("lorawan")
    .AddConstructor<AdrComponent> ();
    return tid;
  }

  AdrComponent::AdrComponent () {}

  AdrComponent::~AdrComponent () {}

  void AdrComponent::OnReceivedPacket (Ptr<const Packet> packet,
                                       Ptr<EndDeviceStatus> status,
                                       Ptr<NetworkStatus> networkStatus)
  {
    NS_LOG_FUNCTION (this->GetTypeId() << packet << networkStatus);

    // We will only act just before reply, when all Gateways will have received
    // the packet, since we need their respective received power.
  }

  void
  AdrComponent::BeforeSendingReply (Ptr<EndDeviceStatus> status,
                                    Ptr<NetworkStatus> networkStatus)
  {
    NS_LOG_FUNCTION (this << status << networkStatus);

    Ptr<Packet> myPacket = status->GetLastPacketReceivedFromDevice()->Copy();
    LoraFrameHeader fHdr;
    fHdr.SetAsUplink();
    myPacket->RemoveHeader(fHdr);

    //Execute the ADR algotithm only if the request bit is set
    if(fHdr.GetAdr())
    {
      if(status->GetReceivedPacketList().size() < historyRange)
        NS_LOG_DEBUG ("Not enough packets received by this device for the algorithm to work!\n");
      else
      {
        //The device request an ADR tuning, so it is going to require answering
        status->m_reply.needsReply = true;

        //New parameters for the end-device
        int newDataRate;
        int newTxPower;

        //ADR Algorithm
        AdrImplementation(&newDataRate,
                          &newTxPower,
                          status);

        //Create a list with mandatory channel indexes
        int channels[] = {1, 2, 3};
        std::list<int> enabledChannels(channels,
                                       channels + sizeof(channels) /
                                       sizeof(int));

        //Repetitions Setting
        const int rep = 1;

        NS_LOG_DEBUG ("Sending LinkAdrReq with DR = "<<newDataRate<<" and TP = "<<newTxPower<<" dB.\n");

        status->m_reply.frameHeader.AddLinkAdrReq(newDataRate,
                                                  newTxPower,
                                                  enabledChannels,
                                                  rep);
        status->m_reply.frameHeader.SetAsDownlink();
        status->m_reply.macHeader.SetMType(LoraMacHeader::UNCONFIRMED_DATA_DOWN);
      }
    }
    else
    {
        // Do nothing
    }
  }

  void AdrComponent::OnFailedReply (Ptr<EndDeviceStatus> status,
                                    Ptr<NetworkStatus> networkStatus)
  {
    NS_LOG_FUNCTION (this->GetTypeId() << networkStatus);
  }

  void AdrComponent::AdrImplementation(int *newDataRate,
                         int *newTxPower,
                         Ptr<EndDeviceStatus> status)
  {
    //Compute the maximum or median SNR, based on the boolean value historyAveraging
    double m_SNR;
    if(historyAveraging)
      m_SNR = GetAverageSNR(status->GetReceivedPacketList(),
                            historyRange);
    else
      m_SNR = GetMaxSNR(status->GetReceivedPacketList(),
                        historyRange);

      //Get the SF used by the device
      int spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

      //Get the device data rate and use it to get the SNR demodulation treshold
      double req_SNR = treshold[SfToDr(spreadingFactor)];

      //Get the device transmission power (dB)
      double transmissionPower = status->GetMac()->GetTransmissionPower();

      //Compute the SNR margin taking into consideration the SNR of
      //previously received packets
      double margin_SNR = m_SNR - req_SNR - offset;

      //Number of steps to decrement the SF (thereby increasing the Data Rate)
      //and the TP.
      int steps = std::floor(margin_SNR / 3);

      //If the number of steps is positive (margin_SNR is positive, so its
      //decimal value is high) increment the data rate, if there are some
      //leftover steps after reaching the maximum possible data rate
      //(corresponding to the minimum SF) decrement the transmission power as
      //well for the number of steps left.
      //If, on the other hand, the number of steps is negative (margin_SNR is
      //negative, so its decimal value is low) increase the transmission power
      //(note that the SF is not incremented as this particular algorithm
      //expects the node itself to raise its SF whenever necessary).
      while(steps > 0 && spreadingFactor > min_spreadingFactor)
      {
        spreadingFactor--;
        steps--;
      }
      while(steps > 0 && transmissionPower > min_transmissionPower)
      {
        transmissionPower -= 3;
        steps--;
      }
      while(steps < 0 && transmissionPower < max_transmissionPower)
      {
        transmissionPower += 3;
        steps++;
      }

      *newDataRate = SfToDr(spreadingFactor);
      *newTxPower = transmissionPower;
  }

  int AdrComponent::SfToDr(int sf)
  {
    return sf - 7;
  }

  double AdrComponent::TxPowerToSNR (double transmissionPower)
  {
  //The following conversion ignores interfering packets
    return transmissionPower + 174 - 10 * log10(B) - NF;
  }

  //Get the maximum received power (it considers the values in dB!)
  double AdrComponent::GetMaxTxFromGateways (EndDeviceStatus::GatewayList gwList)
  {
    double max = min_transmissionPower;

    for(EndDeviceStatus::GatewayList::iterator it = gwList.begin(); it != gwList.end(); it++)
    {
      if(it->second.rxPower > max)
        max = it->second.rxPower;
    }

    return max;
  }

  //Get the maximum received power
  double AdrComponent::GetAverageTxFromGateways (EndDeviceStatus::GatewayList gwList)
  {
    double sum = 0;

    for(EndDeviceStatus::GatewayList::iterator it = gwList.begin(); it != gwList.end(); it++)
    {
      sum += it->second.rxPower;
    }

    return sum / gwList.size();
  }

  double AdrComponent::GetReceivedPower (EndDeviceStatus::GatewayList gwList)
  {
    if(tpAveraging)
      return GetAverageTxFromGateways(gwList);
    else
      return TxPowerToSNR(GetMaxTxFromGateways(gwList));
  }

  double AdrComponent::GetMaxSNR (EndDeviceStatus::ReceivedPacketList packetList,
                                  int historyRange)
  {
    double max = TxPowerToSNR(min_transmissionPower);
    double m_SNR;

    //Take elements from the list starting at the end
    auto it = packetList.rbegin();
    for(int i = 0; i < historyRange; i++, it++)
    {
      m_SNR = TxPowerToSNR(GetReceivedPower(it->second.gwList));

      if(m_SNR > max)
        max = m_SNR;
    }

    return max;
  }

  double AdrComponent::GetAverageSNR (EndDeviceStatus::ReceivedPacketList packetList,
                                      int historyRange)
  {
    double sum = 0;
    double m_SNR;

  //Take elements from the list starting at the end
    auto it = packetList.rbegin();
    for(int i = 0; i < historyRange; i++, it++)
    {
      m_SNR = TxPowerToSNR(GetReceivedPower(it->second.gwList));

      sum += m_SNR;
    }

    return m_SNR / historyRange;
  }
  
  int AdrComponent::GetTxPowerIndex (int txPower)
  {
    if(txPower >= 16)
      return 0;
    else if(txPower >= 14)
      return 1;
    else if(txPower >= 12)
      return 2;
    else if(txPower >= 10)
      return 3;
    else if(txPower >= 8)
      return 4;
    else if(txPower >= 6)
      return 5;
    else if(txPower >= 4)
      return 6;
    else
      return 7;
  }
}
