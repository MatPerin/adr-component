/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 * Author: Matteo Perin <matteo.perin.2@studenti.unipd.2>
 */

#ifndef ADR_COMPONENT_H
#define ADR_COMPONENT_H

#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/network-status.h"
#include "ns3/network-controller-components.h"

namespace ns3 {

  ////////////////////////////////////////
  // LinkAdrRequest commands management //
  ////////////////////////////////////////

  class AdrComponent : public NetworkControllerComponent
  {
    public:

      static TypeId GetTypeId (void);

      //Constructor
      AdrComponent ();
      //Destructor
      virtual ~AdrComponent ();

      void OnReceivedPacket (Ptr<const Packet> packet,
                             Ptr<EndDeviceStatus> status,
                             Ptr<NetworkStatus> networkStatus);

      void BeforeSendingReply (Ptr<EndDeviceStatus> status,
                               Ptr<NetworkStatus> networkStatus);

      void OnFailedReply (Ptr<EndDeviceStatus> status,
                          Ptr<NetworkStatus> networkStatus);

    private:

      void AdrImplementation(uint8_t *newDataRate,
                             uint8_t *newTxPower,
                             Ptr<EndDeviceStatus> status);

      uint8_t SfToDr (uint8_t sf);

      double TxPowerToSNR (double transmissionPower);

      double GetMaxTxFromGateways (EndDeviceStatus::GatewayList gwList);

      double GetAverageTxFromGateways (EndDeviceStatus::GatewayList gwList);

      double GetReceivedPower (EndDeviceStatus::GatewayList gwList);

      double GetMaxSNR (EndDeviceStatus::ReceivedPacketList packetList,
                        uint8_t historyRange);

      double GetAverageSNR (EndDeviceStatus::ReceivedPacketList packetList,
                            uint8_t historyRange);

      int GetTxPowerIndex (int txPower);

      //TX power from gateways policy:
      //0 - max TX power between all connected GW
      //1 - average TX power considering all connected GW
      const bool tpAveraging = 1;

      //Number of previous packets to consider
      const uint8_t historyRange = 20;

      //Received SNR history policy:
      //0 - max SNR between the latest historyRange packets
      //1 - average SNR between the latest historyRange packets
      const bool historyAveraging = 1;

      //SF lower limit
      const int min_spreadingFactor = 7;

      //Minimum transmission power (dBm) (Europe)
      const int min_transmissionPower = 2;

      //Maximum transmission power (dBm) (Europe)
      const int max_transmissionPower = 16;

      //Device specific SNR margin (dB)
      const int offset = 10;

      //Bandwidth (Hz)
      const int B = 125000;

      //Noise Figure (dB)
      const int NF = 6;

      //Vector containing the required SNR for the 6 allowed SF levels
      //ranging from 7 to 12 (the SNR values are in dB).
      double treshold[6] = {-20.0, -17.5, -15.0, -12.5, -10.0, -7.5};
};
}

#endif
