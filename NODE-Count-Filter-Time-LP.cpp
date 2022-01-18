/**
 * @file main.cpp
 * @author Ahmed MJADI (a.mjadi@nextronic.io)
 * @brief Node CC1101 LoRa
 * @version 0.1
 * @date 2022-01-08 modified 15/01/2022
 * @copyright Copyright (c) 2022
 * 
 */

/*
* SAMD21 consumption:
* Sleep: 0.61 mA
* while Radio listening & LoRa sending: 5.28 mA
*/

/////////////////// Libraries //////////////////

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "ArduinoLowPower.h"

/////////////////// LoRa Definitions //////////////////
char LPacket[65];

// SAMD21 Pins Names
#define Lss 4
#define Lrst 6
#define Ldio0 2
//scl  SCL
//miso MISO
//mosi MOSI

/////////////////// CC1101 //////////////////
// SCK_PIN = SCK;
// MISO_PIN = MISO;
// MOSI_PIN = MOSI;
// SS_PIN = 12;

byte buffer[61] = {0};

int RF_RSSI = 0;
String anchorsData[10];
//////////// Functions Def //////////////

/* LOW POWER */

int TimeOut = 30000; // 30000   30 s
int StartTime = 0;

void SAMD_DeepSeep(int period_ms)
{

  SerialUSB.print("========= Going To Sleep for ");
  SerialUSB.print(period_ms / 1000);
  SerialUSB.println(" s ! =========");
  SerialUSB.println("-----------------------------------------------------");

  //wait 2 sec before going to sleep
  delay(2000);

  // ELECHOUSE_cc1101.goSleep();
  // before
  SerialUSB.end();
  USBDevice.detach();

  LowPower.deepSleep(period_ms); //300000

  USBDevice.init();
  USBDevice.attach();
  USB->DEVICE.CTRLB.bit.UPRSM = 0x01u;
  while (USB->DEVICE.CTRLB.bit.UPRSM)
    ;
  SerialUSB.begin(9600);
  // after
  delay(50);
  SerialUSB.print("Wake Up for 1 min !");
  delay(5000);
  setup();
}

//////////////////////////////////// Setup //////////////////////////////////////

void setup()
{
  ELECHOUSE_cc1101.setSpiPin(SCK, MISO, MOSI, 12);

  // LOW POWER Setup
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, NULL, CHANGE);

  //initialize SerialUSB Monitor
  SerialUSB.begin(9600);
  /*CC1101 init*/
  SerialUSB.println("CC1101 SPI Initializing  ");
  SerialUSB.println("CC1101 SPI Initializing ok ! ");

  // while (!SerialUSB);
  SerialUSB.println("==================== TAG/Node starts ====================");

  //setup LoRa transceiver module
  LoRa.setPins(Lss, Lrst, Ldio0);

  SerialUSB.println("LoRa Initializing...");

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6))
  {
    SerialUSB.println(".");
    delay(100);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  SerialUSB.println("[i] LoRa SPI Initializing OK!");

  SerialUSB.println("CC1101 Initializing...");

  if (ELECHOUSE_cc1101.getCC1101())
  { // Check the CC1101 SPI connection.
    SerialUSB.println("[i] CC1101 Initializing OK!");
  }
  else
  {
    SerialUSB.println("[e] CC1101 Connection Error [e]");
  }

  ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(0);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(868);           // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
  ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
  ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
  ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
  ELECHOUSE_cc1101.setDRate(99.97);       // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
  ELECHOUSE_cc1101.setPA(12);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
  ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
  ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
  ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
  ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous SerialUSB mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous SerialUSB mode, Data in on GDO0 and data out on either of the GDOx pins.
  ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
  ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
  ELECHOUSE_cc1101.setCrc(1);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
  ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
  ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
  ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
  ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

  SerialUSB.println("[d] Parameters set, CC1101 RX Mode !");
}
//// ReceiveandSend

bool ReceiveSend(int rec_count)
{

  // Count usefull Received data
  // int rec_count = 0;
  // while (rec_count < iteration)
  // {

  bool buffer_is_in = false;

  //Checks whether something has been received via CC1101.
  //When something is received we give some time to receive the message in full.(time in millis)
  if (ELECHOUSE_cc1101.CheckRxFifo(100) && ELECHOUSE_cc1101.CheckCRC())
  {

    //CRC Check. If "setCrc(false)" crc returns always OK!

    //Get received Data and calculate length
    int len = ELECHOUSE_cc1101.ReceiveData(buffer);
    buffer[len] = '\0';
    //check reppetitive ID of anchors
    for (int i = 0; i < rec_count; i++)
    {
      if (strstr(anchorsData[i].c_str(), (char *)buffer) != NULL)
      {
        buffer_is_in = true;
      }
    }
    if (buffer_is_in)
    {
      SerialUSB.println("");
      SerialUSB.println("** Anchor is already detected **");
      // continue;
    }
    else
    {

      // SerialUSB.print("<< " + rec_count+1);
      SerialUSB.print("<<< Recieved Packets from Anchors: ");

      //Rssi Level in dBm
      SerialUSB.print("Rssi: ");
      RF_RSSI = ELECHOUSE_cc1101.getRssi();
      SerialUSB.print(RF_RSSI);
      SerialUSB.print("  || ");
      SerialUSB.println((char *)buffer);
      anchorsData[rec_count] = (char *)buffer;

      //Print received in char format.
      // SerialUSB.println((char *)buffer); // removed to print new filter

      // LORA SENDER:
      SerialUSB.print(">>> Sending Packet via LoRa :  ");
      SerialUSB.print(" Rssi:");
      SerialUSB.print(RF_RSSI);
      SerialUSB.print("|| Anchor: ");
      SerialUSB.println((char *)buffer);

      LoRa.beginPacket();
      LoRa.print("{\"n\":\"4E4558414243B005\",\"a\":\"");
      LoRa.print((char *)buffer);
      LoRa.print("\",\"RSSI\":");
      LoRa.print(RF_RSSI);
      LoRa.print("}");
      LoRa.endPacket();
      SerialUSB.println("---------------- LoRa end sending ----------------");

      //rec_count++;
      delay(500);
      SerialUSB.println("[d] succefully sent data via LoRa");
    }

    return true;
  }
  // SerialUSB.println("[d] did not find any radio closer");
  return false;
  //}
}

//////////////////////////////////// Loop //////////////////////////////////////

void loop()
{

  // Start counting timeout
  StartTime = millis();
  // SerialUSB.println(StartTime);
  SerialUSB.print("RF Listening for: ");
  SerialUSB.print(TimeOut / 1000);
  SerialUSB.println(" sec... ");
  while (((int)millis()) - StartTime < TimeOut)
  {

    // /* Demo: receives, sends and sleeps ALL signals, but after 30 sec */
    // ReceiveSend();

    /* Demo: receives, sends and sleeps ONE signal, but after 30 sec */

    if (ReceiveSend(3))
    {
      break;
    }

    //SerialUSB.println("[d] did not find any radio closer");
    // SerialUSB.println("[i] Time spent: " + String(millis() - StartTime));
    // delay(50);
  }

  SerialUSB.print("[i] Time spent: ");
  int now = (int)millis() - StartTime;
  SerialUSB.print(now);
  SerialUSB.println(" ms.");
  SerialUSB.println("[i] Tag ID: 4E4558414243B005");

  //////////////////////////////////// SLEEP //////////////////////////////////////
  // delay(5000);

  //Sleeping Deep
  // SAMD_DeepSeep(180000); //3 min= 180000 ms, 5 min = 300000
}
