//INCLUDE
#include <MBee.h>
#include <SoftwareSerial.h>
#include <PacketSerial.h>
//DEFINE
#define DEBUG nss
#define MBee_Serial Serial
#define ssRX 8
#define ssTX 9
#define Check_For_Wire_Connection_Pin 10 // really 10 pin on arduino
SoftwareSerial nss(ssRX, ssTX);
PacketSerial WireSerial;

SerialStar mbee = SerialStar();

//TX SETUP
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.

//RX SETUP
RxResponse rx = RxResponse();
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t Telemetry_Pack [] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
// number_of_packet , CommandForRaper , ERROR Code?
uint8_t Pack_From_TX [] = {0, 1, 2};
const uint16_t remoteAddress = 0x0001;
bool WireConnectionFlag = false;
uint8_t Counter_From_RC = 0;
uint8_t RX_counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(Check_For_Wire_Connection_Pin, INPUT);
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  WireSerial.begin(115200);
  WireSerial.setPacketHandler(&onPacketReceived);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  Check_And_Refresh_WireConnectionFlag_RX();
}

void loop() {
  // put your main code here, to run repeatedly:
  DEBUG.print("RX ");

  //Fill Telemetry_Pack | (FAKE) random x13
  Collect_Telemetry();

  //send telemetry
  if (2 != 3) //
  {
    if (Check_And_Refresh_WireConnectionFlag_RX() == true)
    {
      DEBUG.println("Wire send RX");
      WireSerial.send(Telemetry_Pack, sizeof(Telemetry_Pack) / sizeof(Telemetry_Pack[0]) ) ; // sizeof(package)/sizeof(package[0])
    }
    else // WireConnectionFlag Flag false
    {
      DEBUG.println("WireLESS send");
      tx.setRemoteAddress(remoteAddress);
      tx.setPayload((uint8_t*)Telemetry_Pack); //Устанавливаем указатель на тестовый массив
      tx.setPayloadLength(sizeof(Telemetry_Pack));  //Устанавливаем длину поля данных
      mbee.send(tx);
    }
  }

  //

  WireSerial.update();
  // Check for a receive buffer overflow (optional).
  if (WireSerial.overflow())
  {
    // Send an alert via a pin (e.g. make an overflow LED) or return a
    // user-defined packet to the sender.
    //
    // Ultimately you may need to just increase your recieve buffer via the
    // template parameters (see the README.md).
    DEBUG.println("WireSerial.overflow() oO");
  }

  if (WireConnectionFlag != true)
  {
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      DEBUG.println("mbee TX packet is OK");
    }
  }
  else
  {
    DEBUG.println("Wait for PacketSerial ");
  }

  //
  delay(100);   // 500 ok
  //
}

boolean Check_And_Refresh_WireConnectionFlag_RX()
{
  bool First_WireConnection_Check = digitalRead(Check_For_Wire_Connection_Pin);
  if (WireConnectionFlag != First_WireConnection_Check)
  {
    delay(5);
    bool Second_WireConnection_Check = digitalRead(Check_For_Wire_Connection_Pin);
    if (Second_WireConnection_Check == First_WireConnection_Check)
    {
      if ( Second_WireConnection_Check == true) // WIRE ON
      {

        DEBUG.println(" -|- Change to -> Wire connect");
        WireConnectionFlag = true;
      }
      else                                      // WIRE OFF
      {
        DEBUG.println("-))  ((- Change to -> WireLESS connect");
        WireConnectionFlag = false;
      }
    }
  }
  return WireConnectionFlag;
}

void Check_Recieve_Buffer( bool WireFlag)
{
  //Need Refresh this for PacketSerial if WireFlag == true
  if (WireFlag == true)
  {
    WireSerial.update();
    if (WireSerial.overflow())
    {
      // Send an alert via a pin (e.g. make an overflow LED) or return a
      // user-defined packet to the sender.
      //Alarm Check + disarm
//      if (AlarmFlag == false)
//        Alarm_ON();
      DEBUG.println("WireSerial Buffer Overflow! need increase buffer!");
      // Ultimately you may need to just increase your recieve buffer via the
      // template parameters (see the README.md).
    }
  }
  else  // WireConnectionFlag_TX == false
  {
    //read input Telemetry from Mbee
    // Telemetry from WireConnect discribed on -> onPacketReceived(const uint8_t* buffer, size_t size)
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      DEBUG.println("mbee packet is readable");
      // processing mbee received packet()
    }
    else
    {
      mbee.getResponse().isError();
      DEBUG.println("mbee packet isError() == true");
      delay(500);
    }
  }
  return;
}


void Collect_Telemetry()
{
  //fake collect
  Telemetry_Pack[0] = Counter_From_RC;
  Telemetry_Pack[1] = RX_counter;
  for (int i = 2; i < (sizeof(Telemetry_Pack) / sizeof(Telemetry_Pack[0]) ) ; i++)
  {
    Telemetry_Pack[i] = random(0, 256);
  }
  DEBUG.print("RX_Telemetry_Pack[0]= ");
  DEBUG.println(Telemetry_Pack[0]);
}

// input command packet
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag != true)
  {
    DEBUG.println("Packet for Mbee processing, not my problem=)");
    return;
  }
  DEBUG.println("TX Packet is Read");
  uint8_t tempBuffer[size];
  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);
  DEBUG.println("Buffer:");
  for (int i = 0 ; i < size ; i++)
  {
    DEBUG.print("[");
    DEBUG.print(i);
    DEBUG.print("]=");
    DEBUG.print(tempBuffer[i]);
    DEBUG.print("; ");
  }
  DEBUG.println("");
  Counter_From_RC = tempBuffer[0];
  RX_counter++;
  delay(100);
  return;
}




//
