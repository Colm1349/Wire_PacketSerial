//INCLUDE
#include <MBee.h>
#include <SoftwareSerial.h>
#include <PacketSerial.h>
#include <LiquidCrystalRus.h>
//DEFINE
#define DEBUG nss
#define MBee_Serial Serial
#define ssRX 8
#define ssTX 9
#define CheckPin_For_Wire_Connection 10
SoftwareSerial nss(ssRX, ssTX);
PacketSerial WireSerial;

SerialStar mbee = SerialStar();
LiquidCrystalRus disp(6, 7, 2, 3, 4, 5); // создаем объект

//TX SETUP
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.

//RX SETUP
RxResponse rx = RxResponse();
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

uint8_t  Telemetry_Pack [] = {1, 2, 3, 4, 5, 6, 1388};
bool WireConnectionFlag = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(CheckPin_For_Wire_Connection, INPUT);
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

  //Fill Telemetry_Pack
  Collect_Telemetry();

  //send telemetry
  if (2 < 1) //
  {
    if (Check_And_Refresh_WireConnectionFlag_RX() == true)
    {
      DEBUG.println("Wire send RX");
      WireSerial.send(Telemetry_Pack, sizeof(Telemetry_Pack) / sizeof(Telemetry_Pack[0]) ) ; // sizeof(package)/sizeof(package[0])
    }
    else // WireConnectionFlag Flag false
    {
      DEBUG.println("WireLESS send");
      tx.setPayload((uint8_t*)Telemetry_Pack); //Устанавливаем указатель на тестовый массив
      tx.setPayloadLength(sizeof(Telemetry_Pack));  //Устанавливаем длину поля данных
      mbee.send(tx);
    }
  }
  //
  delay(500);
  ///
}

boolean Check_And_Refresh_WireConnectionFlag_RX()
{
  bool First_WireConnection_Check = digitalRead(CheckPin_For_Wire_Connection);
  if (WireConnectionFlag != First_WireConnection_Check)
  {
    delay(5);
    bool Second_WireConnection_Check = digitalRead(CheckPin_For_Wire_Connection);
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


void Collect_Telemetry()
{

}

// input command packet
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  uint8_t tempBuffer[size];
  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);
  

}


///
