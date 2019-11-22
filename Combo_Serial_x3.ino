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

uint8_t  package [] = {1, 2, 3, 4, 5, 6, 1388};
bool WireConnectionFlag = false;

void setup() {
  pinMode(CheckPin_For_Wire_Connection, INPUT);
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  WireSerial.begin(115200);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  Check_And_Refresh_WireConnectionFlag_TX();
}



void loop() {
  DEBUG.print("ping X ");
  // wire / wireless
  if (Check_And_Refresh_WireConnectionFlag_TX() == true )
  {
    DEBUG.println("Wire send");
    uint8_t x [] = {2, 4};
    WireSerial.send(package, sizeof(package)/sizeof(package[0]) ) ; // sizeof(package)/sizeof(package[0])
  }
  else // WireConnectionFlag Flag false
  {
    DEBUG.println("WireLESS send");
    tx.setPayload((uint8_t*)package); //Устанавливаем указатель на тестовый массив
    tx.setPayloadLength(sizeof(package));  //Устанавливаем длину поля данных
    mbee.send(tx);
  }
  delay(500);
}

boolean Check_And_Refresh_WireConnectionFlag_TX()
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
