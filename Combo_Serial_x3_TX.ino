//INCLUDE
#include <MBee.h>
#include <SoftwareSerial.h>
#include <PacketSerial.h>
#include <LiquidCrystalRus.h>
//DEFINE
//Ports Rename
#define DEBUG nss
#define MBee_Serial Serial
//Const
#define Stop 0
#define Forward 5
#define Backward 2
#define CorruptedCommand 10
//Pins
#define Forward_Check_Pin  2    // 80% certainty
#define Backward_Check_Pin 3    // 80% certainty
#define BUZZER_Pin  4    // specify in the scheme
#define ERROR_Led   5    // specify in the scheme
#define SWITCHER_Pin 6   // specify in the scheme
#define Check_For_Wire_Connection_Pin 10  // specify in the scheme
#define ssRX 8           // 100% certainty
#define ssTX 9           // 100% certainty
#define Forward_Led  11  // specify in the scheme
#define Backward_Led 12  // specify in the scheme
#define Stop_Led     13  // specify in the scheme?
//Analog Pins
#define  ADC_A0 0  // check on the scheme
#define  ADC_A1 1  // check on the scheme
#define  ADC_A2 2  // check on the scheme
#define  ADC_A3 3  // check on the scheme
#define  ADC_A4 4  // check on the scheme
#define  ADC_A5 5  // check on the scheme
#define  ADC_A6 6  // check on the scheme
#define ADC_Battey_LvL_Pin 7 // specify in the scheme

SoftwareSerial nss(ssRX, ssTX);
PacketSerial WireSerial;
SerialStar mbee = SerialStar();

// specify in the scheme
LiquidCrystalRus disp(6, 7, 2, 3, 4, 5); // создаем объект

//TX SETUP
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.

//RX SETUP
RxResponse rx = RxResponse();
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

uint8_t Command_Pack [] = {0, 1, 23};  // number_of_packet , CommandForRaper , ERROR Code?
//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t Telemetry_Pack_From_RX [] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
bool WireConnectionFlag = false;
int number_of_packet = 0;
int CommandForRaper = 0;
bool AlarmFlag = false;

void setup() {
  //check pins setup
  pinMode(Check_For_Wire_Connection_Pin, INPUT);
  pinMode(Forward_Check_Pin , INPUT);
  pinMode(Backward_Check_Pin, INPUT);
  //Leds and control pins setup
  pinMode(Forward_Led, OUTPUT);
  pinMode(Backward_Led, OUTPUT);
  pinMode(Stop_Led, OUTPUT);
  pinMode(ERROR_Led, OUTPUT);
  pinMode(BUZZER_Pin, OUTPUT);
  pinMode(SWITCHER_Pin, OUTPUT);
  //ADC Pins setup
  pinMode(ADC_A0, INPUT);
  pinMode(ADC_A1, INPUT);
  pinMode(ADC_A2, INPUT);
  pinMode(ADC_A3, INPUT);
  pinMode(ADC_A4, INPUT);
  pinMode(ADC_A5, INPUT);
  pinMode(ADC_A6, INPUT);
  pinMode(ADC_Battey_LvL_Pin, INPUT); // 100% input

  //Ports setup
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  WireSerial.begin(115200);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  Check_And_Refresh_WireConnectionFlag_TX();
  // Normal
  //  WireSerial.setPacketHandler(&onPacketReceived);
  //DEBUG Starter handler
  WireSerial.setPacketHandler(&onPacketReceived);
  //display setup
  // OLED_setup();
  delay(666);
}

void loop() {
  //Read command (and maybe error situation)
  //  CommandForRaper = ReadCommandFromSwither();
  CommandForRaper = Forward;
  if (CommandForRaper == CorruptedCommand)
  {
    CommandForRaper = Stop;
    DEBUG.println("CorruptedCommand write");
  }

  //Fill the Command_Pack
  Fill_Command_Pack();

  // wire / wireless send
  bool CheckWirePin = Check_And_Refresh_WireConnectionFlag_TX();
  if (CheckWirePin == true)
  {
    DEBUG.println("Wire send");
    WireSerial.send(Command_Pack, 3 ) ; // sizeof(Command_Pack) / sizeof(Command_Pack[0])
    number_of_packet++;
  }
  else // WireConnectionFlag false
  {
    DEBUG.println("WireLESS send");
    tx.setPayload((uint8_t*)Command_Pack);      //Устанавливаем указатель на тестовый массив
    tx.setPayloadLength(sizeof(Command_Pack));  //Устанавливаем длину поля данных
    mbee.send(tx);
    number_of_packet++;
  }

  DEBUG.print("number_of_packet -> ");
  DEBUG.println(number_of_packet);
  delay(500); // need ~50ms

  WireSerial.update();
  // Check for a receive buffer overflow (optional).
  if (WireSerial.overflow())
  {
    // Send an alert via a pin (e.g. make an overflow LED) or return a
    // user-defined packet to the sender.
    //
    // Ultimately you may need to just increase your recieve buffer via the
    // template parameters (see the README.md).
  }

  //read input Telemetry from Mbee
  // Telemetry from WireConnect discribed on -> onPacketReceived(const uint8_t* buffer, size_t size)
  if (WireConnectionFlag != true)
  {
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      DEBUG.println("mbee packet is readable");
    }
  }
  else
  {
    DEBUG.println("Wait for PacketSerial ");
  }
  return;
}

int ReadCommandFromSwither()
{
  //Check  a move command from  switch(remote controller)
  bool Rush = digitalRead(Forward_Check_Pin);
  bool Back = digitalRead(Backward_Check_Pin);
  int Command = CorruptedCommand;
  // Forward command
  if (Rush == true & Back == false)
  {
    Command = Forward;
    digitalWrite(Backward_Led, LOW);
    digitalWrite(Forward_Led, HIGH);
    digitalWrite(Stop_Led, LOW);
    DEBUG.println("Command RUSH ");
  }
  // Backward command
  if (Rush == false & Back == true)
  {
    Command = Backward;
    digitalWrite(Backward_Led, HIGH);
    digitalWrite(Forward_Led, LOW);
    digitalWrite(Stop_Led, LOW);
    DEBUG.println("Command BACK ");
  }
  // Stop command
  if (Rush == false & Back == false)
  {
    Command = Stop;
    digitalWrite(Backward_Led,  LOW);
    digitalWrite(Forward_Led, LOW);
    digitalWrite(Stop_Led, HIGH);
    DEBUG.println("Command STOP !!! ");
  }
  // Error
  if (Rush == true & Back == true)
  {
    DEBUG.println();
    DEBUG.println("I take Error from \"switcher\" , =( ");
    DEBUG.println();
    Command = Stop;
    digitalWrite(Backward_Led,  HIGH);
    digitalWrite(Forward_Led, HIGH);
    digitalWrite(ERROR_Led, HIGH);
    Alarm_ON();
    DEBUG.println(" NO MOVE, we have a ERROR ((( ");
  }

  DEBUG.print("CommandFor XXX --> ");
  DEBUG.println(Command);
  return Command;
}

void Fill_Command_Pack()
{
  Command_Pack[0] = number_of_packet;
  Command_Pack[1] = CommandForRaper;
  if (AlarmFlag == false)
    Command_Pack[2] = 88;
  else
    Command_Pack[2] = 1;
}

boolean Check_And_Refresh_WireConnectionFlag_TX()
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
        digitalWrite(SWITCHER_Pin , HIGH);  //specify in the scheme
        WireConnectionFlag = true;
      }
      else                                      // WIRE OFF
      {
        DEBUG.println("-))  ((- Change to -> WireLESS connect");
        digitalWrite(SWITCHER_Pin , LOW);  //specify in the scheme
        WireConnectionFlag = false;
      }
    }
  }
  return WireConnectionFlag;
}

// In this example, we  will only simply print the contents of the array=)
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag == false)
  {
    DEBUG.println("This packet is for Mbee. This is not my Job guys *going to relax*");
    return;
  }
  if (AlarmFlag == true)
  {
    Alarm_OFF();
  }

  // Make a temporary buffer.
  uint8_t tempBuffer[size];

  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);

  // Print our temporaray buffer.
  Print_Telemetry_Packet(tempBuffer, size);

  // Send the reversed buffer back to the sender. The send() method will encode
  // the whole buffer as as single packet, set packet markers, etc.
  // The `tempBuffer` is a pointer to the `tempBuffer` array and `size` is the
  // number of bytes to send in the `tempBuffer`.
  //myPacketSerial.send(tempBuffer, size);
  return;
}

void DebugHandler(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag == false)
  {
    DEBUG.println("This packet is for Mbee. This is not my Job guys *going to relax*");
    return;
  }
  if (AlarmFlag == true) {
    Alarm_OFF();
  }
  DEBUG.println("Telemetry Packet from RX RECEIVED!");
  delay(100);
  return;
}

void Print_Telemetry_Packet(uint8_t* buffer, size_t size)
{
  DEBUG.println("Telemtry message:");
  for (int i = 0 ; i < size ; i++)
  {
    DEBUG.print("[");
    DEBUG.print(i);
    DEBUG.print("]=");
    DEBUG.print(buffer[i]);
    DEBUG.print("; ");
  }
  delay(50);
  return;
}

void Alarm_ON()
{
  DEBUG.println("Alarm ON");
  digitalWrite(BUZZER_Pin, HIGH);
  digitalWrite(ERROR_Led, HIGH);
  AlarmFlag = true;
  // GOTO
  // Watch Dog Timer on?
  return;
}

void Alarm_OFF()
{
  // digitalWrite(BUZZER_Pin, HIGH);
  DEBUG.println("Alarm OFF");
  digitalWrite(BUZZER_Pin, LOW);
  digitalWrite(ERROR_Led, LOW);
  AlarmFlag = false;
  // GOTO
  // Watch Dog Timer OFF?
  return;
}





























//
