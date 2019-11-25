//Описание
/*  Пульт радиоуправления способный передать команду самоходу по радиуправлению через MBee или по кабелю
    и принять телеметрию , далее отобразить основные парамерты на дисплее.
    Имеет возможность активировать звуковую индикацию и посылать отладочные сообщения через пины 8 и 9 на PC (SoftWareSerial)
    Отображение на дисплее скорости, нагрузки на двигатели, качества сигнала (RSSI) для беспроводного управения,
    напряжения батареи пульта и заряда аккумлятора самохода.
    Защита от одновременного зажатия ВПЕРЕД И НАЗАД

    Пока отсутствует перезагрузка пульта, не найдено весомых причин для перезагрузки.

    //GO TO Quests

    // EASY
    !! добавить отладку параметров с самохода через DEBUG Serial

    //MEDIUM
    !! добавить индикацию светодиодами для Проводного/Беспроводного управления
    !! добавить управление ключами через SWITCHER_Pin для работы через нужные "рельсы"
    !- добавить функции для работы с дисплеем

    //HARD
    + дальнейшая проверка работы через кабель/провода и используя дисплей выводить нужные данные
    + дальнейшая работа через кабель и Mbee Wireless и отображая текущие значения на дисплее!

*/

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
//const for PC Processing debug
#define ADC_Speed_RPM_Step 16.67
#define ADC_Current_Step 0.0125
#define ADC_Climber_Voltage_Step 0.0047
#define ADC_RC_Voltage_Step 1
#define ADC_PWM_Step 0.4

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

#define Green_WireLess_Led YesOrNo //does it exist?
//Analog Pins
#define  ADC_A0 A0  // check on the scheme
#define  ADC_A1 A1  // check on the scheme
#define  ADC_A2 A2  // check on the scheme
#define  ADC_A3 A3  // check on the scheme
#define  ADC_A4 A4  // check on the scheme
#define  ADC_A5 A5  // check on the scheme
#define  ADC_A6 A6  // check on the scheme
#define ADC_Battey_LvL_Pin A7 // specify in the scheme

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

//Arrays
// number_of_packet , CommandForRaper , ERROR Code?
uint8_t Command_Pack [] = {0, 1, 23};
//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t Telemetry_Pack_From_RX [] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

//Telemetry members
uint8_t RSSI_LvL = 0;
float Speed_1_RPM = 0;
float Speed_2_RPM = 0;
float I_1_A = 0;
float I_2_A = 0;
float Battery_Roper_Voltage = 0;
float PWM_Value_Percent = 12.3;
uint8_t Roper_Direction = 1;
//Data from TX
float Battery_RC_Voltage = 0;  //
// raw Telemetry H+L
int Speed_1_raw = 0;
int Speed_2_raw = 0;
int I_1_raw = 0;
int I_2_raw = 0;
int V_Roper_raw = 0; // 0-1023
//Raw Data from TX
int V_RC_raw = 0;    // 0-1023


//Variables
bool WireConnectionFlag_TX = false;
int number_of_packet = 0;
uint8_t CommandForRaper = 0;
bool AlarmFlag = false;

//Create Custom Sign for Disp
byte Triangle0[8] = {
  0b11111,
  0b01110,
  0b00100,
  0b00100,
  0b00100,
  0b00000,
  0b00001,
  0b00111
};

byte Triangle1[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00001,
  0b00111,
  0b11111,
  0b11111
};

byte Triangle2[8] = {
  0b00000,
  0b00000,
  0b00001,
  0b00111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte Triangle3[8] = {
  0b00001,
  0b00111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte RC_Char[8] = {
  0b01000,
  0b01000,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
  0b11101,
  0b11111
};

byte Roper[8] = {
  0b11111,
  0b11011,
  0b10001,
  0b00100,
  0b10001,
  0b11011,
  0b11111,
  0b11111
};


void setup() {
  //check pins setup
  pinMode(Check_For_Wire_Connection_Pin, INPUT);
  pinMode(Forward_Check_Pin , INPUT);
  pinMode(Backward_Check_Pin, INPUT);
  //Leds and control pins setup
  pinMode(Forward_Led, OUTPUT);
  pinMode(Backward_Led, OUTPUT);
  pinMode(Stop_Led, OUTPUT);     // does it exist??
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

  //Display setup
  disp.begin(20, 4);  // инициализируем дисплей 4 строки по 20 символов
  reset_textmode();
  OLED_Display_Const_Data();
  disp.createChar(2, Roper);
  disp.createChar(3, RC_Char);
  disp.createChar(4, Triangle0);
  disp.createChar(5, Triangle1);
  disp.createChar(6, Triangle2);
  disp.createChar(7, Triangle3);

  //Ports setup
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  WireSerial.begin(115200);
  //DEBUG Starter handler
  WireSerial.setPacketHandler(&onPacketReceived);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  Refresh_WireConnectionFlag_TX();
  delay(666);
}

void loop() {
  //Read RC BatteryLvL
  V_RC_raw = analogRead(ADC_Battey_LvL_Pin);

  //Read command (and maybe error situation)  //kost
  //CommandForRaper = ReadCommandFromSwitcher();
  CommandForRaper = Forward;

  //Fill the Command_Pack
  Fill_Command_Pack();

  // wire / wireless send with depends on WireConnectionFlag_TX
  Refresh_WireConnectionFlag_TX();
  Set_SWITCHER_Pin ( WireConnectionFlag_TX );
  //  while(1);
  Send_Command_Pack( WireConnectionFlag_TX );

  //Check recieve buffer (depends on WireConnectionFlag_TX)
  Check_Recieve_Buffer( WireConnectionFlag_TX );

  //Read Telemetry from Mbee
  if (WireConnectionFlag_TX == false)
  {
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      Processing_Mbee_Telemetry_Packet();
    }
    else
      DEBUG.println("---");
  }

  // Display data from Telemetry
  OLED_Display_Dynamic_Data();

  delay(500); // WireConnectionFlag_TX
  return;
}

int ReadCommandFromSwitcher()
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
    DEBUG.println("I take Error from \"RC_Control_Switch\" , =( ");
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

void Set_SWITCHER_Pin(bool WireFlag)
{
  DEBUG.print("WireFlag =>");
  DEBUG.println(WireFlag);
  if (WireFlag == true)
  {
    digitalWrite(SWITCHER_Pin, LOW); // inverted COM(4) <-> NC (3)
    //    digitalWrite(Green_WireLess_Led, LOW); // does it exist?
  }
  if (WireFlag == false)
  {
    digitalWrite(SWITCHER_Pin, HIGH); // inverted D3A/D6A COM(4) <-> NO (1)
    //    digitalWrite(Green_WireLess_Led, HIGH); //  does it exist?
  }
  return;
}

void Refresh_WireConnectionFlag_TX()
{
  bool First_WireConnection_Check = digitalRead(Check_For_Wire_Connection_Pin);
  if (WireConnectionFlag_TX != First_WireConnection_Check)
  {
    delay(5);
    bool Second_WireConnection_Check = digitalRead(Check_For_Wire_Connection_Pin);
    if (Second_WireConnection_Check == First_WireConnection_Check)
    {
      if (Second_WireConnection_Check == true) // WIRE ON
      {
        DEBUG.println(" -|- Change to -> Wire connect");
        WireConnectionFlag_TX = true;
      }
      else                                      // WIRE OFF
      {
        DEBUG.println("-))  ((- Change to -> WireLESS connect");
        WireConnectionFlag_TX = false;
      }
    }
  }
  return;
}

void Send_Command_Pack(bool CommunicationTypeWireFlag)
{
  if (CommunicationTypeWireFlag == true)
  {
    DEBUG.println("Wire send");  // GO TO Send
    WireSerial.send(Command_Pack, 3) ; // sizeof(Command_Pack) / sizeof(Command_Pack[0]) // GO TO Send actual size
    number_of_packet++;
  }
  else // CommunicationTypeWireFlag false
  {
    DEBUG.println("WireLESS send");
    tx.setPayload((uint8_t*)Command_Pack);      //Устанавливаем указатель на тестовый массив
    tx.setPayloadLength(sizeof(Command_Pack));  //Устанавливаем длину поля данных
    mbee.send(tx);
    number_of_packet++;
  }
  DEBUG.print("number_of_packet -> ");
  DEBUG.println(number_of_packet);
  return;
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
      if (AlarmFlag == false)
        Alarm_ON();
      DEBUG.println("WireSerial Buffer Overflow! need increase buffer!");
      // Ultimately you may need to just increase your recieve buffer via the
      // template parameters (see the README.md).
    }
  }
  else  // WireConnectionFlag_TX == false
  {
    //read input Telemetry from Mbee
    // Telemetry from WireConnect discribed on -> onPacketReceived(const uint8_t* buffer, size_t size)
    if (WireConnectionFlag_TX != true)
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
  }
  return;
}


// In this example, we  will only simply print the contents of the array=)
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag_TX == false)
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

  // Fill Telemetry_Pack_From_RXtempBuffer
  Fill_Telemetry_Pack_From_RX(tempBuffer, size);
  // Print our temporaray buffer.
  Print_Telemetry_Packet(Telemetry_Pack_From_RX, size);

  //

  // Send the reversed buffer back to the sender. The send() method will encode
  // the whole buffer as as single packet, set packet markers, etc.
  // The `tempBuffer` is a pointer to the `tempBuffer` array and `size` is the
  // number of bytes to send in the `tempBuffer`.
  //myPacketSerial.send(tempBuffer, size);
  return;
}

void Fill_Telemetry_Pack_From_RX(const uint8_t* buffer, size_t size)
{
  DEBUG.println("Fill the Telemetry_Pack");
  for (int i = 0 ; i < size ; i++)
  {
    Telemetry_Pack_From_RX[i] = buffer[i];
  }
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

void Processing_Mbee_Telemetry_Packet()
{
  // Print API ID
  DEBUG.println("TX Packet from Rx READED, NICE!");
  DEBUG.print("API ID === ");
  DEBUG.println( mbee.getResponse().getApiId() );

  // Print RSSI
  DEBUG.print("RSSI -> ");
  DEBUG.println(rx.getRssi());

  //    Debug_information_About_Rx_Packet(); // debug info

  // Fill Telemetry_Pack_From_RX
  if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME)
  {
    // Alarm check
    if (AlarmFlag == true) Alarm_OFF();

    // Processing
    mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными. //вписываем данные в "rx"
    if (rx.getDataLength() > 1)
    {
      int j = 0;
      DEBUG.println("TX Packet have many Symbols");
      // 0 - 7 system info , 8 - X important info
      for (int i = 0 ; i < rx.getDataLength() ; i++ )
      {
        //print all Array
        if ( i >= 0)
        {
          DEBUG.print("Value [");
          DEBUG.print(i);
          DEBUG.print("] = ");
          DEBUG.println(rx.getData()[i]);
        }

        //Fill Telemetry Pack
        if ( i >= 8 )
        {
          Telemetry_Pack_From_RX[j] = rx.getData()[i];
          j++;
        }
      }
    }
    else
    {
      DEBUG.println("1 symbols from RX");
    }
    Calculate_Data_From_Telemetry_Pack();
    //Printing_Values_On_A_PC_Monitor();
  }
  else
    DEBUG.println("Unexpected API ID from RX");

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

void Calculate_Data_From_Telemetry_Pack()
{
  DEBUG.println("Calculate_Data_From_Telemetry_Pack()...");
  int temp = 0;
  //1-2
  Speed_1_raw = Telemetry_Pack_From_RX[1] + (Telemetry_Pack_From_RX[2] << 8);
  Speed_1_RPM = (540 - temp) * ADC_Speed_RPM_Step; //because invertor after ADC
  //3-4
  Speed_2_raw = Telemetry_Pack_From_RX[3] + (Telemetry_Pack_From_RX[4] << 8);
  Speed_2_RPM = (540 - temp) * ADC_Speed_RPM_Step;
  //5-6
  I_1_raw = Telemetry_Pack_From_RX[5] + (Telemetry_Pack_From_RX[6] << 8);
  I_1_A = (550 - temp) * ADC_Current_Step;
  //7-8
  I_2_raw = Telemetry_Pack_From_RX[7] + (Telemetry_Pack_From_RX[8] << 8);
  I_2_A = temp * ADC_Current_Step;
  //9-10 Get Roper_Voltage
  V_Roper_raw = Telemetry_Pack_From_RX[9] + (Telemetry_Pack_From_RX[10] << 8); //0-1023 from ADC
  Battery_Roper_Voltage = V_Roper_raw * ADC_Climber_Voltage_Step;              //0-6 V
  //11
  PWM_Value_Percent = Telemetry_Pack_From_RX[11] * ADC_PWM_Step;
  //12
  Roper_Direction = Telemetry_Pack_From_RX[12];  // 0-S;2-B;5-F
  //Get RSSI 12+1
  RSSI_LvL = rx.getRssi();
  //Get RC Voltage
  //V_RC_raw - 0 -1023
  Battery_RC_Voltage = V_RC_raw * ADC_RC_Voltage_Step; // 0-24 V

  return;
}


void Printing_Values_On_A_PC_Monitor()
{
  DEBUG.print("Speed_1=");
  DEBUG.print(Speed_1_RPM);
  DEBUG.print("~Speed_2=");
  DEBUG.print(Speed_2_RPM);
  DEBUG.print("~I_1=");
  DEBUG.print(I_1_A);  //I_1_A
  DEBUG.print("~I_2=");
  DEBUG.print(I_2_A);  //I_2_A
  DEBUG.print("~V_Roper=");
  DEBUG.print(Battery_Roper_Voltage);
  DEBUG.print("~PWM_Value=");
  DEBUG.print(PWM_Value_Percent);
  DEBUG.print("~Direction=");
  DEBUG.print(Roper_Direction);
  // GO TO  //mb REWORK this V_Control_Panel !!!!
  DEBUG.print("~Battery_RC_Voltage=");  // x.y volt  5.2 V
  DEBUG.print( Battery_RC_Voltage );
  DEBUG.print("~RSSI=");
  DEBUG.print(rx.getRssi());
  // End Marker
  DEBUG.println("~ #");
  DEBUG.println();

  delay(30);
  return;
}





//Display Functions
///OLED_Set_Start_Settings
void OLED_Display_Const_Data()
{
  //Zero line
  //  disp.setCursor(0, 0);
  //  disp.print("TEST");

  //Second line
  disp.setCursor(0, 1);
  disp.print("SPD ");
  disp.setCursor(9, 1);
  disp.print("%");
  disp.setCursor(16, 1);
  disp.print("%");

  //Third line
  disp.setCursor(0, 2);
  disp.print("LOAD");
  disp.setCursor(9 , 2);
  disp.print("%");
  disp.setCursor(16 , 2);
  disp.print("%");

  //Fourth line
  disp.setCursor(0, 3);
  disp.print("BAT");
  disp.setCursor(5, 3);
  disp.write(byte(2));  //symbol Climber
  disp.setCursor(9, 3);
  disp.print("%");
  disp.setCursor(12, 3);
  disp.write(byte(3));  //symbol RC
  disp.setCursor(16, 3);
  disp.print("%");
  return;
}

void OLED_Display_Dynamic_Data()
{
  //mute blink
  disp.noBlink();
  uint8_t RSSI_LvL_for_Display = 0;
  int offset = 0;
  int LvL = 0;
  //  Calculate_Speed(&S1, &S2);
  //  Calculate_Current(&CUR1, &CUR2);
  //  Calculate_BatteryLevel(&Bat_Climb, &Bat_RC);

  //Second line
  String buffer1 = "-100";
  String buffer2 = "-200"; // goto
  buffer1 = Compiler_for_OLED (Speed_1_raw);
  buffer2 = Compiler_for_OLED (Speed_2_raw);
  disp.setCursor(5, 1);
  disp.print(buffer1);
  disp.setCursor(12, 1);
  disp.print(buffer2);

  //Third line // goto
  buffer1 = Compiler_for_OLED (I_1_raw);
  buffer2 = Compiler_for_OLED (I_2_raw);
  disp.setCursor(5, 2);
  disp.print(buffer1);
  disp.setCursor(12, 2);
  disp.print(buffer2);

  //Fourth line // goto
  buffer1 = Compiler_Battery_LvL_for_OLED(V_Roper_raw);
  buffer2 = Compiler_Battery_LvL_for_OLED(V_RC_raw);
  disp.setCursor(6, 3);
  disp.print(buffer1);
  disp.setCursor(13, 3);
  disp.print(buffer2);
  //  int minusVal = random (-100, 100);
  //  String small = "-100";
  RSSI_LvL_for_Display = RSSI_Processing();
  if (WireConnectionFlag_TX == false)
    RSSI_plot(RSSI_LvL_for_Display);
  else
    Wire_Symbol_plot();
  delay(1);
  return;
}

String Compiler_Battery_LvL_for_OLED(int input)
{
  String small = "100";
  if (input >= 0)
  {
    small = String(input);
    if (input >= 0 & input < 10)
      small = "  " + String(input);
    if (input >= 10 & input < 100)
      small = " " + String(input);
    if (input >= 100)
      small = String(input);
    delay(1);
    return small;
  }
  else
    return "ERR";
}

String Compiler_for_OLED(int input)
{
  String small = "-100";
  // compile the buffer
  if (input < 0)
  {
    small = String(input);
    if (input < 0 & input > -10)
      small = "  " + String(input);
    if (input <= -10 & input > -100)
      small = " " + String(input);
    if (input <= -100)
      small = String(input);
    delay(5);
  }
  else // minus val >= 0
  {
    small = String(input);
    if (input >= 0 & input < 10)
      small = "   " + String(input);
    if (input >= 10 & input < 100)
      small = "  " + String(input);
    if (input >= 100)
      small = " " + String(input);
    delay(5);
  }
  return small;
}

uint8_t RSSI_Processing()
{
  uint8_t LvL = 0;
  if (RSSI_LvL >= 0 & RSSI_LvL < 32)
    LvL = 1;
  if (RSSI_LvL >= 32 & RSSI_LvL < 64)
    LvL = 2;
  if (RSSI_LvL >= 64 & RSSI_LvL < 96)
    LvL = 3;
  if (RSSI_LvL >= 96 & RSSI_LvL < 128)
    LvL = 4;
  if (RSSI_LvL >= 128 & RSSI_LvL < 160)
    LvL = 5;
  if (RSSI_LvL >= 160 & RSSI_LvL < 192)
    LvL = 6;
  if (RSSI_LvL >= 192 & RSSI_LvL < 224)
    LvL = 7;
  if (RSSI_LvL >= 224 & RSSI_LvL < 255)
    LvL = 8;
  return LvL;
}

// Plot
void RSSI_plot(int LvL)
{
  if (LvL == 1 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
    // disp.setCursor(16, 0);
    // disp.blink();
  }
  if (LvL == 2 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
  }
  if (LvL == 3 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
    disp.setCursor(17, 0);
    //    disp.blink();
  }
  if (LvL == 4 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
  }
  if (LvL == 5 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
    disp.setCursor(18, 0);
    //    disp.blink();
  }
  if (LvL == 6 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
  }
  if (LvL == 7 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
    disp.setCursor(19, 0);
    //    disp.blink();
  }
  if (LvL == 8 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
  }
  return;
}

void Wire_Symbol_plot()
{
  //GOTO maybe plot special symbols? like "-==-"
  disp.setCursor(16, 0);
  disp.print("WIRE");
  return;
}

void reset_textmode() //функция для установки графического режима
{
  disp.command(0x08);//выключили экран
  disp.command(0x17);//переключение в текстовый режим
  disp.command(0x01);//очистили от мусора ОЗУ
  disp.command(0x04 | 0x08);//включили экран
  return;
}

//
