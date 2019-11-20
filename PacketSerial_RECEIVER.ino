//
// Copyright (c) 2012 Christopher Baker <https://christopherbaker.net>
//
// SPDX-License-Identifier: MIT
//


#include <PacketSerial.h>
#include <SoftwareSerial.h>

//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t TelemetryArray [] = {0, 111, 22, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; // 13

// Instances of this class can recieve data packets when registered.
class MyClass
{
  public:
    void processPacketFromSender(const PacketSerial& sender, const uint8_t* buffer, size_t size)
    {
      // Just send the buffer back to the sender.
      sender.send(buffer, size);
    }
};

// By default, PacketSerial automatically wraps the built-in `Serial` object.
// While it is still possible to use the Serial object directly, it is
// recommended that the user let the PacketSerial object manage all serial
// communication. Thus the user should not call Serial.write(), Serial.print(),
// etc. Additionally the user should not use the serialEvent() framework.
//
// By default, PacketSerial uses COBS encoding and has a 256 byte receive
// buffer. This can be adjusted by the user by replacing `PacketSerial` with
// a variation of the `PacketSerial_<COBS, 0, BufferSize>` template found in
// PacketSerial.h.
PacketSerial myPacketSerial;


// Note that SoftwareSerial is not compatible with SAMD_ZERO

// An additional PacketSerial instance.
SoftwareSerial RX_DebugSerial(8, 9);

// An instance of our custom class.
MyClass myClassInstance;

bool need_send_answer = false;

void setup()
{
  // We begin communication with our PacketSerial object by setting the
  // communication speed in bits / second (baud).
  myPacketSerial.begin(9600);

  // If we want to receive packets, we must specify a packet handler function.
  // The packet handler is a custom function with a signature like the
  // onPacketReceived function below.
  myPacketSerial.setPacketHandler(&onPacketReceived);

  // Set up a second custom Serial connection (SOFTWARE)
  RX_DebugSerial.begin(9600);
  RX_DebugSerial.println("RX_Start Debug");

  //  uint8_t Arra[2] = { 600, 10 };
  //  uint8_t StartArra[1] = { 42  };
  //  myPacketSerial.send(Arra , 2);
  //  myOtherPacketSerial.send(StartArra, 1);
}


void loop()
{
  // Do your program-specific loop() work here as usual.
  if (need_send_answer == true)
  {
    send_telemetry();
  }

  // The PacketSerial::update() method attempts to read in any incoming serial
  // data and emits received and decoded packets via the packet handler
  // function specified by the user in the void setup() function.
  //
  // The PacketSerial::update() method should be called once per loop(). Failure
  // to call the PacketSerial::update() frequently enough may result in buffer
  // serial overflows.
  myPacketSerial.update();

  // Check for a receive buffer overflow (optional).
  if (myPacketSerial.overflow())
  {
    // Send an alert via a pin (e.g. make an overflow LED) or return a
    // user-defined packet to the sender.
    //
    // Ultimately you may need to just increase your recieve buffer via the
    // template parameters (see the README.md).
  }
}

// This is our handler callback function.
// When an encoded packet is received and decoded, it will be delivered here.
// The sender is a pointer to the sending PacketSerial instance. The `buffer` is
// a pointer to the decoded byte array. `size` is the number of bytes in the
// `buffer`.
void onPacketReceived(const void* sender, const uint8_t* buffer, size_t size)
{
  if (sender == &myPacketSerial)
  {
    // In this example, we will simply reverse the contents of the array and send
    // it back to the sender.
    // Make a temporary buffer.
    uint8_t tempBuffer[size];

    // Copy the packet into our temporary buffer.
    memcpy(tempBuffer, buffer, size);

    RX_DebugSerial.print("Buf " );
    for ( int i = 0 ; i < size ; i++)
    {
      RX_DebugSerial.print("[");
      RX_DebugSerial.print(i);
      RX_DebugSerial.print("]=");
      RX_DebugSerial.print(tempBuffer[i]);
      RX_DebugSerial.print("; ");
    }
    RX_DebugSerial.println();
    RX_DebugSerial.println(random(1000, 2000));
    
    // Reverse our temporaray buffer.
    //reverse(tempBuffer, size);

    // Send the reversed buffer back to the sender. The send() method will encode
    // the whole buffer as as single packet, set packet markers, etc.
    // The `tempBuffer` is a pointer to the `tempBuffer` array and `size` is the
    // number of bytes to send in the `tempBuffer`.

    //    myPacketSerial.send(tempBuffer, size);
    need_send_answer = true;
  }
  else
  {
    RX_DebugSerial.print("decoded but....");
    need_send_answer = true;
  }
}

//

void send_telemetry ()
{
  //fill
  RX_DebugSerial.print("TelemetryArray");
  for (int i = 0 ; i < sizeof(TelemetryArray) / sizeof(TelemetryArray[0]) ; i++)
  {
    TelemetryArray[i] = random(0, 250);
    RX_DebugSerial.print("[");
    RX_DebugSerial.print(i);
    RX_DebugSerial.print("]=");
    RX_DebugSerial.print(TelemetryArray[i]);
    RX_DebugSerial.print("; ");
  }
  RX_DebugSerial.println();
  RX_DebugSerial.println("------");
  //send
  myPacketSerial.send(TelemetryArray, sizeof(TelemetryArray) / sizeof(TelemetryArray[0]) );
  need_send_answer = false;
  return;
}

// This function takes a byte buffer and reverses it.
void reverse(uint8_t* buffer, size_t size)
{
  uint8_t tmp;

  for (size_t i = 0; i < size / 2; i++)
  {
    tmp = buffer[i];
    buffer[i] = buffer[size - i - 1];
    buffer[size - i - 1] = tmp;
  }
}
