# QuadSoftSerial (4 serial port for arduino)

# my Software serial, include 4 transmit port, 1 receive port

No more serial port? The orginal softwareserial don't have transmate/receive buffer. so, it cann't work at same time. During transmitting, the received data will be miss.


## this is my software serial: 

- Working at 9600~38400 bps(38400bps for 16mhz) 
- 4 transmit ports and 1 receiving ports 
- Simultaneous sending and receiving 
- The transmit must work at the same baud rate 
- Tested on arduino pro mini 8/16MHz 
- Use timet1 interrupt for transmission 
- Use pin change interrupt and Timer2 to receive

## how to install

this is not a arduino library, just copy all file into your project folder.


## how to use?

QuadSerialDef.h:

    //#define RX_1_PIN IO_RXD
    //#define TX_1_PIN IO_TXD
    #define TX_2_PIN 8
    #define TX_3_PIN 9
    //#define TX_3_PIN 9
    //#define TX_4_PIN 11
    
    //comment out the port that you're not use
    //define the transmate pin:  TX_1_PIN ... TX_4_PIN
    //define the receive pin:  RX_1_PIN    

example code here:

```cpp
#include "QuadSerial.h"
#include "QuadSerialDef.h"


// use must define RX_1_PIN, TX_1_PIN~TX_4_PIN in "myTxDef.h"

SoftwareSerial mySerial2(0, TX_2_PIN);
SoftwareSerial mySerial3(0, TX_3_PIN);

void setup()
{
  mySerial2.begin(38400);
  mySerial3.begin(2400);

  mySerial2.print("22222222220123456789");

```

## Chinese:

- 工作在9600〜38400 bps
- 4個發送和1個接收端口
- 同時發送和接收
- 傳輸必須以相同的波特率工作
- 在arduino pro mini 16MHz上測試
- 使用 timet1 interrupt 進行傳輸
- 使用 pin change interrupt 和Timer2接收
