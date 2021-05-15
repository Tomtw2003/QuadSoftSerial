# QuadSoftSerial

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


## Chinese:

- 工作在9600〜38400 bps
- 4個發送和1個接收端口
- 同時發送和接收
- 傳輸必須以相同的波特率工作
- 在arduino pro mini 16MHz上測試
- 使用 timet1 interrupt 進行傳輸
- 使用 pin change interrupt 和Timer2接收
