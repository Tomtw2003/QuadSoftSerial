/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) -
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
//#define _DEBUG 0
//#define _DEBUG_PIN1 11
//#define _DEBUG_PIN2 13
//
// Includes
//
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <util/delay_basic.h>

#include "QuadSerialDef.h"
#include "QuadSerial.h"
#include "fastio.h"


//int debug_cnt;

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------

#if (RX_1_PIN == 0)
#define _NO_RX_PIN

#elif ((RX_1_PIN==2) || (RX_1_PIN==3) )
#define _RX_USE_attachInterrupt // only pin 2 3

#elif (RX_1_PIN <= 7)
#define _RX_USE_pci_2

#elif (RX_1_PIN <= 13)
#define _RX_USE_pci_0

#else
#define _RX_USE_pci_1

#endif


// -------------------------------------------------------------------------------
// transmite
// -------------------------------------------------------------------------------

static uint8_t tx_div_cnt;

#if defined(TX_1_PIN)

static uint8_t tx_byte_1;
static uint8_t tx_bit_1;
static volatile uint8_t tx_buffer_head_1;
static volatile uint8_t tx_buffer_tail_1;
#define TX_BUFFER_SIZE 64
static volatile uint8_t tx_buffer_1[TX_BUFFER_SIZE];
static uint8_t tx_next_1;
static uint8_t tx_div_1;

#endif

#if defined(TX_2_PIN)

static uint8_t tx_byte_2;
static uint8_t tx_bit_2;
static volatile uint8_t tx_buffer_head_2;
static volatile uint8_t tx_buffer_tail_2;
#define TX_BUFFER_SIZE 64
static volatile uint8_t tx_buffer_2[TX_BUFFER_SIZE];
static uint8_t tx_next_2;
static uint8_t tx_div_2;

#endif


#if defined(TX_3_PIN)

static uint8_t tx_byte_3;
static uint8_t tx_bit_3;
static volatile uint8_t tx_buffer_head_3;
static volatile uint8_t tx_buffer_tail_3;
#define TX_BUFFER_SIZE 64
static volatile uint8_t tx_buffer_3[TX_BUFFER_SIZE];
static uint8_t tx_next_3;
static uint8_t tx_div_3;

#endif



#if defined(TX_4_PIN)

static uint8_t tx_byte_4;
static uint8_t tx_bit_4;
static volatile uint8_t tx_buffer_head_4;
static volatile uint8_t tx_buffer_tail_4;
#define TX_BUFFER_SIZE 64
static volatile uint8_t tx_buffer_4[TX_BUFFER_SIZE];
static uint8_t tx_next_4;
static uint8_t tx_div_4;

#endif

// ------------------------------------
//
// ------------------------------------



ISR(TIMER1_COMPA_vect)
{
// ------------------------------------
// ------------------------------------

#if defined(TX_1_PIN)
  if(tx_next_1)
     WRITE_HIGH(TX_1_PIN);
  else
     WRITE_LOW(TX_1_PIN);
#endif
#if defined(TX_2_PIN)
  if(tx_next_2)
     WRITE_HIGH(TX_2_PIN);
  else
     WRITE_LOW(TX_2_PIN);
#endif
#if defined(TX_3_PIN)
  if(tx_next_3)
     WRITE_HIGH(TX_3_PIN);
  else
     WRITE_LOW(TX_3_PIN);
#endif
#if defined(TX_4_PIN)
  if(tx_next_4)
     WRITE_HIGH(TX_4_PIN);
  else
     WRITE_LOW(TX_4_PIN);
#endif

  //tx_div_cnt = (((tx_div_cnt>>1)+1)<<1) | 1;
  tx_div_cnt++;

// ------------------------------------
// ------------------------------------
#if defined(TX_1_PIN)
  if(!(tx_div_cnt & tx_div_1))
  {
    if(tx_bit_1 == 10)
    {
       //start bit
       tx_next_1 = (tx_byte_1 & 1);
       tx_byte_1 >>= 1;
       tx_bit_1--;
       //working = 1;
    }
    else
    if(tx_bit_1==2)
    {
      // end bit
      tx_next_1 = 1;
      tx_bit_1--;
      //working = 1;
    }
    else
    if(tx_bit_1==1)
    {
      // wait
      tx_bit_1--;
      //working = 1;
    }
    else
    if(tx_bit_1)
    {
       tx_next_1 = (tx_byte_1 & 1);
       tx_byte_1 >>= 1;
       tx_bit_1--;
       //working = 1;
    }
    else
    {
    	if (tx_buffer_head_1 != tx_buffer_tail_1)
    	{
    		if (++tx_buffer_tail_1 >= TX_BUFFER_SIZE) tx_buffer_tail_1 = 0;
    		tx_byte_1 = tx_buffer_1[tx_buffer_tail_1];
    		tx_bit_1 = 10;
    		tx_next_1 = 0;
    		//working = 1;
    	}
  	}
	}
#endif

// ------------------------------------
// ------------------------------------
#if defined(TX_2_PIN)
  if(!(tx_div_cnt & tx_div_2))
  {
    if(tx_bit_2 == 10)
    {
       //start bit
       tx_next_2 = (tx_byte_2 & 1);
       tx_byte_2 >>= 1;
       tx_bit_2--;
       //working = 1;
    }
    else
    if(tx_bit_2==2)
    {
      // end bit
      tx_next_2 = 1;
      tx_bit_2--;
      //working = 1;
    }
    else
    if(tx_bit_2==1)
    {
      // wait
      tx_bit_2--;
      //working = 1;
    }
    else
    if(tx_bit_2)
    {
       tx_next_2 = (tx_byte_2 & 1);
       tx_byte_2 >>= 1;
       tx_bit_2--;
       //working = 1;
    }
    else
    {
    	if (tx_buffer_head_2 != tx_buffer_tail_2)
    	{
    		if (++tx_buffer_tail_2 >= TX_BUFFER_SIZE) tx_buffer_tail_2 = 0;
    		tx_byte_2 = tx_buffer_2[tx_buffer_tail_2];
    		tx_bit_2 = 10;
    		tx_next_2 = 0;
    		//working = 1;
    	}
  	}
  }
#endif
// ------------------------------------
// ------------------------------------
#if defined(TX_3_PIN)
  if(!(tx_div_cnt & tx_div_3))
  {
    if(tx_bit_3 == 10)
    {
       //start bit
       tx_next_3 = (tx_byte_3 & 1);
       tx_byte_3 >>= 1;
       tx_bit_3--;
       //working = 1;
    }
    else
    if(tx_bit_3==2)
    {
      // end bit
      tx_next_3 = 1;
      tx_bit_3--;
      //working = 1;
    }
    else
    if(tx_bit_3==1)
    {
      // wait
      tx_bit_3--;
      //working = 1;
    }
    else
    if(tx_bit_3)
    {
       tx_next_3 = (tx_byte_3 & 1);
       tx_byte_3 >>= 1;
       tx_bit_3--;
       //working = 1;
    }
    else
    {
    	if (tx_buffer_head_3 != tx_buffer_tail_3)
    	{
    		if (++tx_buffer_tail_3 >= TX_BUFFER_SIZE) tx_buffer_tail_3 = 0;
    		tx_byte_3 = tx_buffer_3[tx_buffer_tail_3];
    		tx_bit_3 = 10;
    		tx_next_3 = 0;
    		//working = 1;
    	}
  	}
  }
#endif

// ------------------------------------
// ------------------------------------
#if defined(TX_4_PIN)
  if(!(tx_div_cnt & tx_div_4))
  {
    if(tx_bit_4 == 10)
    {
       //start bit
       tx_next_4 = (tx_byte_4 & 1);
       tx_byte_4 >>= 1;
       tx_bit_4--;
       //working = 1;
    }
    else
    if(tx_bit_4==2)
    {
      // end bit
      tx_next_4 = 1;
      tx_bit_4--;
      //working = 1;
    }
    else
    if(tx_bit_4==1)
    {
      // wait
      tx_bit_4--;
      //working = 1;
    }
    else
    if(tx_bit_4)
    {
       tx_next_4 = (tx_byte_4 & 1);
       tx_byte_4 >>= 1;
       tx_bit_4--;
       //working = 1;
    }
    else
    {
    	if (tx_buffer_head_4 != tx_buffer_tail_4)
    	{
    		if (++tx_buffer_tail_4 >= TX_BUFFER_SIZE) tx_buffer_tail_4 = 0;
    		tx_byte_4 = tx_buffer_4[tx_buffer_tail_4];
    		tx_bit_4 = 10;
    		tx_next_4 = 0;
    		//working = 1;
    	}
  	}
  }
#endif



// ------------------------------------
// ------------------------------------

  if(!(tx_div_cnt & 127))
  {
    byte working = 0;

#if defined(TX_1_PIN)
    if(tx_bit_1 || (tx_buffer_head_1 != tx_buffer_tail_1))  working = 1;
#endif
#if defined(TX_2_PIN)
    if(tx_bit_2 || (tx_buffer_head_2 != tx_buffer_tail_2))  working = 1;
#endif
#if defined(TX_3_PIN)
    if(tx_bit_3 || (tx_buffer_head_3 != tx_buffer_tail_3))  working = 1;
#endif
#if defined(TX_4_PIN)
    if(tx_bit_4 || (tx_buffer_head_4 != tx_buffer_tail_4))  working = 1;
#endif
    if(!working)
    {
      // disable timer compare interrupt
      TIMSK1 = 0;
      //debug_cnt++;
    }
  }

}


// ------------------------------------
//
// ------------------------------------


static void mySoftwareTx_init(long speed)
{
  cli();//stop interrupts

  //set timer1 interrupt

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0

  // set compare match register
	//OCR1A = 1666; 	   //16m/9600=1667 delay

	//clock_div = speed / BASE_TX_BAUD_RATE;
	OCR1A = (F_CPU / BASE_TX_BAUD_RATE);
	//OCR1A = (F_CPU / speed);
	//OCR1A = ((F_CPU+speed/2) / speed);

  // Set prescaler
	//TCCR1B = (1<<CS10) | (1<<CS12); //set the pre-scalar as 1024
	TCCR1B = (1<<CS10) | //set freg as CLK
           (1 << WGM12); // turn on CTC mode

	//initialize counter value to 0
	TCNT1 = 0;

  // enable timer compare interrupt
  //TIMSK1 = (1 << OCIE1A);

  // disable timer compare interrupt
  TIMSK1 = 0;

	//IFR |= (1<<OCF1A) ; //clear timer1 overflow flag

  sei();//allow interrupts

#if defined(TX_1_PIN)
	tx_buffer_head_1 = 0;
	tx_buffer_tail_1 = 0;
#endif

#if defined(TX_2_PIN)
	tx_buffer_head_2 = 0;
	tx_buffer_tail_2 = 0;
#endif

#if defined(TX_3_PIN)
	tx_buffer_head_3 = 0;
	tx_buffer_tail_3 = 0;
#endif

#if defined(TX_4_PIN)
	tx_buffer_head_4 = 0;
	tx_buffer_tail_4 = 0;
#endif


}


/****************************************/
/**           Transmission             **/
/****************************************/
#if defined(TX_1_PIN)

static void mySoftwareWrite_1(uint8_t b)
{
	uint8_t intr_state, head;

	head = tx_buffer_head_1 + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;
	while (tx_buffer_tail_1 == head) ; // wait until space in buffer

	//intr_state = SREG;
	//cli();
	tx_buffer_1[head] = b;
	tx_buffer_head_1 = head;
	//SREG = intr_state;

}
#endif


// --------------------------------------
#if defined(TX_2_PIN)

static void mySoftwareWrite_2(uint8_t b)
{
	uint8_t intr_state, head;

	head = tx_buffer_head_2 + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;
	while (tx_buffer_tail_2 == head) ; // wait until space in buffer

	//intr_state = SREG;
	//cli();
	tx_buffer_2[head] = b;
	tx_buffer_head_2 = head;
	//SREG = intr_state;
}

#endif

// --------------------------------------
#if defined(TX_3_PIN)

static void mySoftwareWrite_3(uint8_t b)
{
	uint8_t intr_state, head;

	head = tx_buffer_head_3 + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;
	while (tx_buffer_tail_3 == head) ; // wait until space in buffer

	//intr_state = SREG;
	//cli();
	tx_buffer_3[head] = b;
	tx_buffer_head_3 = head;
	//SREG = intr_state;

}

#endif


// --------------------------------------
#if defined(TX_4_PIN)

static void mySoftwareWrite_4(uint8_t b)
{
	uint8_t intr_state, head;

	head = tx_buffer_head_4 + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;
	while (tx_buffer_tail_4 == head) ; // wait until space in buffer

	//intr_state = SREG;
	//cli();
	tx_buffer_4[head] = b;
	tx_buffer_head_4 = head;
	//SREG = intr_state;

}

#endif



// -------------------------------------------------------------------------------
//  receive
// -------------------------------------------------------------------------------

#if !(ARDUINO > 106)

#define NOT_AN_INTERRUPT -1

#endif



static void blink();

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------

static uint8_t rx_byte;
static uint8_t rx_bit;

static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;
#define RX_BUFFER_SIZE 32
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

const uint8_t BITS_1[] = {
  0b00000000,
  0b10000000,
  0b11000000,
  0b11100000,
  0b11110000,
  0b11111000,
  0b11111100,
  0b11111110,
  0b11111111,
  0b11111111,
};

static uint8_t bittick;


// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------
//uint8_t _receiveBitMask;
//volatile uint8_t *_receivePortRegister;

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------

static void myRxBegin(long bps)
{

  int prescaler;

 if( (F_CPU/ bps) < ((256/9)*8)  )  prescaler = 8;
 else
 if( (F_CPU/ bps) < ((256/9)*32) )  prescaler = 32;
 else
 if( (F_CPU/ bps) < ((256/9)*64) )  prescaler = 64;
 else
 {
   // error
   return;
 }

 bittick = ((F_CPU/prescaler)+(bps/2)) / bps;

  // 16m / prescaler /bps = bittick ;  n must below 25.6 (256/10)
  /*
  switch(bps)
  {
     case   9600:  prescaler = 64; bittick = ((F_CPU/64)+( 9600/2))/ 9600; break;  // > 0.5  = +1
     case  19200:  prescaler = 32; bittick = ((F_CPU/32)+(19200/2))/19200; break;  // > 0.5  = +1
     case  38400:  prescaler = 32; bittick = ((F_CPU/32)+(38400/2))/38400; break;  // > 0.5  = +1
  }
  */

  cli();//stop interrupts

  //set timer1 interrupt

  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B

  if(prescaler==32)
  {
    TCCR2B = (1<<CS21) | (1<<CS20) ; //set freg as CLK /32
  }
  if(prescaler==64)
  {
    TCCR2B = (1<<CS22) ; //set freg as CLK /64
  }

	//initialize counter value to 0
	TCNT2 = 0;

  //TIMSK2 = (1 << TOIE2);
  TIMSK2 = 0;  // disable timer interrupt

	TIFR2 |= (1<<TOV2) ; //clear timer1 overflow flag

  sei();//allow interrupts


	rx_buffer_head = 0;
	rx_buffer_tail = 0;

  rx_byte = 0;
	rx_bit = 0;

}

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------

static void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------

/*

 中斷處理程序

ISR(PCINT0_vect){}    // Port B, PCINT0 - PCINT7    (D8 to D13)
ISR(PCINT1_vect){}    // Port C, PCINT8 - PCINT14   (A0 to A5)
ISR(PCINT2_vect){}    // Port D, PCINT16 - PCINT23  (D0 to D7)
*/

//volatile unsigned int interruptCount=0;


#if defined(_RX_USE_pci_0)
#warning "_RX_USE_pci_0".
ISR(PCINT0_vect)    // Port B, PCINT0 - PCINT7    (D8 to D13)
#elif defined(_RX_USE_pci_1)
#warning "_RX_USE_pci_1".
ISR(PCINT1_vect)    // Port C, PCINT8 - PCINT14   (A0 to A5)
#elif defined(_RX_USE_pci_2)
#warning "_RX_USE_pci_2".
ISR(PCINT2_vect)    // Port D, PCINT16 - PCINT23  (D0 to D7)
#elif defined(_RX_USE_attachInterrupt)
#warning "_RX_USE_attachInterrupt"
void blink()
#endif

//void blink()
//ISR(PCINT2_vect)

#ifndef _NO_RX_PIN
{

//interruptCount++;

  uint8_t head;
  uint8_t t2 = TCNT2;

  //if(TIFR2 & (1<<TOV2)) //timer1 overflow
  //  t2 |= 0x100;

  TCNT2 = 0;
  TIFR2 |= (1<<TOV2) ; //clear timer1 overflow flag
  // enable timer interrupt
  TIMSK2 = (1 << TOIE2);

  //

  //uint8_t bits = (((uint16_t)t2*16)+(bittick16/2)) / bittick16; // > 0.5 , then +1
  uint8_t bits = (((uint16_t)t2)+(bittick/2)) / bittick; // > 0.5 , then +1
  //8.680555556

  if(bits==0) return;

  if(READ(RX_1_PIN))

  //if(*_receivePortRegister & _receiveBitMask)
  {
    if((rx_bit + bits) >= 9)
    {
        bits = 9 - rx_bit;
        rx_byte >>= bits;
        goto _DOWN_BYTE;  // 完成 1 byte
    }
    else
    {
      rx_bit += bits;
      rx_byte >>= bits;
    }
  }
  else
  {
    // 低 LOW
    if(rx_bit==0)
    {
      // start bit
      rx_bit = 0;
      rx_byte = 0;
    }
    else
    {
      if((rx_bit + bits) >= 9)
      {
        bits = 9 - rx_bit;
        rx_byte >>= bits;
        rx_byte |= BITS_1[bits];
        goto _DOWN_BYTE;  // 完成 1 byte
      }
      else
      {
        rx_bit += bits;
        rx_byte >>= bits;
        rx_byte |= BITS_1[bits];
      }
    }
  }


return;

_DOWN_BYTE:

  rx_buffer_head++;
	if (rx_buffer_head >= RX_BUFFER_SIZE) rx_buffer_head = 0;
	if (rx_buffer_head != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = rx_byte;
		//rx_buffer_head = head;
	}
	// new start bit
  rx_bit = 0;
  rx_byte = 0;

  TIMSK2 = 0;  // disable timer interrupt

return;

}
#endif


// -------------------------------------------------------------------------------
//   處理 overflow 或是 end bit
// -------------------------------------------------------------------------------

#ifndef _NO_RX_PIN
ISR(TIMER2_OVF_vect)
{
  uint8_t bits,head;

  if(READ(RX_1_PIN))
  //if(*_receivePortRegister & _receiveBitMask)
  {
    // 收到 end bit 資料
    bits = 9 - rx_bit;
    rx_byte >>= bits;
    rx_byte |= BITS_1[bits];
    rx_buffer_head++;
		if (rx_buffer_head >= RX_BUFFER_SIZE) rx_buffer_head = 0;
		if (rx_buffer_head != rx_buffer_tail)
		{
			rx_buffer[rx_buffer_head] = rx_byte;
		}
  }
  else
  {
    // error
  }

  rx_bit = 0;
  rx_byte = 0;

  //TIFR2 |= (1<<TOV2) ; //clear timer1 overflow flag
  TIMSK2 = 0;  // disable timer interrupt
}
#endif

// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------



// -------------------------------------------------------------------------------
//
// -------------------------------------------------------------------------------



//
// Statics
//
//SoftwareSerial *SoftwareSerial::active_object = 0;

uint8_t SoftwareSerial::_receivePin = 0; // only one receive port

//
// Private methods
//
// ------------------------------------
//
// ------------------------------------

//
// The receive routine called by the interrupt handler
//

//
// Interrupt handling
//

/* static */

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin)
{
  setTX(transmitPin);
  setRX(receivePin);

}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  _tx_pin = tx;

  if(!_tx_pin) return;


  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  //_transmitBitMask = digitalPinToBitMask(tx);
  //uint8_t port = digitalPinToPort(tx);
  //_transmitPortRegister = portOutputRegister(port);



  //pinMode(_tx_pin, OUTPUT);



}

void SoftwareSerial::setRX(uint8_t rx)
{
#ifndef _NO_RX_PIN
  if(!rx)  return;
  if(rx != RX_1_PIN)  return;
  if(_receivePin)  return;

  _receivePin = rx;

  pinMode(rx, INPUT);
  digitalWrite(rx, HIGH);  // pullup for normal logic!
  //_receiveBitMask = digitalPinToBitMask(rx);
  //uint8_t port = digitalPinToPort(rx);
  //_receivePortRegister = portInputRegister(port);


  //pinMode(rx, INPUT_PULLUP);

#if defined(_RX_USE_attachInterrupt)
  attachInterrupt(digitalPinToInterrupt(rx), blink, CHANGE);
#elif defined(_RX_USE_pci_0) || defined(_RX_USE_pci_1)  || defined(_RX_USE_pci_2)
  pciSetup(rx);
#endif

#endif

}




//
// Public methods
//



void SoftwareSerial::begin(long speed)
{

    // Enable the PCINT for the entire port here, but never disable it
    // (others might also need it, so we disable the interrupt by using
    // the per-pin PCMSK register).
    //*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    // Precalculate the pcint mask register and value, so setRxIntMask
    // can be used inside the ISR without costing too much time.
    //_pcint_maskreg = digitalPinToPCMSK(_receivePin);
    //_pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

    mySoftwareTx_init(speed);

#if defined(TX_1_PIN)
  if(_tx_pin == TX_1_PIN)
  {
    tx_div_1 = (BASE_TX_BAUD_RATE / speed)-1;
  }
#endif
#if defined(TX_2_PIN)
  if(_tx_pin == TX_2_PIN)
  {
    tx_div_2 = (BASE_TX_BAUD_RATE / speed)-1;
  }
#endif
#if defined(TX_3_PIN)
  if(_tx_pin == TX_3_PIN)
  {
    tx_div_3 = (BASE_TX_BAUD_RATE / speed)-1;
  }
#endif
#if defined(TX_4_PIN)
  if(_tx_pin == TX_4_PIN)
  {
    tx_div_4 = (BASE_TX_BAUD_RATE / speed)-1;
  }
#endif



    if(_receivePin) myRxBegin(speed);



    // ------------


//#if _DEBUG
//  pinMode(_DEBUG_PIN1, OUTPUT);
//  pinMode(_DEBUG_PIN2, OUTPUT);
//#endif

  //listen();
}

/*
void SoftwareSerial::setRxIntMsk(bool enable)
{
    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;
}
*/

void SoftwareSerial::end()
{
  //stopListening();
}


// Read data from buffer
int SoftwareSerial::read()
{
	uint8_t tail, out;

	tail = rx_buffer_tail;

	if (rx_buffer_head != tail)
	{
  	if (++tail >= RX_BUFFER_SIZE) tail = 0;
  	out = rx_buffer[tail];
  	rx_buffer_tail = tail;
    return out;
  }
}

int SoftwareSerial::available()
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head >= tail) return head - tail;
	return RX_BUFFER_SIZE + head - tail;

  //if (!isListening())
//    return 0;

  //return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
#if defined(TX_1_PIN)
  if(_tx_pin == TX_1_PIN)
  {
    mySoftwareWrite_1(b);
  }
#endif
#if defined(TX_2_PIN)
  if(_tx_pin == TX_2_PIN)
  {
    mySoftwareWrite_2(b);
  }
#endif
#if defined(TX_3_PIN)
  if(_tx_pin == TX_3_PIN)
  {
    mySoftwareWrite_3(b);
  }
#endif
#if defined(TX_4_PIN)
  if(_tx_pin == TX_4_PIN)
  {
    mySoftwareWrite_4(b);
  }
#endif

  if(!(TIMSK1 & (1 << OCIE1A)))
  {
  	//initialize counter value to 0
  	TCNT1 = 0;

    // enable timer compare interrupt
    TIMSK1 = (1 << OCIE1A);
  }
}

void SoftwareSerial::flush()
{
  // There is no tx buffering, simply return
}

int SoftwareSerial::peek()
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	return rx_buffer[tail];

}


