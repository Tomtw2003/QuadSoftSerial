//#include<avr/io.h>
//#include <util/delay.h>

#include "fastio.h"

#include "QuadSerial.h"
#include "QuadSerialDef.h"


// use must define RX_1_PIN, TX_1_PIN~TX_4_PIN in "myTxDef.h"



//SoftwareSerial mySerial1(RX_1_PIN, TX_1_PIN);
SoftwareSerial mySerial2(0, TX_2_PIN);
SoftwareSerial mySerial3(0, TX_3_PIN);

//extern volatile unsigned int interruptCount;


void setup()
{

  pinMode(13, OUTPUT);
  //pinMode(, OUTPUT);
  //pinMode(8, OUTPUT);

  digitalWrite(13,HIGH);
  //digitalWrite(9,HIGH);
  //digitalWrite(1,HIGH);

  //mySerial1.begin(9600);
  mySerial2.begin(38400);
  mySerial3.begin(2400);

  Serial.begin(9600);
  delay(500);
  Serial.println("Hello!");

  //ptr = hello;

// 9600 bps = 0.104166667 ms


}

//extern int debug_cnt;

int aa = 0;
int aa2 = 0;
void loop()
{

    //Serial.println(debug_cnt);
    //mySerial2.print("012345");
    //delay(1);
    //mySerial3.print("ABCDE");
    //delay(1);
    mySerial2.print("22222222220123456789");
    delay(8); // 38400
    mySerial3.print("33333333330123456789");
    delay(1000);





    /*mySerial1.print("12345");
    mySerial2.print("12345");
    mySerial3.print("12345");
    Serial.print("12345");
    delay(10);
    return;
    */

  /*
  if (mySerial1.available())
  {
    char c = mySerial1.read();
    Serial.write(c);
    mySerial3.write(c);
  }

  if (Serial.available())
  {
    char c = Serial.read();
    //mySerial1.write('1');
    mySerial1.write(c);
    mySerial2.write(c);
  }
  */


  //mySerial2.write('1');
  //mySoftwareWrite_2('1');        // bug --> mySerial1.read() ????


/*
if(aa!=interruptCount)
{
mySerial2.println(interruptCount);
aa = interruptCount;
}
*/

  //delay(1);



/*
digitalWrite(13,LOW);
mySoftwareWrite_1('H');
mySoftwareWrite_1('E');
mySoftwareWrite_1('L');
mySoftwareWrite_1('L');
mySoftwareWrite_1('O');
mySoftwareWrite_1(0x0d);
mySoftwareWrite_1(0x0a);

mySoftwareWrite_2('1');
mySoftwareWrite_2('2');
mySoftwareWrite_2('3');
mySoftwareWrite_2('4');
mySoftwareWrite_2('5');
mySoftwareWrite_2(0x0d);
mySoftwareWrite_2(0x0a);


digitalWrite(13,HIGH);
*/

//delay(10);

}



