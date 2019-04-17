#include <SoftwareSerial.h>

//update from SAnwandter
#define ROW_1 2
#define ROW_2 3
#define ROW_2 3
#define ROW_3 4
#define ROW_4 5
#define ROW_5 6
#define ROW_6 7
#define ROW_7 8
#define ROW_8 9

#define COL_1 10
#define COL_2 11
#define COL_3 12
#define COL_4 13
#define COL_5 A0
#define COL_6 A1
#define COL_7 A2
#define COL_8 A3

const byte rows[] = {
    ROW_1, ROW_2, ROW_3, ROW_4, ROW_5, ROW_6, ROW_7, ROW_8
};
const byte col[] = {
  COL_1,COL_2, COL_3, COL_4, COL_5, COL_6, COL_7, COL_8
};

// The display buffer
// It's prefilled with a smiling face (1 = ON, 0 = OFF)
byte ALL[] = {B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111};
byte EX[]  = {B00000000,B00010000,B00010000,B00010000,B00010000,B00000000,B00010000,B00000000};
byte A[]   = {B00000000,B00111100,B01100110,B01100110,B01111110,B01100110,B01100110,B01100110};
byte B[]   = {B01111000,B01001000,B01001000,B01110000,B01001000,B01000100,B01000100,B01111100};
byte C[]   = {B00000000,B00011110,B00100000,B01000000,B01000000,B01000000,B00100000,B00011110};
byte D[]   = {B00000000,B00111000,B00100100,B00100010,B00100010,B00100100,B00111000,B00000000};
byte E[]   = {B00000000,B00111100,B00100000,B00111000,B00100000,B00100000,B00111100,B00000000};
byte F[]   = {B00000000,B00111100,B00100000,B00111000,B00100000,B00100000,B00100000,B00000000};
byte G[]   = {B00000000,B00111110,B00100000,B00100000,B00101110,B00100010,B00111110,B00000000};
byte H[]   = {B00000000,B00100100,B00100100,B00111100,B00100100,B00100100,B00100100,B00000000};
byte I[]   = {B00000000,B00111000,B00010000,B00010000,B00010000,B00010000,B00111000,B00000000};
byte J[]   = {B00000000,B00011100,B00001000,B00001000,B00001000,B00101000,B00111000,B00000000};
byte K[]   = {B00000000,B00100100,B00101000,B00110000,B00101000,B00100100,B00100100,B00000000};
byte L[]   = {B00000000,B00100000,B00100000,B00100000,B00100000,B00100000,B00111100,B00000000};
byte M[]   = {B00000000,B00000000,B01000100,B10101010,B10010010,B10000010,B10000010,B00000000};
byte N[]   = {B00000000,B00100010,B00110010,B00101010,B00100110,B00100010,B00000000,B00000000};
byte O[]   = {B00000000,B00111100,B01000010,B01000010,B01000010,B01000010,B00111100,B00000000};
byte P[]   = {B00000000,B00111000,B00100100,B00100100,B00111000,B00100000,B00100000,B00000000};
byte Q[]   = {B00000000,B00111100,B01000010,B01000010,B01000010,B01000110,B00111110,B00000001};
byte R[]   = {B00000000,B00111000,B00100100,B00100100,B00111000,B00100100,B00100100,B00000000};
byte S[]   = {B00000000,B00111100,B00100000,B00111100,B00000100,B00000100,B00111100,B00000000};
byte T[]   = {B00000000,B01111100,B00010000,B00010000,B00010000,B00010000,B00010000,B00000000};
byte U[]   = {B00000000,B01000010,B01000010,B01000010,B01000010,B00100100,B00011000,B00000000};
byte V[]   = {B00000000,B00100010,B00100010,B00100010,B00010100,B00010100,B00001000,B00000000};
byte W[]   = {B00000000,B10000010,B10010010,B01010100,B01010100,B00101000,B00000000,B00000000};
byte X[]   = {B00000000,B01000010,B00100100,B00011000,B00011000,B00100100,B01000010,B00000000};
byte Y[]   = {B00000000,B01000100,B00101000,B00010000,B00010000,B00010000,B00010000,B00000000};
byte Z[]   = {B00000000,B00111100,B00000100,B00001000,B00010000,B00100000,B00111100,B00000000}; 

byte LETTERS[]   = {
  B00000000,B00111100,B01100110,B01100110,B01111110,B01100110,B01100110,B01100110,
  B01111000,B01001000,B01001000,B01110000,B01001000,B01000100,B01000100,B01111100,
  B00000000,B00011110,B00100000,B01000000,B01000000,B01000000,B00100000,B00011110,
  B00000000,B00111000,B00100100,B00100010,B00100010,B00100100,B00111000,B00000000,
  B00000000,B00111100,B00100000,B00111000,B00100000,B00100000,B00111100,B00000000,
  B00000000,B00111100,B00100000,B00111000,B00100000,B00100000,B00100000,B00000000,
  B00000000,B00111110,B00100000,B00100000,B00101110,B00100010,B00111110,B00000000,
  B00000000,B00100100,B00100100,B00111100,B00100100,B00100100,B00100100,B00000000,
  B00000000,B00111000,B00010000,B00010000,B00010000,B00010000,B00111000,B00000000,
  B00000000,B00011100,B00001000,B00001000,B00001000,B00101000,B00111000,B00000000,
  B00000000,B00100100,B00101000,B00110000,B00101000,B00100100,B00100100,B00000000,
  B00000000,B00100000,B00100000,B00100000,B00100000,B00100000,B00111100,B00000000,
  B00000000,B00000000,B01000100,B10101010,B10010010,B10000010,B10000010,B00000000,
  B00000000,B00100010,B00110010,B00101010,B00100110,B00100010,B00000000,B00000000,
  B00000000,B00111100,B01000010,B01000010,B01000010,B01000010,B00111100,B00000000,
  B00000000,B00111000,B00100100,B00100100,B00111000,B00100000,B00100000,B00000000,
  B00000000,B00111100,B01000010,B01000010,B01000010,B01000110,B00111110,B00000001,
  B00000000,B00111000,B00100100,B00100100,B00111000,B00100100,B00100100,B00000000,
  B00000000,B00111100,B00100000,B00111100,B00000100,B00000100,B00111100,B00000000,
  B00000000,B01111100,B00010000,B00010000,B00010000,B00010000,B00010000,B00000000,
  B00000000,B01000010,B01000010,B01000010,B01000010,B00100100,B00011000,B00000000,
  B00000000,B00100010,B00100010,B00100010,B00010100,B00010100,B00001000,B00000000,
  B00000000,B10000010,B10010010,B01010100,B01010100,B00101000,B00000000,B00000000,
  B00000000,B01000010,B00100100,B00011000,B00011000,B00100100,B01000010,B00000000,
  B00000000,B01000100,B00101000,B00010000,B00010000,B00010000,B00010000,B00000000,
  B00000000,B00111100,B00000100,B00001000,B00010000,B00100000,B00111100,B00000000,
  B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111}; 

int timeCount = 0;

void setup() 
{
    // Open serial port
    Serial.begin(9600);
    
    // Set all used pins to OUTPUT
    // This is very important! If the pins are set to input
    // the display will be very dim.
#define ROW_1 2
    for (byte i = 2; i <= 13; i++)
        pinMode(i, OUTPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
}

void test_serial()
{
  // delayMicroseconds(1000);

  if (Serial.available())
  {
    char* input = Serial.read();
    char sep[] = "|";
 
    int count = 0;
    char* tab[10] = { 0 };
 
    byte x = 0;
    byte y = 0;

    char* token = strtok(input, sep);
    x = atoi(token);
    token = strtok (NULL, sep);
    y = atoi(token);

    /*
    while ( token != NULL ) {
      if (count < 10) {
          int val = atoi(token);
          if (count == 0) x = val;
          if (count == 1) y = val;
          //Serial.print(token);
          //Serial.print(" val = ");
          //Serial.println(val);
          count += 1;
      } else {
          break;
      }
      token = strtok (NULL, sep);
    }
    */

    x = random(256);
    y = random(256);

    int c = coord2row(y, 0, 256);
    int r = coord2row(x, 0, 256);

    drawPoint(c, r);
  }
}

int loop_letters(int t)
{  
  int delta = 200;

  int letter_index = int(t / delta);
  if (letter_index < 26)
  {
    // Serial.println(letter_index);
    // Serial.println(t);
    drawScreen(letter_index * 8);
    delayMicroseconds(100);
  } 
  else {
  // back to the start
    t = 0;
  }

  return t;
}

void loop() {
  // This could be rewritten to not use a delay, which would make it appear brighter
  delay(2);
  timeCount += 1;
  
  int mode = 2;

  if (mode == 1)
  {
    timeCount = loop_letters(timeCount);
  }

  if (mode == 2)
  {
    int delta = 1000;
    int run_it = timeCount % delta;
    int N = 1;
    if (run_it == 0)
    {
      byte c[N];
      byte r[N];
      for (int i = 0; i < N; i++)
      {
        int x = random(256);
        int y = random(256);
          c[i] = coord2row(x, 0, 256);
          r[i] = coord2row(y, 0, 256);
          // c[i] = random(8);
          // r[i] = random(8);
          /*
          Serial.print("x=");
          Serial.print(x);
          Serial.print(" y=");
          Serial.print(y);

          Serial.print(" c=");
          Serial.print(c[i]);
          Serial.print(" r=");
          Serial.println(r[i]);
          */
      }
      drawPoints(N, c, r);
    } 
    else {
    // back to the start
      timeCount = 0;
    }
  }

  if (mode == 3)
  {
    test_serial();
  }
}

void drawScreen(int index)
{ 
   // Turn on each row in series
    for (byte r = 0; r < 8; r++)        // count next row
     {
        digitalWrite(rows[r], HIGH);    //initiate whole row
        for (byte c = 0; c < 8; c++)    // count next row
        {
          // if You set (~buffer2[r] >> c) then You will have positive
          digitalWrite(col[c], (~LETTERS[index + r] >> c) & 0x01); // initiate whole column
          // delay(100);       // uncoment deley for diferent speed of display
          for (int t = 0; t < 10; t++)
          {
            delayMicroseconds(10);
          }
          
          digitalWrite(col[c], 1);      // reset whole column
        }
        digitalWrite(rows[r], LOW);     // reset whole row
        // otherwise last row will intersect with next row
    }
}


byte coord2row(int data, int data_min, int data_max)
{
  // peut s'appliquer à une donnée x => column ou y = row 
  byte led = int((float(data - data_min) / float(data_max - data_min)) * 8.0);
  return led;
}


void drawPoints(int N, byte* column, byte* row) {
    byte LEDS[] = {
      B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000
    };

    for (int r = 0; r < 8; r++)
    {
      LEDS[r] = 0;
    }

    for (int r = 0; r < 8; r++)
    {
       for (int i = 0; i < N; i++)
       {
          if (r == row[i])
          {
             byte d = 1 << column[i];
             LEDS[r] |= d;
          }
       }
    }

    // Turn on each row in series
    for (byte r = 0; r < 8; r++)
    {
        digitalWrite(rows[r], HIGH); // initiate whole row
        for (byte c = 0; c < 8; c++) // count next col
        {
            digitalWrite(col[c], (~LEDS[r] >> c) & 0x01); // initiate whole column

            delay(1); // uncoment deley for diferent speed of display
            // delayMicroseconds(30000);
            // delayMicroseconds(10); // uncoment deley for diferent speed of display
            digitalWrite(col[c], 1); // reset whole column
        }
        digitalWrite(rows[r], LOW); // reset whole row
        // otherwise last row will intersect with next row
    }
}

void drawPoint(byte column, byte row) {
    byte LEDS[] = {
      B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000
    };

    for (int r = 0; r < 8; r++)
    {
      LEDS[r] = 0;
    }

    for (int r = 0; r < 8; r++)
    {
      if (r == row)
      {
         byte d = 1 << column;
         LEDS[r] |= d;
      }
    }

    // Turn on each row in series
    for (byte r = 0; r < 8; r++)
    {
        digitalWrite(rows[r], HIGH); // initiate whole row
        for (byte c = 0; c < 8; c++) // count next col
        {
            digitalWrite(col[c], (~LEDS[r] >> c) & 0x01); // initiate whole column

            delay(1); // uncoment deley for diferent speed of display
            // delayMicroseconds(30000);
            // delayMicroseconds(10); // uncoment deley for diferent speed of display
            digitalWrite(col[c], 1); // reset whole column
        }
        digitalWrite(rows[r], LOW); // reset whole row
        // otherwise last row will intersect with next row
    }
}
