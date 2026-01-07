#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


typedef enum PourState_e
{
  IDLE,
  POURING,
  FULL
} PourState_t;

PourState_t PourState = IDLE;

#define START_POUR() digitalWrite(relayPin, HIGH)
#define STOP_POUR() digitalWrite(relayPin, LOW)

// Ultra Sonic Sensor Declarations
//  defines pins numbers
#define TRIG_PIN 9
#define ECHO_PIN 10

//LCD
#define SDA_PIN 11
#define SCL_PIN 12

#define LCD_ADDR 0x27   // common: 0x27 or 0x3F
#define LCD_COLS 20
#define LCD_ROWS 4

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

enum SpriteSet {
  SPRITE_BEER,
  SPRITE_WALLE
};

// Beer icon...
const byte beerChars[8][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06}, // foam1
  {0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x09}, // foam2
  {0x18,0x10,0x11,0x0E,0x08,0x09,0x09,0x09}, // mug1
  {0x00,0x08,0x0F,0x10,0x00,0x0A,0x0A,0x0A}, // mug2
  {0x18,0x08,0x10,0x10,0x1E,0x13,0x11,0x11}, // handle1
  {0x09,0x09,0x09,0x09,0x09,0x08,0x08,0x07}, // mug3
  {0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x00,0x1F}, // mug4
  {0x11,0x11,0x11,0x13,0x1E,0x10,0x10,0x00}  // handle2
};


// Wall-e
const byte walleChars[8][8] = {
  {0x03,0x0C,0x16,0x16,0x10,0x0F,0x01,0x01}, // left eye
  {0x18,0x06,0x0D,0x0D,0x01,0x1E,0x10,0x10}, // right eye
  {0x07,0x1C,0x16,0x09,0x1A,0x0B,0x0C,0x18}, // left body
  {0x1C,0x07,0x0D,0x12,0x0B,0x1A,0x06,0x03}, // right body
  {0x00,0x00,0x00,0x00,0x03,0x02,0x02,0x03}, // left wheel
  {0x00,0x00,0x00,0x00,0x18,0x08,0x08,0x18}, // right wheel
  {0,0,0,0,0,0,0,0},                         // free
  {0,0,0,0,0,0,0,0}                          // free
};

byte streaml[8] = {0x03,0x03,0x03,0x03,0x03,0x03, 0x03,0x03};

byte stream2[8] = {
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18
};



// defines variables
long Duration;
int Distance;
unsigned long TimePourStarted;

// declare function
int readDistance();

// Linked List to Take Moving Average

// Drink Dispenser
// Distance Requirements & State

#define POUR_RADIUS 3  // Measured in cm
#define POUR_CENTER 20 // Measured in cm
#define POUR_DURATION_MS 3000

#define POUR_HIGH (POUR_CENTER + POUR_RADIUS) // If between POUR_HIGH
#define POUR_LOW (POUR_CENTER - POUR_RADIUS)  // and POUR_LOW then pour
#define TOO_FAR (POUR_HIGH + 1)               // If lower than this do not pour
#define TOO_CLOSE (POUR_LOW - 1)              // If greater than this do not pour

// define relay pin
#define relayPin 2

void setup()
{
  //I2C setup
  Wire.begin(SDA_PIN, SCL_PIN);

  //LCD setup
  lcd.init();
  lcd.backlight();

  //lcd.createChar(0, beerfoam1);
  //lcd.createChar(1, beerfoam2);
  //lcd.createChar(2, mug1);
  //lcd.createChar(3, mug2);
  //lcd.createChar(4, handle1);
  //lcd.createChar(5, mug3);
  //lcd.createChar(6, mug4);
  //lcd.createChar(7, handle2);

  //lcd.createChar(8, lefteye);
  //lcd.createChar(9, righteye);
  //lcd.createChar(10, leftbody);
  //lcd.createChar(11, rightbody);
  //lcd.createChar(12, leftwheel);
  //lcd.createChar(13, rightwheel);

  //Ultrasonic sensor setup 
  digitalWrite(TRIG_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output (Ultrasonice)
  pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input (Ultrasonice)

  STOP_POUR();
  pinMode(relayPin, OUTPUT); // Sets the relayPin as an Output (Relay)

  Serial.begin(9600); // Starts the serial communication

  // Defines an empty linked list use for filtering the data (????)
}

void loop()
{
  readDistance(); // Takes Distance using USS to determine state

  delay(5);

  // This if statement needs to be combined with a state machine to ensure that 2 things don't happen
  // 1. The drink stops dispensing after one dispense
  // 2. The dispenser stops when the cup is out of range no matter what

  switch (PourState)
  {
  case IDLE:
    // if (Distance >= TOO_FAR || Distance <= TOO_CLOSE)
    // {
    //   digitalWrite(relayPin, LOW); // Turns off the relay
    //   Serial.print("Status: No cup detected\n");
    // }
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print("Ready to pour!");   // left eye    

    delay(1000);

    lcd.clear();
    loadSpriteSet(beerChars);
    drawBeerMug(0,0);

    delay(1000);

    lcd.clear();
    loadSpriteSet(walleChars);
    drawWalle(6, 1);

    delay(1000);

    lcd.clear();
    loadSpriteSet(beerChars);
    drawBeerMug(16,0);

    

    if (Distance <= POUR_HIGH && Distance >= POUR_LOW)
    {
      loadSpriteSet(beerChars);
      drawBeerMug(0, 0);
      PourState = POURING;
      START_POUR(); // Turns on the relay
      TimePourStarted = millis();
      Serial.print("Status: Cup detected... Pouring!\n");

    }
    break;
  case POURING:
    // check if cup is removed or timeout exceeded
    if (Distance >= TOO_FAR || Distance <= TOO_CLOSE || millis() - TimePourStarted > POUR_DURATION_MS)
    {
      PourState = FULL;
      STOP_POUR();
    }
    break;
  case FULL:
    // check distance for rearming
    if (Distance >= TOO_FAR || Distance <= TOO_CLOSE)
    {
      PourState = IDLE;
    }
    break;
  }
}

int readDistance()
{

  // Clears the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delay(200);

  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delay(200);
  digitalWrite(TRIG_PIN, LOW);

  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  Duration = pulseIn(ECHO_PIN, HIGH);

  // Calculating the Distance
  Distance = Duration * 0.034 / 2;

  // Anounces that it is reading data
  Serial.println("Reading Distance...");
  Serial.print("Distance: ");
  Serial.println(Distance);

  return Distance;
}

void loadSpriteSet(const byte sprites[8][8])
{
  for (uint8_t i = 0; i < 8; i++)
  {
    lcd.createChar(i, (byte*)sprites[i]);
  }
}

void drawBeerMug(uint8_t x, uint8_t y) 
{
  // Top row
  lcd.setCursor(x, y);
  lcd.write(byte(0));
  lcd.setCursor(x + 1, y);
  lcd.write(byte(1));

  // Middle row
  lcd.setCursor(x, y + 1);
  lcd.write(byte(2));
  lcd.setCursor(x + 1, y + 1);
  lcd.write(byte(3));
  lcd.setCursor(x + 2, y + 1);
  lcd.write(byte(4));

  // Bottom row
  lcd.setCursor(x, y + 2);
  lcd.write(byte(5));
  lcd.setCursor(x + 1, y + 2);
  lcd.write(byte(6));
  lcd.setCursor(x + 2, y + 2);
  lcd.write(byte(7));
}

void drawWalle(uint8_t x, uint8_t y)
{
   // Top row (eyes with padding)
  lcd.setCursor(x + 1, y);
  lcd.write(byte(0));   // left eye

  lcd.setCursor(x + 2, y);
  lcd.write(byte(1));   // right eye

  // Bottom row (wheels + body)
  lcd.setCursor(x, y + 1);
  lcd.write(byte(4));   // left wheel

  lcd.setCursor(x + 1, y + 1);
  lcd.write(byte(2));   // left body

  lcd.setCursor(x + 2, y + 1);
  lcd.write(byte(3));   // right body

  lcd.setCursor(x + 3, y + 1);
  lcd.write(byte(5));   // right wheel
} 