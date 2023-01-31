#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <FastLED.h>

CRGB leds[16];
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define Led_Pin 6
#define NumLed 16
#define MCP_RESET 7
#define LED_Brightness 20

#define checkTime 200

#define MCP_OUTPUT 0
#define MCP_INPUT 1

/**Begin Register**/
#define IODIRA 0x00   // Port A Direction
#define IPOLA 0x02    // Input polarity (Flip read bit or not)
#define GPINTENA 0x04 // Interrupt on change (Interrupt Enable)
#define DEFVALA 0x06  // Default Value Register
#define INTCONA 0x08  // Interrupt On Change control Reguster
#define IOCONA 0x0A   // IO Config
#define GPPUA 0x0C    // PULLUP Resistor
#define INTFA 0x0E    // Interrupt Flag
#define INTCAPA 0x10  // Interrupt Captured Value
#define GPIOA 0x12    // GPIOA Register
#define OLATA 0x14    // Output Latch

#define IODIRB 0x01   // Port B Direction
#define IPOLB 0x03    // Input polarity (Flip read bit or not)
#define GPINTENB 0x05 // Interrupt on change (Interrupt Enable)
#define DEFVALB 0x07  // Default Value Register
#define INTCONB 0x09  // Interrupt On Change control Reguster
#define IOCONB 0x0B   // IO Config
#define GPPUB 0x0D    // PULLUP Resistor
#define INTFB 0x0F    // Interrupt Flag
#define INTCAPB 0x11  // Interrupt Captured Value
#define GPIOB 0x13    // GPIOB Register
#define OLATB 0x15    // Output Latch
/***End Register***/



char bootScreen1[] = "IC Tester";
char bootScreen2[] = "by Chanathip";

char instruction[] = "Select an IC";
static int index;

int nColor[3] = {};
int lColor[3] = {};
int dirList[3] = {};
int ledIndex;
int ledDir = 1;
uint32_t ledTimeNow;
uint32_t ledInterval = 33;//22;


/*
0111111110111111 -> 32703 // Normal IC
        Normal IC
In_1_1 -|1    16|- VCC
In_1_2 -|2    15|- In_4_1
 Out_1 -|3    14|- In_4_2
In_2_1 -|4    13|- Out_4
In_2_2 -|5    12|- In_3_1
 Out_2 -|6    11|- In_3_2
   GND -|7    10|- Out_3
    NC -|8_____9|- NC

1110111111110111 -> 61431 // For 7473
     -|1    16|-
     -|       |-
     -|       |-
 GND(-|4    13|-)VCC
     -|       |-
     -|       |-
     -|       |-
     -|_______|-

*/
const int ICList[] PROGMEM = {
    //IC Number, Power pins, IO Pins
    7400,0,0, // 0
    7402,0,3, // 1
    7404,0,1, // 2
    7408,0,0, // 3 
    7411,0,4, // 7411
    7432,0,0, // 4
    //7438,0,0, // 7438 OC And
    //7473,1,2, // 5 JK
    7486,0,0, // 6
};

String ICName[] = {
  "NAND",
  "NOR",
  "NOT",
  "AND",
  "3-AND",
  "OR",
  //"OC-AND",
  //"JK",
  "XOR"  
};

int powerPins[][2] = {
    {7, 16},
    {4, 13} // 7473
};

int ICInput[][9] = {
  {1,2,4,5,11,12,14,15},  // IC Input CAT0 (Normal)
  {1,3,5,11,13,15},       // IC Input CAT1 (Not)
  {1,2,3,14,5,6,7,10},    // IC Input CAT2 (JK)
  {2,3,5,6,10,11,13,14},  // IC Input CAT3 (NOR)
  {1,2,15,3,4,5,13,12,11} // IC Input CAT4 (7411)
};

int ICOutput[][6] = {
  {3,6,10,13},            // IC Output CAT0 (Normal)
  {2,4,6,10,12,14},       // IC Output CAT1 (Not)
  {10,11,12,14,15,16},    // IC Output CAT2 (JK)
  {1,4,12,15},            // IC Output CAT3 (NOR)
  {14,6,10}               // IC Output CAT4 (7411)
};


void resetMCP()
{
  digitalWrite(7, LOW);
  delay(100);
  digitalWrite(7, HIGH);
};
void Center(String Input, int row)
{
  lcd.setCursor((16 - Input.length()) / 2, row);
  lcd.print(Input);
};

void homePage()
{
  lcd.setCursor(0, 0);
  lcd.print("                ");
  Center(String(instruction), 0);
};

void printIC()
{
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(pgm_read_word(&ICList[index*3]));
  lcd.setCursor(16 - ICName[index].length(),1);
  lcd.print(ICName[index]);
  /*Serial.print(String(index) + " ");
  Serial.println(String(pgm_read_word(&ICList[4*index][0])));
  lcd.setCursor(16 - String(pgm_read_word(&ICList[(4*index)+1])).length(), 1);
  lcd.print(String(pgm_read_word(&ICList[(4*index)+1])));*/
};

void writeToReg(uint8_t reg, uint8_t data, int address = 38)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
};

int readFromReg(uint8_t reg, int address = 38)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return -1;
  }
};

int MCP_pinMode(int pin[], int Mode, int size)
{
  uint8_t pinModeLow = readFromReg(IODIRA);
  uint8_t pinModeHigh = readFromReg(IODIRB);  
  for (int i = 0; i < size; i++)
  {
    if (pin[i] < 1 || pin[i] > 16)
    {
      continue;
    }
    //pin[i] -= 1;
    if (pin[i] > 7)
    {
      //pin[i] -= 8;
      if (Mode == MCP_OUTPUT)
      {
        pinModeHigh = pinModeHigh & ~(1 << (pin[i] - 9));
      }
      else if (Mode == MCP_INPUT)
      {
        pinModeHigh = pinModeHigh | (1 << (pin[i] - 9));
      }
    }
    else if (pin[i] <= 7)
    {
      if (Mode == MCP_OUTPUT)
      {
        pinModeLow = pinModeLow & ~(1 << (pin[i] - 1)); // 7
      }
      else if (Mode == MCP_INPUT)
      {
        pinModeLow = pinModeLow | (1 << (pin[i] - 1));
      }
    }
  }
  Serial.println();
  writeToReg(IODIRA, pinModeLow);
  writeToReg(IODIRB, pinModeHigh);
  return pinModeLow | (pinModeHigh << 8);
};

void MCP_digitalWrite(int pin, int Mode)
{
  uint8_t writeLow = readFromReg(GPIOA);
  uint8_t writeHigh = readFromReg(GPIOB);

  if (pin > 0 && pin <= 16)
  {
    pin -= 1;
    if (pin > 7)
    {
      pin -=8;
      if (Mode == LOW)
      {
        writeHigh = writeHigh & ~(1 << pin);
      }
      else if (Mode == HIGH)
      {
        writeHigh = writeHigh | (1 << pin);
      }
    }
    else if (pin <= 7)
    {
      if (Mode == LOW)
      {
        writeLow = writeLow & ~(1 << pin); // 7
      }
      else if (Mode == HIGH)
      {
        writeLow = writeLow | (1 << pin);
      }
    }
    writeToReg(GPIOA, writeLow);
    writeToReg(GPIOB, writeHigh);
  }
};

void checkAndShow(CRGB OutOn,CRGB OutOff,CRGB InOn,CRGB InOff){
  int mode;
  int read;
  for (int k = 0; k < NumLed; k++)
  {
    if (k == 0)
    {
      mode = readFromReg(IODIRA);
      read = readFromReg(GPIOA);
    }
    else if (k == 8)
    {
      mode = readFromReg(IODIRB);
      read = readFromReg(GPIOB);
    }
    leds[k] = ((read >> (k > 7 ? k - 8 : k)) & 1) ? (mode >> (k > 7 ? k - 8 : k) & 1) ? InOn : OutOn : (mode >> (k > 7 ? k - 8 : k) & 1) ? InOff : OutOff;
  }
  FastLED.show();
}
void initLED(){
  FastLED.addLeds<WS2812B, Led_Pin, GRB>(leds, NumLed);
  FastLED.setBrightness(LED_Brightness);
  for (int i = 0; i < NumLed; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

int MCP_DigitalRead(int pinToRead){
  return ((readFromReg(GPIOB) << 8 | readFromReg(GPIOA)) >> (pinToRead - 1)) & 1;
};

void summaryShow(int counted,int countExpected){
    if (counted == countExpected){
      lcd.clear();
      Center(ICName[index],0);
      Center("All Passed",1);

      for (int i = 0; i < 256;i++){
        for(int j = 0; j < NumLed; j++)  {
          leds[j] = CRGB(0,i,0);
        }
        FastLED.show();
        delay(5);
      }
      delay(1000);
      for (int i = 255; i > 0;i--){
        for(int j = 0; j < NumLed; j++)  {
          leds[j] = CRGB(0,i,0);
        }
        FastLED.show();
        delay(3);
      }      
    }
    else{
      lcd.clear();
      Center(ICName[index],0);
      Center("Something Wrong",1);

      for (int i = 0; i < 256;i++){
        for(int j = 0; j < NumLed; j++)  {
          leds[j] = CRGB(i,0,0);
        }
        FastLED.show();
        delay(3);
      }
      delay(1000);
      for (int i = 255; i > 0;i--){
        for(int j = 0; j < NumLed; j++)  {
          leds[j] = CRGB(i,0,0);
        }
        FastLED.show();
        delay(3);
      } 
    }
}

void Select()
{
  lcd.clear();
  Center("Checking",0);
  
  Center(String(pgm_read_word(&ICList[index*3])),1);
  int ICPower = pgm_read_word(&ICList[3*index + 1]);
  int ICParam = pgm_read_word(&ICList[3*index + 2]);  
  MCP_pinMode(powerPins[ICPower],MCP_OUTPUT,sizeof(ICInput[ICParam]));
  writeToReg(GPIOA,0);
  writeToReg(GPIOB,0);
  writeToReg(GPPUA, 255);
  writeToReg(GPPUB, 255);
  MCP_digitalWrite(powerPins[ICPower][0],LOW);
  MCP_digitalWrite(powerPins[ICPower][1],HIGH);
  MCP_pinMode(ICInput[ICParam],MCP_OUTPUT,sizeof(ICInput[ICParam])/2);
  MCP_pinMode(ICOutput[ICParam],MCP_INPUT,sizeof(ICOutput[ICParam])/2);
  if (ICParam == 0 || ICParam == 3){ // Default Pinout
    int passCount = 0;
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 4; j++){
        MCP_digitalWrite(ICInput[ICParam][i*2],j & 1);
        MCP_digitalWrite(ICInput[ICParam][i*2+1],(j >> 1) & 1);
        int readTotal = readFromReg(GPIOB) << 8 | readFromReg(GPIOA);
        checkAndShow(CRGB::Red,CRGB::Black,CRGB::Green,CRGB::Black);        
        Serial.print(ICName[index] + " ");
        Serial.print(i*4 + j + 1);
        Serial.print(" ");
        int A = j & 1;
        int B = (j >> 1) & 1;
        if (index == 0){ // NAND
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == !(A&B) ){
            passCount++;
          }
        }
        else if (index == 1){ // NOR
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == !(A | B) ){
            passCount++;
          }
        }
        else if (index == 3){ // AND
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == (A & B) ){
            passCount++;
          }
        }
        else if (index == 4){ // AND Open Collector
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == (A & B) ){
            passCount++;
          }
        }        
        else if (index == 5){ // OR 7432
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == (A | B) ){
            passCount++;
          }
        } 
        else if (index == 6){ // XOR 86
          if ( ((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == (A^B) ){
            passCount++;
          }
        }
        delay(checkTime);
      }
      MCP_digitalWrite(ICInput[ICParam][i*2],0);
      MCP_digitalWrite(ICInput[ICParam][i*2+1],0);
    }
    summaryShow(passCount,16);
  }
  else if (ICParam == 1){ // Not Gate
    int passCount = 0;
    for (int i = 0; i < 6; i++){
      for (int j = 0; j < 2; j++){
        MCP_digitalWrite(ICInput[ICParam][i],j);
        checkAndShow(CRGB::Red,CRGB::Black,CRGB::Green,CRGB::Black);
        int readTotal = readFromReg(GPIOB) << 8 | readFromReg(GPIOA);
        if (((readTotal >> (ICOutput[ICParam][i] - 1)) & 1) == !(j&1)){
          passCount++;
        }
        delay(checkTime);
        MCP_digitalWrite(ICInput[ICParam][i],!j);
      }
      Serial.println();
    }
    Serial.println(passCount);
    summaryShow(passCount,12);
  }

  else if (ICParam == 4){ //7411 3-AND
    int passCount = 0;
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 8; j++){
        int A = j & 1;
        int B = (j >> 1) & 1;
        int C = (j >> 2) & 1;
        MCP_digitalWrite(ICInput[ICParam][i*3],A);
        MCP_digitalWrite(ICInput[ICParam][(i*3) + 1],B);
        MCP_digitalWrite(ICInput[ICParam][(i*3) + 2],C);
        if (MCP_DigitalRead(ICOutput[ICParam][i]) == (A&B&C)){
          passCount++;
        }
        checkAndShow(CRGB::Red,CRGB::Black,CRGB::Green,CRGB::Black);
        delay(checkTime);
      }
      delay(checkTime);
    }
    summaryShow(passCount,24);
  }

  for (int i = 0; i < NumLed; i++){
    leds[i] = CRGB::Black;
  }
  MCP_digitalWrite(powerPins[ICPower][0],LOW);
  MCP_digitalWrite(powerPins[ICPower][1],LOW);
};

void setup()
{
  DDRD = DDRD & ~0B00111100;
  DDRD = DDRD | 0B11000000;
  PORTD = PORTD | 0B00111100;

  lcd.init();
  lcd.backlight();
  resetMCP();
  initLED();
  Wire.begin();
  Serial.begin(115200);
  writeToReg(GPPUA,255);
  writeToReg(GPPUB,255);
  Center(bootScreen1, 0);
  Center(bootScreen2, 1);
  delay(1000);
  homePage();
  printIC();
  nColor[1] = 255;

}
int n;
int l;

int lIndex[] = {0,0,0,0,0,0,0,0,0,0};
void loop(){  
  //checkAndShow(CRGB::Red,CRGB::Black,CRGB::Green,CRGB::Black);
  if (millis() - ledTimeNow > ledInterval){

    leds[ledIndex] = CRGB(nColor[0],180,nColor[2]);
    FastLED.show();
    int subVal = 32;//255 / (sizeof(lIndex));  
    //Serial.println();
    lIndex[0] = ledIndex;
    nColor[0] += dirList[0];
    dirList[0] = nColor[0] >= 255 ? -4 : nColor[0] <= 0 ? 6 : dirList[0];
    nColor[0] = nColor[0] >= 255 ? 255 : nColor[0] <= 0 ? 0 : nColor[0];
    nColor[2] += dirList[2];
    dirList[2] = nColor[2] >= 255 ? -2 : nColor[2] <= 0 ? 5 : dirList[2];
    nColor[2] = nColor[2] >= 255 ? 255 : nColor[2] <= 0 ? 0 : nColor[2];
    for(int i = 9; i>0; i--){
      lIndex[i] = lIndex[i-1];
      //Serial.println(lIndex[i]);
    }
    for (int i = 9 ; i > 0; i--){
      leds[lIndex[9-i]] = CRGB(nColor[0],(subVal * i),nColor[2]);
    }
    ledIndex += 1;
    ledIndex = ledIndex > (NumLed - 1) ? 0 : ledIndex;        
    ledTimeNow = millis();
  }

  n = (PIND >> 2) & 15;
  if (n != l)
  {
    if ((n & 1) == 0)
    { // Left
      index = index < 1 ? (sizeof(ICList) / 6) - 1 : --index;
      printIC();
      delay(50);
    }

    else if (((n >> 1) & 1) == 0)
    { // OK Press
      Select();
      ledIndex = 0;
    }

    else if (((n >> 1) & 1) == 1 && ((l >> 1) & 1) == 0)
    { // OK Release
      homePage();
      printIC();
    }

    else if (((n >> 2) & 1) == 0)
    { // Right
      Serial.println("Right");
      index = index > (sizeof(ICList) / 6) - 2 ? 0 : ++index;
      printIC();
      delay(50);
    }

    else if (((n >> 3) & 1) == 0)
    { // Cancel
      lcd.clear();
    }
    l = n;
  }

}
