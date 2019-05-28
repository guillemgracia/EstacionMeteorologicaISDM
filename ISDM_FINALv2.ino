/*********************** Meteo Station ***********************
               By Guillem Gracia and Pere Camprubi
               v1.0
               Last update on 23/05/2019
This code as well as all hardware designs are opensource and 
anyone who wants to use them is allowed to do it freely.
*************************************************************/

/*NOTICE!! data is send to LabView in the following format:
 
  timestamp valueS1 valueS2 valueS3 frq tempBMP pressureBMP altitudeBMP

all send data is in "int" format

*/

//include files
#include <FreqCounter.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal.h>
#include <SD.h>

//constants
#define delay_ms 1000             //delay in ms
#define pressureSeaLevel 101500   //pressure at sea level
#define S1 A0                     //temperature sensor NTC10000
#define S2 A1                     //temperature sensor NTC2200
#define S3 A3                     //air velocity sensor
#define airdensity 1.1839         //kg/m3 at 25ºC
#define trans 2                   //transistor digital pin

//global variables
Adafruit_BMP085 bmp;
unsigned long frq;
int valueS1;                      //temperature of sensor S1 NTC10000
int valueS2;                      //temperature of sensor S2 NTC2200
int valueS3;                      //temperature of sensor S2 NTC2200
float tempS1;                     
float tempS2;                     
float airvelocity;                     
float tempBMP;
float pressureBMP;
float altitudeBMP;
float humidity;
float on;
float temp;
unsigned int timestamp;
unsigned int initialtimestamp;
bool transistor;
const int rs = 3, en = 4, d4 = 6, d5 = 7, d6 = 8, d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
File logFile;

//functions
void setup(){
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("   Setting up");
  lcd.setCursor(0,1);
  lcd.print(" Meteo Station");
  Serial.begin(57600);

  pinMode(trans, OUTPUT);
  transistor = false;
  if (!bmp.begin()) {
    lcd.clear();
    lcd.print("BMP no encontrado");
    while (1) {}
  }

  if (!SD.begin(10)) {
    Serial.print("SD init failed :(");
    while (1) {}
  }

  delay(1000);
  lcd.clear();
  lcd.print("Setup done :)");
  delay(1000);
  initialtimestamp = millis();
  transistor = false;
  on = millis();
}

void loop() {
  readData();
  processData();
  sendData();
  logData();
  printLCD();
  delay(delay_ms);
} 

void readData(){
  timestamp = (millis()-initialtimestamp)/1000;

  FreqCounter::f_comp=10;   // Cal Value / Calibrate with professional Freq Counter
  FreqCounter::start(500);  // 500 ms Gate Time
  while (FreqCounter::f_ready == 0) 
  frq = FreqCounter::f_freq;
  frq = frq*2;
  
  valueS1 = analogRead(S1);
  valueS2 = analogRead(S2);

  tempBMP = bmp.readTemperature();
  pressureBMP = bmp.readPressure();
  altitudeBMP = bmp.readAltitude(pressureSeaLevel);

  if((millis()-on) > 5000){
    if(transistor == false){
      digitalWrite(trans, HIGH);
      transistor = true;
      on = millis();    
    } else {
      digitalWrite(trans, LOW);
      transistor = false;
      on = millis();    
    }
  }
  valueS3 = analogRead(S3);
}

void sendData(){
  Serial.print(timestamp);
  Serial.print(" ");
  Serial.print(valueS1);
  Serial.print(" ");
  Serial.print(valueS2);
  Serial.print(" ");
  Serial.print(round(temp*10));
  Serial.print(" ");
  Serial.print(frq);
  Serial.print(" ");
  Serial.print(round(tempBMP*100));
  Serial.print(" ");
  Serial.print(round(pressureBMP*100));
  Serial.print(" ");
  Serial.print(round(altitudeBMP*100));
  Serial.print(" \n");
}

void processData(){   //TO DO
  float voltage = (float)valueS1*5/1023;
  float Rs = voltage*10000/(5-voltage);
  tempS1 = 1/(0.003354016+0.000256524*log(Rs/10000)+0.00000260597*log(Rs/10000)*log(Rs/10000)+0.0000000632926*log(Rs/10000)*log(Rs/10000)*log(Rs/10000));
  tempS1 -= 273;

  voltage = ((float)valueS2)*5/1023;
  Rs = voltage*2200/(5-voltage);
  tempS2 = 1/(0.003354016+0.000256985*log(Rs/2200)+0.00000262013*log(Rs/2200)*log(Rs/2200)+0.0000000638309*log(Rs/2200)*log(Rs/2200)*log(Rs/2200));
  tempS2 -= 273;

  tempS1 = (tempS1+tempS2+tempBMP)/3;

  if(transistor == true){
    voltage = (float)valueS3*5/1023;
    Rs = voltage*2200/(5-voltage);
    temp = 1/(0.003354016+0.000292415*log(Rs/2060)+0.00000165418*log(Rs/2060)*log(Rs/2060)+0.0000000740746*log(Rs/2060)*log(Rs/2060)*log(Rs/2060));
    temp -= 273;
  } else {
    voltage = (float)valueS3*5/1023;
    Rs = voltage*140/(5-voltage);
    temp = 1/(0.003354016+0.000292415*log(Rs/2060)+0.00000165418*log(Rs/2060)*log(Rs/2060)+0.0000000740746*log(Rs/2060)*log(Rs/2060)*log(Rs/2060));
    temp -= 273;
  }

  int c = (1E12/(1.4*frq*430000))-160;
  humidity = 2*c-695;
  if(humidity >99) humidity = 99;
}

void logData(){
    logFile = SD.open("datalog.txt", FILE_WRITE);
  if (logFile) { 
        logFile.print("Time(s)=");
        logFile.print(timestamp);
        logFile.print("\t, valueS1 =");
        logFile.print(valueS1);
        logFile.print("\t, valueS2 =");
        logFile.print(valueS2);
        logFile.print("\t, valueS3 =");
        logFile.print(valueS3);
        logFile.print("\t, freq =");
        logFile.print(frq);
        logFile.print("\t, tempBMP =");
        logFile.print(tempBMP);
        logFile.print("\t, pressureBMP =");
        logFile.print(pressureBMP);
        logFile.print("\t, altitudeBMP =");
        logFile.println(altitudeBMP);
        logFile.close();
  } 
  else {
    Serial.println("SD failure :(");
  }
}

void printLCD(){    //TO DO
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print(tempS1);
  lcd.setCursor(4,0);
  lcd.print("C");
  
  lcd.setCursor(0,1);
  lcd.print(round(pressureBMP));
  lcd.setCursor(4,1);
  lcd.print("hPa");
  
  lcd.setCursor(5,0);
  lcd.print(round(humidity));
  lcd.setCursor(7,0);
  lcd.print("%HR");

  lcd.setCursor(11,0);
  lcd.print(round(airvelocity));
  lcd.setCursor(13,0);
  lcd.print("m/s");

  lcd.setCursor(10,1);
  if(altitudeBMP < 0) lcd.print("-");
  else lcd.print("+");
  lcd.setCursor(11,1);
  lcd.print(round(altitudeBMP));
  lcd.setCursor(15,1);
  lcd.print("m"); 
}
