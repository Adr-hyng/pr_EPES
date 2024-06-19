#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define OLED_RESET -1
Adafruit_SH1106 display(OLED_RESET);

#if (SH1106_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SH1106.h!");
#endif


#define DT A0
#define SCK A1
long sample=0;
float val=0;
long count=0;
int w = 0;
int knownWeight = 5989; // in grams
// double knownWeight = 569; // in grams
unsigned long readCount(void)
{
  unsigned long Count;
  unsigned char i;
  pinMode(DT, OUTPUT);
  digitalWrite(DT,HIGH);
  digitalWrite(SCK,LOW);
  Count=0;
  pinMode(DT, INPUT);
  while(digitalRead(DT));
  for (i=0;i<24;i++)
  {
    digitalWrite(SCK,HIGH);
    Count=Count<<1;
    digitalWrite(SCK,LOW);
    if(digitalRead(DT)) 
    Count++;
  }
  digitalWrite(SCK,HIGH);
  Count=Count^0x800000;
  digitalWrite(SCK,LOW);
  return(Count);
}


void setup()
{
  Serial.begin(9600);
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  pinMode(SCK, OUTPUT);
  Serial.print("Weight Measurement: ");
  display.display();
  delay(1000);
  display.clearDisplay();

  // display.setTextSize(3);
  // display.setTextColor(WHITE);
  // display.setCursor(32,17);
  // display.println(String(constrain(map(w, 0, knownWeight, 0, 100), 0, 100)) + "%");
  // display.display();
  // delay(2000);
  // display.clearDisplay();

  calibrate();
}


void loop()
{
  count= readCount();
  w=(((count-sample)/val)-2*((count-sample)/val));
  Serial.print(String(count) + " | " + String(sample) + " | " + String(val) + " |     ");
  Serial.print("weight: " + String(w));
  
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(32,17);
  display.println(String(constrain(map(w, 0, knownWeight, 0, 100), 0, 100)) + "%");
  display.display();
  delay(1000);
  display.clearDisplay();

  Serial.println("g | " + String(constrain(map(w, 0, knownWeight, 0, 100), 0, 100)) + "%");
}


void calibrate()
{
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("    Calibrating \n    Please wait..");
  display.display();

  for(int i=0;i<100;i++)
  {
    count=readCount();
    sample+=count;
  }
  display.clearDisplay();

  sample/=100;
  Serial.print("\nAvg: ");
  Serial.print(sample);

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println(" Put Water Tank and\nwait a sec");
  display.display();
  count=0;
  while(count > -5000)
  {
    count=readCount();
    count=sample-count;
  }
  display.clearDisplay();

  // val = -110.82; // Remove this, and assign it to the value being produced by calibration.
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,17);
  display.println("  Please \n   wait");
  display.display();
  delay(2000);
  for(int i=0;i<100;i++)
  {
    count=readCount();
    val+=sample-count;
  }
  display.clearDisplay();
  val=val/100.0;
  val=val/knownWeight;        // put here your calibrating weight
}

