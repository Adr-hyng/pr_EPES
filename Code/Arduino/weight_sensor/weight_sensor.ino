#define DT A0
#define SCK A1
long sample=0;
float val=0;
long count=0;
double knownWeight = 5989; // in grams
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
  pinMode(SCK, OUTPUT);
  Serial.print("Weight Measurement: ");
  delay(1000);
  calibrate();
}


void loop()
{
  count= readCount();
  int w=(((count-sample)/val)-2*((count-sample)/val));
  Serial.print(String(count) + " | " + String(sample) + " | " + String(val) + " |     ");
  Serial.print("weight: " + String(w));
  
  Serial.println("g | " + String(constrain(map(w, 0, knownWeight, 0, 100), 0, 100)) + "%");
}


void calibrate()
{
  Serial.print("Calibrating...");
  Serial.print("\nPlease Wait...");
  for(int i=0;i<100;i++)
  {
    count=readCount();
    sample+=count;
  }

  sample/=100;
  Serial.print("\nAvg: ");
  Serial.print(sample);

  Serial.print("\nPut ");
  Serial.print(knownWeight);
  Serial.print("g & wait for a sec"); 
  count=0;
  while(count > -5000)
  {
    count=readCount();
    count=sample-count;
  }

  // val = -110.82; // Remove this, and assign it to the value being produced by calibration.

  Serial.println("\nPlease Wait....\n\n");
  delay(2000);
  for(int i=0;i<100;i++)
  {
    count=readCount();
    val+=sample-count;
  }
  val=val/100.0;
  val=val/knownWeight;        // put here your calibrating weight
}

