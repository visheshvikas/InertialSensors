// http://www.mouser.com/ds/2/54/EMS22A-50229.pdf
// The encoder is designed for Daisy chain arrangement
// Need to send activate signal to read (50ns or more high signal to CS pin)
// Pin 1 = Digital Input (DI) => Left empty otherwise connect to last encoder
// Pin 2 = Clock (CLK) => PIN_CLK
// Pin 3 = GND 
// Pin 4 = Digital Output (DO) => PIN_DATA
// Pin 5 = VCC = 3.3V (for EMS22A-30, otherwise 5V for EMS22A-50)
// Pin 6 = CS => PIN_CS


const int PIN_CLOCK = 6; // pin 2
const int PIN_DATA = 7; // pin 4
const int PIN_CS = 5; //
int contVar;
int refpos = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, INPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);
  contVar = 0;
  Serial.println("Reddy to go Gaiz?");
}


//byte stream[16];
void loop() {
  if(Serial.available()>0)
  {
    contVar = 1;
    Serial.println("Unleash the Kraken");
    refpos = readEncoder();
    if(Serial.available())
    {
      Serial.read();
    }
  }
  if(contVar)
  {
    float pos = readEncoder() - refpos;
    if(pos<0)
    {
      pos=pos+1024;
    }
    
    Serial.print(pos);
    Serial.print("\t");
    Serial.println((float)pos/1024*360,2);
  }
}


int readEncoder()
{
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_CS, LOW);
    int pos = 0;
    for (int i=0; i<10; i++) {
      digitalWrite(PIN_CLOCK, LOW);
      digitalWrite(PIN_CLOCK, HIGH);
      
      byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
      pos += b * pow(2, 10-(i+1));
    }
    for (int i=0; i<6; i++) {
      digitalWrite(PIN_CLOCK, LOW);
      digitalWrite(PIN_CLOCK, HIGH);
    }
    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);
    
    return pos;
}
