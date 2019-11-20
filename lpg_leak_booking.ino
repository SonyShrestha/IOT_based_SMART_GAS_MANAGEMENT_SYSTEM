#include<LiquidCrystal.h>
float f1;
int buz=5,gas=A1,fire=A2,m1=A4,m2=A5;
//RS=8,RW=Gnd,EN=9,D4=0,D5=1,D6=2,D7=3;
LiquidCrystal lcd(13,12,11,10,9,8);
//-----------Device-1-----------------//
byte PD_SCK;  // Power Down and Serial Clock Input Pin
byte DOUT;    // Serial Data Output Pin
byte GAIN;    // amplification factor
long OFFSET = 0;  // used for tare weight
float SCALE = 1;  // used to return weight in grams, kg, ounces, whatever
void setup() 
{
  lcd.begin(16, 2);
  Serial.begin(9600);
  pinMode(buz,OUTPUT);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  //pinMode(gas,INPUT);
  pinMode(fire,INPUT);
  delay(500);
  lcd.setCursor(0,0);  //lcd.setCursor(column,line) starts with 0
  lcd.print(" IOT BASED GAS  ");
  lcd.setCursor(0,1);
  lcd.print(" MANAGEMENT SYS ");
  delay(500);
  //------------------- HX711 -----------------
  scale1_begin(2,3,128);  //(Dt,Sck,Gain)
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale1_read());     // print a raw reading from the ADC
  Serial.print("read average: \t\t");
  Serial.println(scale1_read_average(20));   // print the average of 20 readings from the ADC
  Serial.print("get value: \t\t");
  Serial.println(scale1_get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)
  Serial.print("get units: \t\t");
  Serial.println(scale1_get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided 
                                           // by the SCALE parameter (not set yet)
  scale1_set_scale(-110010);    // this value is obtained by calibrating the scale with known weights; see the README for details
  scale1_tare(5);               // reset the scale to 0
  Serial.println("After setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale1_read());                 // print a raw reading from the ADC
  Serial.print("read average: \t\t");
  Serial.println(scale1_read_average(20));       // print the average of 20 readings from the ADC
  Serial.print("get value: \t\t");
  Serial.println(scale1_get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()
  Serial.print("get units: \t\t");
  Serial.println(scale1_get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
                                                // by the SCALE parameter set with set_scale
  lcd.setCursor(0,0);  //lcd.setCursor(column,line) starts with 0
  lcd.print("LPG : WEIG: FIRE");
  lcd.setCursor(0,1);
  lcd.print("    :     :     ");
  delay(1000);
}
void loop() 
{
  //----- Read weights -----
  int f = digitalRead(fire);
  float g = analogRead(gas);
  f1=scale1_get_units(5);
  lcd.setCursor(0,1);
  lcd.print("    :     :     ");
  lcd.setCursor(0,1);
  lcd.print(g);
  lcd.setCursor(5,1);
  lcd.print(f1);
  lcd.setCursor(12,1);
  lcd.print(f);
  Serial.println("*"+String(g)+"/"+String(f)+"/"+String(f1)+"#");
  if(g>500 || f==1)
  {
    digitalWrite(buz,HIGH);
  }
  else
  {
    digitalWrite(buz,LOW);
  }
  scale1_power_down();             // put the ADC in sleep mode
  delay(1000);
  scale1_power_up();
  delay(2000);
}
//***********************************************************
//                 Device-1 Functions
//***********************************************************
void scale1_begin(byte dout, byte pd_sck, byte gain)
{
  PD_SCK = pd_sck;
  DOUT = dout;

  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);

  scale1_set_gain(gain);
}
bool scale1_is_ready() 
{
  return digitalRead(DOUT) == LOW;
}
void scale1_set_gain(byte gain) 
{
  switch (gain) {
    case 128:   // channel A, gain factor 128
      GAIN = 1;
      break;
    case 64:    // channel A, gain factor 64
      GAIN = 3;
      break;
    case 32:    // channel B, gain factor 32
      GAIN = 2;
      break;
  }

  digitalWrite(PD_SCK, LOW);
  scale1_read();
}
void scale1_yield(void) {};
long scale1_read()
{
  // wait for the chip to become ready
  while (!scale1_is_ready()) {
    // Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
    scale1_yield();
  }

  unsigned long value = 0;
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // pulse the clock pin 24 times to read the data
  data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
  data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
  data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

  // set the channel and the gain factor for the next reading using the clock pin
  for (unsigned int i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (data[2] & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }

  // Construct a 32-bit signed integer
  value = ( static_cast<unsigned long>(filler) << 24
      | static_cast<unsigned long>(data[2]) << 16
      | static_cast<unsigned long>(data[1]) << 8
      | static_cast<unsigned long>(data[0]) );

  return static_cast<long>(value);
}
long scale1_read_average(byte times) 
{
  long sum = 0;
  for (byte i = 0; i < times; i++) {
    sum += scale1_read();
    scale1_yield();
  }
  return sum / times;
}
double scale1_get_value(byte times)
{
  return scale1_read_average(times) - OFFSET;
}
float scale1_get_units(byte times) 
{
  return scale1_get_value(times) / SCALE;
}
void scale1_tare(byte times) 
{
  double sum = scale1_read_average(times);
  scale1_set_offset(sum);
}
void scale1_set_scale(float scale)
{
  SCALE = scale;
}
float scale1_get_scale() 
{
  return SCALE;
}
void scale1_set_offset(long offset)
{
  OFFSET = offset;
}
long scale1_get_offset()
{
  return OFFSET;
}
void scale1_power_down()
{
  digitalWrite(PD_SCK, LOW);
  digitalWrite(PD_SCK, HIGH);
}
void scale1_power_up() 
{
  digitalWrite(PD_SCK, LOW);
}
