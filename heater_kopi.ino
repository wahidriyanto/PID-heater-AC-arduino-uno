#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <RBDdimmer.h>

//---------------------------Input output pin-------------------------------//
//rbd dimmer
const int output_pin = 3;
const int zerocross = 2;

const int ONE_WIRE_BUS = 7; // sensor ds
const int led_pin = 13; // led pin
const int ir_pin = 11; // ir pin
const int switch_pin = 12; //btn pin

//relay pin
const int rel0 = 9;
const int rel1 = 10;

//analog pin
int turbidity_pin = A0;
int pot_pin = A1;

//---------------------------create object-------------------------------//
//lcd
LiquidCrystal_I2C lcd(0x27, 16, 2);
//dimmer
//dimmerLamp dimmer(output_pin, zerocross);
dimmerLamp dimmer(output_pin);
//sensor ds
OneWire ds(ONE_WIRE_BUS);

int refresh = 0;

float suhu_ds = 0;
int turb_value = 0;

int switch_val;

int pot_val = 0;
int pot_level = 0;

String kekeruhan = " ";
String pouring = " ";
String pengaduk = " ";

int setpoin = 0;
float suhu_real = 0;

//pid variable
float PID_error = 0;
float prev_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;

int maxPIDout = 100; //RBDdimmer value scale 0-100%
int minPIDout = 9; //RBDdimmer minimal value is 9

//pid
float kp = 20.566; float ki = 0.06; float kd = 2.8; //set kp,ki and kd
int PID_p = 0; int PID_i = 0; int PID_d = 0;

int mode_level = 0;

//read the temperature
float dallas(OneWire& ds, byte start = false) {
  int16_t temp;
  do {
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    ds.read_bytes((uint8_t*) &temp, sizeof(temp));
    ds.reset();
    ds.write(0xCC);
    ds.write(0x44, 1);
    if (start) delay(250);
  } while (start--);
  return (temp * 0.0625);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
  pinMode(ir_pin, INPUT);
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(rel0, OUTPUT);
  pinMode(rel1, OUTPUT);

  digitalWrite(rel0, 1);
  digitalWrite(rel1, 1);

  //led indicator
  lcd.init();
  for (int i = 0; i < 10; i++) {
    digitalWrite(led_pin, 1);
    delay(80);
    digitalWrite(led_pin, 0);
    delay(80);
  }
  lcd.backlight();

  dimmer.begin(NORMAL_MODE, ON);
  dallas(ds, true);

  lcd.print("test heater");
  delay(500);

  lcd.clear();
}

void loop() {

  sensor_ds_read();
  level(); //level function

  //refresh lcd
  if (++refresh > 5) {
    lcd.clear();
    refresh = 0;
  }

  //switch on
  if (digitalRead(switch_pin) == 1) {
    digitalWrite(rel1, 0);
    turbidity_read();
    pengaduk = "on";
    dimmer.setPower(9);
    
    lcd.setCursor(0, 0);
    lcd.print(F("Air:")); lcd.print(kekeruhan);
    
    lcd.setCursor(0, 1);
    lcd.print(F("Pump:")); lcd.print(pouring);
    lcd.print(F(" Aduk:")); lcd.print(pengaduk);
    //pump on
    if (digitalRead(ir_pin) == 0) {
      digitalWrite(rel0, 0);
      pouring = "on";
    }
    //pump off
    if (digitalRead(ir_pin) == 1) {
      digitalWrite(rel0, 1);
      pouring = "off";
    }
  }

  //switch off
  if (digitalRead(switch_pin) == 0) {
    digitalWrite(rel1, 1);
    turb_value = 0;
    kekeruhan = "Wait";
    pengaduk = "off";
    pouring = "off";
    pemanas(); //pid heater
    
    lcd.setCursor(0,0);
    lcd.print("heat:"); lcd.print(PID_value);
    lcd.print(" er:"); lcd.print(prev_error);
    
    lcd.setCursor(9, 1);
    lcd.print("sp:"); lcd.print(pot_level); 
    lcd.print((char)223); lcd.print("C");
    
    lcd.setCursor(0, 1);
    lcd.print(suhu_ds); lcd.print((char)223); lcd.print("C");
  }
  //digitalWrite(led_pin, digitalRead(led_pin)^1);
}

//read trubidity sensor
void turbidity_read() {
  turb_value = analogRead(turbidity_pin);
  if (turb_value > 600) {
    kekeruhan = "JERNIH";
  }
  if (turb_value < 200) {
    kekeruhan = "KERUH";
  }
}

void sensor_ds_read() {
  suhu_ds = dallas(ds);
  //turn off the heater if temperature > 110 degree celcius
  if (suhu_ds > 110) dimmer.setPower(9);
}

void level() {
  pot_val = analogRead(pot_pin);
  //map potensiometer for setpoin level
  pot_level = map(pot_val, 0, 1023, 40, 100);

  //kondisi pembagian level untuk set parameter pid
  if (pot_level > 40 && pot_level < 60) mode_level = 1;
  if (pot_level > 60 && pot_level < 80) mode_level = 2;
  if (pot_level > 80 && pot_level < 100) mode_level = 3;

  Serial.println ((String)mode_level + " " + kp + " " + ki + " " + kd);
}

//PID heater
void pemanas() {
  digitalWrite(led_pin, digitalRead(led_pin) ^ 1);

  suhu_real = suhu_ds;
  setpoin = pot_level;

  PID_error = setpoin - suhu_real;

  if (PID_error > 30) PID_error = 30;
  if (PID_error < -30) PID_error = -30;

  PID_p = kp * PID_error;
  PID_i = PID_i + (ki * PID_error);

  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 100;

  PID_d = kd * ((PID_error - prev_error) / elapsedTime);

  PID_value = PID_p + PID_i + PID_d;

  //limit pid output
  if (PID_value < minPIDout) PID_value = minPIDout;
  if (PID_value > maxPIDout) PID_value = maxPIDout;

  prev_error = PID_error;

  //set power dimmer based on pid output
  dimmer.setPower(PID_value);

  Serial.print("error: ");
  Serial.print(prev_error);
  Serial.print("\t pemanas: ");
  Serial.print(dimmer.getPower());
  Serial.print("\t");
}
