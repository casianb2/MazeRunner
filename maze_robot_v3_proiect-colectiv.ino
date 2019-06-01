//Trebuie PULL UP de 50kOhm pe SDA_PIN si SCL_PIN//pt micro 340kOhm

#include <Wire.h>
#include <TimerOne.h>
#include <AccelStepper.h>
#include <EnableInterrupt.h>

// standard I2C address for Smart Battery packs
byte deviceAddress = 11;
byte player_address = 8;


#define SLEEP 13
AccelStepper stepper1(AccelStepper::DRIVER, 12, 11); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5 STEP, DIR
AccelStepper stepper2(AccelStepper::DRIVER, 3, 2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5


String str = "";
String substr = "";
int index_colon = 0;

// Standard and common non-standard Smart Battery commands
#define BATTERY_MODE             0x03
#define TEMPERATURE              0x08
#define VOLTAGE                  0x09
#define CURRENT                  0x0A
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define REMAINING_CAPACITY       0x0F
#define FULL_CHARGE_CAPACITY     0x10
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define DESIGN_CAPACITY          0x18
#define DESIGN_VOLTAGE           0x19
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define SERIAL_NUM               0x1C
#define MFG_NAME                 0x20   // String
#define DEV_NAME                 0x21   // String
#define CELL_CHEM                0x22   // String
#define MFG_DATA                 0x23   // String
#define CELL4_VOLTAGE            0x3C   // Indidual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE            0x3D
#define CELL2_VOLTAGE            0x3E
#define CELL1_VOLTAGE            0x3F
#define STATE_OF_HEALTH          0x4F

#define bufferLen 32
uint8_t i2cBuffer[bufferLen];

#define SOUND_SPEED_METERS_SEC_@0ALT 350
#define TEMPERATURE_CELSIUS 20
#define MM_PER_1MICROSEC 0.331

#define TRIG_PIN_LEFT 4    //4
#define ECHO_PIN_LEFT 5    //2 //interrupt pin


#define TRIG_PIN_FRONT 6   //5
#define ECHO_PIN_FRONT 7   //3 //interrupt pin

#define TRIG_PIN_RIGHT A0   //6
#define ECHO_PIN_RIGHT 8   //7 //interrupt pin

#define pinOfPin(P)(((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)

#define irSensor_pin1 A6
#define irSensor_pin2 A1

volatile unsigned long echo_timer_left = 0;
volatile unsigned long echo_timer_front = 0;
volatile unsigned long echo_timer_right = 0;

volatile unsigned long echo_timer_left_copy = 0;
volatile unsigned long echo_timer_front_copy = 0;
volatile unsigned long echo_timer_right_copy = 0;

int leftUltraDistanceMM;
int frontUltraDistanceMM;
int rightUltraDistanceMM;


const int trigger_pulse_time = 50; //us
const long ultra_measurement_cycle = 5000;//60000
unsigned long timer_ultra_measurement_cycle = 0;
byte ultra_sensor_turn = 0;
bool timer_wait_for_trigger_low = false;
bool stepper_go_flag = true;
uint16_t irSensor_value = 0;

unsigned long timer = 0;
unsigned long x = 0;
int speedd = 300;
int spd = 50;
int accc = 1000;
int pasi_90_grade = 267.5 / 2 + 5; //- 10; //267.5
#define FORWARDS 0
#define BACKWARDS 1
#define LEFT 2
#define RIGHT 3
int go = 0;

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  digitalWrite(TRIG_PIN_LEFT, LOW);
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  digitalWrite(TRIG_PIN_FRONT, LOW);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  pinMode(irSensor_pin1, INPUT);
  pinMode(irSensor_pin2, INPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  enableInterrupt(ECHO_PIN_LEFT, echoLeftReceived, CHANGE);
  enableInterrupt(ECHO_PIN_FRONT, echoFrontReceived, CHANGE);
  enableInterrupt(ECHO_PIN_RIGHT, echoRightReceived, CHANGE);

  Timer1.initialize(trigger_pulse_time);
  Timer1.attachInterrupt(triggerUltraSensors);

  Serial.begin(115200);  // start serial for output

  pinMode(SLEEP, OUTPUT);
  digitalWrite(SLEEP, LOW);
  Serial.setTimeout(10);
  stepper1.setMaxSpeed(speedd);//steps/sec
  stepper1.setAcceleration(accc);//steps/sec/sec
  // stepper1.moveTo(500);
  //stepper1.setSpeed(3000);
  stepper2.setMaxSpeed(speedd);//steps/sec
  stepper2.setAcceleration(accc);//steps/sec/sec
  // stepper2.moveTo(1000000);
  // stepper2.setSpeed(3000);

  Wire.beginTransmission(player_address); // transmit to device #8
  Wire.write(3);
  Wire.endTransmission();

  Serial.println("ready...");
}

uint16_t reading = 0;
double a = 99;
bool useLightSensor = 1;

void loop()
{
  if (useLightSensor == 1) {
    irSensor_value = analogRead(irSensor_pin1);
    if (irSensor_value > 50 && go == 1) {
      go = 0;
      stepper1.stop();
      stepper2.stop();
      delay(1000);
      if (analogRead(irSensor_pin1) > 50) {
        delay(500);
        if (analogRead(irSensor_pin1) > 50) {
          mazeEnd();
        }
        else {
          go = 1;
        }
      } else {
        go = 1;
      }
    }
  }
  /*
    Serial.print("sensor 1: ");
    Serial.print(analogRead(irSensor_pin1));
    Serial.print("---sensor 2: ");
    Serial.println(analogRead(irSensor_pin2));
    delay(100);
  */
  // Keep reading from Arduino Serial Monitor and send to HC-05
  /*
    if (stepper1.distanceToGo() == 0)
      stepper1.moveTo(-stepper1.currentPosition());

    if (stepper2.distanceToGo() == 0)
      stepper2.moveTo(-stepper2.currentPosition());
  */
  if (Serial.available())
  {
    //  if (str.indexOf('\n') != -1) {
    str = Serial.readStringUntil('\n');
    //   }
    //  else {
    //     str =  Serial.readStringUntil('\r');
    //   }
    //delay(5000);
    index_colon = str.indexOf(':');
    if (index_colon != -1) { //if ':' is present in the string
      substr = str.substring(0, index_colon);
      if (substr == "forward_steps") {
        int steps_to_do = (str.substring(index_colon + 1)).toInt();
        do_steps(FORWARDS, steps_to_do);
      } else {
        if (substr == "backward_steps") {
          int steps_to_do = (str.substring(index_colon + 1)).toInt();
          do_steps(BACKWARDS, steps_to_do);
        } else {
          if (substr == "forward_mm") {
            int steps_to_do = (str.substring(index_colon + 1)).toInt();
            do_mm(FORWARDS, steps_to_do);
          } else {
            if (substr == "backward_mm") {
              int steps_to_do = (str.substring(index_colon + 1)).toInt();
              do_mm(BACKWARDS, steps_to_do);
            } else {
              if (substr == "rotate_left_degrees") {
                int degrees_to_move = (str.substring(index_colon + 1)).toInt();
                rotate_degrees(LEFT, degrees_to_move);
              } else {
                if (substr == "rotate_right_degrees") {
                  int degrees_to_move = (str.substring(index_colon + 1)).toInt();
                  rotate_degrees(RIGHT, degrees_to_move);
                } else {
                  if (substr == "stsp") {
                    spd = (str.substring(index_colon + 1)).toInt();

                  }
                }
              }
            }
          }
        }
      }
    }
    else {
      if (str == "f") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() + 1000);
        stepper2.moveTo(stepper2.currentPosition() - 1000);
      }
      if (str == "b") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() - 1000);
        stepper2.moveTo(stepper2.currentPosition() + 1000);
      }
      if (str == "ff") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() + 10000);
        stepper2.moveTo(stepper2.currentPosition() - 10000);
      }
      if (str == "bb") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() - 10000);
        stepper2.moveTo(stepper2.currentPosition() + 10000);
      }

      if (str == "r") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() + 2000);
        stepper2.moveTo(stepper2.currentPosition() + 2000);
      }
      if (str == "l") {
        stepper1.stop();
        stepper2.stop();
        stepper1.moveTo(stepper1.currentPosition() - 2000);
        stepper2.moveTo(stepper2.currentPosition() - 2000);
      }
      if (str == "s") {

        Serial.println("stopping motors:");
        //BTserial.println("pulling battery info:");
        stepper1.stop();
        stepper2.stop();
        //  stepper_go_flag = false;
      }
      if (str == "pb") {
        Serial.println("pulling battery info:");
        //BTserial.println("pulling battery info:");
        sendBatteryInfoThroughBluetooth();
      }
      if (str == "song1") {
        Serial.println("playing song1:");
        Wire.beginTransmission(player_address); // transmit to device #8
        Wire.write(1);
        Wire.endTransmission();
      }
      if (str == "song2") {
        Serial.println("playing song2:");
        Wire.beginTransmission(player_address); // transmit to device #8
        Wire.write(2);
        Wire.endTransmission();
      }
      if (str == "c") {
        Serial.println("battery_current: ");
        sendBatteryCurrent();
      }
      if (str == "c2") {
        Serial.println("battery_current: ");
        sendBatteryCurrent2();
      }
      if (str == "go") {
        Serial.println("going...");
        go = 1;
        Wire.beginTransmission(player_address); // transmit to device #8
        Wire.write(1);
        Wire.endTransmission();
      }
      if (str == "st") {
        Serial.println("stopping...");
        go = 0;
        stepper1.stop();
        stepper2.stop();
      }
      if (str == "sz") {
        Serial.println("pulling ir sensor values...");
        Serial.print("sensor 1:");
        Serial.println(analogRead(irSensor_pin1));
        Serial.print("sensor 2:");
        Serial.println(analogRead(irSensor_pin2));
      }
      if (str == "toggle_sensor") {
        if (useLightSensor == 0) {
          useLightSensor = 1;
          Serial.println("sensor enabled");
        } else {
          useLightSensor = 0;
          Serial.println("sensor disabled");
        }
      }
    }
    /*
      if (str == "pb") {
      Serial.println("pulling battery info:");
      //BTserial.println("pulling battery info:");
      sendBatteryInfoThroughBluetooth();
      }
      else {
      if (str == "s") {
        Serial.println("stopping motors:");
        //BTserial.println("pulling battery info:");
        stepper1.stop();
        stepper2.stop();
        stepper_go_flag = false;
      } else {
        if (str == "go") {
          Serial.println("go:");
          //BTserial.println("pulling battery info:");

          stepper_go_flag = true;
        }
        else {

          speedd = str.toInt();
          if (speedd > 0) {
            //stepper1.setSpeed(speedd);
            //stepper2.setSpeed(speedd);

             stepper1.setAcceleration(speedd);
             stepper1.setMaxSpeed(speedd);
             stepper2.setAcceleration(speedd);
             stepper2.setMaxSpeed(speedd);
          }
        }
      }
      }
    */
    // Serial.println(str);
    // Serial.println();

  }
  else {
    // get_battery_data();
    // print_all_battery_data();
    //    sendBatteryInfoThroughBluetooth();
    //   delay(1000);
  }

  //   Serial.println(MM_PER_1MICROSEC * echo_timer_left_copy / 2);

  /*
    Serial.print("left: ");
    Serial.print(MM_PER_1MICROSEC * echo_timer_left_copy / 2);
    Serial.print("  | front: ");
    Serial.print(MM_PER_1MICROSEC * echo_timer_front_copy / 2);
    Serial.print("  | right: ");
    Serial.println(MM_PER_1MICROSEC * echo_timer_right_copy / 2);//
    Serial.println();
  */
  // Serial.println(irSensor_value);
  // delay(1000);
  int fr = 60;//110
  int lftLower = 70;
  int lftUpper = 80;

  if (((MM_PER_1MICROSEC * echo_timer_left_copy / 2) < lftLower) && ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) >= fr)) {
    //  if (((MM_PER_1MICROSEC * echo_timer_left_copy / 2) < 140) && ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) >= fr)) {
    //  turn_right();
    // stepper2.setMaxSpeed(spd);
    if (go == 1) {
      stepper1.stop();
      stepper2.moveTo(1000000); //leftMotor
    }
  }
  if (((MM_PER_1MICROSEC * echo_timer_left_copy / 2) > lftUpper) && ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) >= fr)) {
    //  turn_left();
    // stepper2.setMaxSpeed(300);
    if (go == 1) {
      stepper1.moveTo(-1000000);
      stepper2.stop();
    }
  }
  if (((MM_PER_1MICROSEC * echo_timer_left_copy / 2) >= lftLower) && ((MM_PER_1MICROSEC * echo_timer_left_copy / 2) <= lftUpper) && ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) >= fr)) {
    //  go_straight();
    // stepper2.setMaxSpeed(300);
    if (go == 1) {
      stepper1.moveTo(-1000000);
      stepper2.moveTo(1000000);
    }
  }
  if ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) < fr) {
    if ((MM_PER_1MICROSEC * echo_timer_left_copy / 2) <= lftUpper + 40) {
      if (go == 1) {
        turn_right_90_deg();
        //  Serial.print("asdsafadsad: ");
        // Serial.println((MM_PER_1MICROSEC * echo_timer_front_copy / 2));
        delay(2000);
      }
    } else if ((MM_PER_1MICROSEC * echo_timer_left_copy / 2) > lftUpper + 40) {
      //stepper1.moveTo(-1000000);
      //stepper2.stop();
      if (go == 1) {
        turn_left_90_deg();
        delay(1000);
        stepper1.moveTo(-1000000);
        stepper2.moveTo(1000000);
        delay(1000);
      }
    }
  }

  /*
    if ((MM_PER_1MICROSEC * echo_timer_front_copy / 2) < fr) {
    //  turn_right();
    // stepper2.setMaxSpeed(300);
    stepper1.moveTo(1000000);
    stepper2.moveTo(1000000);

    }
  */


}


void stepper_update() {
  // x = micros();
  // if (x - time_us >= timer) {
  if (stepper_go_flag == true) {
    stepper1.run();
    stepper2.run();
    // stepper1.runSpeed();
    // stepper2.runSpeed();
  }

  //   timer = x;
  // }
}

void triggerUltraSensors()
{
  //Serial.println("triggered");
  stepper_update();
  if (timer_wait_for_trigger_low == true) {
    if (ultra_sensor_turn == 1) {
      digitalWrite(TRIG_PIN_LEFT, LOW);
      //   timerUltraTriggerLeft = micros() + trigger_pulse_time; //start timer
      //  Serial.println("left went low");
    }
    if (ultra_sensor_turn == 2) {
      digitalWrite(TRIG_PIN_FRONT, LOW);
      //    timerUltraTriggerFront = micros() + trigger_pulse_time;
      //    Serial.println("front went low");
    }
    if (ultra_sensor_turn == 0) {
      digitalWrite(TRIG_PIN_RIGHT, LOW);
      //    timerUltraTriggerRight = micros() + trigger_pulse_time;
      //  Serial.println("right went low");
    }
    timer_wait_for_trigger_low = false;
    //Timer1.setPeriod(ultra_measurement_cycle);
    //Timer1.start();
    return;
  }
  else {
    if (micros() - timer_ultra_measurement_cycle >= ultra_measurement_cycle) {
      timer_ultra_measurement_cycle = micros();

      if (ultra_sensor_turn == 0) { //left
        //   Serial.println("left went high");
        ultra_sensor_turn = 1;
        digitalWrite(TRIG_PIN_LEFT, HIGH);
        timer_wait_for_trigger_low = true;
        //Timer1.setPeriod(100000);
        //Timer1.start();
        return;
      }
      if (ultra_sensor_turn == 1) {//front
        //  Serial.println("front went high");
        ultra_sensor_turn = 2;
        digitalWrite(TRIG_PIN_FRONT, HIGH);
        timer_wait_for_trigger_low = true;
        //Timer1.setPeriod(100000);
        //Timer1.start();
        return;
      }
      if (ultra_sensor_turn == 2) {//right
        // Serial.println("right went high");
        ultra_sensor_turn = 0;
        digitalWrite(TRIG_PIN_RIGHT, HIGH);
        timer_wait_for_trigger_low = true;
        //Timer1.setPeriod(100000);
        //Timer1.start();
        return;
      }
    }
  }
}

void echoLeftReceived() {
  //if (digitalRead(ECHO_PIN_LEFT) == HIGH) {
  // if (bitRead(PORTD, 5) == 1) {
  volatile unsigned long tmr = micros();
  if (isHigh(ECHO_PIN_LEFT)) {
    echo_timer_left = tmr;
    //   Serial.println("echo left high");
    return;
  }
  //if (digitalRead(ECHO_PIN_LEFT) == LOW) {
  // if (bitRead(PORTD, 5) == 0) {
  if (isLow(ECHO_PIN_LEFT)) {

    //   Serial.println("echo left low");
    echo_timer_left_copy = tmr - echo_timer_left;
    //   Serial.print("time: ");
    //   Serial.println(echo_timer_left_copy);
    return;

    //  leftUltraDistanceMM = MM_PER_1MICROSEC * echo_timer_left_copy;
    //  Serial.print("left: ");
    //  Serial.println(leftUltraDistanceMM);
  }
}
void echoFrontReceived() {
  volatile unsigned long tmr = micros();

  if (isHigh(ECHO_PIN_FRONT)) {
    echo_timer_front = tmr;
    //   Serial.println("echo front high");
    return;
  }
  if (isLow(ECHO_PIN_FRONT)) {
    //   Serial.println("echo front low");

    echo_timer_front_copy = tmr - echo_timer_front;
    //  frontUltraDistanceMM = MM_PER_1MICROSEC * echo_timer_front_copy;
    //  Serial.print("left: ");
    //  Serial.println(leftUltraDistanceMM);
    return;
    //   Serial.print("front: ");
    //   Serial.println(echo_timer_front_copy);
  }
}
void echoRightReceived() {
  volatile unsigned long tmr = micros();

  if (isHigh(ECHO_PIN_RIGHT)) {
    echo_timer_right = tmr;
    //   Serial.println("echo right high");
    return;
  }
  if (isLow(ECHO_PIN_RIGHT)) {
    //   Serial.println("echo right low");
    echo_timer_right_copy = tmr - echo_timer_right;
    //   rightUltraDistanceMM = MM_PER_1MICROSEC * echo_timer_right_copy;
    //   Serial.print("right: ");
    //   Serial.println(echo_timer_right_copy);
    return;
  }
}

void do_steps(int direction, long steps_to_do) {
  Serial.print("doing steps ");
  Serial.print(direction);
  Serial.print(" ");
  Serial.println(steps_to_do);

  stepper1.stop();
  stepper2.stop();
  if (direction == FORWARDS) {
    stepper1.moveTo(stepper1.currentPosition() + steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() - steps_to_do);
  }
  if (direction == BACKWARDS) {
    stepper1.moveTo(stepper1.currentPosition() - steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() + steps_to_do);
  }
}
void do_mm(int direction, long mm) {
  Serial.print("doing mm ");
  Serial.print(direction);
  Serial.print(" ");
  Serial.println(mm);
  float steps_per_mm = 0.1;
  long steps_to_do = mm * steps_per_mm;
  stepper1.stop();
  stepper2.stop();
  if (direction == FORWARDS) {
    stepper1.moveTo(stepper1.currentPosition() + steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() - steps_to_do);
  }
  if (direction == BACKWARDS) {
    stepper1.moveTo(stepper1.currentPosition() - steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() + steps_to_do);
  }
}
void rotate_degrees(int direction, int deg) {
  Serial.print("rotating deg ");
  Serial.print(direction);
  Serial.print(" ");
  Serial.println(deg);
  int steps_to_do = deg;
  stepper1.stop();
  stepper2.stop();
  if (direction == LEFT) {
    stepper1.moveTo(stepper1.currentPosition() - steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() - steps_to_do);
  }
  if (direction == RIGHT) {
    stepper1.moveTo(stepper1.currentPosition() + steps_to_do);
    stepper2.moveTo(stepper2.currentPosition() + steps_to_do);
  }
}

void sendBatteryCurrent() {
  Serial.println(fetchWord(CURRENT));   // print the reading
}
void sendBatteryCurrent2() {
  Serial.println(fetchWord2(CURRENT));   // print the reading
}
void sendBatteryInfoThroughBluetooth() {
  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(RELATIVE_SOC));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  Serial.print("RELATIVE_SOC: ");
  Serial.println(reading);   // print the reading

  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(ABSOLUTE_SOC));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  Serial.print("ABSOLUTE_SOC: ");
  Serial.println(reading);   // print the reading

  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(TEMPERATURE));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  unsigned int tempk = (float)reading / 10.0 - 273.15;
  Serial.print("TEMPERATURE: ");
  Serial.println(tempk);   // print the reading

  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(CELL1_VOLTAGE));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  Serial.print("CELL1_VOLTAGE: ");
  Serial.println(reading);   // print the reading

  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(CELL2_VOLTAGE));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  Serial.print("CELL2_VOLTAGE: ");
  Serial.println(reading);   // print the reading

  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(byte(CELL3_VOLTAGE));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  reading |= (Wire.read() << 8);  // shift high byte to be high 8 bits
  Serial.print("CELL3_VOLTAGE: ");
  Serial.println(reading);   // print the reading
}
int fetchWord(byte func)
{
  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(func);      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  byte b1 = Wire.read();
  byte b2 = Wire.read();
  int ret = 0;
  ret = ((int)b2) << 8;
  ret = ret | b1;
  //return (int)b1 | ((( int)b2) << 8);
  return ret;
}
int fetchWord2(byte func)
{
  Wire.beginTransmission(11); // transmit to device #112 (0x70)
  Wire.write(func);      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting
  Wire.requestFrom(11, 2, true);   // request 2 bytes from slave device #112
  byte b2 = Wire.read();
  byte b1 = Wire.read();
  int ret = 0;
  ret = ((int)b2) << 8;
  ret = ret | b1;
  //return (int)b1 | ((( int)b2) << 8);
  return ret;
}
void turn_right_90_deg() {
  stepper1.stop();
  stepper2.stop();
  delay(2000);
  stepper1.move(pasi_90_grade);
  stepper2.move(pasi_90_grade);
}
void turn_left_90_deg() {
  stepper1.stop();
  stepper2.stop();
  delay(2000);
  stepper1.move(-pasi_90_grade);
  stepper2.move(-pasi_90_grade);
}
void mazeEnd() {
  Serial.println("reached goal...");
  go = 0;
  stepper1.stop();
  stepper2.stop();
  Serial.println("playing song2:");
  Wire.beginTransmission(player_address); // transmit to device #8
  Wire.write(2);
  Wire.endTransmission();
}