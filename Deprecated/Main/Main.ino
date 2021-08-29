// ================================================================
// ===                         ONEDUINO                         ===
// ================================================================

// https://forum.arduino.cc/index.php?topic=408980.0 Post nr 4

#define DEBUG
#define __ASSERT_USE_STDERR //NDEBUG in the case of leaving out asserts

#ifdef DEBUG
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));} //StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#define DEBUGMODE          true
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#define DEBUGMODE          false
#endif

#define sgn(x) ((x) < 0 ? -1 : 1)
#define CONTROL_FREQUENCY 10 // Hz

// ================================================================
// ===                        LIBRARIES                         ===
// ================================================================

// Installed libraries
#include <assert.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Encoder.h>

// Custom libraries
#include "Kalman.h"
#include "LQR.h"

#define LED_PIN 13

#define M1A 7
#define M1B 8

#define M2A 2
#define M2B 3

// ================================================================
// ===                      I2C SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
}

// ================================================================
// ===                  CONTROL LOOP INTERRUPT                  ===
// ================================================================
// Up to 4 intervaltimers can run simultaneously. They trigger the
// associated interrupts methods at a fixed frequency.
volatile bool controlInterrupt = false;

void controlInterruptHandler() {
  controlInterrupt = true;
}

IntervalTimer controlTimer;

void timersSetup() {
  controlTimer.priority(1);
  controlTimer.begin(controlInterruptHandler, 1000000/CONTROL_FREQUENCY); // Expressed in microseconds
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
MPU6050 mpu;

int FifoAlive = 0;     // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;       // [x, y, z]            rotational velocities around all three axes

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100;     // lets get 100 readings from the MPU before we start trusting them

int MPUOffsets[6] = {-8423, -565, 1008, 58, -38, 64};

void MPU6050Connect() {
  static int MPUInitCntr = 0; // Static: not deleted when function ends which allows values to be reused in subsequent function calls
  // Initialize device
  mpu.initialize();
  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    const char* StatStr[5] { "No Error", "Initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times (works because static)
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  
  // Assign offsets
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);
  
  // No calibration is required since this has been done beforehand
  //mpu.CalibrateAccel(6);
  //mpu.CalibrateGyro(6);
  //mpu.PrintActiveOffsets();
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabelize
  mpu.resetFIFO(); // Clear fifo buffer
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { 
  // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  // static unsigned long LastGoodPacketTime;
  // mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    // digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    // LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

// ================================================================
// ===                    MPU Polling Data                      ===
// ================================================================
// Alternative to GetDMP without use of interrupts

float Yaw, Pitch, Roll;
float YawVel, PitchVel, RollVel;

void pollDMP() {
  // Clear MPU value in order to get latest value
  mpu.resetFIFO();
  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  // Read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  // Further calculations
  MPUMath();
}

// ================================================================
// ===                        MPU Math                          ===
// ================================================================

void MPUMath() {
  mpu.dmpGetGyro(&gyro, fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  YawVel = gyro.z / 32768.0 * 2000.0 * DEG_TO_RAD;
  PitchVel = gyro.y / 32768.0 * 2000.0  * DEG_TO_RAD;
  RollVel = gyro.x / 32768.0 * 2000.0  * DEG_TO_RAD;
  
  Yaw = ypr[0]; // Z
  Pitch = ypr[1]; // Y
  Roll = ypr[2]; // X
}

// ================================================================
// ===                      Encoder METHODS                     ===
// ================================================================

// Encoder for wheel rotation
Encoder wheelEnc(21, 22);
float wheelEncPos, wheelEncVel;

// Encoder for flywheel rotation
Encoder flyEnc(16, 17);
float flyEncPos, flyEncVel;

void encoderSetup() {
  // Both encoder position can be initialised as 0
  // The initial position of the flywheel is of no importance
  // The initial position of the wheel is 0 at startup and thus also of no importance
  wheelEnc.write(0);
  flyEnc.write(0);
}

void pollEncoders() {
  // Get position information
  float oldWheelEncPos = wheelEncPos;
  wheelEncPos = wheelEnc.read() / (18.75 * 64.0) * 2 * PI;
  wheelEncVel = CONTROL_FREQUENCY * (wheelEncPos - oldWheelEncPos);
  float oldFlyEncPos = flyEncPos;
  flyEncPos = flyEnc.read() / (18.75 * 64.0) * 2 * PI;
  flyEncVel = CONTROL_FREQUENCY * (flyEncPos - oldFlyEncPos);
}

// ================================================================
// ===                       Motor METHODS                      ===
// ================================================================

// Source: https://www.pjrc.com/teensy/td_pulse.html#frequency
// Pins M1A = 8 and M1B = 7 are controlled by FlexPWM1.3 timer
// Pins M2A = 3 and M2B = 2 are controlled by FlexPWM4.2 timer
void motorSetup() { 
  // Check if pins still correct, otherwise setup will fail
  assert((M1A == 7) && (M1B == 8));
  assert((M2A == 2) && (M2B == 3));
  // Setup pins as output pins
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  // Setup PWM resolution
  analogWriteResolution(12);
  // Now use: analogWrite(port, 0 - 4095)
  // Ideal frequency: 36621.09 Hz
  analogWriteFrequency(M1A, 36621.09); // M1B frequency changes as well
  analogWriteFrequency(M2A, 36621.09); // M2B frequency changes as well
}

void driveMotor(float input, uint8_t portA, uint8_t portB) {
  // Quick action for input 0
  if (input == 0.0){
    analogWrite(portA, 0);
    analogWrite(portB, 0);
    return;
  }
  // Constrain input to motor voltage
  float constrainedInput = constrain(input, -12.0, 12.0);
  // Sign and Map to new range
  int8_t signInput = sgn(constrainedInput);
  uint16_t pwmInput = abs(constrainedInput) * 4095 / 12; // Automatically floored to int
  // Write output for motor
  analogWrite((signInput == 1) ? portB : portA, 0); // In case 0 isn't perfectly crossed, turn off other PWM signal
  analogWrite((signInput == 1) ? portA : portB, pwmInput);
  DPRINTSFN(10, " PWM:", pwmInput/4095.0, -6, 2);
  DPRINTLN();
}

void testMotors() {
  // Drive forwards and backwards at different speeds
  driveMotor(1, M1A, M1B); 
  delay(250);
  driveMotor(-1, M1A, M1B);
  delay(250);
  driveMotor(0, M1A, M1B);
  delay(250);
  driveMotor(1, M2A, M2B);
  delay(250);
  driveMotor(-1, M2A, M2B);
  delay(250);
  driveMotor(0, M2A, M2B);
}

// ================================================================
// ===                       Model SETUP                        ===
// ================================================================
// Forward control
// State: [x, dx, theta, dtheta]
//        [distance travelled, speed, forward angle, rotational velocity]
const float A_F[4][4] = {{1,0.0932499806614936,-0.128465898713369,-0.00395403990363919},
                         {0,0.845687546747161,-3.09562497386320,-0.128465898713369},
                         {0,0.0655715718247256,2.72393023041216,0.152926756621594},
                         {0,1.58006908719785,41.7074774283773,2.72393023041216}};
const float B_F[4] = {0.00170886565531809,0.0390664438614782,-0.0166003979303103,-0.400017490429837};
const float C_F[4] = {1,1,1,1};
const float Q_F[4] = {0.5,0.5,2,1}; // State deviation weight LQR
const float V_F[4] = {0.1,0.1,0.1,0.1}; // Verify through measurements
const float R_F = 3; // Input weight LQR
float U_F = 0;

LQR LQR_F(A_F, B_F, Q_F, R_F);
Kalman KALMAN_F(A_F, B_F, C_F, V_F);
ColumnVector4 State_F;
ColumnVector4 Obs_State_F;
ColumnVector4 Des_State_F;

// Sideways control
// State: [phi, dphi, alpha, dalpha]
//        [Side angle, rotational velocity, flywheel angle, rotational velocity]
const float A_S[4][4] = {{1.41146607741256,0.113372087314826,0,-0.000255074418624809},
                               {8.75203510129462,1.41146607741256,0,-0.00560148085730519},
                               {0,0,1,0.111574942057396},
                               {0,0,0,1.24010372302915}};
const float B_S[4] = {0.00161439505458740,0.0354524104892734,-0.0732591269455416,-1.51964381664016};
const float C_S[4] = {};
const float Q_S[4] = {1,1,0.5,0.5}; // State deviation weight LQR
const float V_S[4] = {0.1,0.1,0.1,0.1}; // Verify through measurements
const float R_S = 20; // Input weight LQR
float U_S = 0;  

LQR LQR_S(A_S, B_S, Q_S, R_S);
Kalman KALMAN_S(A_S, B_S, C_S, V_S);
ColumnVector4 State_S;
ColumnVector4 Obs_State_S;
ColumnVector4 Des_State_S;

// ================================================================
// ===                        CONTROL                           ===
// ================================================================

bool detectStart() {
  // Pitch and roll inside start range
  if ((abs(Pitch) < (5 * PI / 180.0)) && (abs(Roll) < (5 * PI / 180))) {
    // Turn off LED
    digitalWrite(LED_PIN, LOW);
    delay(500);
    return true;
  }
  return false;
}

bool detectFall() {
  // Pitch and roll outside safety range
  if ((abs(Pitch) > (30 * PI / 180.0)) || (abs(Roll) > (20 * PI / 180.0))) {
    // Safety procedure
    // Stop motors
    driveMotor(0, M1A, M1B);
    driveMotor(0, M2A, M2B);
    // Turn on LED
    digitalWrite(LED_PIN, HIGH);
    return true;
  }
  return false;
}

void updateStates() {
  // Get IMU and Encoder data
  pollDMP();
  pollEncoders();
  // Update observations
  Obs_State_F.setElement(0, wheelEncPos * 0.04); // distance travelled
  Obs_State_F.setElement(1, wheelEncVel * 0.04); // speed
  Obs_State_F.setElement(2, -1 * Pitch); // forward angle
  Obs_State_F.setElement(3, PitchVel); // rotational velocity
  Obs_State_S.setElement(0, Roll); // side angle
  Obs_State_S.setElement(1, RollVel); // rotational velocity sideways
  Obs_State_S.setElement(2, flyEncPos); // flywheel angle
  Obs_State_S.setElement(3, flyEncVel); // rotational velocity flywheel
  // Use Kalman filter to determine velocities (or low pass filter)
  KALMAN_F.updateState(State_F, Obs_State_F, U_F);
  KALMAN_S.updateState(State_S, Obs_State_S, U_S);
  // DEBUG
  /*DPRINTSFN(10, " STATE 1:", wheelEncPos * 0.04, -6, 2);
  DPRINTSFN(10, " STATE 2:", wheelEncVel * 0.04, -6, 2);
  DPRINTSFN(10, " STATE 3:", -1 * Pitch, -6, 2);
  DPRINTSFN(10, " STATE 4:", PitchVel, -6, 5);
  DPRINTLN();*/
  DPRINTSFN(10, " Roll:", Roll, -6, 3);
  DPRINTSFN(10, " RollVel:", RollVel, -6, 3);
  DPRINTLN();
}

void computeInputs() {
  // LQR model for computing inputs
  int its_F = LQR_F.computeInput(Obs_State_F, Des_State_F, U_F);
  /*DPRINTSFN(10, " INPUT:", U_F, -6, 2);
  DPRINTSFN(10, " ITS:", its, -6, 2);
  DPRINTLN();*/
  int its_S = LQR_S.computeInput(Obs_State_S, Des_State_S, U_S);
  DPRINTSFN(10, " INPUT:", U_S, -6, 2);
  DPRINTSFN(10, " ITS:", its_S, -6, 2);
  DPRINTLN();
}

void applyControl() {
  // Apply calculated inputs to correct motors
  driveMotor(U_S, M1A, M1B); // Flywheel
  //driveMotor(U_F, M2A, M2B); // Wheel
}

// ================================================================
// ===                         SETUP                            ===
// ================================================================
void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor to be ready
  #ifdef DEBUG
    //while (!Serial); 
  #endif
  Serial.println(F("Setting up I2C..."));
  i2cSetup();
  Serial.println(F("Initialising MPU6050..."));
  MPU6050Connect();
  Serial.println(F("Setting up motor encoders..."));
  encoderSetup();
  Serial.println(F("Setting up motors..."));
  motorSetup();
  Serial.println(F("Testing motors in 2s..."));
  delay(2000);
  testMotors();
  Serial.println(F("Motor test completed..."));
  Serial.println(F("Setting up interrupt timers..."));
  timersSetup();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  
  while (!detectStart()) {
    // Update IMU values
    pollDMP();
    delay(50);
  }
  
  while (!detectFall()) {
    // Check for control interrupt
    if (controlInterrupt) {
      // Turn update flag off
      controlInterrupt = false;
      // Update States
      updateStates();
      // Calculate control
      computeInputs();
      // Apply control
      applyControl();
    }
  }
}

// ================================================================
// ===                Debug Matrix/Vector Class                 ===
// ================================================================
// Put this in a unit test module or something
void debugMatVec() {
  // Initialize
  float matArr[4][4] {{19,7,16,2},{19,19,8,3},{10,8,5,19},{10,3,9,20}};
  float rowArr[4] {12,2,5,8};
  float colArr[4] {17,1,1,4};
  Matrix4 mat(matArr);
  RowVector4 rowvec(rowArr);
  ColumnVector4 colvec(colArr);
  
  // Multiplication
  float mat2Arr[4][4] {{674,400,458,403},{832,567,523,307},{582,319,420,519},{537,259,409,600}};
  float rowmat[4] {396,186,305,285};
  float matcol[4] {354,362,259,262};
  float vecvec[4][4] {{204,34,85,136},{12,2,5,8},{12,2,5,8},{48,8,20,32}};

  assert((mat * mat) == Matrix4(mat2Arr));
  assert((rowvec * mat) == RowVector4(rowmat));
  assert((mat * colvec) == ColumnVector4(matcol));
  assert((rowvec * colvec) == 243);
  assert((colvec * rowvec) == Matrix4(vecvec));
  assert(rowvec.T() == ColumnVector4(rowArr));
}

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link. 
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();
}
