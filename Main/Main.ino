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

// ================================================================
// ===                        LIBRARIES                         ===
// ================================================================

// Installed libraries
#include <assert.h>
#include <Wire.h>
#include <I2Cdev.h>
// #include "MPU6050.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include <Encoder.h>

// Custom libraries
#include "Kalman.h"
#include "LQR.h"

#define LED_PIN 13

#define M1A 8 
#define M1B 7

#define M2A 3
#define M2B 2

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
  controlTimer.begin(controlInterruptHandler, 100000); // Update frequency of 10 Hz
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
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    // LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

// ================================================================
// ===                    MPU Polling Data                      ===
// ================================================================
// Alternative to GetDMP without use of interrupts

float Yaw, Pitch, Roll;

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
  // Blink LED
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // DEBUG Purposes
  DPRINTSFN(15, " Yaw:", Yaw, -6, 2);
  DPRINTSFN(15, " Pitch:", Pitch, -6, 2);
  DPRINTSFN(15, " Roll:", Roll, -6, 2);
  DPRINTLN();
}

// ================================================================
// ===                        MPU Math                          ===
// ================================================================

void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  Yaw = (ypr[0] * 180.0 / M_PI); // Z
  Pitch = (ypr[1] * 180.0 / M_PI); // Y
  Roll = (ypr[2] * 180.0 / M_PI); // X
}

// ================================================================
// ===                      Encoder METHODS                     ===
// ================================================================

// Encoder for wheel rotation
Encoder wheelEnc(21, 22);
float wheelEncPos;

// Encoder for flywheel rotation
Encoder flyEnc(16, 17);
float flyEncPos;

void encoderSetup() {
  // Both encoder position can be initialised as 0
  // The initial position of the flywheel is of no importance
  // The initial position of the wheel is 0 at startup and thus also of no importance
  wheelEnc.write(0);
  flyEnc.write(0);
}

void pollEncoders() {
  // Get position information
  wheelEncPos = wheelEnc.read() / (18.75 * 64.0) * 360.0;
  flyEncPos = flyEnc.read() / (18.75 * 64.0) * 360.0;
  // DEBUG
  DPRINTSFN(10, " Wheel Encoder:", wheelEncPos, -6, 2);
  DPRINTSFN(10, " Fly Encoder:", flyEncPos, -6, 2);
  DPRINTLN();
}

// ================================================================
// ===                       Motor METHODS                      ===
// ================================================================

// Source: https://www.pjrc.com/teensy/td_pulse.html#frequency
// Pins M1A = 8 and M1B = 7 are controlled by FlexPWM1.3 timer
// Pins M2A = 3 and M2B = 2 are controlled by FlexPWM4.2 timer
void motorSetup() { 
  // Check if pins still correct, otherwise setup will fail
  assert((M1A == 8) && (M1B == 7));
  assert((M2A == 3) && (M2B == 2));
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
const float A_F[4][4] = {{1,0.09393943,-0.11034489,-0.00341160},
                               {0,0.86322326,-2.63312777,-0.11034489},
                               {0,0.06076874,2.59774256,0.14927222},
                               {0,1.45010660,38.28278264,2.59774256}};
const float B_F[4] = {0.00153432,0.03462702,-0.01538449,-0.36711559};
const float Q_F[4] = {10,2,2,1};
const float R_F = 0.3;

LQR forwardLQR(A_F, B_F, Q_F, R_F);
float forwardState[4] = {0,0,0,0};
float forwardKsi[4] = {0,0,0,0};

// Sideways control
// State: [phi, dphi, alpha, dalpha]
//        [Side angle, rotational velocity, flywheel angle, rotational velocity]
const float A_S[4][4] = {{1.41146607741256,0.113372087314826,0,-0.000255074418624809},
                               {8.75203510129462,1.41146607741256,0,-0.00560148085730519},
                               {0,0,1,0.111574942057396},
                               {0,0,0,1.24010372302915}};
const float B_S[4] = {0.00161439505458740,0.0354524104892734,-0.0732591269455416,-1.51964381664016};
const float Q_S[4] = {1,1,0.5,0.5};
const float R_S = 1;

LQR sidewayLQR(A_S, B_S, Q_S, R_S);
float sidewayState[4] = {0,0,0,0};
float sidewayKsi[4] = {0,0,0,0};

// ================================================================
// ===                        CONTROL                           ===
// ================================================================

void detectStart() {
  // 
}

void detectFall() {
  // Pitch and roll outside safety range
  
}

void updateState() {
  // Get current state
  
  // Get IMU and Encoder data
  pollDMP();
  pollEncoders();
  // Use Kalman filter to determine velocities (or low pass filter)
  
  // Updat state vectors accordingly
  forwardState = {}
}

// ================================================================
// ===                         SETUP                            ===
// ================================================================
void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor to be ready
  // while (!Serial); 
  Serial.println(F("Setting up I2C..."));
  i2cSetup();
  Serial.println(F("Initialising MPU6050..."));
  MPU6050Connect();
  Serial.println(F("Setting up interrupt timers..."));
  timersSetup();
  Serial.println(F("Setting up motor encoders..."));
  encoderSetup();
  Serial.println(F("Setting up motors..."));
  motorSetup();
  Serial.println(F("Testing motors in 2s..."));
  delay(2000);
  testMotors();
  Serial.println(F("Motor test completed..."));
  
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  if (controlInterrupt) {
    // Turn update flag off
    controlInterrupt = false;
    // Get updated position data
    pollDMP();
    // Get encoder position data
    pollEncoders();
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
