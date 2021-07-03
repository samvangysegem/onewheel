// ================================================================
// ===                         ONEDUINO                         ===
// ================================================================

// https://forum.arduino.cc/index.php?topic=408980.0 Post nr 4

#define DEBUG

#ifdef DEBUG
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));} //StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

// ================================================================
// ===                        LIBRARIES                         ===
// ================================================================

// Installed libraries
#include <Wire.h>
#include <I2Cdev.h>
// #include "MPU6050.h"
#include <MPU6050_6Axis_MotionApps20.h>


// Custom libraries
#include "Kalman.h"
#include "LQR.h"

#define LED_PIN 13

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

int MPUOffsets[6] = {-990,-1285,1185,-14,71,95};

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
  
  // Calibration Time: generate offsets and calibrate our MPU6050 (6 loops -> 600 readings)
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
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
  
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] * 180.0 / M_PI);
  Roll = (ypr[2] * 180.0 / M_PI);
}

// ================================================================
// ===                       Model Setup                        ===
// ================================================================
const float A[4][4] PROGMEM = {{1,0.09393943,-0.11034489,-0.00341160},
                               {0,0.86322326,-2.63312777,-0.11034489},
                               {0,0.06076874,2.59774256,0.14927222},
                               {0,1.45010660,38.28278264,2.59774256}};
const float B[4] PROGMEM = {0.00153432,0.03462702,-0.01538449,-0.36711559};
const float Q[4] PROGMEM = {10,2,2,1};
const float R PROGMEM = 0.3;

float state[4] = {0.2,-0.1,-PI/72,0};
float ksi[4] = {0,0,0,0};

LQR forwardLQR(A, B, Q, R);

// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  Serial.begin(115200);
  // Wait for Serial Monitor to be ready
  while (!Serial); 
  Serial.println(F("i2cSetup"));
  i2cSetup();
  Serial.println(F("MPU6050Connect"));
  MPU6050Connect();
  Serial.println(F("Interrupt Timers Setup"));
  timersSetup();
  Serial.println(F("Setup complete"));
  Serial.println(F("Debug section"));
  debugMatVec();
  Serial.println(F("Debug complete"));
  pinMode(LED_PIN, OUTPUT);

  unsigned long time1 = micros();
  float input = forwardLQR.getControl(state, ksi);
  unsigned long time2 = micros();
  Serial.println(input, 4);
  Serial.println(time2-time1);
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
  }
}

// ================================================================
// ===                Debug Matrix/Vector Class                 ===
// ================================================================
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

  Serial.print("Mat * Mat: ");
  Serial.println((mat * mat) == Matrix4(mat2Arr));
  Serial.print("Vec * Mat: ");
  Serial.println((rowvec * mat) == RowVector4(rowmat));
  Serial.print("Mat * Vec: ");
  Serial.println((mat * colvec) == ColumnVector4(matcol));
  Serial.print("Vec[row] * Vec[col]: ");
  Serial.println((rowvec * colvec) == 243);
  Serial.print("Vec[col] * Vec[row]: ");
  Serial.println((colvec * rowvec) == Matrix4(vecvec));
  Serial.print("Vec Transpose: ");
  Serial.println(rowvec.T() == ColumnVector4(rowArr));
  Serial.println("Matrix Inverse");
  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      Serial.println(mat.Inv().getElement(i, j), 5);
    }
  }
}
