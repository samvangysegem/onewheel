## Mechanical Design

### Flywheel Design
3D printed wheel lacks the weight to fulfill the role of a flywheel. Therefore, a number of bolts are added at a large outside diameter to increase the total moment of inertia.

Source: https://itafasteners.com/weight-chart.php
According to the site mentioned above, the total weight of 100 M8 bolts is approximately 1.593kg, resulting in a weight of 15.9g x 12 at a radius of 40mm (with nuts).

Outside diameter: 95mm
Inside diameter: 7mm
Diameter of bols: 80mm
Pitch diameter fastening nuts: 19.06mm
Diameter fastening nuts: 3.4mm

**Components**
* Bolts
    * Online store: https://nl.rs-online.com/web/p/hex-bolts/0190254
* Nuts
    * Online store: https://nl.rs-online.com/web/p/hex-nuts/1224404


### Belt/Pulley Design
Calculator: https://www.bbman.com/belt-length-calculator/

* Center distance
    * 70 mm

**Components**
* Pulleys
    * 16 teeth
    * 25.5 mm pitch diameter
    * 3D Model: https://www.traceparts.com/en/product/gates-standard-pulley-t5-pitch-5-mm-width-10-mm-number-of-teeth-16?CatalogPath=TRACEPARTS%3ATP01005001001002&Product=32-28092010-116015&PartNumber=P21T5162FA

* Belt: 220mm length
    * Online store: https://nl.rs-online.com/web/p/timing-belts/4745347/

## Software Design
Control is split up in forwards and sideways control, assuming independent systems (which off course is not correct but for sufficiently small deviations should provide a good approach to control). 

### MPU6050
Calibration is required to guarantee correct operating behaviour. This can be achieved through the MPU 6050 library in the Arduino IDE, example sketch "IMU_Zero.ino". Calibration is best performed before installing the sensor on the robot in order to yield optimal results on a flat surface.

General operation: https://mjwhite8119.github.io/Robots/mpu6050

### Motor Encoders

### Kalman Filter
The Kalman filter is responsible for estimating the necessary velocity estimates based on the position estimates and mathematical model. Immediately deriving the velocity from the position estimate would induces a high amount of noise in the velocity estimates. Integrating the acceleration values on the other hand would be another good approach, however these are quite difficult to access.

### LQR Control

### Matrix4 and Vector4
Custom classes written to perform matrix calculations for Kalman filter and LQR control. Row and column vectors are separated in order to easily implement the multiplication overloading of these different vectors with each other and matrices.
- [ ] Inverse operator for Matrix class (efficient implementation)
- [ ] Copy constructors

## Git Commands
git status
git log --stat
git add
git commit -m "Message"
git push
git pull
git rm --cached *filename*