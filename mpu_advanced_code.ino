/*
================================================================================
                        MPU6050 REGISTER MAP
================================================================================

SENSOR DATA REGISTERS (Read-only):
----------------------------------
0x3B    ACCEL_XOUT_H    Accelerometer X-axis High Byte
0x3C    ACCEL_XOUT_L    Accelerometer X-axis Low Byte
0x3D    ACCEL_YOUT_H    Accelerometer Y-axis High Byte
0x3E    ACCEL_YOUT_L    Accelerometer Y-axis Low Byte
0x3F    ACCEL_ZOUT_H    Accelerometer Z-axis High Byte
0x40    ACCEL_ZOUT_L    Accelerometer Z-axis Low Byte

0x41    TEMP_OUT_H      Temperature High Byte
0x42    TEMP_OUT_L      Temperature Low Byte

0x43    GYRO_XOUT_H     Gyroscope X-axis High Byte
0x44    GYRO_XOUT_L     Gyroscope X-axis Low Byte
0x45    GYRO_YOUT_H     Gyroscope Y-axis High Byte
0x46    GYRO_YOUT_L     Gyroscope Y-axis Low Byte
0x47    GYRO_ZOUT_H     Gyroscope Z-axis High Byte
0x48    GYRO_ZOUT_L     Gyroscope Z-axis Low Byte

I2C ADDRESS[MPU6050 ADDRESS]:
------------
Default: 0x68 (if AD0 pin is LOW)
Alternative: 0x69 (if AD0 pin is HIGH)

================================================================================
*/

#include <Wire.h>
#include <Math.h>
#define PI 3.14159265
int i; //used for gyro calibration loop
float ax,ay,az,gx,gy,gz;
float sumx=0,sumy=0,sumz=0,uncalibratedx,uncalibratedy,uncalibratedz,offsetx,offsety,offsetz; //used in gyroCalibration() function
float angleX=0, angleY=0, angleZ=0;
float accAngleX=0, accAngleY=0;
float prevtime, nowtime, dt;
void setup() {
  Serial.begin(9600);
  Wire.begin(); //Sets up the SDA (data) and SCL (clock) pins
  Wire.beginTransmission(0x68); //transmits to mpu
  Wire.write(0x6B); //points to the sleep register and when the next write function is called the bits are written into this register
  Wire.write(00000000); //here the 6th bit is 0 that means the sleep mode is tunred to 0
  Wire.endTransmission(true); //You've told the MPU what to do, now you're done talking.
  Serial.println("MPU woke up.");

  Serial.println("Gyro calibration starting.");
  gyroCalibration();
  Serial.println("Gyro calibration ended");
  prevtime = millis();
  Serial.println("nowTime(ms)\tdt(s)\tgx(ω)\tgy(ω)\tgz(ω)\tangleX\tangleY\tangleZ");
  Serial.println("------------------------------------------------------------------------");
}

void loop() {
Wire.beginTransmission(0x68); // so now wire is specifically selected towards the mpu address so now Everything after (Wire.) will be specifically redirected to the sensor
Wire.write(0x3B); //this is the starting register from where the i2c begins to read the information we get mpu
Wire.endTransmission(false); //grab the attention to the mpu only. i.e do not stop the communication with the mpu  
Wire.requestFrom(0x68,14); 
//the data for acceleration is of 16 bit and the gyro is for 16 bit but the register is of 8 bit so we read twice
ax = (Wire.read() << 8 | Wire.read())/16384.0;
ay = (Wire.read() << 8 | Wire.read())/16384.0;
az = (Wire.read() << 8 | Wire.read())/16384.0;

//we ignore the temp for now //this just reads past the registers and does nothing
Wire.read();
Wire.read();

gx =  ((Wire.read() << 8 | Wire.read())/131.0)-offsetx; //we subtract the offset so that we get 0 degree when the bot is still
gy =  ((Wire.read() << 8 | Wire.read())/131.0)-offsety;
gz =  ((Wire.read() << 8 | Wire.read())/131.0)-offsetz;


//GET THE ANGLES FROM ACCELEROMETER DATA
accAngleX = atan2(ay/az) * (180/PI);
accAngleY = atan2(-ax,sqrt(ay*ay+az*az)) * (180/PI);
//
//get the dt
nowtime = millis();
dt = (nowtime - prevtime)/1000; // change in time frame in seconds
prevtime = nowtime;
//the upper values are angular velocity i.e degrees per second (gx,gy,gz)
//so we convert them into angles using (degree = angular velocity * time taken) here, gx * dt = angle
//+ ADDITION OF THE COMPLEMENTARY FILTER SO THAT THE TRUE BIASNESS FROM THE DRIFT OF GYRO BECOMES NEGLIGIBLE
angleX = 0.98*(angleX + (gx * dt)) + 0.02 * accAngleX; // ∫gx dt [finalPosition = initialPosition + Change in position ]
angleY = 0.98*(angleX + (gx * dt)) + 0.02 * accAngleX;
angleZ = 0.98*(angleX + (gx * dt)) + 0.02 * accAngleX;

//tabulated output
  Serial.print(nowtime); Serial.print("\t");
  Serial.print(dt, 4); Serial.print("\t");
  Serial.print(gx, 2); Serial.print("\t");
  Serial.print(gy, 2); Serial.print("\t");
  Serial.print(gz, 2); Serial.print("\t");
  Serial.print(angleX, 2); Serial.print("°\t");
  Serial.print(angleY, 2); Serial.print("°\t");
  Serial.print(angleZ, 2); Serial.print("°\n");
//
delay(5);
}

void gyroCalibration(){
  for (i=1;i<=500;i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x43); //the first register for gyro //mpu6050 revision 4.2 page 31
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6); //hey "0x68" send me 6 bytes starting from where your pointer is "0x43" 

    uncalibratedx = (Wire.read() << 8 | Wire.read())/131.0;
    uncalibratedy = (Wire.read() << 8 | Wire.read())/131.0;
    uncalibratedz = (Wire.read() << 8 | Wire.read())/131.0;

    //we are summing up the biases produced by the hardware limitation of the gyro 
    //suppose the gyro shows 0.88 degrees when still which is a bias so we sum up biases up to 500 and then subtract the offsets

    sumx = (sumx + uncalibratedx); //find the sum of the total misleading x values when in still i.e when the robot does not move 
    sumy = (sumy + uncalibratedy); //find the sum of the total misleading y values when in still i.e when the robot does not move
    sumz = (sumz + uncalibratedz); //find the sum of the total misleading z values when in still i.e when the robot does not move
  
    delay(2);
  }
  offsetx = sumx/500; //get the average bias of the x values which is then subtracted later
  offsety = sumy/500; //get the average bias of the x values which is then subtracted later
  offsetz = sumz/500; //get the average bias of the x values which is then subtracted later
}
