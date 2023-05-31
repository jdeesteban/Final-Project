#include "LSM6DS3.h"
#include "Wire.h"

float accx, accy, accz;
float vx, vy;
float v0x, v0y;
float posx, pos0x;
float posy, pos0y;
float gx, gy, gz;
float agx, agy, agz;
float angx, angy, angz;
float ang0x, ang0y, ang0z;
long prev_time, dt;
//Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
 
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}
 
void loop() {

  accx= myIMU.readFloatAccelX()*9.81;
  accy= myIMU.readFloatAccelY()*9.81;
  accz= myIMU.readFloatAccelZ()*9.81;
  gx=myIMU.readFloatGyroX()*M_PI/180;
  gy=myIMU.readFloatGyroY()*M_PI/180;
  gz=myIMU.readFloatGyroZ()*M_PI/180;

  dt=(millis()-prev_time)*pow(10,-3);
  prev_time=millis()*pow(10,-3);
  //Position Reading
  if(accx>0.5 || accx<-0.5){
    vx= v0x + accx*dt;
    posx=pos0x + v0x*dt + 0.5*accx*dt*dt;
    v0x=vx;
    pos0x=posx;
  }   

  if(accy>0.2 || accy<-0.2){
    vy= v0y + accy*dt;
    posy=pos0y + v0y*dt + 0.5*accy*dt*dt;
    v0y=vy;
    pos0y=posy;
  }   

  //Orientation Reading
  /* /
  angx = (gx )*dt + ang0x;
  angy = (gy )*dt + ang0y;//*/
  //Calcular los ángulos con acelerometro
  float agx = atan(accy / sqrt(pow(accx, 2) + pow(accz, 2)))*(180.0 / 3.14);
  float agy = atan(-accx / sqrt(pow(accy, 2) + pow(accz, 2)))*(180.0 / 3.14);
  //Calcular angulo de rotación con giroscopio y filtro complementario
  angx = 0.98*(ang0x + (gx )*dt) + 0.02*agx;
  angy = 0.98*(ang0y + (gy)*dt) + 0.02*agy;
  ang0x=angx;
  ang0y=angy;

    //Accelerometer
 
    Serial.print(accx);
    Serial.print(",");
    Serial.print(accy);
    Serial.print(",");
    Serial.print(accz);
 
    //Gyroscope
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(angx);
    Serial.print(",");
    Serial.print(angy);
    Serial.print(",");
    Serial.println(0);
    //Thermometer
    /*Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C1 = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F1 = ");
    Serial.println(myIMU.readTempF(), 4);*/
 
    delay(1000);
}