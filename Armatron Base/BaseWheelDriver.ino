#define FDEBUG
#define BUZZER_PIN 15

#include <ESP32Encoder.h>
#include <fWiFiManager.h>
#include <PID_v1.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

double gyroDeadzone = 1; //degrees / second

int pinA = 13;
int pinB = 14;
long currentPos;

/*
void encoderInterruptAFalling() {
    if (digitalRead(pinB))
        currentPos++;
    else
        currentPos--;
}

void encoderInterruptARising() {
    if (!digitalRead(pinB))
        currentPos++;
    else
        currentPos--;
}
*/

class WheelAssembly {
public:
    WheelAssembly(int encA, int encB, int motorA, int motorB) : pid(&Input, &Output, &Setpoint, 0.1, 0.25, 0.00075, DIRECT), encoder() {
        encoderPinA = encA;
        encoderPinB = encB;

        motorPinA = motorA;
        motorPinB = motorB;
    }

    void Begin() {
        //ESP32Encoder::useInternalWeakPullResistors = UP;
        encoder.attachHalfQuad(encoderPinA, encoderPinB);

        pid.SetMode(AUTOMATIC);
        pid.SetOutputLimits(-255, 255);
    }

    void Drive() {
        long time = millis();
        double dT = (double)(time - lastRPSCheckMillis) / 1000;
        double dX = (double)(encoder.getCount() - lastEncoderPos) / 823.1;

        lastEncoderPos = encoder.getCount();
        lastRPSCheckMillis = time;

        RPS = dX / dT;

        Input = RPS * 823.1;
        Setpoint = TargetRPS * 823.1;
        pid.Compute();

        if (Setpoint != 0) {
            if (Setpoint > 0) {
                analogWrite(motorPinA, 255 - Output);
                analogWrite(motorPinB, 255);
            }
            else {
                analogWrite(motorPinB, 255 + Output);
                analogWrite(motorPinA, 255);
            }
        }
        else {
            analogWrite(motorPinA, 255);
            analogWrite(motorPinB, 255);
        }
    }

    double RPS, TargetRPS;
private:
    PID pid;
    ESP32Encoder encoder;
    double Setpoint, Input, Output;

    int encoderPinA, encoderPinB;
    int motorPinA, motorPinB;

    long lastRPSCheckMillis;
    long lastEncoderPos;
};

WheelAssembly wheel1 = WheelAssembly(13, 27, 4, 5);    //LEFT  FRONT
WheelAssembly wheel2 = WheelAssembly(18, 19, 34, 35);  //RIGHT FRONT
WheelAssembly wheel3 = WheelAssembly(22, 23, 26, 25);  //RIGHT BACK
WheelAssembly wheel4 = WheelAssembly(35, 34, 32, 33);  //LEFT  BACK

void setup() {
    //attachInterrupt(digitalPinToInterrupt(pinA), encoderInterruptARising, RISING);
    fDebugUtils::startup_beep();
    fSerialParser::AddCommand("set", &setSpeeds);
    fSerialParser::AddCommand("getVel", &getSpeeds);
    fSerialParser::AddCommand("getPos", &getPos);
    fSerialParser::AddCommand("calibrate", &ResetIMU);

    fSerialParser::Begin(115200);
    //fWiFiManager::Start();

    wheel1.Begin();
    wheel2.Begin();
    wheel3.Begin();
    wheel4.Begin();

    Wire.begin();
    if (!myMPU9250.init()) {
        fDebugUtils::Log("Can't communicate with IMU!");

        while (true)
            fDebugUtils::error_tone();
    }
    if (!myMPU9250.initMagnetometer()) {
        fDebugUtils::Log("Can't init magnetometer!");

        while (true)
            fDebugUtils::error_tone();
    }
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_8HZ);

    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);


    fDebugUtils::Log("Calibrating IMU!");
    fDebugUtils::alert_beep();
    myMPU9250.autoOffsets();
}

void setSpeeds() {
    //Serial.println("Setting speeds");
    double xspeed = Serial.parseFloat();
    double zspeed = Serial.parseFloat();
    double thspeed = Serial.parseFloat();

    wheel1.TargetRPS = xspeed - zspeed - thspeed;
    wheel2.TargetRPS = -xspeed - zspeed + thspeed;
    wheel3.TargetRPS = xspeed - zspeed + thspeed;
    wheel4.TargetRPS = -xspeed - zspeed - thspeed;

    //Serial.println("Speeds are: X= " + String(xspeed) + ", Y=" + String(zspeed) + ", THETA = " + String(thspeed));
}

double xpos;
double zpos;
double heading;
double gyroHeading;
long lastMS;

void getSpeeds() {
    double xspeed = wheel1.RPS + wheel3.RPS - wheel2.RPS - wheel4.RPS;
    double zspeed = -(wheel1.RPS + wheel2.RPS + wheel3.RPS + wheel4.RPS);
    double theta = myMPU9250.getAngles().x;

    Serial.println("xVel=" + String(xspeed) + ",zVel" + String(zspeed) + ",heading" + String(heading * 180 / 3.141) + "theta=" + String(theta) + "gyroH=" + String(gyroHeading));
}

void getPos() {
    Serial.println("xPos=" + String(xpos) + ",zPos" + String(zpos) + ",heading" + String(heading));
}

void Odometry() {
    long time = millis();

    double dX = wheel1.RPS + wheel3.RPS - wheel2.RPS - wheel4.RPS;
    double dZ = -(wheel1.RPS + wheel2.RPS + wheel3.RPS + wheel4.RPS);
    double dT = (double)(time - lastMS) / 1000;
    lastMS = time;

    double magX = myMPU9250.getMagValues().x;
    double magY = myMPU9250.getMagValues().y;
    double magZ = myMPU9250.getMagValues().z;

    /* Normalize magnetometer data */
    double h = sqrtf(magX * magX + magY * magY + magZ * magZ);
    magX /= h;
    magY /= h;
    magZ /= h;
    /* Compute euler angles */
    heading = atan2f(-magY, magX);


    double gyroSpeed = myMPU9250.getGyrValues().z;
    double gyroDelta = gyroSpeed * dT;

    gyroHeading += abs(gyroSpeed) > gyroDeadzone ? gyroDelta : 0;

    xpos += ((sin(heading) * dX) + (cos(heading) * dZ)) * dT;
    zpos += ((sin(heading) * dZ) + (cos(heading) * dX)) * dT;
}

void ResetIMU() {
    fDebugUtils::Log("Calibrating IMU");
    fDebugUtils::alert_beep();
    myMPU9250.autoOffsets();
    fDebugUtils::Log("Calibration done");
}

// the loop function runs over and over again until power down or reset
long lastPrintMS;
long lastLoopMs;
double rate;

void loop() {
    fSerialParser::parseSerial();

    wheel1.Drive();
    wheel2.Drive();
    wheel3.Drive();
    wheel4.Drive();

    Odometry();

    long time = millis();
    rate = 1000.0 / (double)(time - lastLoopMs);
    lastLoopMs = time;


    if (millis() - lastPrintMS > 50) {
        Serial.println("OK");
        Serial.println("Rate=" + String(rate) + " hz");
        lastPrintMS = millis();

        getSpeeds();
    }
    //Serial.println("RPS = " + String(wheel1.RPS));
    //Serial.println("TARGET = " + String(wheel1.TargetRPS));
    //Serial.println("OUTPUT = " + String(wheel1.Output));
}
