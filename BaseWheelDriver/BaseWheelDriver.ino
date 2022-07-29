#define FDEBUG
//#define BUZZER_PIN 15
#define USE_FGUI

#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <MPU9250_WE.h>
#include <Wire.h>

#include "flib.h"
#include "fComms.h"
#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

double gyroDeadzone = 1; //degrees / second

int pinA = 13;
int pinB = 14;
long currentPos;

double xspeed;
double zspeed;
double thspeed;

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
    WheelAssembly(int encA, int encB, int motorA, int motorB) : pid(&Input, &Output, &Setpoint, 0.05, 0.25, 0.00075, DIRECT), encoder() {
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

    double GetPosition() {
        return encoder.getCount();
    }

    void SetP(double val) {
        pid.SetTunings(val, pid.GetKi(), pid.GetKd());
    }

    void SetI(double val) {
        pid.SetTunings(pid.GetKp(), val, pid.GetKd());
    }

    void SetD(double val) {
        pid.SetTunings(pid.GetKp(), pid.GetKi(), val);
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
WheelAssembly wheel2 = WheelAssembly(19, 18, 32, 33);  //RIGHT FRONT REVERSED
WheelAssembly wheel3 = WheelAssembly(14, 15, 25, 26);  //RIGHT BACK  REVERSED
WheelAssembly wheel4 = WheelAssembly(23, 12, 17, 16);  //LEFT  BACK

void setup() {
    //attachInterrupt(digitalPinToInterrupt(pinA), encoderInterruptARising, RISING);
    fDebugUtils::startup_beep();
    fSerialParser::AddCommand("set", &setSpeeds);
    fSerialParser::AddCommand("getVel", &getSpeeds);
    fSerialParser::AddCommand("getPos", &getPos);
    fSerialParser::AddCommand("calibrate", &calibGyro);

    fComms::AddCommand("calibrate", [](String param) {
        calibGyro();
        });

    fComms::AddCommand("get_encoders", [](String param) {
        fComms::TCPSend("W1: " + String(wheel1.GetPosition()));
        fComms::TCPSend("W2: " + String(wheel2.GetPosition()));
        fComms::TCPSend("W3: " + String(wheel3.GetPosition()));
        fComms::TCPSend("W4: " + String(wheel4.GetPosition()));
        });

    fComms::AddCommand("set_speed", [](String data) {
        String command;
        String args;

        for (int i = 0; i < data.length(); i++) {
            if (data[i] == ' ') {
                args = data.substring(i + 1);
                break;
            }

            command += data[i];
        }

        if (command == "x") {
            xspeed = atof(args.c_str());
            fComms::TCPSend("set x speed: " + String(xspeed));
        }

        if (command == "z") {
            zspeed = atof(args.c_str());
            fComms::TCPSend("set z speed: " + String(zspeed));
        }

        if (command == "th") {
            thspeed = atof(args.c_str());
            fComms::TCPSend("set th speed: " + String(thspeed));
        }

        });

    fComms::AddCommand("set_pid", [](String data) {
        String command;
        String args;

        for (int i = 0; i < data.length(); i++) {
            if (data[i] == ' ') {
                args = data.substring(i + 1);
                break;
            }

            command += data[i];
        }

        if (command == "p") {
            double p = atof(args.c_str());

            wheel1.SetP(p);
            wheel2.SetP(p);
            wheel3.SetP(p);
            wheel4.SetP(p);

            fComms::TCPSend("set PID P: " + String(xspeed));
        }

        if (command == "i") {
            double i = atof(args.c_str());

            wheel1.SetI(i);
            wheel2.SetI(i);
            wheel3.SetI(i);
            wheel4.SetI(i);

            fComms::TCPSend("set PID I: " + String(zspeed));
        }

        if (command == "d") {
            float d = atof(args.c_str());

            wheel1.SetD(d);
            wheel2.SetD(d);
            wheel3.SetD(d);
            wheel4.SetD(d);

            fComms::TCPSend("set PID D: " + String(thspeed));
        }

        });

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


    flib_Startup();
    fComms::StartAsTask("WHEELDRV");

    calibGyro();

    xTaskCreate(OdomDriveTask, "DRIVE", 20000, NULL, 2, NULL);
}

void calibGyro() {
    fDebugUtils::beep();
    fGUI::StartMenu();
    fDebugUtils::success_background();
    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("CALIB GYRO", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 6, true, true);

        fGUI::Flush(true);

        delay(200);
    }
    fDebugUtils::beep();

    fDebugUtils::Log("Calibrating IMU!");
    myMPU9250.autoOffsets();

    long startms = millis();
    float voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;

    fGUI::SetFont(u8g2_font_6x10_tr, true);
    fGUI::PrintCentered("CALIB DONE", 64, 16, true);
    fGUI::ProgressBar(64, 36, 96, 16, 1, true, true);
    fGUI::Flush(true);

    delay(500);
    fGUI::EndMenu();
}

void setSpeeds() {
    //Serial.println("Setting speeds");
    xspeed = Serial.parseFloat();
    zspeed = Serial.parseFloat();
    thspeed = Serial.parseFloat();
    //Serial.println("Speeds are: X= " + String(xspeed) + ", Y=" + String(zspeed) + ", THETA = " + String(thspeed));
}

void updateSpeeds() {
    wheel1.TargetRPS = xspeed - zspeed - thspeed;
    wheel2.TargetRPS = -xspeed - zspeed + thspeed;
    wheel3.TargetRPS = xspeed - zspeed + thspeed;
    wheel4.TargetRPS = -xspeed - zspeed - thspeed;
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

void OdomDriveTask(void* param) {
    while (true) {
        wheel1.Drive();
        wheel2.Drive();
        wheel3.Drive();
        wheel4.Drive();

        Odometry();

        updateSpeeds();
        delay(50);
    }
}

// the loop function runs over and over again until power down or reset
long lastPrintMS;
long lastLoopMs;
double rate;

void loop() {
    double secs = (double)millis() / 1000;

    if (fmod(secs, 2) < 1)
        fGUI::ProgressBar(124, 62, 8, 4, fmod(secs, 2), true);
    else
        fGUI::ProgressBar(124, 62, 8, 4, 2 - fmod(secs, 2), false);

    fGUI::SetFont(u8g2_font_8x13_tr);
    fGUI::Print("RPS:", 0, 21);
    fGUI::Print(String((double)floor(wheel1.RPS * 10) / 10) + " " + String((double)floor(wheel2.RPS * 10) / 10), 56, 14);
    fGUI::Print(String((double)floor(wheel4.RPS * 10) / 10) + " " + String((double)floor(wheel3.RPS * 10) / 10), 56, 28);

    fGUI::Print("Hdg: ", 0, 45);
    fGUI::PrintCentered(String(gyroHeading), 82, 45);


    fGUI::SetFont(u8g2_font_5x7_tr);
    if (fWiFiManager::GetStatus() != "Not started")
        fGUI::Print(fWiFiManager::GetStatus(), 2, 64);

    fGUI::Print(fComms::GetStatus(), 2, 56);

    fGUI::Flush();
    delay(50);
}
