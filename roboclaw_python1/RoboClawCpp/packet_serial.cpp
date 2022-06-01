//
// Created by Jarred Sampayan on 5/18/22.
//

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <cstdlib> //This is to use the sleep function for Linux or Windows

//#include "RoboClaw.h"
#include <ioStream>
#include <time.h>
#include "Arduino.h"
#include "Roboclaw.cpp"


cout << "Test Start" << endl;

using namespace std;
//Test out forwards and backwards operation of a single roboclaw
class packet_serial {
public:
    int address1;
    int address2;
    int roboclaw;
    int S1M2_1;
    int S2M1_1;
    //void Roboclaw(String alpha, int beta) const;
};

int main() {

    Roboclaw roboclaw;
    address1 = 128;
    address2 = 129;
    //address1 = 0x80;
    //address2 = 0x81;

    roboclaw.Roboclaw("/dev/ttyS0",38400); //Create the RoboClaw object, passing the serial port and baudrate
    roboclaw.begin();

    roboclaw.SpeedAccelM1M2_2(address1,2000, 500,2000,0);
    roboclaw.SpeedAccelM1M2_2(address1,2000, 0,2000,500);
    roboclaw.SpeedAccelM1M2_2(address1,2000, 0,2000,0);
    roboclaw.SpeedAccelM1M2_2(address2,2000, 500,2000,0);
    roboclaw.SpeedAccelM1M2_2(address2,2000, 0,2000,500);
    roboclaw.SpeedAccelM1M2_2(address2,2000, 0,2000,0);

    roboclaw.ForwardM1(address1,0);
    roboclaw.ForwardM1(address2,0);
    roboclaw.ForwardM2(address1,0);
    roboclaw.ForwardM2(address2,0);

    sleep(0.5);

    //SpeedAccelM1M2_2(self,address,accel1,speed1(RPM),accel2,speed2(RPM)):
    roboclaw.SpeedAccelM1M2_2(address1,1000,500,1000,2220) //max rpm ~8220 @~16VDC
    roboclaw.SpeedAccelM1M2_2(address2,1000,500,1000,500)
    sleep(2);

    S1M2_1 = roboclaw.ReadSpeedM2(address1); //Speed Roboclaw 1, motor 2, instance 1
    S2M1_1 = roboclaw.ReadSpeedM1(address2);

    cout << S1M2_1 << endl;
    cout << S2M1_1 << endl;

    roboclaw.SpeedAccelM1M2_2(address1,1000,-500,2000,0);
    roboclaw.SpeedAccelM1M2_2(address2,1000,-500,2000,0);
    sleep(1);
    roboclaw.SpeedAccelM1M2_2(address1,1000,0,50,-1200);
    roboclaw.SpeedAccelM1M2_2(address2,1000,0,50,-1200);
    sleep(1);

    S1M2_2 = roboclaw.ReadSpeedM2(address1);  //Speed Roboclaw 1, motor 2, instance 1
    S2M1_2 = roboclaw.ReadSpeedM1(address2);

    cout << S1M2_2 << endl;
    cout << S2M1_2 << endl;

    roboclaw.ForwardM1(address1,0);
    roboclaw.ForwardM1(address2,0);
    roboclaw.ForwardM2(address1,0);
    roboclaw.ForwardM2(address2,0);

    cout << "Done" << endl;

    return 0;
}

