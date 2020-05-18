/*
 *  Example for Moving RC servos
 */

#include <stdio.h>

#include <roboard.h>  // include this to use the RoBoIO library

int main(void) {
    unsigned long home[32]  = {0L};
    unsigned long frame[32] = {0L};
    
    // first set the correct RoBoard version
    roboio_SetRBVer(RB_100);
    //roboio_SetRBVer(RB_100RD);  // if your RoBoard is RB-100RD
    //roboio_SetRBVer(RB_110);    // if your RoBoard is RB-110
    //roboio_SetRBVer(RB_050);    // if your RoBoard is RB-050

    rcservo_SetServo(RCSERVO_PINS3, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S1 as non-feedback servo
    rcservo_SetServo(RCSERVO_PINS4, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S2 as non-feedback servo
    if (rcservo_Init(RCSERVO_USEPINS3 + RCSERVO_USEPINS4) == false)  // set PWM/GPIO pins S1 & S2 as Servo mode
    {
        printf("ERROR: fail to init RC Servo lib (%s)!\n", roboio_GetErrMsg());
        return -1;
    }

    home[2] = home[3] = 1500L;  // set the initial home position of all servos as 1500us
    rcservo_EnterPlayMode_HOME(home);  // enter Action Playing Mode for moving servos

    printf("Kalau sudah selesai tekan enter\n"); getchar();
    printf("Complete.\n");

    rcservo_Close();  // close RC Servo lib
    return 0;
}

