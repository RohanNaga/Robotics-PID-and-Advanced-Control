#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.438; //-0.37; // offset calculated by taking read values at zero position
float offset_Enc3_rad = 0.225; //0.27;  // offset calculated by taking read values at zero position


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

#pragma DATA_SECTION(custom, ".my_vars")
float custom = 0.0;

long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;  // variable used to output theta1motor value
float printtheta2motor = 0;  // variable used to output theta2motor value
float printtheta3motor = 0;  // variable used to output theta3motor value

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0; 
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;
float len = .254;  // length of each of the links
float x = 0;       // x value of end-effector
float y = 0;       // y value of end-effector
float theta2 = 0;  // theta 2 value in DH parameter values
float theta3 = 0;  // theta 3 value in DH parameter values
float r = 0;       // r value computing end effector distance in the x0-y0 plane away from the x0-y0 origin
float z = 0;       // z value of end effector from z0 origin
float desmotortheta1 = 0;   // calculated motor theta 1 from inverse kinematics
float desmotortheta2 = 0;   // calculated motor theta 2 from inverse kinematics
float desmotortheta3 = 0;   // calculated motor theta 3 from inverse kinematics
float thetag = 0;       // intermediate theta calculation to calculate theta 3
float h = 0;            // hypotenuse of the triangle connecting end effector position, origin, and r value. Used to calculate theta1 and theta2
float thetax = 0;       // intermediate angle calculation for theta 2
float thetay = 0;       // intermediate angle calculation for theta 2
float d1 = .254;        // value for d1 in DH table
float q2 = 0;
float q3 = 0;
float theta1des;
float theta2des;
float theta3des;
float theta1motorlast;
float theta2motorlast;
float theta3motorlast;
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;
float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;
float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float error1old = 0;
float error2old = 0;
float error3old = 0;
float error1 = 0;
float error2 = 0;
float error3 = 0;
float Integralold1 = 0;
float Integral1 = 0;
float Integralold2 = 0;
float Integral2 = 0;
float Integralold3 = 0;
float Integral3 = 0;
float T = 0;
float KP1 = 50;
float KD1 = 2.35;
float KP2 = 45;
float KD2 = 2.1;
float KP3 = 60;
float KD3 = 1.55;
float Ki1 = 300;
float Ki2 = 800;
float Ki3 = 1400;
float threshold1 = .02;
float threshold2 = .015;
float threshold3 = .015;
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;
float q = 0;
float v = 0;
float a = 0;
float t = 0;
float radius = 0.05;
float w = 2;

void mains_code(void);

//
// Main
//
void main(void)
{
	mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    t = (mycount)/1000.0;
    x = 0.35;
    y = radius*cos(w*t);
    z = radius*sin(w*t) + .3;

    // Inverse Kinematic Equations
    r = sqrt(x*x+y*y);              // r value taking the distance of the end-effector position from the origin to its position on the x0-y0 plane
    h = sqrt(r*r+(z-d1)*(z-d1));        // hypotenuse of the triangle connecting end effector position, origin, and r value. Used to calculate theta1 and theta2
    desmotortheta1 = atan2(y, x);       // motor 1 theta value from inverse kinematics. Theta1 in DH parameters and motor theta 1 are equal.
    thetax = atan2(z-d1,r);         // portion of theta 2 value calculated
    thetay = acos((h*h+len*len-len*len)/(2*len*h)); // second portion of theta 2 value calculated
    theta2 = thetax+thetay;             // Theta 2 calculated by adding subsidiary theta values
    desmotortheta2 = -theta2 + PI/2;            // motor theta 2 calculate
    thetag = acos((-(h*h)+len*len+len*len)/(2*len*len));// complement of theta 3 angle used to calculate theta 3
    theta3 = PI - thetag;               // theta 3 value calculated from thetag
    desmotortheta3 = theta3 + desmotortheta2 -PI/2; // Motor theta 3 calculated from theta 3

    theta1des = desmotortheta1;
    theta2des = desmotortheta2;
    theta3des = desmotortheta3;


    //if (mycount % 2000 < 1000){
    //   theta1des = 0;
    //    theta2des = 0;
    //    theta3des = 0;
    //} else {
    //    theta1des = PI/6;
    //    theta2des = PI/6;
    //    theta3des = PI/6;
    //}
    T = 0.001;

    error1 = theta1des - theta1motor;
    error2 = theta2des - theta2motor;
    error3 = theta3des - theta3motor;
    Integral1 = Integralold1 +((error1+error1old)/2)*T;
    Integral2 = Integralold2 +((error2+error2old)/2)*T;
    Integral3 = Integralold3 +((error3+error3old)/2)*T;
    Integralold1 = Integral1;
    Integralold2 = Integral2;
    Integralold3 = Integral3;
    error1old = error1;
    error2old = error2;
    error3old = error3;

    if(fabs(error1) > threshold1) {
        Integral1 = 0;
        Integralold1 = 0;
    }

    if(fabs(error2) > threshold2) {
            Integral2 = 0;
            Integralold2 = 0;
        }

    if(fabs(error3) > threshold3) {
            Integral3 = 0;
            Integralold3 = 0;
        }
    //getting Omega values
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    //order matters here. Why??
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega1_old2 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    //order matters here. Why??
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    //order matters here. Why??
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


    if (mycount % 2000 < 1000) {
        q = - t*t*t + (3*t*t)/2;
        v = - 3*t*t + 3*t;
        a = 3 - 6*t;
    } else {
        q = t*t*t - (9*t*t)/2 + 6*t - 2;
        v = 3*t*t - 9*t + 6;
        a = 6*t - 9;
    }
    // PID Control
    *tau1 = KP1*(theta1des-theta1motor)-KD1*Omega1 + Ki1 * Integral1;
    *tau2 = KP2*(theta2des-theta2motor)-KD2*Omega2 + Ki2 * Integral2;
    *tau3 = KP3*(theta3des-theta3motor)-KD3*Omega3 + Ki3 * Integral3;

    //Forward Feed control
    //*tau1 = J1*a + KP1*(q-theta1motor)+KD1*(v-Omega1) + Ki1 * Integral1;
    //*tau2 = J2*a + KP2*(q-theta2motor)+KD2*(v-Omega2) + Ki2 * Integral2;
    //*tau3 = J3*a + KP3*(q-theta3motor)+KD3*(v-Omega3) + Ki3 * Integral3;

    //Motor torque limitation(Max: 5 Min: -5)
    if (*tau1 > 5) {
        *tau1 = 5;
    } else if(*tau1 < -5){
        *tau1 = -5;
    }

    if (*tau2 > 5) {
            *tau2 = 5;
        } else if(*tau2 < -5) {
            *tau2 = -5;
        }
    if (*tau3 > 5) {
            *tau3 = 5;
        } else  if(*tau3 < -5) {
            *tau3 = -5;
        }

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    printtheta1motor = theta1motor;		// utilized to print theta 1 value
    printtheta2motor = theta2motor;		// utilized to print theta 2 value
    printtheta3motor = theta3motor;		// utilized to print theta 3 value

    Simulink_PlotVar1 = theta1des;
    Simulink_PlotVar2 = theta2des;
    Simulink_PlotVar3 = theta3des;
    Simulink_PlotVar4 = y;

    //x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    //y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    //z = (127.0*cos(theta2motor))/500 - (127*sin(theta3motor))/500 + 127.0/500.0;

    theta1motorlast = theta1motor;
    theta2motorlast = theta2motor;
    theta3motorlast = theta3motor;


    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f %.2f,%.2f,%.2f,%.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor,desmotortheta1,desmotortheta2, desmotortheta3);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

