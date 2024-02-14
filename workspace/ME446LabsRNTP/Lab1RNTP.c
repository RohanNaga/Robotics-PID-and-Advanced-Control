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
float x = 0;	   // x value of end-effector
float y = 0;	   // y value of end-effector
float theta2 = 0;  // theta 2 value in DH parameter values
float theta3 = 0;  // theta 3 value in DH parameter values
float r = 0;	   // r value computing end effector distance in the x0-y0 plane away from the x0-y0 origin
float z = 0;	   // z value of end effector from z0 origin
float desmotortheta1 = 0;	// calculated motor theta 1 from inverse kinematics
float desmotortheta2 = 0;	// calculated motor theta 2 from inverse kinematics
float desmotortheta3 = 0;	// calculated motor theta 3 from inverse kinematics
float thetag = 0;		// intermediate theta calculation to calculate theta 3
float h = 0;			// hypotenuse of the triangle connecting end effector position, origin, and r value. Used to calculate theta1 and theta2
float thetax = 0;		// intermediate angle calculation for theta 2
float thetay = 0;		// intermediate angle calculation for theta 2
float d1 = .254;		// value for d1 in DH table

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


    *tau1 = 0;		// torque value for motor 1
    *tau2 = 0;		// torque value for motor 2
    *tau3 = 0;		// torque value for motor 3

    //Motor torque limitation(Max: 5 Min: -5)

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

    Simulink_PlotVar1 = theta1motor;		// sending theta 1 value to Simulink
    Simulink_PlotVar2 = theta2motor;		// sending theta 2 value to Simulink
    Simulink_PlotVar3 = theta3motor;		// sending theta 3 value to Simulink
    Simulink_PlotVar4 = 0;

	// Forward Kinematic Equations
    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;    // Forward kinematic equation for end-effector x-coordinate
    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;	 // Forward kinematic equation for end-effector y-coordinate
    z = (127.0*cos(theta2motor))/500 - (127*sin(theta3motor))/500 + 127.0/500.0; // Forward kinematic equation for end-effector z-coordinate

	// Inverse Kinematic Equations
    r = sqrt(x*x+y*y);				// r value taking the distance of the end-effector position from the origin to its position on the x0-y0 plane
    h = sqrt(r*r+(z-d1)*(z-d1));		// hypotenuse of the triangle connecting end effector position, origin, and r value. Used to calculate theta1 and theta2
    desmotortheta1 = atan2(y, x);		// motor 1 theta value from inverse kinematics. Theta1 in DH parameters and motor theta 1 are equal.
    thetax = atan2(z-d1,r);			// portion of theta 2 value calculated
    thetay = acos((h*h+len*len-len*len)/(2*h*len));	// second portion of theta 2 value calculated
    theta2 = thetax+thetay;				// Theta 2 calculated by adding subsidiary theta values
    desmotortheta2 = -theta2 + PI/2;			// motor theta 2 calculate
    thetag = acos((-(h*h)+len*len+len*len)/(2*len*len));// complement of theta 3 angle used to calculate theta 3
    theta3 = PI - thetag;				// theta 3 value calculated from thetag
    desmotortheta3 = theta3 + desmotortheta2 -PI/2;	// Motor theta 3 calculated from theta 3
    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f %.2f,%.2f,%.2f,%.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor,desmotortheta1,desmotortheta2, desmotortheta3);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

