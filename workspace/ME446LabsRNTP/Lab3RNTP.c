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


//Variables used for forward kinematics

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
float d1 = .254;        // value for d1 in DH table
float q2 = 0;
float q3 = 0;

//Trajectory tracking variables
float theta1des;
float theta2des;
float theta3des;
float theta1motorlast;
float theta2motorlast;
float theta3motorlast;

//Joint Angular Velocities and variables to get filtered omega values
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
float T = 0;

//Tuned KP and KD values to reduce error between desired path trajectory and actual path trajectory
float KP1 = 50;
float KD1 = 2.35;
float KP2 = 45;
float KD2 = 2.1;
float KP3 = 60;
float KD3 = 1.55;
float Ki1 = 300;
float Ki2 = 800;
float Ki3 = 1400;
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;
float q = 0;
float v = 0;
float a = 0;
float t = 0;
float radius = 0.05;
float w = 2;

// Friction control inputs to decide wether friction is viscous or coulombic and at what speed to switch between each type of friction
float minimum_velocity1 = 0.1;
float Viscous_positive1 = .185;
float Coulomb_positive1 = .18;
float Viscous_negative1 = .2;
float Coulomb_negative1 = -.25;
float slope_between_minimums1 = 3.6;

float minimum_velocity2 = 0.05;
float Viscous_positive2 = .3;
float Coulomb_positive2 = .4759;
float Viscous_negative2 = .31;
float Coulomb_negative2 = -.51;
float slope_between_minimums2 = 5;

float minimum_velocity3 = 0.05;
float Viscous_positive3 = .26;
float Coulomb_positive3 = .5339;
float Viscous_negative3 = .28;
float Coulomb_negative3 = -.5190;
float slope_between_minimums3 = 5;

// Friction torque variable
float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;

// Friction torque coefficients
float ff1 = .55;
float ff2 = .75;
float ff3 = .8;

// Virables and Values in matrices used to calculate the torques at each joint with a weakened rotated axis

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;
float JT_1[3];
float JT_2[3];
float JT_3[3];
float KPxyz[3];

// Velocity filter for x, y, z dot values
float x_old = 0;
float xdot_old1 = 0;
float xdot_old2 = 0;
float y_old = 0;
float ydot_old1 = 0;
float ydot_old2 = 0;
float z_old = 0;
float zdot_old1 = 0;
float zdot_old2 = 0;

// variables for path trajectory, speed and acceleration
float xd = 0;
float yd = 0;
float zd = 0;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;
float xdot = 0;
float ydot = 0;
float zdot = 0;

// KP and KD gains for state space and Impedence control
float KPx = 175;
float KDx = 14;
float KPy = 170;
float KDy = 14;
float KPz = 150;
float KDz = 11;

// Variables to set feedforward force
float Fzcmd = 0;
float Kt =  6;

// Variables to caluclate intermediate steps
float taux = 0;
float tauy = 0;
float tauz = 0;

// Straight line point a
float xa = .25;
float ya = 0;
float za = .25;

// Straight line point b
float xb = .45;
float yb = 0;
float zb = .5;

// time control for straight line 
float t_start = 0;
float t_total = 5;

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

    /* This section of code is used to get the Omega values for each joint.
    Because Omega values can fluctuate very quickly and have momentarily large values,
    we have utilized an FIR filter to average out the last three omega values to get a
    reasonable estimate for the current actual omega value */

    //getting Omega values
        Omega1 = (theta1motor - Theta1_old)/0.001;
        Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
        Theta1_old = theta1motor;
        //order matters here. Why??
        Omega1_old2 = Omega1_old1;
        Omega1_old1 = Omega1;

        Omega2 = (theta2motor - Theta2_old)/0.001;
        Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
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

    /* This code identifies which range of values each joint's angular velocity(omega) is in.
    This is important as the friction coefficient changes with the joints angular velocity.
    When the angular velocity of the joint is below or above the minimum velocity, the friction
    torque is equal to the Viscous coeefficient multiplied by the omega value plus the Coulombic friction offset.
    When it is below the minimum velocity value, the friction torque is equal to the known slope in this region
    multiplied by the omega value. It is importnant to use different vairable for the positive and negative sides
    of the omega values as there are different levels of friction.
    */
    
    if (Omega1 > minimum_velocity1) {
        u_fric1 = Viscous_positive1*Omega1 + Coulomb_positive1;
    } else if (Omega1 < -minimum_velocity1) {
        u_fric1 = Viscous_negative1*Omega1 + Coulomb_negative1;
    } else {
        u_fric1 = slope_between_minimums1*Omega1;
    }

    if (Omega2 > minimum_velocity2) {
        u_fric2 = Viscous_positive2*Omega2 + Coulomb_positive2;
    } else if (Omega2 < -minimum_velocity2) {
        u_fric2 = Viscous_negative2*Omega2 + Coulomb_negative2;
    } else {
        u_fric2 = slope_between_minimums2*Omega2;
    }

    if (Omega3 > minimum_velocity3) {
        u_fric3 = Viscous_positive3*Omega3 + Coulomb_positive3;
    } else if (Omega3 < -minimum_velocity3) {
        u_fric3 = Viscous_negative3*Omega3 + Coulomb_negative3;
    } else {
        u_fric3 = slope_between_minimums3*Omega3;
    }

    /* These are the set of equations which calculate the end effector positon of the robot based on its geometry
    */
    
    //Forward Kinematics
    x = .254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = .254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = .254*(1+cos(theta2motor)-sin(theta3motor));
    

    /* This code is utilized to set the desired trajectory for the robot to swtich 
    between 2 different points every second */
    /*
    if (mycount % 2000 < 1000){
        xd = .2;
        yd = 0;
        zd = .5;
    } else {
        xd = .25;
        yd = .25;
        zd = .44;
    }
    */

    // This sets the variable t to the time in seconds from the start of the code running.
    // mycount increments every millisecond so it is the time in milliseconds since the program started running
        
    t = mycount/1000.0;

    /* This code is used for the straight line path trajectory. We first find the total distance each way by subtracting the two desired points.
        We then set a variable equal to the percentage of time that has passed out of the total time for 1 straight line pass.
        Then using an if-else statement, if we are at less than half the total time, the path trajectory is calculated to go from point a to point b.
        If we are past half the total time, then the trajectory is calculated to go back from point b to point a. If we reach the total time, we set the
        start tinme equal to the current time to reset the loop.
    */
    float deltax = xb - xa;
    float deltay = yb - ya;
    float deltaz = zb - za;
    float t_percent = (t-t_start)/t_total;

    if (t-t_start < t_total/2) {
        xd = deltax*2*t_percent+xa;
        yd = deltay*2*t_percent+ya;
        zd = deltaz*2*t_percent+za;

        xd_dot = deltax/(2*t_total);
        yd_dot = deltay/(2*t_total);
        zd_dot = deltaz/(2*t_total);
    } else if (t-t_start < t_total){
        xd = deltax*(1-2*t_percent)+xb;
        yd = deltay*(1-2*t_percent)+yb;
        zd = deltaz*(1-2*t_percent)+zb;

        xd_dot = -deltax/(2*t_total);
        yd_dot = -deltay/(2*t_total);
        zd_dot = -deltaz/(2*t_total);
    } else {
        t_start = t;
    }

    /* This code is used to calculate the variables needed to calculate the final torques at each joint.
    We first calculat the velocities values by using an FIR filter. This FIR filter averages the last three velocity values
    to get an accurate estimation for the current velocity value. This is done by storing the last two velocity values
    in global variables.
    */
    
    //Finding x,y,z dot
    xdot = (x - x_old)/0.001;
    xdot = (xdot + xdot_old1 + xdot_old2)/3.0;
    x_old = x;
    //order matters here. Why??
    xdot_old2 = xdot_old1;
    xdot_old1 = xdot;

    ydot = (y - y_old)/0.001;
    ydot = (ydot + ydot_old1 + ydot_old2)/3.0;
    y_old = y;
    //order matters here. Why??
    ydot_old2 = ydot_old1;
    ydot_old1 = ydot;

    zdot = (z - z_old)/0.001;
    zdot = (zdot + zdot_old1 + zdot_old2)/3.0;
    z_old = z;
    //order matters here. Why??
    zdot_old2 = zdot_old1;
    zdot_old1 = zdot;


    /* This code was taken from the lab manual and is used to calculate the rotation and Jacobian matrices
    based on the thetax, thetay, and thetaz rotation values. */

    
    //Rotation xyz and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    //Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;
    JT_1[0] = JT_11;
    JT_1[1] = JT_12;
    JT_1[2] = JT_13;
    JT_2[0] = JT_21;
    JT_2[1] = JT_22;
    JT_2[2] = JT_23;
    JT_3[0] = JT_31;
    JT_3[1] = JT_32;
    JT_3[2] = JT_33;

    
    /* This code is used to calculate intermediary steps in the torque calculations to make debugging easier by checking intermediary values.
    In the below section these values will be used in the torque equation to calculate the torque with simple impedance control from the 
    equation given in the lab manual.
    */
    
    // This is the typical KP and KD control calculation but specified to each axis
    KPxyz[0] = KPx*(xd-x)+KDx*(xd_dot-xdot);
    KPxyz[1] = KPy*(yd-y)+KDy*(yd_dot-ydot);
    KPxyz[2] = KPz*(zd-z)+KDz*(zd_dot-zdot);

    // This is the error in desired x,y,z values and actual values
    float e1 = (xd-x);
    float e2 = (yd-y);
    float e3 = (zd-z);

    // This is the error in desired x,y,z velocities and the actual velocities
    float edot1 = (xd_dot-xdot);
    float edot2 = (yd_dot-ydot);
    float edot3 = (zd_dot-zdot);

    // This is an intermediary step to calculate repeated values in the torque equation with simple impedance control.
    taux = KPx*RT11*e1 + KPx*RT12*e2 + KPx*RT13*e3 + KDx*RT11*edot1 + KDx*RT12*edot2 + KDx*RT13*edot3;
    tauy = KPy*RT21*e1 + KPy*RT22*e2 + KPy*RT23*e3 + KDy*RT21*edot1 + KDy*RT22*edot2 + KDy*RT23*edot3;
    tauz = KPz*RT31*e1 + KPz*RT32*e2 + KPz*RT33*e3 + KDz*RT31*edot1 + KDz*RT32*edot2 + KDz*RT33*edot3;

    /* Below are the set of equations utilized to calcualte the torques for each joint of the robot. We have created torque equations to control
    the robot utilizing simple impedance control which can weaken the robots control in an axis direction. We have coded Task space PD control 
    that accounts for the torque needed to overcome friction which is important to tune KP and KD values to each of the x, y and z axes.
    We have created a control law for just friction control. We also still have PID and Feedforward control laws below from previous labs.
    */
    // This is the set of equations given in the lab manual utilized to calculate the torques with simple impedance cotrol
    // Simple Impedence Control
    *tau1 = ((JT_11*R11+JT_12*R21+JT_13*R31)*taux+(JT_11*R12+JT_12*R22+JT_13*R32)*tauy+(JT_11*R13+JT_12*R23+JT_13*R33)*tauz)+ff1*u_fric1;
    *tau2 = ((JT_21*R11+JT_22*R21+JT_23*R31)*taux+(JT_21*R12+JT_22*R22+JT_23*R32)*tauy+(JT_21*R13+JT_22*R23+JT_23*R33)*tauz)+ff2*u_fric2;;
    *tau3 = ((JT_31*R11+JT_32*R21+JT_33*R31)*taux+(JT_31*R12+JT_32*R22+JT_33*R32)*tauy+(JT_31*R13+JT_32*R23+JT_33*R33)*tauz)+ff3*u_fric3;;


    // Task Space PD Control with friction accounted for with feed forward control
    //*tau1 = JT_1[0]*KPxyz[0]+JT_1[1]*KPxyz[1]+JT_1[2]*KPxyz[2]+ff1*u_fric1+JT_1[2]*Fzcmd/Kt;
    //*tau2 = JT_2[0]*KPxyz[0]+JT_2[1]*KPxyz[1]+JT_2[2]*KPxyz[2]+ff2*u_fric2+JT_2[2]*Fzcmd/Kt;
    //*tau3 = JT_3[0]*KPxyz[0]+JT_3[1]*KPxyz[1]+JT_3[2]*KPxyz[2]+ff3*u_fric3+JT_3[2]*Fzcmd/Kt;

    //Friction Control
    //*tau1 = ff1*u_fric1;
    //*tau2 = ff2*u_fric2;
    //*tau3 = ff3*u_fric3;

    // PID Control
    //*tau1 = KP1*(theta1des-theta1motor)-KD1*Omega1 + Ki1 * Integral1;
    //*tau2 = KP2*(theta2des-theta2motor)-KD2*Omega2 + Ki2 * Integral2;
    //*tau3 = KP3*(theta3des-theta3motor)-KD3*Omega3 + Ki3 * Integral3;

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

    printtheta1motor = x;		// utilized to print theta 1 value
    printtheta2motor = y;		// utilized to print theta 2 value
    printtheta3motor = z;		// utilized to print theta 3 value

    Simulink_PlotVar1 = z;
    Simulink_PlotVar2 = zd;
    Simulink_PlotVar3 = y;
    Simulink_PlotVar4 = yd;

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

