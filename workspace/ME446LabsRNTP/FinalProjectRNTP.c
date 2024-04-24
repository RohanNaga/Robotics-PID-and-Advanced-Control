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

float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;

float ff1 = .55;
float ff2 = .75;
float ff3 = .8;

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

float x_old = 0;
float xdot_old1 = 0;
float xdot_old2 = 0;
float y_old = 0;
float ydot_old1 = 0;
float ydot_old2 = 0;
float z_old = 0;
float zdot_old1 = 0;
float zdot_old2 = 0;

float xd = 0;
float yd = 0;
float zd = 0;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;
float xdot = 0;
float ydot = 0;
float zdot = 0;

float KPx = 200;
float KDx = 8;
float KPy = 150;
float KDy = 6;
float KPz = 300;
float KDz = 5;

float KPxn = 175;
float KPyn = 170;
float KPzn = 150;
float KDxn = 14;
float KDyn = 14;
float KDzn = 11;

float Fzcmd = 0;
float Kt =  6;

float taux = 0;
float tauy = 0;
float tauz = 0;

float xa = 0.14;
float ya = 0;
float za = 0.43;

float xb = .254;
float yb = 0;
float zb = .508;
float t_start = 0;
float t_total = 1;

float waypoint_length = 0;
int waypoint_count = 0;

void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}

typedef struct waypoints {
    float x;
    float y;
    float z;
}point;


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    point start = {.254,0,.508};
    point above_sun = {0.05,0.1,0.5};
    point above_peghole = {0.0314,0.3539,0.3};
    point above_peghole_lowerz = {0.0314,0.3539,0.19};
    point in_peghole = {0.0314,0.3539,0.12};
    point in_peghole2 = {0.0314,0.3539,0.12};
    point out_peghole = {0.0314,0.3539,0.3};
    point peghole_to_zigzag = {0.28,0.05,0.39};
    point align_zigzag = {0.4021,0.1148,0.3};
    point align_zigzag_height = {0.3839,0.1067,0.205};
    point start_zigzag = {0.4019,0.0963,0.2047};
    point through_zigzag1 = {0.417,0.0585,0.2047};

    point through_zigzag2 = {0.4185,0.0493,0.2047};

    point through_zigzag3 = {0.4107,0.0422,0.2047};
    point through_zigzag4 = {0.3484,0.0555,0.2047};

    point through_zigzag5 = {0.3461,0.0518,0.2047};
    point through_zigzag6 = {0.3325,0.0406,0.2047};

    point through_zigzag7 = {0.3334,0.0331,0.2047};
    point through_zigzag8 = {0.3856,-0.0308,0.2047};
    /*
    point through_zigzag1 = {0.4019,0.0858,0.2047};
    point through_zigzag2 = {0.4200,0.0608,0.2047};
    point through_zigzag3 = {0.4213,0.0518,0.2047};
    point through_zigzag4 = {0.4186,0.0466,0.2047};
    point through_zigzag5 = {0.4047,0.04014,0.2047};
    point through_zigzag6 = {0.3996,0.0394,0.2047};
    point through_zigzag7 = {0.3584,0.0490,0.2047};
    point through_zigzag8 = {0.34,0.0506,0.2047};
    point through_zigzag9 = {0.333,0.0482,0.2047};
    point through_zigzag10 = {0.329,0.0363,0.2047};
    point through_zigzag11 = {0.334,0.0268,0.2047};
    point through_zigzag12 = {0.384,-0.0309,0.2047};
    */
    point out_zigzag = {0.3978,-0.0659,0.3};
    point zigzag_to_egg = {0.2486,0.1875,0.4};
    point start_egg = {0.2486,0.1875,0.3};
    point end_egg = {0.2486,0.1875,0.29};
    point egg_hold1 = {0.2486,0.1875,0.279233};
    point egg_hold2 = {0.2486,0.1875,0.279233};
    point egg_hold3 = {0.2486,0.1875,0.279233};
    point egg_hold4 = {0.2486,0.1875,0.279233};
    point final_pos = {.254,0,.508};
    point final_pos2 = {.254,0,.508};
    point final_pos3 = {.254,0,.508};
    point final_pos4 = {.254,0,.508};

    point waypoints [] = {start, above_sun, above_peghole, above_peghole_lowerz, in_peghole, in_peghole2, out_peghole, peghole_to_zigzag, align_zigzag, align_zigzag_height, start_zigzag, through_zigzag1,through_zigzag2, through_zigzag3, through_zigzag4, through_zigzag5, through_zigzag6, through_zigzag7, through_zigzag8, out_zigzag, zigzag_to_egg, start_egg, end_egg, egg_hold1, egg_hold2, egg_hold3, egg_hold4, final_pos, final_pos2, final_pos3, final_pos4};
    int waypoints_length = 29;
    int variable_time [] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 , 1, .75, .75, .75 , .75, .75 , .75, .75, 1, 1, 1 , 1 , 1 , 1};

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


    //Forward Kinematics
    x = .254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = .254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = .254*(1+cos(theta2motor)-sin(theta3motor));

    t = mycount/1000.0;
    float deltax = xb - xa;
    float deltay = yb - ya;
    float deltaz = zb - za;
    if (waypoint_count >= waypoints_length) {
        waypoint_count = waypoints_length  - 1 ;
    }
    t_total = variable_time[waypoint_count];
    float t_percent = (t-t_start)/t_total;

    if (t-t_start < t_total) {
        xd = deltax*t_percent+xa;
        yd = deltay*t_percent+ya;
        zd = deltaz*t_percent+za;

        xd_dot = deltax/(t_total);
        yd_dot = deltay/(t_total);
        zd_dot = deltaz/(t_total);
    } else {
        t_start = t;
        xa = xb;
        ya = yb;
        za = zb;
        waypoint_count++;
        xb = waypoints[waypoint_count].x;
        yb = waypoints[waypoint_count].y;
        zb = waypoints[waypoint_count].z;
    }

    if (waypoint_count > waypoints_length - 1) {
        xb = .254;
        yb = 0;
        zb = .508;
    }

    if (waypoint_count > 9 && waypoint_count <= 12) {
        thetaz = .6435;
        KPx = 25;
        KDx = 5;
        KPy = 700;
        KDy = 8;
        KPz = 400;
        KDz = 7;
    } else if (waypoint_count == 13) {
        thetaz = 0;
        KPx = 800;
        KDx = 8;
        KPy = 500;
        KDy = 6;
        KPz = 400;
        KDz = 8;
    } else if (waypoint_count > 13 && waypoint_count <= 15) {
        thetaz = -.2618;
        KPy = 10;
        KDy = 5;
        KPx = 900;
        KDx = 11;
        KPz = 400;
        KDz = 9;
    } else if (waypoint_count == 16 || waypoint_count == 17) {
        thetaz = -.2618;
        KPy = 600;
        KDy = 5;
        KPx = 1000;
        KDx = 4;
        KPz = 400;
        KDz = 8;
    } else if (waypoint_count > 17 && waypoint_count < 18) {
        thetaz = .6435;
        KPx = 50;
        KDx = 5;
        KPy = 500;
        KDy = 10;
        KPz = 400;
        KDz = 11;
    } else if (waypoint_count >= 18) {
        thetaz = 0;
        KPx = 800;
        KDx = 11;
        KPy = 500;
        KDy = 10;
        KPz = 400;
        KDz = 9;
    }

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


    KPxyz[0] = KPx*(xd-x)+KDx*(xd_dot-xdot);
    KPxyz[1] = KPy*(yd-y)+KDy*(yd_dot-ydot);
    KPxyz[2] = KPz*(zd-z)+KDz*(zd_dot-zdot);

    float e1 = (xd-x);
    float e2 = (yd-y);
    float e3 = (zd-z);

    float edot1 = (xd_dot-xdot);
    float edot2 = (yd_dot-ydot);
    float edot3 = (zd_dot-zdot);

    taux = KPx*RT11*e1 + KPx*RT12*e2 + KPx*RT13*e3 + KDx*RT11*edot1 + KDx*RT12*edot2 + KDx*RT13*edot3;
    tauy = KPy*RT21*e1 + KPy*RT22*e2 + KPy*RT23*e3 + KDy*RT21*edot1 + KDy*RT22*edot2 + KDy*RT23*edot3;
    tauz = KPz*RT31*e1 + KPz*RT32*e2 + KPz*RT33*e3 + KDz*RT31*edot1 + KDz*RT32*edot2 + KDz*RT33*edot3;

    // Simple Impedence Control
    *tau1 = ((JT_11*R11+JT_12*R21+JT_13*R31)*taux+(JT_11*R12+JT_12*R22+JT_13*R32)*tauy+(JT_11*R13+JT_12*R23+JT_13*R33)*tauz)+ff1*u_fric1;
    *tau2 = ((JT_21*R11+JT_22*R21+JT_23*R31)*taux+(JT_21*R12+JT_22*R22+JT_23*R32)*tauy+(JT_21*R13+JT_22*R23+JT_23*R33)*tauz)+ff2*u_fric2;;
    *tau3 = ((JT_31*R11+JT_32*R21+JT_33*R31)*taux+(JT_31*R12+JT_32*R22+JT_33*R32)*tauy+(JT_31*R13+JT_32*R23+JT_33*R33)*tauz)+ff3*u_fric3;;

    if (mycount < 10 ) {
        *tau1 = 0;
        *tau2 = 0;
        *tau3 = 0;
    }

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

    Simulink_PlotVar1 = x;
    Simulink_PlotVar2 = xd;
    Simulink_PlotVar3 = yd;
    Simulink_PlotVar4 = zd;

    theta1motorlast = theta1motor;
    theta2motorlast = theta2motor;
    theta3motorlast = theta3motor;


    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.4f %.4f,%.4f,%.2f,%.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor,desmotortheta1,desmotortheta2, desmotortheta3);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

