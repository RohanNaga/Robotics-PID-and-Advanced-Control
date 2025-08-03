% %Rotation zxy and its T
% cosz = cos(thetaz);
% sinz = sin(thetaz);
% cosx = cos(thetax);
% sinx = sin(thetax);
% cosy = cos(thetay);
% siny = sin(thetay);
% R11 = cosz*cosy-sinz*sinx*siny;
% R12 = -sinz*cosx;
% R13 = cosz*siny+sinz*sinx*cosy;
% R21 = sinz*cosy+cosz*sinx*siny;
% R22 = cosz*cosx;
% R23 = sinz*siny-cosz*sinx*cosy;
% R31 = -cosx*siny;
% R32 = sinx;
% R33 = cosx*cosy;
% %Jacobian Transpose
% cosq1 = cos(theta1motor);
% sinq1 = sin(theta1motor);
% cosq2 = cos(theta2motor);
% sinq2 = sin(theta2motor);
% cosq3 = cos(theta3motor);
% sinq3 = sin(theta3motor);
% JT_11 = -0.254*sinq1*(cosq3 + sinq2);
% JT_12 = 0.254*cosq1*(cosq3 + sinq2);
% JT_13 = 0;
% JT_21 = 0.254*cosq1*(cosq2 - sinq3);
% JT_22 = 0.254*sinq1*(cosq2 - sinq3);
% JT_23 = -0.254*(cosq3 + sinq2);
% JT_31 = -0.254*cosq1*sinq3;
% JT_32 = -0.254*sinq1*sinq3;
% JT_33 = -0.254*cosq3;

syms KPx KPy KPz KDx KDy KDz
JT = sym('JT',[3,3]);
R = sym('R',[3,3]);
RT = sym('RT',[3,3]);
error = sym('e',[3,1]);
errordot = sym('edot',[3,1]);
KP = diag([KPx,KPy,KPz]);
KD = diag([KDx,KDy,KDz]);
%A = JT*R
B = KP*RT*error+KD*RT*errordot
