x = 0
y = 1
z = 1

r = sqrt(x*x+y*y);				
h = sqrt(r*r+(z-d1)*(z-d1));
desmotortheta1 = atan2(y, x);
thetax = atan2(z-d1,r);	
thetay = acos((h*h+len*len-len*len)/(2*h*len));	
theta2 = thetax+thetay;			
desmotortheta2 = -theta2 + PI/2;	
thetag = acos((-(h*h)+len*len+len*len)/(2*len*len));
theta3 = PI - thetag;
desmotortheta3 = theta3 + desmotortheta2 -PI/2;

