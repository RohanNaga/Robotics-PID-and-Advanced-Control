syms a0 a1 a2 a3 b0 b1 b2 b3 t 

theta0 = 0;
theta1 = .5;
theta2 = 0;
dtheta0 = 0;
dtheta1 = 0;
dtheta2 = 0;

q = a0 + a1*t+a2*t*t+a3*t*t*t;
dq = a1 +2*a2*t + 3*a3*t*t;
ddq = 2*a2 + 6*a3*t;

eq1 = subs(q,t,0) == theta0;
eq2 = subs(q,t,1) == theta1;
eq3 = subs(dq,t,0) == dtheta0;
eq4 = subs(dq,t,1) == dtheta1;

q_b = b0 + b1*t+b2*t*t+b3*t*t*t;
dq_b = b1 +2*b2*t + 3*b3*t*t;
ddq_b = 2*b2 + 6*b3*t;

eq5 = subs(q_b,t,1) == theta1;
eq6 = subs(q_b,t,2) == theta2;
eq7 = subs(dq_b,t,1) == dtheta1;
eq8 = subs(dq_b,t,2) == dtheta2;

sol_a = solve([eq1,eq2,eq3,eq4],[a0,a1,a2,a3]);
sol_b = solve([eq5,eq6,eq7,eq8],[b0,b1,b2,b3]);

a0 = sol_a.a0;
a1 = sol_a.a1;
a2 = sol_a.a2;
a3 = sol_a.a3;
b0 = sol_b.b0;
b1 = sol_b.b1;
b2 = sol_b.b2;
b3 = sol_b.b3;

q = a0 + a1*t+a2*t*t+a3*(t*t*t);
dq = a1 +2*a2*t + 3*a3*t*t;
ddq = 2*a2 + 6*a3*t;


q_b = b0 + b1*t+b2*t*t+b3*(t*t*t);
dq_b = b1 +2*b2*t + 3*b3*t*t;
ddq_b = 2*b2 + 6*b3*t;