alpha1 = -pi/2;
d1 = 60.25;
t1 = theta1;

% Link 2
a2 = 128;
alpha2 = 0;
d2 = 0;
t2 = theta2 - pi/2;

% Link 3
a3 = 148;
alpha3 = 0;
d3 = 0;
t3 = theta3 + pi/2;

% Link 4
a4 = 133.4;
alpha4 = 0;
d4 = 0;
t4 = theta4;

H0_1 = DH(a0,alpha0,d0,t0);
H1_2 = DH(a1,alpha1,d1,t1);
H2_3 = DH(a2,alpha2,d2,t2);
H3_4 = DH(a3,alpha3,d3,t3);
H4_5 = DH(a4,alpha4,d4,t4);

H0_5 = simplify(subs(H0_1 * H1_2 * H2_3 * H3_4 * H4_5, {theta1,theta2,theta3,theta4}, {0,0,0,0}));

simplify(H0_5)
