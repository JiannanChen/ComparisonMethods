% Fig. 1 -> x1&x1d   
plot(x1d_Our, 'k');
hold on;
plot(x1_Our, '+');
hold on;
plot(x1_PID, 'o');
hold on;
plot(x1_BC, '*');
hold on;
plot(x1_tPPC, 'x');
hold on;
plot(x1_PPTPTC, 's');
hold on;
legend('x1d_Our', 'x1_Our', 'x1_PID', 'x1_BC', 'x1_tPPC', 'x1_PPTPTC');

% Fig. 2 -> x2&x2d 
plot(x2d_Our, 'k');
hold on;
plot(x2_Our, '+');
hold on;
plot(x2_PID, 'o');
hold on;
plot(x2_BC, '*');
hold on;
plot(x2_tPPC, 'x');
hold on;
plot(x2_PPTPTC, 's');
hold on;
legend('x2d_Our', 'x2_Our', 'x2_PID', 'x2_BC', 'x2_tPPC', 'x2_PPTPTC');

% Fig. 3 -> x3
plot(x3_Our, '+');
hold on;
plot(x3_PID, 'o');
hold on;
plot(x3_BC, '*');
hold on;
plot(x3_tPPC, 'x');
hold on;
plot(x3_PPTPTC, 's');
hold on;
legend('x3_Our', 'x3_PID', 'x3_BC', 'x3_tPPC', 'x3_PPTPTC');

% Fig. 4 -> z1  
plot(z1_Our, 'k');
hold on;
plot(kb1_Our, 'k');
hold on;
plot(kb1_Neg_Our, 'k');
hold on;
plot(z1_PID, 'o');
hold on;
plot(z1_BC, '*');
hold on;
plot(z1_tPPC, 'x');
hold on;
plot(rho1_tPPC, 'x');
hold on;
plot(rho1_Neg_tPPC, 'x');
hold on;
plot(z1_PPTPTC, 's');
hold on;
plot(rho1_PPTPTC, 's');
hold on;
plot(rho1_Neg_PPTPTC, 's');
hold on;
legend('z1_Our', 'kb1_Our', 'kb1_Neg_Our', 'z1_PID', 'z1_BC', 'z1_tPPC', 'rho1_tPPC', 'rho1_Neg_tPPC', 'z1_PPTPTC', 'rho1_PPTPTC', 'rho1_Neg_PPTPTC');

% Fig. 5 -> z2  
plot(z2_Our, 'k');
hold on;
plot(kb2_Our, 'k');
hold on;
plot(kb2_Neg_Our, 'k');
hold on;
plot(z2_PID, 'o');
hold on;
plot(z2_BC, '*');
hold on;
plot(z2_tPPC, 'x');
hold on;
plot(rho2_tPPC, 'x');
hold on;
plot(rho2_Neg_tPPC, 'x');
hold on;
plot(z2_PPTPTC, 's');
hold on;
plot(rho2_PPTPTC, 's');
hold on;
plot(rho2_Neg_PPTPTC, 's');
hold on;
legend('z2_Our', 'kb2_Our', 'kb2_Neg_Our', 'z2_PID', 'z2_BC', 'z2_tPPC', 'rho2_tPPC', 'rho2_Neg_tPPC', 'z2_PPTPTC', 'rho2_PPTPTC', 'rho2_Neg_PPTPTC');

% Fig. 6 -> z3 
plot(z3_Our, '+');
hold on;
plot(kb3_Our, 'k');
hold on;
plot(kb3_Neg_Our, 'k');
hold on;
plot(z3_PID, 'o');
hold on;
plot(z3_BC, '*');
hold on;
plot(z3_tPPC, 'x');
hold on;
plot(z3_PPTPTC, 's');
hold on;
legend('z3_Our', 'kb3_Our', 'kb3_Neg_Our', 'z3_PID', 'z3_BC', 'z3_tPPC', 'z3_PPTPTC');










