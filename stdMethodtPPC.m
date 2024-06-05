function [sys,x0,str,ts,simStateCompliance] = stdMethodtPPC(t,x,u,flag)
% General MATLAB S-Function Template

% General structure of an S-function.
switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case 2
    sys=[];
  case 3
    sys=mdlOutputs(t,x,u);
  case 4
    sys=[];
  case 9
    sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 14;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% initial conditions
x0  = [5.5, 0.1, 0];

str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

% Physical paras
M = 200; l = 0.3;
Jv = 10; Jw = 0.005; 
r = 0.1; k = 6; c = 0.05;
a1 = 2*c / (M*r^2 + 2*Jw); b1 = k*r / (M*r^2 + 2*Jw);
a2 = 2*c*l*l / (Jv*r^2 + 2*Jw*l^2); b2 = r*l*k / (Jv*r^2 + 2*Jw*l^2);

% Control objective
x1d = 3*sin(t); dx1d = 3*cos(t);
x2d = 0.2*sin(t); dx2d = 0.2*cos(t); ddx2d = -0.2*sin(t);

% Control paras
k1 = 1; k2 = 1; k3 = 1;
% rho10 = 6; rho1oo = 1; m1 = 10;
% rho20 = 4; rho2oo = 1; m2 = 1;
rho10 = 6; rho1oo = 1; m1 = 1;
rho20 = 4; rho2oo = 1; m2 = 1;

% Control inputs
z1 = x(1) - x1d;
rho1 = (rho10-rho1oo)*exp(-m1*t)+rho1oo;
drho1 = -m1*(rho10-rho1oo)*exp(-m1*t);
s1 = z1/rho1;
z1hat = 1/2*log((s1+1)/(1-s1));
Uv = (1/b1)*(a1*x(1) + dx1d + z1*drho1/rho1 - k1*z1hat);

z2 = x(2) - x2d;
rho2 = (rho20-rho2oo)*exp(-m2*t)+rho2oo;
drho2 = -m2*(rho20-rho2oo)*exp(-m2*t);
ddrho2 = m2^2*(rho20-rho2oo)*exp(-m2*t);
s2 = z2/rho2;
z2hat = 1/2*log((s2+1)/(1-s2));
R2 = 1/rho2/(1-s2^2);
alpha = dx2d - z2*drho2/rho2 - k2*z2hat;
dz2 = x(3) - dx2d;
dz2hat = R2*(dz2-z2*drho2/rho2);
d_alpha = ddx2d - dz2*drho2/rho2 - z2*ddrho2/rho2 + z2*drho2^2/rho2/rho2  - k2*dz2hat;

z3 = x(3) - alpha;
Up = (1/b2)*(a2*x(3) + d_alpha - R2*z2hat - k3*z3);

% System dynamics
sys(1) = -a1*x(1) + b1*Uv;
sys(2) = x(3);
sys(3) = -a2*x(3) + b2*Up;
% end mdlDerivatives

%
%=============================================================================
% mdlOutputs
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

% Physical paras
M = 200; l = 0.3;
Jv = 10; Jw = 0.005; 
r = 0.1; k = 6; c = 0.05;
a1 = 2*c / (M*r^2 + 2*Jw); b1 = k*r / (M*r^2 + 2*Jw);
a2 = 2*c*l*l / (Jv*r^2 + 2*Jw*l^2); b2 = r*l*k / (Jv*r^2 + 2*Jw*l^2);

% Control objective
x1d = 3*sin(t); dx1d = 3*cos(t);
x2d = 0.2*sin(t); dx2d = 0.2*cos(t); ddx2d = -0.2*sin(t);

% Control paras
k1 = 1; k2 = 1; k3 = 1;
% rho10 = 6; rho1oo = 1; m1 = 10;
% rho20 = 4; rho2oo = 1; m2 = 1;
rho10 = 6; rho1oo = 1; m1 = 1;
rho20 = 4; rho2oo = 1; m2 = 1;

% Control inputs
z1 = x(1) - x1d;
rho1 = (rho10-rho1oo)*exp(-m1*t)+rho1oo;
drho1 = -m1*(rho10-rho1oo)*exp(-m1*t);
s1 = z1/rho1;
z1hat = 1/2*log((s1+1)/(1-s1));
Uv = (1/b1)*(a1*x(1) + dx1d + z1*drho1/rho1 - k1*z1hat);

z2 = x(2) - x2d;
rho2 = (rho20-rho2oo)*exp(-m2*t)+rho2oo;
drho2 = -m2*(rho20-rho2oo)*exp(-m2*t);
ddrho2 = m2^2*(rho20-rho2oo)*exp(-m2*t);
s2 = z2/rho2;
z2hat = 1/2*log((s2+1)/(1-s2));
R2 = 1/rho2/(1-s2^2);
alpha = dx2d - z2*drho2/rho2 - k2*z2hat;
dz2 = x(3) - dx2d;
dz2hat = R2*(dz2-z2*drho2/rho2);
d_alpha = ddx2d - dz2*drho2/rho2 - z2*ddrho2/rho2 + z2*drho2^2/rho2/rho2  - k2*dz2hat;

z3 = x(3) - alpha;
Up = (1/b2)*(a2*x(3) + d_alpha - R2*z2hat - k3*z3);

% System outputs
sys(1) = z1;
sys(2) = z2;
sys(3) = rho1;
sys(4) = -rho1;
sys(5) = rho2;
sys(6) = -rho2;
sys(7) = x(1);
sys(8) = x(2);
sys(9) = x(3);
sys(10) = z1;
sys(11) = z2;
sys(12) = z3;
sys(13) = x1d;
sys(14) = x2d;
% end mdlOutputs
