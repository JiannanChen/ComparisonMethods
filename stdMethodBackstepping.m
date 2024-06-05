function [sys,x0,str,ts,simStateCompliance] = stdMethodBackstepping(t,x,u,flag)
% General MATLAB S-Function Template

% General structure of an S-function.
switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=[];
  case 9,
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
sizes.NumOutputs     = 10;
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

% Control inputs
z1 = x(1) - x1d;
Uv = (1/b1)*(a1*x(1) + dx1d - k1*z1);

z2 = x(2) - x2d;
alpha = dx2d - k2*z2;
dz2 = x(3) - dx2d;
d_alpha = ddx2d - k2*dz2;

z3 = x(3) - alpha;
Up = (1/b2)*(a2*x(3) + d_alpha - z2 - k3*z3);

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

% Control inputs
z1 = x(1) - x1d;
Uv = (1/b1)*(a1*x(1) + dx1d - k1*z1);

z2 = x(2) - x2d;
alpha = dx2d - k2*z2;
dz2 = x(3) - dx2d;
d_alpha = ddx2d - k2*dz2;

z3 = x(3) - alpha;
Up = (1/b2)*(a2*x(3) + d_alpha - z2 - k3*z3);

% System outputs
sys(1) = 0;
sys(2) = 0;
sys(3) = x(1);
sys(4) = x(2);
sys(5) = x(3);
sys(6) = z1;
sys(7) = z2;
sys(8) = z3;
sys(9) = x1d;
sys(10) = x2d;
% end mdlOutputs
