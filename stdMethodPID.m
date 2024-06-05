function [sys,x0,str,ts,simStateCompliance] = stdMethodPID(t,x,u,flag)
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

sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 11;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% initial conditions
x0  = [5.5, 0.1, 0, 5.5, 0.1];

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
x2d = 0.2*sin(t); dx2d = 0.2*cos(t);

% Control paras
Kp1 = -20; Ki1 = -10; Kd1 = -10;
Kp2 = -20; Ki2 = -10; Kd2 = -10;

% Control inputs
z1 = x(1) - x1d;
iz1 = x(4);
dz1 = u(1) - dx1d;                  % u(1) is the time derivative of x(1)
Uv = Kp1*z1 + Ki1*iz1 + Kd1*dz1;    % i and d are the short of integral and 
                                    % derivative 

z2 = x(2) - x2d;
iz2 = x(5);
dz2 = x(3) - dx2d;
Up = Kp2*z2 + Ki2*iz2 + Kd2*dz2;

% System dynamics
sys(1) = -a1*x(1) + b1*Uv;
sys(2) = x(3);
sys(3) = -a2*x(3) + b2*Up;
% Auxiliary systems
sys(4) = x(1) - x1d;  % z1 and z2
sys(5) = x(2) - x2d;  % the integrals of z1 and z2 are x(4) and x(5)
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
x2d = 0.2*sin(t); dx2d = 0.2*cos(t);

% Control paras
Kp1 = -20; Ki1 = -10; Kd1 = -10;
Kp2 = -20; Ki2 = -10; Kd2 = -10;

% Control inputs
z1 = x(1) - x1d;
iz1 = x(4);
dz1 = u(1) - dx1d;                  % u(1) is the time derivative of x(1)
Uv = Kp1*z1 + Ki1*iz1 + Kd1*dz1;    % i and d are the short of integral and 
                                    % derivative 

z2 = x(2) - x2d;
iz2 = x(5);
dz2 = x(3) - dx2d;
Up = Kp2*z2 + Ki2*iz2 + Kd2*dz2;

% Others
Oth1 = -a1*x(1) + b1*Uv;

% System Outputs
sys(1) = 0;
sys(2) = 0;
sys(3) = Oth1;
sys(4) = x(1);
sys(5) = x(2);
sys(6) = x(3);
sys(7) = z1;
sys(8) = z2;
sys(9) = 0;
sys(10) = x1d;
sys(11) = x2d;
% end mdlOutputs
