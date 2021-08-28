function [X1] = SolveFODERK4(h, u, X0)
% This function solves the forward nonlinear differential equations of the
% onewheel using a fourth order Runge-Kutta method.
% The function arguments are the initial conditions, input and stepsize h
% The function output is the new calculated state
% Define model constants
M = 1000*10^(-3); %[kg]
L = 76*10^(-3); %[m]
Iyy_g = 2358443*10^(-9); %[kg*m^2] = Lzz in Solidworks
Ixx_g = 1284903*10^(-9); %[kg*m^2] = Lxx in Solidworks
m_w = 54*10^(-3); %[kg]
R_w = 0.04; %[m] 
I_w = (1/2)*m_w*(0.02^2+0.04^2); %[kg*m^2]
R = 38; %[Ohm]
K_phi = 0.158; %[Nm/A]
K_t = 0.158; %[Nm/A]
g = 9.81;

% Forward equations
% dx/dt = w
% dw/dt = d^2x/dt^2 = ...
% dtheta/dt = phi
% dphi/dt = d^2theta/dt^2 = ...
function dxdt = PendulumDxDt(t, x, w, theta, phi)
dxdt = w;
end

function dwdt = PendulumDwDt(t, x, w, theta, phi)
dwdt = (-L*sin(theta)*M*R*R_w^2*(L^2*M + Iyy_g)*phi^2 + K_phi*K_t*(L^2*M + Iyy_g)*w - (-L^2*cos(theta)*sin(theta)*M^2*R*g*R_w + u*K_t*(L^2*M + Iyy_g))*R_w)*1/(L^2*M^2*cos(theta)^2*R_w^2 - ((M + m_w)*R_w^2 + I_w)*(L^2*M + Iyy_g))*1/R;
end

function dthetadt = PendulumDthetaDt(t, x, w, theta, phi)
dthetadt = phi;
end

function dphidt = PendulumDphiDt(t, x, w, theta, phi)
dphidt = M*(L*M*cos(theta)*sin(theta)*phi^2*R*R_w^2 - K_phi*K_t*cos(theta)*w + K_t*cos(theta)*u*R_w - g*((M + m_w)*R_w^2 + I_w)*R*sin(theta))*L*1/(L^2*M^2*cos(theta)^2*R_w^2 - ((M + m_w)*R_w^2 + I_w)*(L^2*M + Iyy_g))*1/R;
end

% Rename initial conditions
x0 = X0(1);
w0 = X0(2);
theta0 = X0(3);
phi0 = X0(4);

% Solve equations using a fourth order Runge-Kutta method
Kx1 = PendulumDxDt(0, x0, w0, theta0, phi0);
Kw1 = PendulumDwDt(0, x0, w0, theta0, phi0);
Ktheta1 = PendulumDthetaDt(0, x0, w0, theta0, phi0);
Kphi1 = PendulumDphiDt(0, x0, w0, theta0, phi0);

Kx2 = PendulumDxDt(h/2, x0+(h/2)*Kx1, w0+(h/2)*Kw1, theta0+(h/2)*Ktheta1, phi0+(h/2)*Kphi1);
Kw2 = PendulumDwDt(h/2, x0+(h/2)*Kx1, w0+(h/2)*Kw1, theta0+(h/2)*Ktheta1, phi0+(h/2)*Kphi1);
Ktheta2 = PendulumDthetaDt(h/2, x0+(h/2)*Kx1, w0+(h/2)*Kw1, theta0+(h/2)*Ktheta1, phi0+(h/2)*Kphi1);
Kphi2 = PendulumDphiDt(h/2, x0+(h/2)*Kx1, w0+(h/2)*Kw1, theta0+(h/2)*Ktheta1, phi0+(h/2)*Kphi1);

Kx3 = PendulumDxDt(h/2, x0+(h/2)*Kx2, w0+(h/2)*Kw2, theta0+(h/2)*Ktheta2, phi0+(h/2)*Kphi2);
Kw3 = PendulumDwDt(h/2, x0+(h/2)*Kx2, w0+(h/2)*Kw2, theta0+(h/2)*Ktheta2, phi0+(h/2)*Kphi2);
Ktheta3 = PendulumDthetaDt(h/2, x0+(h/2)*Kx2, w0+(h/2)*Kw2, theta0+(h/2)*Ktheta2, phi0+(h/2)*Kphi2);
Kphi3 = PendulumDphiDt(h/2, x0+(h/2)*Kx2, w0+(h/2)*Kw2, theta0+(h/2)*Ktheta2, phi0+(h/2)*Kphi2);

Kx4 = PendulumDxDt(h, x0+h*Kx3, w0+h*Kw3, theta0+h*Ktheta3, phi0+h*Kphi3);
Kw4 = PendulumDwDt(h, x0+h*Kx3, w0+h*Kw3, theta0+h*Ktheta3, phi0+h*Kphi3);
Ktheta4 = PendulumDthetaDt(h, x0+h*Kx3, w0+h*Kw3, theta0+h*Ktheta3, phi0+h*Kphi3);
Kphi4 = PendulumDphiDt(h, x0+h*Kx3, w0+h*Kw3, theta0+h*Ktheta3, phi0+h*Kphi3);

x = x0 + (1/6)*(Kx1 + 2*Kx2 + 2*Kx3 + Kx4)*h;
dx = w0 + (1/6)*(Kw1 + 2*Kw2 + 2*Kw3 + Kw4)*h;
theta = theta0 + (1/6)*(Ktheta1 + 2*Ktheta2 + 2*Ktheta3 + Ktheta4)*h;
dtheta = phi0 + (1/6)*(Kphi1 + 2*Kphi2 + 2*Kphi3 + Kphi4)*h;

X1 = [x;dx;theta;dtheta];
end

