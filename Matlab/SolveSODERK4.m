function [X1] = SolveSODERK4(h, u, X0)
% This function solves the forward nonlinear differential equations of the
% onewheel using a fourth order Runge-Kutta method.
% The function arguments are the initial conditions, input and stepsize h
% The function output is the new calculated state
% Define model constants
M = 1000*10^(-3); %[kg]
L = 82*10^(-3); %[m]
Iyy_g = 2311756*10^(-9); %[kg*m^2] = Lzz in Solidworks
Ixx_g = 1437327*10^(-9); %[kg*m^2] = Lxx in Solidworks
m_w = 54*10^(-3); %[kg]
R_w = 0.04; %[m] 
m_bn = 15.9*10^(-3); %[kg]
I_w = 12 * m_bn * (40*10^(-3))^2;
R = 38; %[Ohm]
K_phi = 0.158; %[Nm/A]
K_t = 0.158; %[Nm/A]
g = 9.81;

% Forward equations
% dphi/dt = x
% dx/dt = d^2phi/dt^2 = ...
% dalpha/dt = y
% dy/dt = d^2alpha/dt^2 = ...
function dphidt = PendulumDphiDt(t, phi, x, alpha, y)
dphidt = x;
end

function dxdt = PendulumDxDt(t, phi, x, alpha, y)
dxdt = (-K_phi*K_t*y + M*R*g*(L + R_w)*sin(phi) + K_t*u)*1/R*1/(Ixx_g + M*(L + R_w)^2);
end

function dalphadt = PendulumDalphaDt(t, phi, x, alpha, y)
dalphadt = y;
end

function dydt = PendulumDyDt(t, phi, x, alpha, y)
dydt = K_t*(K_phi*y - u)*1/I_w*1/R;
end

% Rename initial conditions
phi0 = X0(1);
x0 = X0(2);
alpha0 = X0(3);
y0 = X0(4);

% Solve equations using a fourth order Runge-Kutta method
Kphi1 = PendulumDphiDt(0, phi0, x0, alpha0, y0);
Kx1 = PendulumDxDt(0, phi0, x0, alpha0, y0);
Kalpha1 = PendulumDalphaDt(0, phi0, x0, alpha0, y0);
Ky1 = PendulumDyDt(0, phi0, x0, alpha0, y0);

Kphi2 = PendulumDphiDt(h/2, phi0+(h/2)*Kphi1, x0+(h/2)*Kx1, alpha0+(h/2)*Kalpha1, y0+(h/2)*Ky1);
Kx2 = PendulumDxDt(h/2, phi0+(h/2)*Kphi1, x0+(h/2)*Kx1, alpha0+(h/2)*Kalpha1, y0+(h/2)*Ky1);
Kalpha2 = PendulumDalphaDt(h/2, phi0+(h/2)*Kphi1, x0+(h/2)*Kx1, alpha0+(h/2)*Kalpha1, y0+(h/2)*Ky1);
Ky2 = PendulumDyDt(h/2, phi0+(h/2)*Kphi1, x0+(h/2)*Kx1, alpha0+(h/2)*Kalpha1, y0+(h/2)*Ky1);

Kphi3 = PendulumDphiDt(h/2, phi0+(h/2)*Kphi2, x0+(h/2)*Kx2, alpha0+(h/2)*Kalpha2, y0+(h/2)*Ky2);
Kx3 = PendulumDxDt(h/2, phi0+(h/2)*Kphi2, x0+(h/2)*Kx2, alpha0+(h/2)*Kalpha2, y0+(h/2)*Ky2);
Kalpha3 = PendulumDalphaDt(h/2, phi0+(h/2)*Kphi2, x0+(h/2)*Kx2, alpha0+(h/2)*Kalpha2, y0+(h/2)*Ky2);
Ky3 = PendulumDyDt(h/2, phi0+(h/2)*Kphi2, x0+(h/2)*Kx2, alpha0+(h/2)*Kalpha2, y0+(h/2)*Ky2);

Kphi4 = PendulumDphiDt(h, phi0+h*Kphi3, x0+h*Kx3, alpha0+h*Kalpha3, y0+h*Ky3);
Kx4 = PendulumDxDt(h, phi0+h*Kphi3, x0+h*Kx3, alpha0+h*Kalpha3, y0+h*Ky3);
Kalpha4 = PendulumDalphaDt(h, phi0+h*Kphi3, x0+h*Kx3, alpha0+h*Kalpha3, y0+h*Ky3);
Ky4 = PendulumDyDt(h, phi0+h*Kphi3, x0+h*Kx3, alpha0+h*Kalpha3, y0+h*Ky3);

phi = phi0 + (1/6)*(Kphi1 + 2*Kphi2 + 2*Kphi3 + Kphi4)*h;
dphi = x0 + (1/6)*(Kx1 + 2*Kx2 + 2*Kx3 + Kx4)*h;
alpha = alpha0 + (1/6)*(Kalpha1 + 2*Kalpha2 + 2*Kalpha3 + Kalpha4)*h;
dalpha = y0 + (1/6)*(Ky1 + 2*Ky2 + 2*Ky3 + Ky4)*h;

X1 = [phi;dphi;alpha;dalpha];
end
