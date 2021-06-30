%% Clear all
clc;
clear all;

%% Initialize values
% Onewheel
M = 1000*10^(-3); %[kg]
L = 76*10^(-3); %[m]
Iyy_g = 2358443*10^(-9); %[kg*m^2] = Lzz in Solidworks
Ixx_g = 1284903*10^(-9); %[kg*m^2] = Lxx in Solidworks

m_w = 54*10^(-3); %[kg]
R_w = 0.04; %[m]

% In this case, I_w represents the moment of inertia of the flywheel
m_bn = 15.9*10^(-3); %[kg]
I_w = 12 * m_bn * (40*10^(-3))^2; %[kg*m^2]

% Motor
R = 38; %[Ohm]
K_phi = 0.158; %[Nm/A]
K_t = 0.158; %[Nm/A]

% General
g = 9.81;

%% State Space Model (Side Movement) - Continuous Time
% State:
% [phi] -> Robot angle (sideways)
% [dphi]
% [alpha] -> Flywheel angle
% [dalpha]
% Init

A = zeros(4,4);
B = zeros(4,1);
C = zeros(2,4);
D = zeros(2,1);

A(1,2) = 1; % Link dphi to dphi
A(3,4) = 1; % Link dalpha to dalpha
A(2,1) = M*g*(L + R_w)*1/(Ixx_g + M*(L + R_w)^2);
A(2,4) = -K_phi*K_t*1/(Ixx_g + M*(L + R_w)^2)*1/R;
A(4,4) = K_t*K_phi*1/I_w*1/R;

B(2,1) = K_t*1/R*1/(Ixx_g + M*(L + R_w)^2);
B(4,1) = -K_t*1/I_w*1/R;

C(1,1) = 1; % phi is measured (IMU)
C(2,3) = 1; % alpha is measured (motor encoder)

ct_sys = ss(A,B,C,D);

%% Animation
%[K,S,e] = lqr(ct_sys,diag([1 1 0.05 0.05]),1); % Q only attributes small weight to rotation of wheel (try to limit speed)
%ct_sys_cont = ss(A-B*K,B,C,D);
%
%x0 = [pi/36;0;0;0];
%[y,t,x] = initial(ct_sys_cont,x0,5);
%
% Custom input
%t_ss = linspace(0, 10, 1000);
%u_ss = sin(3.1415*t_ss);
%[y,t,x] = lsim(ct_sys_cont, u_ss, t_ss);
%
% Show animation
%shg;
%for i=1:length(t)
%    plot([0,-L*sin(x(i,1))],[0,L*cos(x(i,1))],"ko-"); hold on;
%    plot([-L*sin(x(i,1)),-L*sin(x(i,1))-0.04*sin(x(i,3))],[L*cos(x(i,1)),L*cos(x(i,1))+0.04*cos(x(i,3))],"ko-"); % circle representing flywheel
%    th = 0:pi/50:2*pi;
%    plot(0.04 * cos(th) - L*sin(x(i,1)), 0.04 * sin(th) + L*cos(x(i,1))); hold off;
%    axis([-0.15 0.15 -0.1 0.2]);
%    pause(0.001);
%end

%% LQR Control with state tracking (infinite horizon)
% Samplefrequency
Ts = 0.1;

% DT System
dt_sys = c2d(ct_sys, Ts);
[Ad, Bd, Cd, Dd, Ts_d] = ssdata(dt_sys);

% Desired state
ksi = [0;0;0;0];

% Cost for state and input
Q = diag([1 1 0.05 0.05]); % Q only attributes small weight to rotation of wheel (try to limit speed)
R = 1;

% Constant disturbance
G = (Ad - eye(4))*ksi; % Last vector is desired state

% Ricatti for M
[M,K,] = idare(Ad,Bd,Q,R,[],[]);

% Disturbance vector
r = mldivide(eye(4)-transpose(Ad-(Bd/(R+transpose(Bd)*M*Bd))*transpose(Bd)*M*Ad), transpose(Ad-(Bd/(R+transpose(Bd)*M*Bd))*transpose(Bd)*M*Ad)*M*G);

%% System simulation 
% ! Still a problem with the simulation itself... Feedback isn't working
% out for one reason or another so check formulas...

% Simulation time
Tend = 4;

% Initial conditions
x = [pi/90;0;0;0];
t = 0:Ts:(Tend-Ts);

% Simulation system
Tsim = 0.01;
sim_sys = c2d(ct_sys, Tsim);
[Asim, Bsim, Csim, Dsim] = ssdata(sim_sys);

% Results
t_res = 0:Tsim:Tend;
x_res = zeros(length(t_res), 4);
x_res(1,:) = transpose(x);
u_res = zeros(length(t_res), 1);

% Simulation
index = 2;
u_index = 1;
for i=t
    % Determine input (Feedback matrices included)
    u = -(R+transpose(Bd)*M*Bd)\transpose(Bd)*(M*Ad*(x-ksi)+M*G+r);
    % Input boundaries
    if abs(u)>12
        u = 12 * sign(u);
    end
    % Calculate system response (ZOH input)
    for j=i+Tsim:Tsim:i+Ts
        % System SS
        % u = awgn(x(3),30,'measured'); % Add white gaussian noise to input
        x = Asim*x+Bsim*u;
        % x(3) = awgn(x(3),30,'measured'); % Add white gaussian noise to angle
        y = Csim*x+Dsim*u;
        % Store data
        x_res(index,:) = transpose(x);
        index = index + 1;
        u_res(u_index) = u;
        u_index = u_index + 1;
    end
end

% Show animation
shg;
for i=1:length(t_res)
    plot([0,-(L+R_w)*sin(x_res(i,1))],[0,(L+R_w)*cos(x_res(i,1))],"ko-"); hold on; % [X coordinates], [Y coordinates]
    plot([-(L+R_w)*sin(x_res(i,1)),-(L+R_w)*sin(x_res(i,1))-0.045*sin(x_res(i,3))],[(L+R_w)*cos(x_res(i,1)),(L+R_w)*cos(x_res(i,1))+0.045*cos(x_res(i,3))],"ko-"); % circle representing flywheel
    th = 0:pi/50:2*pi;
    plot(0.045 * cos(th) - (L+R_w)*sin(x_res(i,1)), 0.045 * sin(th) + (L+R_w)*cos(x_res(i,1))); hold off;
    axis([-0.15 0.15 -0.1 0.2]);
    pause(0.001);
end