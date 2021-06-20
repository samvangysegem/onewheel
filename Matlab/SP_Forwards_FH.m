%% Clear all
clc;
clear all;

%% Initialize values
% Samplefrequency
Ts = 0.1;

% Onewheel
M = 750*10^(-3); %[kg]
L = 72*10^(-3); %[m]
Iyy_g = 2049081*10^(-9); %[kg*m^2]
Ixx_g = 810407*10^(-9); %[kg*m^2]

m_w = 54*10^(-3); %[kg]
R_w = 0.04; %[m] 
I_w = (1/2)*m_w*(0.02^2+0.04^2); %[kg*m^2]

% Motor
R = 38; %[Ohm]
K_phi = 0.158; %[Nm/A]
K_t = 0.158; %[Nm/A]

% General
g = 9.81;

%% State Space Model (F/B Movement) - Continuous Time
% State:
% [x] -> Robot distance travelled
% [dx]
% [theta] -> Robot angle
% [dtheta]
% Init

A = zeros(4,4);
B = zeros(4,1);
C = zeros(2,4);
D = zeros(2,1);

A(1,2) = 1;
A(3,4) = 1;
A(2,2) = -K_phi*K_t*(L^2*M + Iyy_g)*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
A(2,3) = -R_w^2*L^2*M^2*g*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g));
A(4,2) = L*K_phi*K_t*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
A(4,3) = L*((M + m_w)*R_w^2 + I_w)*g*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g));

B(2,1) = R_w*K_t*(L^2*M + Iyy_g)*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
B(4,1) = -L*K_t*R_w*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;

C(1,1) = 1; % x is measured (motor encoder)
C(2,3) = 1; % theta is measured (IMU)

% CT System
ct_sys = ss(A,B,C,D);

%% LQR Control
% [K,S,e] = lqr(ct_sys,diag([0.01 20 1 20]),1);
% ct_sys_cont = ss(A-B*K,B,C,D);

%% LQR Control with state tracking (finite horizon)
% DT System
dt_sys = c2d(ct_sys,Ts);
[Ad, Bd, Cd, Dd, Ts_d] = ssdata(dt_sys);
% Desired state
ksi = [0.5;0;0;0];
Tstate = 5-Ts:-Ts:0;
% Solve Ricatti for M
Q = diag([20 0 1 5]);
R = 0.1;
% Constant disturbance
G = (Ad - eye(4))*ksi; % Last vector is desired state
% Ricatti equation solver
M = zeros(4, 4, length(Tstate)+1);
r = zeros(4, 1, length(Tstate)+1);
M(:,:,length(Tstate)+1) = Q;
r(:,:,length(Tstate)+1) = [0;0;0;0];
for t=length(Tstate):-1:1
    r(:,:,t) = transpose(Ad-(Bd/(R+transpose(Bd)*M(:,:,t+1)*Bd))*transpose(Bd)*M(:,:,t+1)*Ad)*(r(:,:,t+1)+M(:,:,t+1)*G);
    M(:,:,t) = transpose(Ad)*M(:,:,t+1)*Ad-transpose(Ad)*M(:,:,t+1)*(Bd/(R+transpose(Bd)*M(:,:,t+1)*Bd))*transpose(Bd)*M(:,:,t+1)*Ad+G;
end

%% System simulation
% Simulation time
Tend = 5;

% Initial conditions
x = [0;0;0;0];
t = 0:Ts:(Tend-1);

% Simulation system
Tsim = 0.01;
sim_sys = c2d(ct_sys, Tsim);
[Asim, Bsim, Csim, Dsim] = ssdata(sim_sys);

% Results
t_res = 0:Tsim:Tend;
x_res = zeros(length(t_res), 4);
x_res(1,:) = transpose(x);

% Simulation
k = 0;
index = 2;
for i=t
    % Determine state using input
    u = -(R+transpose(Bd)*M(:,:,k+1)*Bd)\transpose(Bd)*(M(:,:,k+1)*Ad*(x-ksi)+M(:,:,k+1)*G+r(:,:,k+1));
    k = k + 1;
    % Calculate system response (ZOH input)
    for j=i+Tsim:Tsim:i+Ts
        % System SS
        x = Asim*x+Bsim*u;
        y = Csim*x+Dsim*u;
        % Store data
        x_res(index,:) = transpose(x);
        index = index + 1;
    end
end

%% Animation

% Initial conditions only
% x0 = [0;0;0.05;0.05];
% [y,t,x] = initial(ct_sys_cont,x0);

% Step response
% opt = stepDataOptions;
% opt.StepAmplitude = -3;
% [y,t,x] = step(ct_sys_cont, 5, opt);

% Custom input
%t_ss = linspace(0, 10, 1000);
%u_ss = -4*heaviside(t_ss)-4*heaviside(t_ss-2.5)+16*heaviside(t_ss-5)-8*heaviside(t_ss-7.5);
%[y,t,x] = lsim(ct_sys_cont, u_ss, t_ss);

% Show animation
set(gcf, 'Position',  [100, 100, 1500, 500]); % Set size of figure window
shg; % Show graph
for i=1:length(t_res)
    plot([x_res(i,1),x_res(i,1)+L*sin(x_res(i,3))],[R_w,R_w+L*cos(x_res(i,3))],"ko-"); hold on;
    plot([x_res(i,1),x_res(i,1)+R_w*sin(x_res(i,1)/R_w)],[R_w,R_w+R_w*cos(x_res(i,1)/R_w)],"k-");
    th = 0:pi/20:2*pi;
    plot(0.04 * cos(th) + x_res(i,1), 0.04 * sin(th) + R_w,"k-");
    yline(0); hold off;
    xlim([-0.15 1.15]);
    ylim([-0.1 0.3]);
    pbaspect([3.25 1 1]);
    % axis equal;
    pause(0.0001);
end
