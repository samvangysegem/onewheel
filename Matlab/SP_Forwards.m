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

%% LQR Control with state tracking (infinite horizon)
% DT System
dt_sys = c2d(ct_sys,Ts);
[Ad, Bd, Cd, Dd, Ts_d] = ssdata(dt_sys);
% Desired state
ksi = [1;0;0;0];
% Cost for state and input
Q = diag([20 5 1 1]);
R = 0.30;
% Constant disturbance
G = (Ad - eye(4))*ksi; % Last vector is desired state
% Ricatti for M
[M,K,] = idare(Ad,Bd,Q,R,[],[]);
% Disturbance vector
r = mldivide(eye(4)-transpose(Ad-(Bd/(R+transpose(Bd)*M*Bd))*transpose(Bd)*M*Ad), transpose(Ad-(Bd/(R+transpose(Bd)*M*Bd))*transpose(Bd)*M*Ad)*M*G);

%% System simulation
% Simulation time
Tend = 10;

% Initial conditions
x = [0.2;-0.4;-pi/36;0];
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

%% Animation
% 
% % Show animation
% set(gcf, 'Position',  [100, 100, 1500, 600]); % Set size of figure window
% shg; % Show graph
% for i=1:length(t_res)
%     plot([x_res(i,1),x_res(i,1)+L*sin(x_res(i,3))],[R_w,R_w+L*cos(x_res(i,3))],"ko-"); hold on;
%     plot([x_res(i,1),x_res(i,1)+R_w*sin(x_res(i,1)/R_w)],[R_w,R_w+R_w*cos(x_res(i,1)/R_w)],"k-");
%     th = 0:pi/20:2*pi;
%     plot(0.04 * cos(th) + x_res(i,1), 0.04 * sin(th) + R_w,"k-");
%     yline(0); hold off;
%     % Set axis parameters
%     xlim([-0.15 1.15]);
%     ylim([-0.1 0.3]);
%     pbaspect([3.25 1 1]);
%     % axis equal;
%     pause(0.0001);
% end

%% Initialize video

myVideo = VideoWriter('OneDuine_Mk1','MPEG-4'); %open video file
myVideo.FrameRate = 100;  %can adjust this, 5 - 10 works well for me
open(myVideo)
set(gcf, 'Position',  [100, 100, 1500, 1000]);
for i=1:length(t_res)
    % First subplot => input
    subplot(2,1,1);
    % Plot corresponding input
    plot(t_res(1:i), u_res(1:i),'-r','Linewidth',2); hold on;
    yline(12);
    yline(-12); hold off;
    % Set axis parameters
    xlim([0 t_res(end)]);
    ylim([-15 15]);
    pbaspect([3.25 1 1]);
    % Additional information
    title('Input magnitude');
    xlabel('Time [s]');
    ylabel('Voltage [V]'); 
    % Second subplot => animation
    subplot(2,1,2); cla;
    plot([x_res(i,1),x_res(i,1)+L*sin(x_res(i,3))],[R_w,R_w+L*cos(x_res(i,3))],"ko-"); hold on;
    plot([x_res(i,1),x_res(i,1)+R_w*sin(x_res(i,1)/R_w)],[R_w,R_w+R_w*cos(x_res(i,1)/R_w)],"k-");
    th = 0:pi/20:2*pi;
    plot(0.04 * cos(th) + x_res(i,1), 0.04 * sin(th) + R_w,"k-");
    yline(0); hold off;
    % Set axis parameters
    xlim([-0.15 1.15]);
    ylim([-0.1 0.3]);
    pbaspect([3.25 1 1]);
    % axis equal;
    pause(0.01);
    frame = getframe(1);
    writeVideo(myVideo,frame);
end

close(myVideo)