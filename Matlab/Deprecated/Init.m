%% Clear all
clc;
clear all;

%% Initialize values
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
% Init
A = zeros(4,4);
B = zeros(4,1);

A(1,2) = 1;
A(3,4) = 1;
A(2,2) = -K_phi*K_t*(L^2*M + Iyy_g)*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
A(2,3) = -R_w^2*L^2*M^2*g*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g));
A(4,2) = L*K_phi*K_t*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
A(4,3) = L*((M + m_w)*R_w^2 + I_w)*g*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g));

B(2,1) = R_w*K_t*(L^2*M + Iyy_g)*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;
B(4,1) = -L*K_t*R_w*M*1/(((L^2*m_w + Iyy_g)*M + Iyy_g*m_w)*R_w^2 + I_w*(L^2*M + Iyy_g))*1/R;


