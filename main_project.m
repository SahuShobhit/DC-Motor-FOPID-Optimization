
clc;
clear;
close all;

disp('DC Motor FOPID Project Started');
disp('MATLAB environment is ready.');

%% Step 1: DC Motor Parameters

Ra = 2;       % Armature resistance (ohm)
La = 0.5;     % Armature inductance (H)
J  = 0.02;    % Rotor inertia (kg.m^2)
B  = 0.2;     % Viscous friction coefficient
Kt = 0.1;     % Motor torque constant
Ke = 0.1;     % Back EMF constant

%% Step 2: Transfer Function

num = Kt;
den = [La*J, (La*B + Ra*J), (Ra*B + Kt*Ke)];

G = tf(num, den);

%% Step 3: Display Motor Model

disp('DC Motor Transfer Function:');
G

disp('Poles of the motor:');
pole(G);
disp(pole(G))
disp('Is the open-loop motor stable?');
isstable(G)
%% Step 4: Open-Loop Motor Response

figure;
step(G);
grid on;

title('Open-Loop DC Motor Response');
xlabel('Time (seconds)');
ylabel('Speed Response (rad/s per unit voltage)');


%% Step 5: PID Controller Design

Kp = 10;
Ki = 20;
Kd = 0.5;

C_PID = pid(Kp, Ki, Kd);

T_PID = feedback(C_PID * G, 1);

%% Step 6: Compare Open-Loop and PID Response

figure;
step(G, T_PID);
grid on;

legend('Open-Loop Motor', 'With PID Controller', ...
    'Location', 'best');

title('DC Motor Response: Open-Loop vs PID');
xlabel('Time (seconds)');
ylabel('Speed Response');


%% Step 7: PID Performance Analysis

info_PID = stepinfo(T_PID);

disp('PID Controller Performance:');
disp(info_PID);

final_value = dcgain(T_PID);
steady_state_error = abs(1 - final_value);

disp('Steady-State Error for Unit Reference:');
disp(steady_state_error);


%% Step 8: Fractional-Order PID Controller

lambda = 0.8;
mu = 0.8;

wb = 0.01;
wh = 100;
N = 5;

% Approximate fractional operators
S_lambda = oustaloup(lambda, wb, wh, N);
S_mu = oustaloup(mu, wb, wh, N);

I_lambda = 1 / S_lambda;
D_mu = S_mu;

% Construct FOPID controller
C_FOPID = Kp + Ki*I_lambda + Kd*D_mu;

% Closed-loop system
T_FOPID = feedback(C_FOPID * G, 1);

%% Step 9: Compare PID and FOPID

figure;
step(T_PID, T_FOPID, 10);
grid on;

legend('Conventional PID', 'FOPID', 'Location', 'best');
title('PID vs FOPID Speed Response');
xlabel('Time (seconds)');
ylabel('Speed Response');


%% Step 10: Quantitative Performance Comparison

info_PID = stepinfo(T_PID);
info_FOPID = stepinfo(T_FOPID);

Controller = ["PID"; "FOPID"];

RiseTime = [info_PID.RiseTime; info_FOPID.RiseTime];

SettlingTime = [info_PID.SettlingTime; ...
    info_FOPID.SettlingTime];

Overshoot = [info_PID.Overshoot; ...
    info_FOPID.Overshoot];

FinalValue = [dcgain(T_PID); dcgain(T_FOPID)];

SteadyStateError = abs(1 - FinalValue);

Results = table(Controller, RiseTime, SettlingTime, ...
    Overshoot, SteadyStateError);

disp('PID vs FOPID Performance Comparison:');
disp(Results);
