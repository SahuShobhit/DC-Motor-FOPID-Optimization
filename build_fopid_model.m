
%% Build FOPID Simulink Model

clc;

% Motor parameters
Ra = 2;
La = 0.5;
J  = 0.02;
B  = 0.2;
Kt = 0.1;
Ke = 0.1;

G = tf(Kt, [La*J, La*B + Ra*J, Ra*B + Kt*Ke]);


% Load PSO-optimized FOPID parameters
R = load('fopid_optimization_results.mat');

Kp = R.Kp_opt;
Ki = R.Ki_opt;
Kd = R.Kd_opt;
lambda = R.lambda_opt;
mu = R.mu_opt;

% Oustaloup approximations
H_I = oustaloup(-lambda, 0.01, 100, 5);
H_D = oustaloup(mu, 0.01, 100, 5);


% Convert fractional operators to state-space form
SI = ss(H_I);
SD = ss(H_D);

A_I = SI.A; B_I = SI.B; C_I = SI.C; D_I = SI.D;
A_D = SD.A; B_D = SD.B; C_D = SD.C; D_D = SD.D;

% Create a separate Simulink model
mdl = 'DC_Motor_FOPID_Auto';

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end

new_system(mdl);
open_system(mdl);

% Add blocks
add_block('simulink/Sources/Step', [mdl '/Reference'], ...
    'Position', [30 160 60 190], ...
    'Time', '0', 'Before', '0', 'After', '1');

add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Inputs', '+-', 'Position', [110 160 130 200]);

add_block('simulink/Math Operations/Gain', [mdl '/Kp'], ...
    'Gain', 'Kp', 'Position', [180 70 230 100]);

add_block('simulink/Continuous/State-Space', [mdl '/Fractional_Integrator'], ...
    'A', 'A_I', 'B', 'B_I', 'C', 'C_I', 'D', 'D_I', ...
    'Position', [180 150 290 190]);

add_block('simulink/Math Operations/Gain', [mdl '/Ki'], ...
    'Gain', 'Ki', 'Position', [320 150 370 190]);

add_block('simulink/Continuous/State-Space', [mdl '/Fractional_Derivative'], ...
    'A', 'A_D', 'B', 'B_D', 'C', 'C_D', 'D', 'D_D', ...
    'Position', [180 250 290 290]);

add_block('simulink/Math Operations/Gain', [mdl '/Kd'], ...
    'Gain', 'Kd', 'Position', [320 250 370 290]);

add_block('simulink/Math Operations/Sum', [mdl '/Controller_Sum'], ...
    'Inputs', '+++', 'Position', [420 150 445 225]);

add_block('simulink/Continuous/Transfer Fcn', [mdl '/DC_Motor'], ...
    'Numerator', 'Kt', ...
    'Denominator', '[La*J La*B+Ra*J Ra*B+Kt*Ke]', ...
    'Position', [500 160 600 200]);

add_block('simulink/Sinks/Scope', [mdl '/Speed_Scope'], ...
    'Position', [680 160 710 190]);

% Connect reference and feedback loop
add_line(mdl, 'Reference/1', 'Error/1');
add_line(mdl, 'Error/1', 'Kp/1');
add_line(mdl, 'Error/1', 'Fractional_Integrator/1');
add_line(mdl, 'Error/1', 'Fractional_Derivative/1');

% Sum controller branches
add_line(mdl, 'Kp/1', 'Controller_Sum/1');
add_line(mdl, 'Fractional_Integrator/1', 'Ki/1');
add_line(mdl, 'Ki/1', 'Controller_Sum/2');
add_line(mdl, 'Fractional_Derivative/1', 'Kd/1');
add_line(mdl, 'Kd/1', 'Controller_Sum/3');

% Motor and feedback
add_line(mdl, 'Controller_Sum/1', 'DC_Motor/1');
add_line(mdl, 'DC_Motor/1', 'Speed_Scope/1');
add_line(mdl, 'DC_Motor/1', 'Error/2', 'autorouting', 'on');

set_param(mdl, 'StopTime', '10');

save_system(mdl);
open_system(mdl);

disp('FOPID Simulink model created.');
