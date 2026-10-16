
%% PSO-Based FOPID Optimization
clc;
run('main_project.m');
% Check required model and toolbox
assert(exist('G','var') == 1, ...
    'Run main_project.m first to create G.');

assert(exist('particleswarm','file') == 2, ...
    'Global Optimization Toolbox is required.');

% Parameter bounds: [Kp Ki Kd lambda mu]
lb = [0.1  0.1  0.001  0.2  0.2];
ub = [100  100  5      1.5  1.5];

% PSO settings
nvars = 5;
opts = optimoptions('particleswarm', ...
    'SwarmSize', 20, ...
    'MaxIterations', 30, ...
    'Display', 'iter');

% Run optimization
obj = @(x) fopidCost(x, G);

rng(1,'twister');

[xbest, fbest, exitflag, output] = ...
    particleswarm(obj, nvars, lb, ub, opts);

% Best parameters
Kp_opt = xbest(1);
Ki_opt = xbest(2);
Kd_opt = xbest(3);
lambda_opt = xbest(4);
mu_opt = xbest(5);

disp('Optimized parameters:');
disp(table(Kp_opt, Ki_opt, Kd_opt, ...
    lambda_opt, mu_opt));

disp('Best objective value:');
disp(fbest);

disp('PSO exit information:');
disp(output.message);




%% Step 14: Build Optimized FOPID Controller

S_lambda_opt = oustaloup(lambda_opt, 0.01, 100, 5);
S_mu_opt = oustaloup(mu_opt, 0.01, 100, 5);

C_FOPID_opt = Kp_opt + Ki_opt/S_lambda_opt ...
    + Kd_opt*S_mu_opt;

T_FOPID_opt = feedback(C_FOPID_opt*G, 1);

%% Step 15: Check Stability and Response

disp('Optimized closed-loop stable?');
disp(isstable(T_FOPID_opt));

tTest = (0:0.01:20)';
yOpt = step(T_FOPID_opt, tTest);

infoOpt = stepinfo(yOpt, tTest, 1, ...
    'SettlingTimeThreshold', 0.02);

disp('Optimized FOPID Performance:');
disp(infoOpt);

disp('Final tracking error:');
essOpt = abs(1-yOpt(end));

disp('Final tracking error:');
disp(essOpt);

%% Step 16: Compare Responses

figure;
step(T_PID, T_FOPID, T_FOPID_opt, 20);
grid on;

legend('PID', 'Manual FOPID', 'PSO-FOPID', ...
    'Location', 'best');

title('PID vs Manual FOPID vs PSO-FOPID');
xlabel('Time (seconds)');
ylabel('Speed Response');


%% Step 17: Verify Optimized Controller

disp('Closed-loop stability:');
disp(isstable(T_FOPID_opt));

disp('Closed-loop poles:');
disp(pole(T_FOPID_opt));

figure;
step(T_FOPID_opt, 2);
grid on;
title('PSO-FOPID Response: First 2 Seconds');
xlabel('Time (seconds)');
ylabel('Speed Response');


%% Step 18: Inertia Robustness Test

J_values = [0.8*J, J, 1.2*J];

tRobust = (0:0.01:10)';
figure;
hold on;

for k = 1:length(J_values)
    J_test = J_values(k);

    den_test = [La*J_test, ...
        La*B + Ra*J_test, ...
        Ra*B + Kt*Ke];

    G_test = tf(Kt, den_test);

    T_test = feedback(C_FOPID_opt*G_test, 1);

    fprintf('\nJ = %.4f\n', J_test);
    fprintf('Stable = %d\n', isstable(T_test));

    y_test = step(T_test, tRobust);

    plot(tRobust, y_test, ...
        'DisplayName', sprintf('J = %.4f', J_test));
end

grid on;
legend('Location', 'best');
title('PSO-FOPID Robustness to Inertia Variation');
xlabel('Time (seconds)');
ylabel('Speed Response');
hold off;



%% Step 19: Rebuild Disturbance Transfer Functions

% Load torque to speed transfer function
den_load = [La*J, La*B + Ra*J, Ra*B + Kt*Ke];
G_load = tf([-La, -Ra], den_load);

% Rebuild the feedback denominator explicitly
D_PID = minreal(G_load / (1 + C_PID*G));
D_PSO = minreal(G_load / (1 + C_FOPID_opt*G));

disp('PID disturbance model stable?');
disp(isstable(D_PID));

disp('PSO-FOPID disturbance model stable?');
disp(isstable(D_PSO));

disp('PSO-FOPID disturbance poles:');
disp(pole(D_PSO));


%% Step 20: Diagnose Load Disturbance Model

disp('Motor poles:');
disp(pole(G));

disp('PID closed-loop stable?');
disp(isstable(T_PID));

disp('PSO-FOPID closed-loop stable?');
disp(isstable(T_FOPID_opt));

disp('Load disturbance transfer function poles:');
disp(pole(D_PSO));

disp('Load disturbance transfer function stable?');
disp(isstable(D_PSO));

%% Diagnose disturbance loop

L_PSO = C_FOPID_opt * G;

disp('Reference loop poles:');
disp(pole(feedback(L_PSO, 1)));

disp('Disturbance loop denominator poles:');
disp(pole(D_PSO));

disp('Disturbance loop DC gain:');
disp(dcgain(D_PSO));


%% Step 21: Validated Load Disturbance Response

tDist = (0:0.01:10)';

yDist_PID = step(D_PID, tDist);
yDist_PSO = step(D_PSO, tDist);

figure;
plot(tDist, yDist_PID, 'DisplayName', 'PID');
hold on;
plot(tDist, yDist_PSO, 'DisplayName', 'PSO-FOPID');
grid on;
legend('Location', 'best');
title('Load Torque Disturbance Response');
xlabel('Time (seconds)');
ylabel('Speed Deviation');
hold off;

fprintf('PID peak disturbance response: %.6f\n', ...
    max(abs(yDist_PID)));

fprintf('PSO-FOPID peak disturbance response: %.6f\n', ...
    max(abs(yDist_PSO)));


%% Step 22: Save Disturbance Results

DisturbanceController = ["PID"; "PSO-FOPID"];

PeakDeviation = [max(abs(yDist_PID)); ...
    max(abs(yDist_PSO))];

DisturbanceResults = table( ...
    DisturbanceController, PeakDeviation);

disp('Load Disturbance Comparison:');
disp(DisturbanceResults);

writetable(DisturbanceResults, ...
    'disturbance_results.csv');

save('fopid_optimization_results.mat', ...
    'xbest', 'fbest', 'Kp_opt', 'Ki_opt', ...
    'Kd_opt', 'lambda_opt', 'mu_opt', ...
    'infoOpt', 'essOpt', 'exitflag', 'output', ...
    'DisturbanceResults');


function Jcost = fopidCost(x, G)

Kp = x(1);
Ki = x(2);
Kd = x(3);
lambda = x(4);
mu = x(5);

wb = 0.01;
wh = 100;
N = 5;

Jcost = 1e6;

try
    S_lambda = oustaloup(lambda, wb, wh, N);
    S_mu = oustaloup(mu, wb, wh, N);

    C = Kp + Ki/S_lambda + Kd*S_mu;
    T = feedback(C*G, 1);

    if ~isstable(T)
        return;
    end

    % Fine time resolution for rise time and overshoot
    tFast = (0:0.01:20)';
    yFast = step(T, tFast);

    if any(~isfinite(yFast)) || max(abs(yFast)) > 10
        return;
    end

    info = stepinfo(yFast, tFast, 1, ...
        'SettlingTimeThreshold', 0.02);

    if ~isfinite(info.RiseTime) || ...
            ~isfinite(info.Overshoot)
        return;
    end

    % Longer window for independent settling validation
    tLong = (0:0.1:200)';
    yLong = step(T, tLong);

    if any(~isfinite(yLong)) || max(abs(yLong)) > 10
        return;
    end

    band = abs(yLong - 1) <= 0.02;
    lastOutside = find(~band, 1, 'last');

    if isempty(lastOutside)
        settlingTime = 0;
    elseif lastOutside == length(tLong)
        return;
    else
        settlingTime = tLong(lastOutside + 1);
    end

    ess = abs(1 - yLong(end));

    % Reject poor steady-state tracking
    if ess > 0.02
        return;
    end

    % Objective: fast rise, fast verified settling,
    % low tracking error, and limited overshoot
    Jcost = 0.5*info.RiseTime ...
        + settlingTime ...
        + 50*ess ...
        + 2*max(0, info.Overshoot - 2);

catch
    Jcost = 1e6;
end

end
