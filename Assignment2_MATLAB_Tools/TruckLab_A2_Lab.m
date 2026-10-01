clearvars;
close all;
clc;
%% Load the ref path
load("paths\trucklab_circular_path.mat")
% addpath("KinematicModel_TractorSemitrailer\");

%% Parameters
% Input
p.input.Ld = [];
p.input.V1 = .2;

% Controller setup
controller.path = path;
controller.V1 = p.input.V1;
controller.L1 = TruckParams.L1;
controller.lookahead_dist = p.input.Ld;
controller.lastFoundIndex = 1;
% for longitudinal controller (a.k.a stoping criterion)
controller.proximity_threshold = .2; % [m]

%% Simulation
tmax= 25;          % simulation time [s]
fs  = 120;          % sample frequency for data storage [Hz] =>controller rate 120 Hz
Ts = 1./fs;    

%Simulation
s = sim('Pure_Pursuit_Tactor_Lab');

%% Plot results
% Extract relevant data for plotting
t_vec = s.time.Data; x_pose = s.x1.Data; y_pose = s.y1.Data; psi_orientation = s.psi_1.Data;
x_targets = s.TargetPoints(:,1); y_targets = s.TargetPoints(:,2);

fig5 = figure(5);
theme(fig5, "light")
helpers.fitFigureWithMargin(fig5, .05);
plot(x_pose, y_pose, 'g-', 'LineWidth', 1.5, 'DisplayName', "rear axle trajectory"); 
axis equal; hold on; grid on;
plot(path.rearaxle(:,1), path.rearaxle(:,2), 'b-.s', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', "reference path");
for k = 1:length(path.rearaxle(:,1))
    text(path.rearaxle(k,1), path.rearaxle(k,2), num2str(k));
end
plot(x_targets, y_targets, 'm.', 'MarkerSize', 10, 'DisplayName', "target points");
draw_circle(path.rearaxle(end,1),path.rearaxle(end,2), controller.proximity_threshold);
xlabel('X [m]'); ylabel('Y [m]');
legend('Location','best');
title('Tractor Rear Axle Trajectory');

% Exercise 10.4, Figure 2: Plot steering angle and tractor orientation over time
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

% Exercise 10.4, Figure 3: Plot ATE and RMSE-ATE over time
% (reuse helpers.ATE_error, as in ex8_lookahead_dist_analysis.m)
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
function draw_circle(a, b, r)
    theta = linspace(0, 2*pi, 100);
    x = a + r*cos(theta);
    y = b + r*sin(theta);
    plot(x , y, 'k--', 'LineWidth' , 2);
end