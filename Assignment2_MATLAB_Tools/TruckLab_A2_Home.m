clearvars;
close all;
clc;
%% Load the ref path
load("paths\shortlab_path.mat")
% addpath("KinematicModel_TractorSemitrailer\");

%% Parameters
% Initial conditions
p.init.x10 = -3;
p.init.y10 = -2;
p.init.psi10 = deg2rad(290);
p.init.psi20 = deg2rad(290);
% Input
p.input.Ld = .9;
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
tmax= 30;          % simulation time [s]
fs  = 120;          % sample frequency for data storage [Hz] =>controller rate 120 Hz
Ts = 1./fs;    

%Simulation
s = sim('PurePursuit_Tractor_Home');

%% Animation 
skip = 50; % skiping N # of frames for animation speed
for k = 1:skip:length(s.time.Data) 
    currentX = s.x1.Data(k); currentY = s.y1.Data(k); currentPSI = s.psi_1.Data(k);

    [~, controller.lastFoundIndex] = helpers.kin_feasible_target_search(path.rearaxle, [currentX, currentY, currentPSI],...
                          controller.lookahead_dist, controller.lastFoundIndex, TruckParams.L1, TruckParams.delta_max);

    helpers.pure_pursuit_animation(path.rearaxle, [currentX, currentY, currentPSI],...
                           controller.lookahead_dist, controller.lastFoundIndex);
    title(sprintf('Time: %.2f sec | Index: %d', s.time.Data(k), controller.lastFoundIndex));
    pause(0.03);   % pacing for visibility
end


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

% Exercise 7.1: Plot steering ang, target direction and tractor orientation
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%%%%%%%%%%%% YOUR CODE: END %%%%%%%%%%%%%

%% ex 10.2
fig7 = figure(7);
theme(fig7, 'light')
grid on;
plot(rad2deg(s.delta_in.Data)); grid on;
xlabel('time [s]'); ylabel('\delta [deg]');
title('Steering Angle Over Time');


% ======================= Sub-Fucntions ===================================
function draw_circle(a, b, r)
    theta = linspace(0, 2*pi, 100);
    x = a + r*cos(theta);
    y = b + r*sin(theta);
    plot(x , y, 'k--', 'LineWidth' , 2);
end