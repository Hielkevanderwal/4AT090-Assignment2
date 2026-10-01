clearvars; 
close all;
clc;

% Load path
path_string = "paths/dev_left_turn_path.mat";
load(path_string); % assumes it loads a variable called 'path'

%% Parameters
% Initial conditions
p.init.x10 = -.5;
p.init.y10 = 0;
p.init.psi10 = 3*pi/5;
p.init.psi20 = 3*pi/5;

% Input
p.input.Ld = 1.3;
p.input.V1 = 0.5;

% Simulation
dt = 0.5;
t_end = 120;
t_vec = 0:dt:t_end;
% --- Set up ---
N = length(t_vec);
X = zeros(N,4); % alloc. memory for an array: rows = state vector for each time step
% Initial state
state = [p.init.x10; p.init.y10; p.init.psi10; p.init.psi20];
X(1,:) = state';


% Controller setup
controller.path = path;
controller.V1 = p.input.V1;
controller.lookahead_dist = p.input.Ld;
controller.lastFoundIndex = 1;
controller.proximity_threshold = .2;

% memory allocation
x_targets = zeros(1, N-1);
y_targets = zeros(1, N-1);
target_flag = false(1, N-1);
deltas = zeros(1, N-1);
alphas = zeros(1, N-1);

%% Simulation Loop
for k = 1:length(t_vec)-1
    x1 = X(k,1);
    y1 = X(k,2);
    psi1 = X(k,3);
    psi2 = X(k,4);

    % Step 1: Target Point Selection and lastFoundIndex update (if needed)
    [target_point, controller.lastFoundIndex] = helpers.target_search(controller.path.rearaxle, [x1, y1, psi1], ...
                                                                      controller.lookahead_dist, controller.lastFoundIndex);

    % [intermediante step] logging target point history
    if all(~isnan(target_point))
        target_flag(k) = true;
        x_targets(k) = target_point(1);
        y_targets(k) = target_point(2);
    end

    % Step 2: Compute the target direction alpha
    alpha = helpers.target_direction(target_point, [x1, y1, psi1]);
    alphas(k) = alpha;

    % Step 3: Compute steering angle delta based on the obtained alpha
    delta_in = helpers.TargetDir2SteerAng(TruckParams.L1, alpha, controller.lookahead_dist);
    deltas(k) = delta_in;
    
    % Exercise 6: STOPING CRITERION
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
    % TODO: Check the distance between the vehicle's current position and
    % the final waypoint (controller.path.rearaxle(end,:)). If it is below
    % controller.proximity_threshold, set the vehicle velocity to zero,
    % trim the pre-allocated arrays (X, alphas, deltas, x_targets,
    % y_targets, target_flag) to the current index, and terminate the
    % simulation loop.
    
    %%%%%%%%%%%% YOUR CODE: END %%%%%%%%%%%%%

    % Step 4: Feed the obtained delta into the kinematic model and retrive
    % necessary derivatives
    [~, ~, ~, ~, ~, ~, ~, ~, ~, x1_dot, y1_dot, psi1_dot, psi2_dot] = ...
        helpers.kinematic_model(controller.V1, delta_in, x1, y1, psi1, psi2);

    % Step 5: Discretly integrate the derivates and progress the simulation
    % in time by a fixed time step 'dt'
    % Exercise 5.1: Euler integration
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

    %%%%%%%%%%%% YOUR CODE: END %%%%%%%%%%%%%
    
    % Animate
    helpers.pure_pursuit_animation(controller.path.rearaxle, [x1, y1, psi1], controller.lookahead_dist, controller.lastFoundIndex);
    title(sprintf('Time: %.2f sec | Index: %d', t_vec(k+1), controller.lastFoundIndex));
    % drawnow limitrate % <— flush graphics without overloading CPU
    pause(0.01)       % <— optional, gives time for the figure to repaint
end

% trimming the time vector in case the stoping criterion gets triggered
if size(X,1) < N
    t_vec = t_vec(1:size(X,1));
end

% for convinience grab x y psi
x_pose = X(:,1); y_pose = X(:,2); psi_orientation = X(:,3);

%% Plot results
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
xlabel('X [m]'); ylabel('Y [m]');
legend('Location','best');
title('Tractor Rear Axle Trajectory');

% Exercise 7.1: Plot steering ang, target direction and tractor orientation
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%% Quantitative evaluation
[ATEs, RMSE_ATEs, dense_ref_path] = helpers.ATE_error([x_pose, y_pose], [path.rearaxle(:,1), path.rearaxle(:,2)]);

%% Plotting ATE results
 fig7 = figure(7);
 theme(fig7, "light")
 hold on; grid on; 
 plot(t_vec, ATEs, 'r-', DisplayName='ATE');
 plot(t_vec, RMSE_ATEs, 'r--', DisplayName='RMSE ATE')
 xlabel('time [s]')
 ylabel('absolute trajectory error [m]')
 legend(Location='best')
 hold off;

 %% Plotting resampled ref path
 fig8 = figure(8);
 theme(fig8, 'light')
 hold on; grid on;
 plot(dense_ref_path(:,1), dense_ref_path(:,2), 'b-o')