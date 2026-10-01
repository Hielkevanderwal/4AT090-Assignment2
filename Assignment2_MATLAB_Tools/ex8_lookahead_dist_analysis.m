clearvars; close all; clc;

path_string = "paths/dev_left_turn_path.mat";
load(path_string);  % -> path struct with .rearaxle

% Base paramters
base.V1 = 0.5;
base.dt = 0.5;
base.t_end = 120;
base.proximity_threshold = 0.2;

% Initial conditions
base.x10 = 0.0;
base.y10 =  0.0;
base.psi10 = pi/2;
base.psi20 = pi/2;

% Lookahead distances to test
Ld_list = [0.6 0.9 1.3 1.8];

% Prepare results folder
outdir = fullfile(pwd, 'results_Ld_sweep');
if ~exist(outdir, 'dir'), mkdir(outdir); end

% Container for all runs
ALL = struct('Ld', {}, 't_vec', {}, 'x_pose', {}, 'y_pose', {}, ...
             'ATE', {}, 'RMSE_ATE', {}, 'refDense', {});

for i = 1:numel(Ld_list)
    Ld = Ld_list(i);
    out = helpers.simulate_pure_pursuit(Ld, path, base);   % one run

    % Save per-run MAT with a readable name
    fname = sprintf('traj_Ld_%0.2f.mat', Ld);
    save(fullfile(outdir, fname), '-struct', 'out');

    % (Optional) also export CSV with trajectory for quick inspection
    T = table(out.t_vec(:), out.x_pose(:), out.y_pose(:), out.ATE(:), out.RMSE_ATE(:), ...
              'VariableNames', {'t','x','y','ATE','RMSE_ATE'});
    writetable(T, fullfile(outdir, sprintf('traj_Ld_%0.2f.csv', Ld)));

    ALL(i) = struct('Ld', out.Ld, 't_vec', out.t_vec, ...
                    'x_pose', out.x_pose, 'y_pose', out.y_pose, ...
                    'ATE', out.ATE, 'RMSE_ATE', out.RMSE_ATE, ...
                    'refDense', out.refDense);
end

% Save combined results too (handy to reload)
save(fullfile(outdir, 'all_results.mat'), 'ALL', 'Ld_list', 'path', 'base');

% ---------- Plot comparisons ----------
colors = ["#F5AD27", "#F40B0B", "#00D131", "#0B35E3"]; % hex [yellow red green blue]
x_ref = path.rearaxle(:,1); y_ref = path.rearaxle(:,2); 
y_range =  max(y_ref) - min(y_ref);
% Trajectories overlayed
fig1 = figure(101); 
clf;
theme(fig1,"light")
hold on; grid on; axis equal;
plot(x_ref, y_ref, 'k--o', 'LineWidth', 2 , 'DisplayName','reference');
for i = 1:numel(ALL)
    plot(ALL(i).x_pose, ALL(i).y_pose, 'LineWidth', 1.5, ...
        'Color', colors(i), ...
        'DisplayName', sprintf('Ld = %.2f m', ALL(i).Ld));
end
xlabel('x [m]'); ylabel('y [m]'); ylim([min(y_ref) - .1*y_range , max(y_ref) + .1*y_range]);
legend('Location','best'); title('Trajectory vs. time sweeped over various L_d');

% ATE over time
fig2 = figure(102); 
clf; 
theme(fig2,"light"); 
hold on; grid on;
for i = 1:numel(ALL)
    plot(ALL(i).t_vec, ALL(i).ATE, 'LineWidth', 1.2, ...
        'Color', colors(i), ...
        'DisplayName', sprintf('ATE: Ld = %.2f', ALL(i).Ld));
    plot(ALL(i).t_vec, ALL(i).RMSE_ATE, '--', 'LineWidth', 1.2, ...
        'Color', colors(i), ...
        'DisplayName', sprintf('RMSE-ATE: Ld = %.2f', ALL(i).Ld));
end
xlabel('time [s]'); ylabel('ATE/RMSE-ATE [m]'); legend('Location','best'); title('ATE and RMSE-ATE per timestep sweeped over various L_d');