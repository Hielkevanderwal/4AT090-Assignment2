function out = simulate_pure_pursuit(Ld, path, baseParams)

    V1   = baseParams.V1;
    dt   = baseParams.dt;
    t_end= baseParams.t_end;

    t_vec = 0:dt:t_end;
    N = numel(t_vec);
    X = zeros(N,4);
    X(1,:) = [baseParams.x10, baseParams.y10, baseParams.psi10, baseParams.psi20];

    controller.path = path;
    controller.V1   = V1;
    controller.lookahead_dist = Ld;
    controller.lastFoundIndex = 1;
    controller.proximity_threshold = baseParams.proximity_threshold;

    x_targets = NaN(1, N-1);
    y_targets = NaN(1, N-1);
    alphas    = NaN(1, N-1);
    deltas    = NaN(1, N-1);

    for k = 1:N-1
        x1 = X(k,1); y1 = X(k,2); psi1 = X(k,3); psi2 = X(k,4);

        [target_point, controller.lastFoundIndex] = helpers.kin_feasible_target_search( ...
            controller.path.rearaxle, [x1, y1, psi1], ...
            controller.lookahead_dist, controller.lastFoundIndex, ...
            TruckParams.L1, TruckParams.delta_max);

        if all(~isnan(target_point))
            x_targets(k) = target_point(1);
            y_targets(k) = target_point(2);
        end

        alpha = helpers.target_direction(target_point, [x1, y1, psi1]);
        alphas(k) = alpha;

        delta_in = helpers.TargetDir2SteerAng(TruckParams.L1, alpha, controller.lookahead_dist);
        deltas(k) = delta_in;

        if helpers.euclidean_distance([x1, y1], controller.path.rearaxle(end,:)) < controller.proximity_threshold
            % Trim and stop
            X = X(1:k, :); t_vec = t_vec(1:k);
            x_targets = x_targets(1:k-1); 
            y_targets = y_targets(1:k-1);
            alphas = alphas(1:k-1);   
            deltas = deltas(1:k-1);
            break;
        end

        % Kin model
        [~, ~, ~, ~, ~, ~, ~, ~, ~, x1_dot, y1_dot, psi1_dot, psi2_dot] = ...
            helpers.kinematic_model(controller.V1, delta_in, x1, y1, psi1, psi2);

        % Euler step
        X(k+1, :) = X(k,:) + dt * [x1_dot, y1_dot, psi1_dot, psi2_dot];
    end

    x_pose = X(:,1); y_pose = X(:,2); psi_orientation = X(:,3);

    % ATE / RMSE (pointwise)
    [ATE, RMSE_ATE, refDense] = helpers.ATE_error([x_pose, y_pose], path.rearaxle);

    % Pack output
    out = struct( ...
        'Ld', Ld, 't_vec', t_vec, 'X', X, ...
        'x_pose', x_pose, 'y_pose', y_pose, 'psi_orientation', psi_orientation, ...
        'x_targets', x_targets, 'y_targets', y_targets, ...
        'alphas', alphas, 'deltas', deltas, ...
        'ATE', ATE, 'RMSE_ATE', RMSE_ATE, 'refDense', refDense);
end
