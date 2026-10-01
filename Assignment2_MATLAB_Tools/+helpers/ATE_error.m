function [ATE, RMSE_ATE, dense_path] = ATE_error(pose, sparse_path)
    % Step 1. reasample theref path density to match the simulated trajectory 
    N = size(pose,1);
    dense_path = resample_path_arclength(sparse_path, N);

    % Step 2. Preallocate outputs for
    ATE      = zeros(N,1);             % pointwise error
    RMSE_ATE = zeros(N,1);             % running RMSE
    sum_sq   = 0;                      % running sum of squared errors

    % Step 3. Loop over time and compute ATE errors
    for k = 1:N
        xk     = pose(k,1);            
        yk     = pose(k,2);            
        x_ref  = dense_path(k,1);      
        y_ref  = dense_path(k,2);      

        % Exercise 8.1: ATE implementation
        %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

        %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

        % Exercise 8.2: running RMSE-ATE implementation
        %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

        %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
    end
end

function dense_path = resample_path_arclength(path, N)
% Resample a polyline (path) to N points with uniform arclength spacing.

    ds = sqrt(sum(diff(path,1,1).^2, 2));  % segment lengths
    s  = [0; cumsum(ds)];                   % arclength parameter
    sQ = linspace(0, s(end), N)';           % uniform arclength samples

    dense_path = [ interp1(s, path(:,1), sQ, 'pchip'), ...
                 interp1(s, path(:,2), sQ, 'pchip')];
end
