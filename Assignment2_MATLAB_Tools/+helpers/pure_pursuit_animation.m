function pure_pursuit_animation(path, currentPose, lookahead_dist, lastFoundIndex)
    % Example setup
    fig1 = figure(1);
    theme(fig1, "light")
    helpers.fitFigureWithMargin(fig1, .1);
    % set(fig1, 'Position', [100, 100, 800, 600]); % <- incorporate dynamic dims HERE
    clf(fig1); axis equal; grid on; hold on; shg;
    set(0,'DefaultFigureRenderer','opengl');

    % path segments
    plot_path_with_label(path(1:lastFoundIndex,:), '--', [0.6 0.3 0], 'traveled path');
    plot_path_with_label(path(lastFoundIndex:end,:), '-', [0.5 0.5 0.5], 'remaining path');

    % highlight traversed waypoints
    highlight_points(path(1:lastFoundIndex,:), 'r'); % b
    highlight_points(path(lastFoundIndex:end,:), [0.5 0.5 0.5]); % grey

    % extract current position and heading
    x1 = currentPose(1); y1 = currentPose(2); psi1 = currentPose(3);
    % current pose 
    plot(x1, y1, '.', 'Color', 'r', 'MarkerSize', 15, 'DisplayName', 'current pose');
    % Plot current orientation 
    plot([x1, x1 + TruckParams.L1*cos(psi1)],[y1, y1 + TruckParams.L1*sin(psi1)], 'k', 'LineWidth', 3);
    
    % search radius 
    draw_circle(currentPose(1), currentPose(2), lookahead_dist);

    % ===== FOV wedge (kinematically feasible area of search) =====
    % R_min = L / tan(delta_max),  alpha_max = asin( min(1, Ld / (2*R_min)) )
    R_min     = TruckParams.L1 / tan(TruckParams.delta_max);
    alpha_max = asin( min(1, lookahead_dist / (2*R_min)) );

    th   = linspace(psi1 - alpha_max, psi1 + alpha_max, 80);
    xArc = x1 + lookahead_dist * cos(th);
    yArc = y1 + lookahead_dist * sin(th);
    patch('XData',[x1, xArc, x1], ...
          'YData',[y1, yArc, y1], ...
          'FaceColor',[0.2 0.8 0.2], 'FaceAlpha',0.15, ...
          'EdgeColor',[0.2 0.8 0.2], 'LineStyle','-', ...
          'DisplayName','FOV');

    % Target search (uses your `target_search` logic)
    [target, ~] = helpers.target_search(path, currentPose, lookahead_dist, lastFoundIndex);

    if any(~isnan(target))
        plot(target(1), target(2), 'pentagram', 'Color', 'm', 'MarkerSize', 15, 'DisplayName', 'goal point');
        plot_path_with_label([currentPose(:,1:2); target], '-', 'k', 'lookahead line');
        text(target(1), target(2), sprintf('(%.2f, %.2f)', target(1), target(2)), ...
             'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    end

    % Start and end markers
    plot(path(1,1), path(1,2), '.', 'Color', 'b', 'MarkerSize', 15, 'DisplayName', 'start');
    plot(path(end,1), path(end,2), '.', 'Color', 'g', 'MarkerSize', 15, 'DisplayName', 'end');

    % Add grid and legend
    grid on;
    legend('Location', 'bestoutside');

    drawnow limitrate nocallbacks
end


% ======================= Sub-Fucntions ===================================
function draw_circle(a, b, r)
    theta = linspace(0, 2*pi, 100);
    x = a + r*cos(theta);
    y = b + r*sin(theta);
    plot(x , y, 'k--', 'LineWidth' , 2);
end

% function plot_path(path, pointColor, lineColor)
%     plot(path(:,1), path(:,2), '.', 'Color', pointColor, 'MarkerSize', 10); hold on;
%     plot(path(:,1), path(:,2), '-', 'Color', lineColor, 'LineWidth', 1.5);
% end

function plot_path_with_label(path, lineStyle, lineColor, lineLabel)
    for i = 1:size(path, 1) - 1
        if i == 1
            plot(path(i:i+1,1), path(i:i+1,2), lineStyle, 'Color', lineColor, ...
                'DisplayName', lineLabel, 'LineWidth', 1.5);
        else
            plot(path(i:i+1,1), path(i:i+1,2), lineStyle, 'Color', lineColor, ...
                'HandleVisibility', 'off', 'LineWidth', 1.5);
        end
        hold on;
    end
end

function highlight_points(points, pointColor)
    if isvector(points) && length(points) == 2
        % Single point (1x2 or 2x1)
        plot(points(1), points(2), '.', 'Color', pointColor, 'MarkerSize', 10); hold on;
    elseif size(points,2) == 2
        % Matrix of points
        plot(points(:,1), points(:,2), '.', 'Color', pointColor, 'MarkerSize', 10); hold on;
    else
        warning('Unexpected input format to highlight_points.');
    end
end