function plot_path(path, pointColor, lineColor)
    plot(path(:,1), path(:,2), '.', 'Color', pointColor, 'MarkerSize', 10); hold on;
    plot(path(:,1), path(:,2), '-', 'Color', lineColor, 'LineWidth', 1.5);
end