function fig_obj = fitFigureWithMargin(fig_obj, marginFrac)
%FITFIGUREWITHMARGIN Create/resize current figure with a % margin on largest monitor.
% marginFrac: fraction of the monitor size for each side (e.g., 0.10 for 10%)
% Returns the figure handle.

    if nargin < 1, marginFrac = 0.10; end
    marginFrac = max(0,min(0.49,marginFrac));    % clamp to [0, 0.49]

    % Get all monitor rectangles: [x y width height] in pixels
    mp = get(groot,'MonitorPositions');          % works for multi-monitor setups
    if size(mp,1) > 1
        % Pick the largest monitor by area
        [~,idx] = max(prod(mp(:,3:4),2));
        mon = mp(idx,:);
    else
        mon = mp(1,:);
    end

    % Compute desired figure rectangle with symmetric margins
    mx = mon(3)*marginFrac;                      % horizontal margin (pixels)
    my = mon(4)*marginFrac;                      % vertical margin (pixels)
    pos = [ mon(1)+mx, mon(2)+my, mon(3)-2*mx, mon(4)-2*my ];

    % Create or resize the current figure
    set(fig_obj,'Units','pixels','WindowState','normal'); % ensure we can set Position
    set(fig_obj,'Position',pos);

    % (Optional) better performance for animation
    set(fig_obj,'Renderer','opengl');
end
