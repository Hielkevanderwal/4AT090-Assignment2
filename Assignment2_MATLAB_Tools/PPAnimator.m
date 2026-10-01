classdef PPAnimator < matlab.System
%PPAnimator  Simulink-safe clone of pure_pursuit_animation.m

%#codegen
properties(Nontunable)
    desiredFrameRate = 30;     % target animation fps
    simStepSize      = 1/120;  % Simulink sample time (s)
end

properties(Access=private)
    frameSkip = 1;     % steps to skip between redraws
    counter   = 0;     % internal counter
end

methods(Access=protected)
    % Timing: make this block run at simStepSize
    function sts = getSampleTimeImpl(obj)
        sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.simStepSize);
    end

    % Output port (dummy "tick" so the block can't be optimized away)
    function num = getNumOutputsImpl(~), num = 1; end
    function varargout = getOutputSizeImpl(~), varargout{1} = [1 1]; end
    function varargout = getOutputDataTypeImpl(~), varargout{1} = 'uint8'; end
    function varargout = isOutputComplexImpl(~), varargout{1} = false; end
    function varargout = isOutputFixedSizeImpl(~), varargout{1} = true; end

    % Setup – mark graphics & helpers as extrinsic and compute frameSkip
    function setupImpl(obj)
        obj.frameSkip = max(1, round(1/(obj.desiredFrameRate*obj.simStepSize)));

        coder.extrinsic('figure','title','set','clf','axis','grid','hold','legend','shg');
        coder.extrinsic('plot','patch','text', 'sprintf');
        coder.extrinsic('linspace','cos','sin');
        coder.extrinsic('theme');
        coder.extrinsic('helpers.fitFigureWithMargin');
        coder.extrinsic('helpers.target_search');
    end

    % Step – returns a tick; plots only every frameSkip steps
    function tick = stepImpl(obj, time, path, currentPose, lookahead_dist, lastFoundIndex)

        % bump counter and decide whether to draw
        obj.counter = obj.counter + 1;
        doDraw = (mod(obj.counter, obj.frameSkip) == 0);

        % Only attempt graphics when running in MATLAB (not generated code)
        if doDraw && isempty(coder.target)
            % ----- clone of your pure_pursuit_animation.m -----
            fig1 = figure(1);
            theme(fig1,"light");              % ok if missing (extrinsic)
            helpers.fitFigureWithMargin(fig1, 0.10);
            clf(fig1); axis equal; grid on; hold on; shg;
            set(0,'DefaultFigureRenderer','opengl');
            title(sprintf('Time: %.2f sec | Index: %d', time, lastFoundIndex))

            % segments and points
            PPAnimator.plot_path_with_label(path(1:lastFoundIndex,:), '--', [0.6 0.3 0], 'traveled path');
            PPAnimator.plot_path_with_label(path(lastFoundIndex:end,:),   '-', [0.5 0.5 0.5], 'remaining path');
            PPAnimator.highlight_points(path(1:lastFoundIndex,:), 'r');
            PPAnimator.highlight_points(path(lastFoundIndex:end,:), [0.5 0.5 0.5]);

            % pose
            x1  = currentPose(1);  y1  = currentPose(2);  psi = currentPose(3);
            plot(x1, y1, '.', 'Color','r','MarkerSize',15,'DisplayName','current pose');
            plot([x1, x1 + TruckParams.L1*cos(psi)], [y1, y1 + TruckParams.L1*sin(psi)], ...
                 'k','LineWidth',3);

            % lookahead circle
            PPAnimator.draw_circle(x1, y1, lookahead_dist);

            % FOV wedge
            R_min     = TruckParams.L1 / tan(TruckParams.delta_max);
            alpha_max = asin( min(1, lookahead_dist/(2*R_min)) );
            th   = linspace(psi - alpha_max, psi + alpha_max, 80);
            xArc = x1 + lookahead_dist*cos(th);
            yArc = y1 + lookahead_dist*sin(th);
            patch('XData',[x1, xArc, x1], 'YData',[y1, yArc, y1], ...
                  'FaceColor',[0.2 0.8 0.2], 'FaceAlpha',0.15, ...
                  'EdgeColor',[0.2 0.8 0.2], 'LineStyle','-', 'DisplayName','FOV');

            % target (uses your helper)
            [target, ~] = helpers.target_search(path, currentPose, lookahead_dist, lastFoundIndex);
            if all(~isnan(target))
                plot(target(1), target(2), 'p', 'Color','m','MarkerSize',12,'DisplayName','goal point');
                PPAnimator.plot_path_with_label([[x1, y1]; target], '-', 'k', 'lookahead line');
                text(target(1), target(2), sprintf('(%.2f, %.2f)', target(1), target(2)), ...
                     'VerticalAlignment','top','HorizontalAlignment','left');
            end

            % endpoints + legend
            plot(path(1,1),  path(1,2),  '.', 'Color','b','MarkerSize',15,'DisplayName','start');
            plot(path(end,1),path(end,2),'.', 'Color','g','MarkerSize',15,'DisplayName','end');
            leg = legend('Location','bestoutside');
            set(leg,'AutoUpdate','off'); % <- no duplicates in legend
            % ---------------------------------------------------
        end

        % dummy output so Simulink keeps executing this block
        tick = uint8(mod(lastFoundIndex, 255));
    end

    function resetImpl(obj)
        obj.counter = 0;
    end
end

% ---- static helpers (copied from your function) ----
methods(Static, Access=private)
    function draw_circle(a,b,r)
        theta = linspace(0,2*pi,100);
        plot(a + r*cos(theta), b + r*sin(theta),'k--','LineWidth',2);
    end

    function plot_path_with_label(seg, ls, clr, lab)
        if isempty(seg) || size(seg,1) < 2, return; end
        for i = 1:size(seg,1)-1
          if i == 1
            plot(seg(i:i+1,1), seg(i:i+1,2), ls, 'Color', clr, ...
                 'LineWidth', 1.5, 'DisplayName', lab, 'HandleVisibility','on');
          else
            plot(seg(i:i+1,1), seg(i:i+1,2), ls, 'Color', clr, ...
                 'LineWidth', 1.5, 'HandleVisibility','off');
          end
          hold on;
        end
    end

    function highlight_points(pts, c)
        if isempty(pts), return; end
        if isvector(pts) && numel(pts)==2
            plot(pts(1),pts(2),'.','Color',c,'MarkerSize',10);
        elseif size(pts,2)==2
            plot(pts(:,1),pts(:,2),'.','Color',c,'MarkerSize',10);
        end
    end
end
end
