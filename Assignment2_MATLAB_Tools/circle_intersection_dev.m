clearvars; 
close all;
clc;

circle_intersection_bounds([1, 2], [1, 0], [1, 4], 1);

function y = sgn_star(x)
    % Exercise 1.1: sgn* implementation
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
    
    if x<0 
        y=-1;
    else
        y=1;
    end
    
    %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
end

function draw_circle(a, b, r)
    theta = linspace(0, 2*pi, 100);
    x = a + r*cos(theta);
    y = b + r*sin(theta);
    plot(x , y, 'w--', 'LineWidth' , 2);
end

function circle_intersection_bounds(Pose, waypoint1, waypoint2, lookahead_dist)
    currentX = Pose(1);
    currentY = Pose(2);
     
    x1 = waypoint1(1);
    y1 = waypoint1(2);
    x2 = waypoint2(1);
    y2 = waypoint2(2);
    fprintf("Waypoint 1 (x1, y1): (%.1f, %.1f)\n", x1, y1);
    fprintf("Waypoint 2 (x2, y2): (%.1f, %.1f)\n", x2, y2);
    
    intersectionFound = false; % flag for the intersection 
    
    % Offset the circle to the origin
    x1_offset = x1 - currentX;
    y1_offset = y1 - currentY;
    x2_offset = x2 - currentX;
    y2_offset = y2 - currentY;
    fprintf("Offsetted Waypoint 1 (x1, y1): (%.1f, %.1f)\n", x1_offset, y1_offset);
    fprintf("Offsetted Waypoint 2 (x2, y2): (%.1f, %.1f)\n", x2_offset, y2_offset);
    
    % Exercise 1.2: implement the Circle-Line Intersection math from
    % Intermezzo 1 eq 1 to 5
    % [NOTE !!!} Please use the offseted variables and name yours according
    % to the 'print' statement seen below.
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

    r = lookahead_dist;

    dx = x2_offset - x1_offset;
    dy = y2_offset - y1_offset;

    dr_sq = dx*dx + dy*dy; % removed sqrt for optimalisation.

    D = x1_offset * y2_offset - x2_offset * y1_offset;
    delta = r*r * dr_sq - D*D;

    %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
    fprintf("dx: " + dx + "\n");
    fprintf("dy: " + dy + "\n");
    fprintf("dy: " + dy + "\n");
    fprintf("dr^2: " + dr_sq + "\n");
    fprintf("D: " + D + "\n");
    fprintf("delta: " + delta + "\n");
    
    if delta >= 0
        % Exercise 1.3: implement the Circle-Line Intersection math from
        % Intermezzo 1 eq 6 and 7
        %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
        
        x1_intersect = (D * dy - sgn_star(dy) * dx * sqrt(delta)) / dr_sq;
        x2_intersect = (D * dy + sgn_star(dy) * dx * sqrt(delta)) / dr_sq;

        y1_intersect = (-D*dx - abs(dy)*sqrt(delta)) / dr_sq;
        y2_intersect = (-D*dx + abs(dy)*sqrt(delta)) / dr_sq;

        %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
        fprintf("x1_intersect: " + x1_intersect + "\n");
        fprintf("x2_intersect: " + x2_intersect + "\n");
        fprintf("y1_intersect: " + y1_intersect + "\n");
        fprintf("y2_intersect: " + y2_intersect + "\n");
        
        % Add 'currentX' and 'currentY'. Offset the system back to its original position
        intersention_1 = [x1_intersect+currentX y1_intersect+currentY];
        fprintf("intersention_1: (%.2f, %.2f)\n", intersention_1(1), intersention_1(2));
        intersention_2 = [x2_intersect+currentX y2_intersect+currentY];
        fprintf("intersention_2: (%.2f, %.2f)\n", intersention_2(1), intersention_2(2));
        
        minX = min(x1, x2);
        maxX = max(x1, x2);
        minY = min(y1, y2);
        maxY = max(y1, y2);
        
        % Exercise 1.4: Conditions for intersections to be in range
        %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

        intersec1_in_range = (minX - currentX)^2 + (minY - currentY)^2 >= r^2;
        intersec2_in_range = (maxX - currentX)^2 + (maxY - currentY)^2 >= r^2;

        %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

        if intersec1_in_range || intersec2_in_range
            intersectionFound = true;

            if intersec1_in_range
                fprintf("Intersection 1 is valid.\n");
            end

            if intersec2_in_range
                fprintf("Intersection 2 is valid.\n");
            end
        end
                      
    end
        
    figure(1)
    plot([x1 x2], [y1 y2], 'g*-', 'LineWidth', 2); hold on;
    axis equal;
    draw_circle(currentX, currentY, lookahead_dist);
    
    if intersectionFound == false
        fprintf("No intersection found.");
    else
        fprintf('Intersection 1 found at [%.4f, %.4f]\n', intersention_1(1), intersention_1(2));
        fprintf('Intersection 2 found at [%.4f, %.4f]\n', intersention_2(1), intersention_2(2));
        plot(intersention_1(1), intersention_1(2), 'r.', 'MarkerSize', 20); hold on;
        plot(intersention_2(1), intersention_2(2), 'b.', 'MarkerSize', 20);
        str1 = sprintf('(%.2f, %.2f)', intersention_1(1), intersention_1(2));
        str2 = sprintf('(%.2f, %.2f)', intersention_2(1), intersention_2(2));
        text(intersention_1(1), intersention_1(2), str1, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
        text(intersention_2(1), intersention_2(2), str2, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
        legend('','','intersection 1', 'intersection 2');   
    end
    
end