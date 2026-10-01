function [target_point, lastFoundIndex] = kin_feasible_target_search(path, currentPose, ...
                                        lookahead_dist, lastFoundIndex, L1, delta_max)
    currentX = currentPose(1);
    currentY = currentPose(2);
    currentPSI = currentPose(3);

    target_point = [NaN NaN];
    startIndex = lastFoundIndex;
    
    R_min = L1/atan(delta_max);
    alpha_max = asin(min(1,lookahead_dist/(2*R_min)));

    for i = startIndex:(length(path) - 1)
        % Step 1. Offset the circle to the origin
        x1 = path(i,1);
        y1 = path(i,2);
        x2 = path(i+1,1);
        y2 = path(i+1,2);
        % zeroed out coordinates of the current pose
        x10 = x1 - currentX;
        y10 = y1 - currentY;
        x20 = x2 - currentX;
        y20 = y2 - currentY;

        dx = x20 - x10;
        dy = y20 - y10;
        dr = sqrt(dx^2 + dy^2);
        D = x10*y20 - x20*y10;
        delta = lookahead_dist^2*dr^2 - D^2;

        if delta >= 0
            % Step 3. Compute points of intersection
            x1_intersect = (D*dy + helpers.sgn_star(dy)*dx*sqrt(delta))/dr^2;
            x2_intersect = (D*dy - helpers.sgn_star(dy)*dx*sqrt(delta))/dr^2;
            y1_interesct = (-D*dx + abs(dy)*sqrt(delta))/dr^2;
            y2_interesct = (-D*dx - abs(dy)*sqrt(delta))/dr^2;
            % Step 4. Add 'currentX' and 'currentY'. Offset the system back to its original position
            intersention_1 = [x1_intersect+currentX y1_interesct+currentY];
            intersention_2 = [x2_intersect+currentX y2_interesct+currentY];
            
            minX = min(path(i,1), path(i+1,1));
            maxX = max(path(i,1), path(i+1,1));
            minY = min(path(i,2), path(i+1,2));
            maxY = max(path(i,2), path(i+1,2));

            intersec1_in_range = intersention_1(1) >= minX && intersention_1(1) <= maxX && ...
                                 intersention_1(2) >= minY && intersention_1(2) <= maxY;
            intersec2_in_range = intersention_2(1) >= minX && intersention_2(1) <= maxX && ...
                                 intersention_2(2) >= minY && intersention_2(2) <= maxY;
                         
            % copute target direction alpha for both intersections
            theta_1 = atan2(intersention_1(2) - currentY, intersention_1(1) - currentX);
            theta_2 = atan2(intersention_2(2) - currentY, intersention_2(1) - currentX);
            alpha_1 = theta_1 - currentPSI;
            alpha_2 = theta_2 - currentPSI;
            
            % Normalizing the angles
            alpha_1 = mod(alpha_1 + pi, 2*pi) - pi;
            alpha_2 = mod(alpha_2 + pi, 2*pi) - pi;
%             alpha_1 = atan2(sin(a_1), cos(a_1));  %theta_1 - currentPSI;
%             alpha_2 = atan2(sin(a_2), cos(a_2));  %theta_2 - currentPSI;
            
            % Feasible target point if "in-range" and |alpha| <= alpha_{max}
            feasible_1 = intersec1_in_range && (abs(alpha_1) <= alpha_max);
            feasible_2 = intersec2_in_range && (abs(alpha_2) <= alpha_max);
            

            if feasible_1 || feasible_2
                
                if feasible_1 && feasible_2 %% both solutions in range

                    if helpers.euclidean_distance(intersention_1, path(i+1,:)) < ...
                       helpers.euclidean_distance(intersention_2, path(i+1,:))
                        target_point = intersention_1;
                    else
                        target_point = intersention_2;
                    end
                else
                    if feasible_1
                        target_point = intersention_1;
                    else
                        target_point = intersention_2;
                    end

                end
                
                if helpers.euclidean_distance(target_point,  path(i+1,:)) < ...
                   helpers.euclidean_distance([currentX currentY], path(i+1,:))   
                    lastFoundIndex = i; % stay on this segment
                    break % we've found a good target
                else
                    lastFoundIndex = i+1; % move search foward to the next segment and KEEP LOOPING
                end
            else % no intersections found
                target_point = path(lastFoundIndex + 1, :);
            end                
        end
    end
end
