function alpha = target_direction(target_point, currentPose)
    currentX = currentPose(1); currentY = currentPose(2); currentPSI = currentPose(3);
    targetX = target_point(1); targetY = target_point(2);
    % Exercise 3.1 Target direction alpha implementation
    %%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

    theta = atan2(targetY - currentY, targetX - currentX);
    a = theta - currentPSI;

    % Normailze alpha to (-pi, pi] 
    alpha = mod(a + pi, 2*pi) - pi;
    
    %%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
end
