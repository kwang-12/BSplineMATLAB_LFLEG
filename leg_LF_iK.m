function jointAngles = leg_LF_iK(current_joint_angles, desired_pos, dimensions)
    theta_1_now = current_joint_angles(1);
    theta_2_now = current_joint_angles(2);
    theta_3_now = current_joint_angles(3);
    X_desired = desired_pos(1);
    Y_desired = desired_pos(2);
    Z_desired = desired_pos(3);
    d2 = dimensions.d2;
    a2 = dimensions.a2;
    a3 = dimensions.a3;
    % determine theta 3
    c3 = (X_desired^2 + Y_desired^2 + Z_desired^2 - d2^2 - a2^2 - a3^2)/(2*a2*a3);
    if c3^2 <= 1         %theta 3 has two solutions
        theta_3_1 = atan2(sqrt(1-c3^2), c3);
        theta_3_2 = atan2(-sqrt(1-c3^2), c3);
        if abs(theta_3_1 - theta_3_now) >= abs(theta_3_2 - theta_3_now)       %closest solutoin is desired
            theta_3_desired = theta_3_2;
        else
            theta_3_desired = theta_3_1;
        end
    else     %theta 3 has no solution
        disp('Solution doesnt exist! - Theta 3 has no solution')
        jointAngles = [0 0 0];
        return 
    end
    % determine theta 2
    a = -a3*sin(theta_3_desired);
    b = -a3*cos(theta_3_desired) - a2;
    c = Z_desired;
    if a^2+b^2-c^2 > 0                  %theta 2 has two solutions
        theta_2_1 = atan2(sqrt(a^2 + b^2 - c^2), c) + atan2(b, a);
        theta_2_2 = atan2(-sqrt(a^2 + b^2 - c^2), c) + atan2(b, a);
        if abs(theta_2_1 - theta_2_now) >= abs(theta_2_2 - theta_2_now)       %closest solution is desired
            theta_2_desired = theta_2_2;
        else
            theta_2_desired = theta_2_1;
        end
    elseif a^2+b^2-c^2 == 0             %theta 2 has one solution
        theta_2_desired = atan2(b, a);
    else                                %theta 2 has no solution
        disp('theta 2 has no solution')
        jointAngles = [0 0 0];
        return 
    end
    % determine theta 1
    a = a2*cos(theta_2_desired) + a3*cos(theta_2_desired+theta_3_desired);
    b = -d2;
    c = X_desired;
    if a^2+b^2-c^2 > 0                  %theta 1 has two solutions
        theta_1_1 = atan2(sqrt(a^2 + b^2 - c^2), c) + atan2(b, a);
        theta_1_2 = atan2(-sqrt(a^2 + b^2 - c^2), c) + atan2(b, a);
        if abs(theta_1_1 - theta_1_now) >= abs(theta_1_2 - theta_1_now)         %closest solution is desired
            theta_1_desired = theta_1_2;
        else
            theta_1_desired = theta_1_1;
        end
    elseif a^2+b^2-c^2 ==0              %theta 1 has one solution
    else                                %theta 1 has no solution
        disp('theta 1 has no solution')
        jointAngles = [0 0 0];
        return
    end
    jointAngles = [theta_1_desired, theta_2_desired, theta_3_desired];
end