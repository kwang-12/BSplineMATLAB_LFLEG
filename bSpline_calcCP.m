%% Calculation--Control point, d---
% Since k is set to 7, the calculation process is greatly simplified.
% 1. A total of properties.n+1 control point need to be found.
% 2. A total of size(q,1)=properties.n+1-6 equations are already provided.
% 3. A total of 6 additional equations need to be determined.
% In this case, a total of 6 equations need to be determined.
% Since closed B-Spline is created, the curve must pass through the first
% and last control points.
% Therefore, the rest of 6 eqns. come from the pre-determined kinematic
% constraints:
% 1. velocities, 2. acceleration, 3. jerk at the beginning and the end of
% the closed B-Spline.
% Note-1: Since the mid point of ground contact is selected as the beginning
% and the end of the trajectory, the acceleration and jerk at the said
% points are prefered to be zero.
% Specifically, the velocity should be the stride speed while the
% acceleration and jerk are set to zero.
% Note-2: The control points are solved by setting a linear combination of
% properties.n equations.

% Input:
% 1. Desired joint trajectory vector, jTraj
% jTraj should be a vertical vector with A element.
% A is the number of desired joint position.
% 2. Corresponding knot vector, U
% 3. Desired velocities at the beginning and the end of the trajectory,
% vDesired, 2x1 vector
% 4. Desired acceleration at the beginning and the end of the trajectory,
% aDesired, 2x1 vector
% 5. Desired jerk at the beginning and the end of the trajectory, 
% jDesired, 2x1 vector
% 6. B-spline properties, properties
% Output:
% 1. Calculated control point vector, cP
% 

function cP = bSpline_calcCP(jTraj, U, vDesired, aDesired, jDesired, properties)
    n = properties.n;
    k = properties.k;
    Ud = U(2:end-1);
    Udd = Ud(2:end-1);
    Uddd = Udd(2:end-1);
    % solve linear system: A * cP = B
    B = [jTraj; vDesired ; aDesired; jDesired];
    % Assemble A
    A = zeros(size(B,1), size(B,1));
    %----------first properties.n+1-6 equations-------
    for i = 1:n+1-6
        for i_tick = 1:size(A,1)
            A(i, i_tick) = bSpline_Nik(U(i+k), i_tick, k, U);
        end
    end
    %----------the last 6 equations determined by kinematic constraint-------
    %-----velocity constraint-----
    i = n+1-6 + 1;      % ini velocities: u_val = 0, value of u_val is related to the corresponding time stamp of the assigned desired kinematics constraints
    u_val = 0;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = -bSpline_deBaCo('v', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U);
    A(i, end) = bSpline_deBaCo('v', n, u_val, properties, U) * bSpline_deCo('v', n, properties, U);
    for i_tick = 2:size(A,1)-1
        A(i, i_tick) = bSpline_deBaCo('v', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('v', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U);
    end
    i = i + 1;      % end velocities: u_val = 1
    u_val = 1;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = -bSpline_deBaCo('v', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U);
    A(i, end) = bSpline_deBaCo('v', n, u_val, properties, U) * bSpline_deCo('v', n, properties, U);
    for i_tick = 2:size(A,1)-1
        A(i, i_tick) = bSpline_deBaCo('v', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('v', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U);
    end
    
    %-----acceleration constraint-----
    i = i + 1;      % ini accel, u_val = 0
    u_val = 0;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = -bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = A(i,2) - bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = A(i,2) + bSpline_deBaCo('a', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U);
    A(i, end-1) = bSpline_deBaCo('a', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U);
    A(i, end) = bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U);
    for i_tick = 3:size(A,1)-2
        A(i, i_tick) = bSpline_deBaCo('a', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('a', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('a', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('a', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U);
    end
    i = i + 1;      % end accel, u_val = 1
    u_val = 1;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = -bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = A(i,2) - bSpline_deBaCo('a', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U);
    A(i, 2) = A(i,2) + bSpline_deBaCo('a', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U);
    A(i, end-1) = bSpline_deBaCo('a', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U);
    A(i, end) = bSpline_deBaCo('a', n-1, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U);
    for i_tick = 3:size(A,1)-2
        A(i, i_tick) = bSpline_deBaCo('a', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('a', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('a', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('a', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U);
    end
    
    %-----jerk constraint-----
    i = i + 1;      % ini jerk, u_val = 0
    u_val = 0;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = -bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = -bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 3, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 3, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 3, properties, U) * bSpline_deCo('j', 3, properties, U);
    A(i, end-2) = bSpline_deBaCo('j', n-4, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-3, properties, U) * bSpline_deCo('j', n-4, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-3, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end) = bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    for i_tick = 4:size(A,1)-3
        A(i, i_tick) = bSpline_deBaCo('j', i_tick-3, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U) * bSpline_deCo('j', i_tick-3, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U) * bSpline_deCo('j', i_tick, properties, U);
    end
    i = i + 1;      % end jerk, u_val = 1
    u_val = 1;
        % co = bSpline_deBaCo(choice, num, val, properties, U)
    A(i, 1) = -bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 1, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 2) = A(i, 2) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = -bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 1, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 2, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 1, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 1, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 2, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) + bSpline_deBaCo('j', 2, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 3, properties, U) * bSpline_deCo('j', 2, properties, U);
    A(i, 3) = A(i, 3) - bSpline_deBaCo('j', 3, u_val, properties, U) * bSpline_deCo('v', 3, properties, U) * bSpline_deCo('a', 3, properties, U) * bSpline_deCo('j', 3, properties, U);
    A(i, end-2) = bSpline_deBaCo('j', n-4, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-3, properties, U) * bSpline_deCo('j', n-4, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-3, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-2, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-2) = A(i, end-2) - bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-2) = A(i, end-2) + bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = bSpline_deBaCo('j', n-3, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-3, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-2, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n-1, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end-1) = A(i, end-1) - bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    A(i, end) = bSpline_deBaCo('j', n-2, u_val, properties, U) * bSpline_deCo('v', n, properties, U) * bSpline_deCo('a', n-1, properties, U) * bSpline_deCo('j', n-2, properties, U);
    for i_tick = 4:size(A,1)-3
        A(i, i_tick) = bSpline_deBaCo('j', i_tick-3, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U) * bSpline_deCo('j', i_tick-3, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-2, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick-1, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick-2, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-2, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick-1, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) + bSpline_deBaCo('j', i_tick-1, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U) * bSpline_deCo('j', i_tick-1, properties, U);
        A(i, i_tick) = A(i, i_tick) - bSpline_deBaCo('j', i_tick, u_val, properties, U) * bSpline_deCo('v', i_tick, properties, U) * bSpline_deCo('a', i_tick, properties, U) * bSpline_deCo('j', i_tick, properties, U);
    end
    cP = linsolve(A,B);
end