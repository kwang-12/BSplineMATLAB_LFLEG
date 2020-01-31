close all
clear all
clc

% 1. TODO: EXPERIMENT WITH HIGHER DEGREE BSPLINE WITH MORE KINEMATIC
% CONSTRAINTS
% 7 DOF BSPLINE -> 6 CONSTRAINTS
% 9 DOF BSPLINE -> 8 CONSTRAINTS
% 2. TODO: Modify the bSpline_calcCP function to allow flexible kinematic
% constraint settings.

% Define:
% LF leg object: 
% units: meters and radians
LF.iniJointAngle.theta1 = 0/180*pi;
LF.iniJointAngle.theta2 = -45/180*pi;
LF.iniJointAngle.theta3 = 90/180*pi;
LF.iniJointAngle = [LF.iniJointAngle.theta1, LF.iniJointAngle.theta2, LF.iniJointAngle.theta3];
LF.geometry.d2 = 0.092;
LF.geometry.a2 = 0.218;
LF.geometry.a3 = 0.230;

% Input the desired operational space trajectory
opTrajSettings.timeStep = 0.05;
opTrajSettings.iniTime = 0;
opTrajSettings.endTime = 1;
opTrajSettings.timeVector = opTrajSettings.iniTime : opTrajSettings.timeStep : opTrajSettings.endTime;
% opTrajSettings.timeVector = 0:0.05:1;
opTrajSettings.xVector = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.33, 0.318, 0.31, 0.31, 0.31, 0.318, 0.33, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35];
opTrajSettings.yVector = ones(1,length(opTrajSettings.xVector))*LF.geometry.d2;
opTrajSettings.zVector = [0.00,-0.02,-0.04,-0.06,-0.08,-0.10,-0.12,-0.13,-0.106,-0.06, 0.00, 0.06, 0.106, 0.13, 0.12, 0.10, 0.08, 0.06, 0.04, 0.02, 0.00];

% opTrajSettings.xVector = [0.35,  0.35,  0.35,  0.35,  0.35,  0.35,  0.35,  0.334, 0.314, 0.2980, 0.2920,  0.290,  0.290,  0.290,  0.290, 0.2920, 0.2980, 0.314,  0.334,  0.35, 0.35, 0.35, 0.35, 0.35,  0.35, 0.35];
% opTrajSettings.yVector = ones(1,length(opTrajSettings.xVector))*LF.geometry.d2;
% opTrajSettings.zVector = [0.00,-0.016,-0.032,-0.048,-0.064,-0.080,-0.096,-0.1045,-0.103,-0.0882,-0.0650,-0.0390,-0.0130, 0.0130, 0.0390, 0.0650, 0.0882, 0.103, 0.1045, 0.096,0.080,0.064,0.048,0.032, 0.016, 0.00];

opTrajSettings.iniVelo = [(opTrajSettings.xVector(2)-opTrajSettings.xVector(1))/opTrajSettings.timeStep];
opTrajSettings.iniVelo = [opTrajSettings.iniVelo; (opTrajSettings.yVector(2)-opTrajSettings.yVector(1))/opTrajSettings.timeStep];
opTrajSettings.iniVelo = [opTrajSettings.iniVelo; (opTrajSettings.zVector(2)-opTrajSettings.zVector(1))/opTrajSettings.timeStep];
opTrajSettings.endVelo = [(opTrajSettings.xVector(end)-opTrajSettings.xVector(end-1))/opTrajSettings.timeStep];
opTrajSettings.endVelo = [opTrajSettings.endVelo; (opTrajSettings.yVector(end)-opTrajSettings.yVector(end-1))/opTrajSettings.timeStep];
opTrajSettings.endVelo = [opTrajSettings.endVelo; (opTrajSettings.zVector(end)-opTrajSettings.zVector(end-1))/opTrajSettings.timeStep];
opTraj.traj = [opTrajSettings.timeVector', opTrajSettings.xVector', opTrajSettings.yVector', opTrajSettings.zVector'];
opTraj.constraint = [opTrajSettings.iniVelo, opTrajSettings.endVelo];

% Solve the inverse kinematics of the operational space trajectory,
% calculate the corresponding joint space trajectory and ini/end velocities
jtTraj.traj = leg_LF_op2JTraj(opTraj.traj, LF);
jtTraj.iniJointVelo = inv(leg_LF_jacobian(leg_LF_iK(LF.iniJointAngle, opTraj.traj(1,2:end), LF.geometry), LF.geometry));
jtTraj.iniJointVelo = jtTraj.iniJointVelo * opTraj.constraint(:,1);
jtTraj.endJointVelo = inv(leg_LF_jacobian(leg_LF_iK(LF.iniJointAngle, opTraj.traj(end,2:end), LF.geometry), LF.geometry));
jtTraj.endJointVelo = jtTraj.endJointVelo * opTraj.constraint(:,2);
jtTraj.jointVelo_constraint = [jtTraj.iniJointVelo, jtTraj.endJointVelo];

% Generate smooth joint space trajectory using B-spline
bSpline.properties.k = 7;
bSpline.properties.n =size(jtTraj.traj,1)-1;
bSpline.properties.delta_time = zeros(size(jtTraj.traj(:,1),1)-1,1);
for tick = 1:size(jtTraj.traj(:,1),1)-1
    bSpline.properties.delta_time(tick) = jtTraj.traj(tick+1,1) - jtTraj.traj(tick,1);
end
fprintf('%i degree B-splines to interpolate %i points.\n', bSpline.properties.k, bSpline.properties.n+1);
bSpline.U = bSpline_KnotVector(bSpline.properties);
bSpline.time = min(opTrajSettings.timeVector):0.001:max(opTrajSettings.timeVector);
bSpline.properties.m = size(bSpline.U,2)-1;
bSpline.properties.n = bSpline.properties.m - bSpline.properties.k - 1;
bSpline.properties_velo.n = bSpline.properties.n - 1;
bSpline.properties_velo.k = bSpline.properties.k - 1;
bSpline.properties_accel.n = bSpline.properties.n - 2;
bSpline.properties_accel.k = bSpline.properties.k - 2;
bSpline.properties_jerk.n = bSpline.properties.n - 3;
bSpline.properties_jerk.k = bSpline.properties.k - 3;
bSpline.AB.CP = bSpline_calcCP(jtTraj.traj(:,2), bSpline.U, jtTraj.jointVelo_constraint(1,:)', [0; 0], [0; 0], bSpline.properties);
bSpline.AB.CP_velo = bSpline_calcDeCP_1st(bSpline.AB.CP, bSpline.properties, bSpline.U);
bSpline.AB.CP_accel = bSpline_calcDeCP_1st(bSpline.AB.CP_velo, bSpline.properties_velo, bSpline.U(2:end-1));
bSpline.AB.CP_jerk = bSpline_calcDeCP_1st(bSpline.AB.CP_accel, bSpline.properties_accel, bSpline.U(3:end-2));
bSpline.HIP.CP = bSpline_calcCP(jtTraj.traj(:,3), bSpline.U, jtTraj.jointVelo_constraint(2,:)', [0; 0], [0; 0], bSpline.properties);
bSpline.HIP.CP_velo = bSpline_calcDeCP_1st(bSpline.HIP.CP, bSpline.properties, bSpline.U);
bSpline.HIP.CP_accel = bSpline_calcDeCP_1st(bSpline.HIP.CP_velo, bSpline.properties_velo, bSpline.U(2:end-1));
bSpline.HIP.CP_jerk = bSpline_calcDeCP_1st(bSpline.HIP.CP_accel, bSpline.properties_accel, bSpline.U(3:end-2));
bSpline.KNEE.CP = bSpline_calcCP(jtTraj.traj(:,4), bSpline.U, jtTraj.jointVelo_constraint(3,:)', [0; 0], [0; 0], bSpline.properties);
bSpline.KNEE.CP_velo = bSpline_calcDeCP_1st(bSpline.KNEE.CP, bSpline.properties, bSpline.U);
bSpline.KNEE.CP_accel = bSpline_calcDeCP_1st(bSpline.KNEE.CP_velo, bSpline.properties_velo, bSpline.U(2:end-1));
bSpline.KNEE.CP_jerk = bSpline_calcDeCP_1st(bSpline.KNEE.CP_accel, bSpline.properties_accel, bSpline.U(3:end-2));
for count = 1:size(bSpline.time,2)
    %AB_curve
    bSpline.AB.curve(count) = bSpline_curve(bSpline.AB.CP, bSpline.time(count), bSpline.U, bSpline.properties);
    bSpline.AB.curve_velo(count) = bSpline_curve(bSpline.AB.CP_velo, bSpline.time(count), bSpline.U(2:end-1), bSpline.properties_velo);
    bSpline.AB.curve_accel(count) = bSpline_curve(bSpline.AB.CP_accel, bSpline.time(count), bSpline.U(3:end-2), bSpline.properties_accel);
    bSpline.AB.curve_jerk(count) = bSpline_curve(bSpline.AB.CP_jerk, bSpline.time(count), bSpline.U(4:end-3), bSpline.properties_jerk);
    %HIP_curve
    bSpline.HIP.curve(count) = bSpline_curve(bSpline.HIP.CP, bSpline.time(count), bSpline.U, bSpline.properties);
    bSpline.HIP.curve_velo(count) = bSpline_curve(bSpline.HIP.CP_velo, bSpline.time(count), bSpline.U(2:end-1), bSpline.properties_velo);
    bSpline.HIP.curve_accel(count) = bSpline_curve(bSpline.HIP.CP_accel, bSpline.time(count), bSpline.U(3:end-2), bSpline.properties_accel);
    bSpline.HIP.curve_jerk(count) = bSpline_curve(bSpline.HIP.CP_jerk, bSpline.time(count), bSpline.U(4:end-3), bSpline.properties_jerk);
    %KNEE_curve
    bSpline.KNEE.curve(count) = bSpline_curve(bSpline.KNEE.CP, bSpline.time(count), bSpline.U, bSpline.properties);
    bSpline.KNEE.curve_velo(count) = bSpline_curve(bSpline.KNEE.CP_velo, bSpline.time(count), bSpline.U(2:end-1), bSpline.properties_velo);
    bSpline.KNEE.curve_accel(count) = bSpline_curve(bSpline.KNEE.CP_accel, bSpline.time(count), bSpline.U(3:end-2), bSpline.properties_accel);
    bSpline.KNEE.curve_jerk(count) = bSpline_curve(bSpline.KNEE.CP_jerk, bSpline.time(count), bSpline.U(4:end-3), bSpline.properties_jerk);
end

% % % % % % % % inv(leg_LF_jacobian(jtTraj.traj(11,2:end), LF.geometry))*[0,0,0.06/0.1]'/pi*180

% Calculate end effector velocity using BSpline joint space trajectory
for count = 1:size(bSpline.time,2)
    opTraj.bSplineTraj(count,:) = leg_LF_jacobian([bSpline.AB.curve(count), bSpline.HIP.curve(count), bSpline.KNEE.curve(count)], LF.geometry) * [bSpline.AB.curve_velo(count), bSpline.HIP.curve_velo(count), bSpline.KNEE.curve_velo(count)]';
    opTraj.bSplineSpeed(count) = sqrt(opTraj.bSplineTraj(count,1)^2 + opTraj.bSplineTraj(count,2)^2 + opTraj.bSplineTraj(count,3)^2);
end

opTraj.settingSpeedX = zeros(size(opTrajSettings.timeVector));
opTraj.settingSpeedY = zeros(size(opTrajSettings.timeVector));
opTraj.settingSpeedZ = zeros(size(opTrajSettings.timeVector));
opTraj.settingSpeed = zeros(size(opTrajSettings.timeVector));
opTraj.settingSpeedX(1) = opTrajSettings.iniVelo(1);
opTraj.settingSpeedY(1) = opTrajSettings.iniVelo(2);
opTraj.settingSpeedZ(1) = opTrajSettings.iniVelo(3);
opTraj.settingSpeed(1) = sqrt(opTraj.settingSpeedX(1)^2 + opTraj.settingSpeedY(1)^2 +opTraj.settingSpeedZ(1)^2);
opTraj.settingSpeedX(end) = opTrajSettings.endVelo(1);
opTraj.settingSpeedY(end) = opTrajSettings.endVelo(2);
opTraj.settingSpeedZ(end) = opTrajSettings.endVelo(3);
opTraj.settingSpeed(end) = sqrt(opTraj.settingSpeedX(end)^2 + opTraj.settingSpeedY(end)^2 +opTraj.settingSpeedZ(end)^2);

for count = 2:size(opTrajSettings.timeVector,2)-1
    opTraj.settingSpeedX(count) = (opTrajSettings.xVector(count) - opTrajSettings.xVector(count-1))/(opTrajSettings.timeVector(count)-opTrajSettings.timeVector(count-1));
    opTraj.settingSpeedY(count) = (opTrajSettings.yVector(count) - opTrajSettings.yVector(count-1))/(opTrajSettings.timeVector(count)-opTrajSettings.timeVector(count-1));
    opTraj.settingSpeedZ(count) = (opTrajSettings.zVector(count) - opTrajSettings.zVector(count-1))/(opTrajSettings.timeVector(count)-opTrajSettings.timeVector(count-1));
    opTraj.settingSpeed(count) = sqrt(opTraj.settingSpeedX(count)^2 + opTraj.settingSpeedY(count)^2 + opTraj.settingSpeedZ(count)^2);
end
%% Visualization
figure(1)
plot(opTrajSettings.zVector, -opTrajSettings.xVector, '-o', 'MarkerSize', 8);
hold on
bSpline_FK_traj = zeros(size(bSpline.time,2),3);
for count = 1:size(bSpline.time,2)
    bSpline_FK_traj(count,:) = leg_LF_fK([bSpline.AB.curve(count), bSpline.HIP.curve(count), bSpline.KNEE.curve(count)], LF.geometry);
end
plot(bSpline_FK_traj(:,3), -bSpline_FK_traj(:,1), 'LineWidth', 3)
title('op-space traj, x-z plane')
xlabel('x')
ylabel('z')
xlim([-0.2, 0.2])
ylim([-0.4, 0])
grid on


figure(2)
plot(jtTraj.traj(:,1),jtTraj.traj(:,2)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'r')
hold on
plot(jtTraj.traj(:,1),jtTraj.traj(:,3)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'g')
plot(jtTraj.traj(:,1),jtTraj.traj(:,4)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'b')
plot(bSpline.time, bSpline.AB.curve/pi*180, 'LineWidth', 3, 'Color', 'r')
plot(bSpline.time, bSpline.HIP.curve/pi*180, 'LineWidth', 3, 'Color', 'g')
plot(bSpline.time, bSpline.KNEE.curve/pi*180, 'LineWidth', 3, 'Color', 'b')
legend('AB-operaPoints','HIP-operaPoints','KNEE-operaPoints', 'AB-bSpline', 'HIP-bSpline', 'KNEE-bSpline', 'Location', 'Best')
grid on
xlabel('time [sec]')
ylabel('degree')
title('jt-space traj')

visual_time_ticks = findTimeTick(jtTraj.traj(:,1), bSpline.time);

figure(3)
plot(bSpline.time, bSpline.AB.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'r')
hold on
plot(bSpline.time, bSpline.HIP.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'g')
plot(bSpline.time, bSpline.KNEE.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'b')
plot(bSpline.time(visual_time_ticks), bSpline.AB.curve_velo(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'r')
plot(bSpline.time(visual_time_ticks), bSpline.HIP.curve_velo(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'g')
plot(bSpline.time(visual_time_ticks), bSpline.KNEE.curve_velo(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'b')
legend('AB-bSpline', 'HIP-bSpline', 'KNEE-bSpline', 'Location', 'Best')
grid on
xlabel('time [sec]')
ylabel('degree/sec')
title('joint velo')

figure(4)
plot(bSpline.time, bSpline.AB.curve_accel/pi*180, 'LineWidth', 3, 'Color', 'r')
hold on
plot(bSpline.time, bSpline.HIP.curve_accel/pi*180, 'LineWidth', 3, 'Color', 'g')
plot(bSpline.time, bSpline.KNEE.curve_accel/pi*180, 'LineWidth', 3, 'Color', 'b')
plot(bSpline.time(visual_time_ticks), bSpline.AB.curve_accel(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'r')
plot(bSpline.time(visual_time_ticks), bSpline.HIP.curve_accel(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'g')
plot(bSpline.time(visual_time_ticks), bSpline.KNEE.curve_accel(visual_time_ticks)/pi*180, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'Color', 'b')
legend('AB-bSpline', 'HIP-bSpline', 'KNEE-bSpline', 'Location', 'Best')
grid on
xlabel('time [sec]')
ylabel('degree/sec2')
title('joint accel')

figure(5)
plot(bSpline.time, bSpline.AB.curve_jerk/pi*180, 'LineWidth', 3, 'Color', 'r')
hold on
plot(bSpline.time, bSpline.HIP.curve_jerk/pi*180, 'LineWidth', 3, 'Color', 'g')
plot(bSpline.time, bSpline.KNEE.curve_jerk/pi*180, 'LineWidth', 3, 'Color', 'b')
legend('AB-bSpline', 'HIP-bSpline', 'KNEE-bSpline', 'Location', 'Best')
grid on
xlabel('time [sec]')
ylabel('degree/sec3')
title('joint jerk')

% figure(6)
% plot(bSpline.time, bSpline.AB.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'r')
% hold on
% plot(bSpline.time, bSpline.AB.curve_accel/pi*180 * (max(bSpline.AB.curve_velo)/max(bSpline.AB.curve_accel)), 'LineStyle','--','LineWidth', 3, 'Color', 'r')
% grid on
% xlabel('time [sec]')
% 
% figure(7)
% plot(bSpline.time, bSpline.HIP.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'g')
% hold on
% plot(bSpline.time, bSpline.HIP.curve_accel/pi*180 * (max(bSpline.HIP.curve_velo)/max(bSpline.HIP.curve_accel)), 'LineStyle','--','LineWidth', 3, 'Color', 'g')
% grid on
% xlabel('time [sec]')
% 
% figure(8)
% plot(bSpline.time, bSpline.KNEE.curve_velo/pi*180, 'LineWidth', 3, 'Color', 'b')
% hold on
% plot(bSpline.time, bSpline.KNEE.curve_accel/pi*180 * (max(bSpline.KNEE.curve_velo)/max(bSpline.KNEE.curve_accel)), 'LineStyle','--','LineWidth', 3, 'Color', 'b')
% grid on
% xlabel('time [sec]')

figure(9)
plot(bSpline.time, opTraj.bSplineTraj, 'LineWidth', 3)
legend('x','y','z')
grid on
xlabel('time [sec]')
ylabel('m/sec')

figure(10)
plot(bSpline.time, opTraj.bSplineSpeed, 'LineWidth', 3)
hold on
plot(opTrajSettings.timeVector, opTraj.settingSpeed, 'LineWidth', 3)
grid on
xlabel('time [sec]')
ylabel('m/sec')

%% Transfer generated joint trajectory into useable numbers
clear knee_pos knee_vel hip_pos hip_vel
send_time_interval = 5;%(7000 micro-second)
knee_offset = 0;
knee_multiplier = 20/16;    %gear_ratio
hip_offset = -5900;
hip_multiplier = -1;
knee_pos = round(knee_multiplier*(bSpline.KNEE.curve(1:send_time_interval:length(bSpline.KNEE.curve))/pi*180/360*2^14*11-knee_offset));
knee_vel = round(knee_multiplier*bSpline.KNEE.curve_velo(1:send_time_interval:length(bSpline.KNEE.curve_velo))/pi*180/360*2^14*11);
hip_pos = round(hip_multiplier*(bSpline.HIP.curve(1:send_time_interval:length(bSpline.HIP.curve))/pi*180/360*2^14*11-hip_offset));
hip_vel = round(hip_multiplier*bSpline.HIP.curve_velo(1:send_time_interval:length(bSpline.HIP.curve_velo))/pi*180/360*2^14*11);

