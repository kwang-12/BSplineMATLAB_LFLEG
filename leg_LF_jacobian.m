function J0_reduced_num = leg_LF_jacobian(theta, geometry)
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    d2 = geometry.d2;
    a2 = geometry.a2;
    a3 = geometry.a3;
    J0_reduced_row1_num = [-d2*cos(theta1)-a2*sin(theta1)*cos(theta2)-a3*sin(theta1)*cos(theta2+theta3), -cos(theta1)*(a2*sin(theta2)+a3*sin(theta2+theta3)), -a3*cos(theta1)*sin(theta2+theta3)];
    J0_reduced_row2_num = [-d2*sin(theta1)+a2*cos(theta1)*cos(theta2)+a3*cos(theta1)*cos(theta2+theta3), -sin(theta1)*(a2*sin(theta2)+a3*sin(theta2+theta3)), -a3*sin(theta1)*sin(theta2+theta3)];
    J0_reduced_row3_num = [0, -a2*cos(theta2)-a3*cos(theta2+theta3), -a3*cos(theta2+theta3)];
    J0_reduced_num = [J0_reduced_row1_num; J0_reduced_row2_num; J0_reduced_row3_num];
end