function coord = leg_LF_fK(jointAngle, dimensions)
    theta1 = jointAngle(1);
    theta2 = jointAngle(2);
    theta3 = jointAngle(3);
    d2 = dimensions.d2;
    a2 = dimensions.a2;
    a3 = dimensions.a3;
    T04_x = -d2 * sin(theta1) + cos(theta1)*(a3*cos(theta2+theta3) + a2*cos(theta2));
    T04_y = d2 * cos(theta1) + sin(theta1)*(a3*cos(theta2+theta3) + a2*cos(theta2));
    T04_z = -a3*sin(theta2+theta3) - a2*sin(theta2);
    coord = [T04_x, T04_y, T04_z];
end