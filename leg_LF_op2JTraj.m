function jtTraj = leg_LF_op2JTraj(opTraj, LF)
    jtTraj(1,:) = leg_LF_iK(LF.iniJointAngle, opTraj(1,2:end), LF.geometry);
    for count = 2:size(opTraj,1)
        jtTraj(count,:) = leg_LF_iK(jtTraj(count-1,:), opTraj(count,2:end), LF.geometry);
    end
    jtTraj = [opTraj(:,1),jtTraj];
end