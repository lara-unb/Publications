function SetJointsR(clientID, vrep, joints_handles_R, q)
%   SETJOINTS 

    n = length(joints_handles_R);
    for i=1:n
        rtnR = vrep.simxSetJointPosition(clientID,joints_handles_R(i),q(i),vrep.simx_opmode_continuous );
    end

end
