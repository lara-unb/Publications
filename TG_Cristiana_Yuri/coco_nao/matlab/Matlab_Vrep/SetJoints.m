function SetJoints(clientID, vrep, joints_handles, q)
%   SETJOINTS 

    n = length(joints_handles);
    for i=1:n
        rtn = vrep.simxSetJointPosition(clientID,joints_handles(i),q(i),vrep.simx_opmode_continuous );
    end

end