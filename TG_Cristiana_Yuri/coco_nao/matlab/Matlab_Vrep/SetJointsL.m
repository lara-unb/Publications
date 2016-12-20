
function SetJointsL(clientID, vrep, joints_handles_L, q)
%   SETJOINTS 

    n = length(joints_handles_L);
    for i=1:n
        rtnL = vrep.simxSetJointPosition(clientID,joints_handles_L(i),q(i),vrep.simx_opmode_continuous );
    end

end