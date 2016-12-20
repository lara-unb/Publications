function curr_joints = GetJointsL(clientID, vrep, joints_handles)
%   GETJOINTS 

% LEFT ARM
curr_joints(1) =  GetCoordJoint(clientID, vrep, joints_handles(1));
curr_joints(2) =  GetCoordJoint(clientID, vrep, joints_handles(2));
curr_joints(3) =  GetCoordJoint(clientID, vrep, joints_handles(3));
curr_joints(4) =  GetCoordJoint(clientID, vrep, joints_handles(4));
curr_joints(5) =  GetCoordJoint(clientID, vrep, joints_handles(5));

curr_joints = curr_joints';
end
