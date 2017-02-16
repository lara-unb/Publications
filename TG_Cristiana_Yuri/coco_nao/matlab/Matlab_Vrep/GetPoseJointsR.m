function pose_joints = GetPoseJointsR(clientID, vrep, world_frame_handle, joints_handles_R)
%   GETPOSEJOINTS 

    n = length(joints_handles_R);
    for i=1:n
        pose_joints(i) = GetPose(clientID, vrep, world_frame_handle, joints_handles_R(i));    
    end

end