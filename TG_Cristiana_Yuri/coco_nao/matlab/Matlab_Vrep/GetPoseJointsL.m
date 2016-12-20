function pose_joints = GetPoseJointsL(clientID, vrep, world_frame_handle, joints_handles_L)
%   GETPOSEJOINTS 

    n = length(joints_handles_L);
    for i=1:n
        pose_joints(i) = GetPose(clientID, vrep, world_frame_handle, joints_handles_L(i));    
    end

end