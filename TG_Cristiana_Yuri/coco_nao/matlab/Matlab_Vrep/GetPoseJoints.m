function pose_joints = GetPoseJoints(clientID, vrep, world_frame_handle, joints_handles)
%   GETPOSEJOINTS 

    n = length(joints_handles);
    for i=1:n
        pose_joints(i) = GetPose(clientID, vrep, world_frame_handle, joints_handles(i));    
    end

end