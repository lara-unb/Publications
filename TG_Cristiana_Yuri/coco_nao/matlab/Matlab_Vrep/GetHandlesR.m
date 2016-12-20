function handles = GetHandlesR(clientID, vrep)

    %Frames
    [~,world_frame_handle] = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25',vrep.simx_opmode_oneshot_wait); % World frame handle

    %NAO
    [~,NAO_handle] = vrep.simxGetObjectHandle(clientID, 'NAO',vrep.simx_opmode_oneshot_wait); % NAO handle

    
    %Joints
    [~,joints_handles(1)] = vrep.simxGetObjectHandle(clientID, 'RShoulderPitch3', vrep.simx_opmode_oneshot_wait); %Shoulder Pitch handle
    [~,joints_handles(2)] = vrep.simxGetObjectHandle(clientID, 'RShoulderRoll3', vrep.simx_opmode_oneshot_wait); %Shoulder Roll handle
    [~,joints_handles(3)] = vrep.simxGetObjectHandle(clientID, 'RElbowYaw3', vrep.simx_opmode_oneshot_wait); %Elbow Yaw handle
    [~,joints_handles(4)] = vrep.simxGetObjectHandle(clientID, 'RElbowRoll3', vrep.simx_opmode_oneshot_wait); %Elbow Roll handle
    [~,joints_handles(5)] = vrep.simxGetObjectHandle(clientID, 'RWristYaw3', vrep.simx_opmode_oneshot_wait); %Wrist Yaw handle

    %Objects
    [~,object_handle] = vrep.simxGetObjectHandle(clientID,'Cup', vrep.simx_opmode_oneshot_wait);

    %OPEN THE CHANNELS TO READ INFO LATER
    for i=1:5
        vrep.simxGetObjectPosition(clientID,joints_handles(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetObjectOrientation(clientID,joints_handles(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joints_handles(i),vrep.simx_opmode_streaming);
    end

    vrep.simxGetObjectPosition(clientID,object_handle,-1,vrep.simx_opmode_streaming);
    vrep.simxGetObjectOrientation(clientID,object_handle,-1,vrep.simx_opmode_streaming);

    %PREPARING OUTPUT
    handles.nao = NAO_handle;
    handles.worldFrame = world_frame_handle;
    handles.joints = joints_handles;
    handles.object = object_handle;

end