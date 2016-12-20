function handles = GetHandlesRight(clientID, vrep)

    %Frames
    [~,world_frame_handle] = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25',vrep.simx_opmode_oneshot_wait); % World frame handle

    %NAO
    [~,NAO_handle] = vrep.simxGetObjectHandle(clientID, 'NAO',vrep.simx_opmode_oneshot_wait); % NAO handle

    
    %Joints Right
    [~,joints_handles_R(1)] = vrep.simxGetObjectHandle(clientID, 'RShoulderPitch3', vrep.simx_opmode_oneshot_wait); %Shoulder Pitch handle
    [~,joints_handles_R(2)] = vrep.simxGetObjectHandle(clientID, 'RShoulderRoll3', vrep.simx_opmode_oneshot_wait); %Shoulder Roll handle
    [~,joints_handles_R(3)] = vrep.simxGetObjectHandle(clientID, 'RElbowYaw3', vrep.simx_opmode_oneshot_wait); %Elbow Yaw handle
    [~,joints_handles_R(4)] = vrep.simxGetObjectHandle(clientID, 'RElbowRoll3', vrep.simx_opmode_oneshot_wait); %Elbow Roll handle
    [~,joints_handles_R(5)] = vrep.simxGetObjectHandle(clientID, 'RWristYaw3', vrep.simx_opmode_oneshot_wait); %Wrist Yaw handle
    
     %Joints Left
    [~,joints_handles_L(1)] = vrep.simxGetObjectHandle(clientID, 'LShoulderPitch3', vrep.simx_opmode_oneshot_wait); %Shoulder Pitch handle
    [~,joints_handles_L(2)] = vrep.simxGetObjectHandle(clientID, 'LShoulderRoll3', vrep.simx_opmode_oneshot_wait); %Shoulder Roll handle
    [~,joints_handles_L(3)] = vrep.simxGetObjectHandle(clientID, 'LElbowYaw3', vrep.simx_opmode_oneshot_wait); %Elbow Yaw handle
    [~,joints_handles_L(4)] = vrep.simxGetObjectHandle(clientID, 'LElbowRoll3', vrep.simx_opmode_oneshot_wait); %Elbow Roll handle
    [~,joints_handles_L(5)] = vrep.simxGetObjectHandle(clientID, 'LWristYaw3', vrep.simx_opmode_oneshot_wait); %Wrist Yaw handle


    %Objects
    [~,object_handle] = vrep.simxGetObjectHandle(clientID,'Cup', vrep.simx_opmode_oneshot_wait);

    %OPEN THE CHANNELS TO READ INFO LATER
    for i=1:5
        vrep.simxGetObjectPosition(clientID,joints_handles_R(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetObjectOrientation(clientID,joints_handles_R(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joints_handles_R(i),vrep.simx_opmode_streaming);
    end
        for i=1:5
        vrep.simxGetObjectPosition(clientID,joints_handles_L(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetObjectOrientation(clientID,joints_handles_L(i),-1,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joints_handles_L(i),vrep.simx_opmode_streaming);
    end

    vrep.simxGetObjectPosition(clientID,object_handle,-1,vrep.simx_opmode_streaming);
    vrep.simxGetObjectOrientation(clientID,object_handle,-1,vrep.simx_opmode_streaming);

    %PREPARING OUTPUT
    handles.nao = NAO_handle;
    handles.worldFrame = world_frame_handle;
    handles.jointsR = joints_handles_R;
    handles.jointsL = joints_handles_L;
    handles.object = object_handle;

end