function pose = GetPose(clientID, vrep, world_frame_handle, handle)
%   GETPOSE 

%CLEAR OUTPUT
pose = [];

%GET POSITION
rtn=1;
while rtn == vrep.simx_return_novalue_flag
    [rtn,pos] = vrep.simxGetObjectPosition(clientID,handle,world_frame_handle,vrep.simx_opmode_streaming);
    [connectionId]=vrep.simxGetConnectionId(clientID);
%Verify if server is connected
    if connectionId==-1
        disp('Server disconnected!');
        return;
    end  
end
pos = double(pos);

%GET ORIENTATION
rtn=1;
while rtn == vrep.simx_return_novalue_flag
    [rtn,ori] = vrep.simxGetObjectOrientation(clientID,handle,world_frame_handle,vrep.simx_opmode_streaming);
    [connectionId]=vrep.simxGetConnectionId(clientID);
%Verify if server is connected
    if connectionId==-1
        disp('Server disconnected!');
        return;
    end  
end
ori = double(ori);

%DUAL QUATERNION REPRESENTING THE POSE
r1 = DQ([cos((ori(1))/2) sin((ori(1))/2) 0 0]);
r2 = DQ([cos((ori(2))/2) 0 sin((ori(2))/2) 0]);
r3 = DQ([cos((ori(3))/2) 0 0 sin((ori(3))/2)]);
r = r1*r2*r3;
t = DQ([0 pos]);
pose = r+0.5*DQ.E*t*r;

end