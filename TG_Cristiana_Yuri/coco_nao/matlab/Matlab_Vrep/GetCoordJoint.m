function q = GetCoordJoint(clientID, vrep, handle)
%   GETCOORDJOINT 

[rtn,q] = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_streaming);
q = double(q);

end