function [vrep clientID] = InitializeVREP

    % V-REP INITIALIZATION
    vrep = remApi('remoteApi'); % Defines the V-REP communication object
    vrep.simxFinish(-1); % Closes any existing communication link

    clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % Defines the Matl ab client for V-REP

    if clientID ~= -1
        disp('V-REP connected!');
    end
    vrep.simxSynchronousTrigger(clientID);

    [connectionId]=vrep.simxGetConnectionId(clientID);
    if connectionId==-1
            disp('Server disconnected!');
            return;
    end 

end