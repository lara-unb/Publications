%Separate controll for both left and right arms

close all;
clear all;
clc;
warning off;

curr_path = pwd;
DQ_lib_path = [curr_path '/DQ_Robotics']; 
addpath(DQ_lib_path);


% V-REP INITIALIZATION
[vrep clientID] = InitializeVREP;

% LOAD HANDLES
%Handles of scene objects
handles = GetHandles(clientID, vrep);

%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.jointsR, [0 0 0 0 0]);
SetJoints(clientID, vrep, handles.jointsL, [0 0 0 0 0]);

% ROBOT MODEL

%Model
naoL = BuildModelL;
naoR = BuildModelR;

%Joint mean values
j_mean_l = [0 0.50615 0 -0.78975 0]; %Esquerdo
j_mean_r = [0 -0.50615 0 0.78975 0]; %Direito

%Set robot base to the Chest
nao_pose = GetPose(clientID, vrep, handles.worldFrame, handles.nao);

naoR.set_base(nao_pose);
naoL.set_base(nao_pose);

% KINEMATIC CONTROL

%GO TO OBJECT AND GRAB

%Object's pose
obj_pose = GetPose(clientID, vrep, handles.worldFrame, handles.object);
qL = GetJoints(clientID, vrep, handles.jointsL);
qR = GetJoints(clientID, vrep, handles.jointsR);
%obj_pose = nao.fkm(q);

%Shift the pose to fit the gripper on the object
x_ref = obj_pose;

%Maximum tolerance
epsilon = 1e-2;

%Gain
K = 0.5;
err_tR = 1e3;
err_tL = 1e3;

err_saveR = [];
err_saveL = [];

joint_saveR = [];
joint_saveL = [];

err_norm = [];

while norm(abs(err_tR))>epsilon
    
    %Verify if V-rep is connected
    [connectionId]=vrep.simxGetConnectionId(clientID);
    if connectionId==-1
            disp('Server disconnected!');
            return;
    end  
    
    %Current joints
    qR = GetJoints(clientID, vrep, handles.jointsR);
    qL = GetJoints(clientID, vrep, handles.jointsL);
    
    joint_saveR = [joint_saveR qR];
    joint_saveL = [joint_saveL qL];
    
    %Current pose
    x_currR = naoR.fkm(qR);
    x_currL = naoL.fkm(qL);

     %Jacobian
    JR = naoR.jacobian(qR);
    JTR = naoR.jacobp(JR, x_currR); %jacobian of the translation only 
   
    JL = naoL.jacobian(qL);
    JTL = naoL.jacobp(JL, x_currL);
    
    %Error
    err_tR = vec4(x_ref.translation-x_currR.translation); %error of the translation only
    err_tL = vec4(x_ref.translation-x_currL.translation);
    
    err_saveR = [err_saveR err_tR];
    err_saveL = [err_saveL err_tL];

    err_norm = [err_norm norm(err_tR)];
    err_norm = [err_norm norm(err_tL)];

    %damped pinv
    lambda = 10e-3;
    pseudo_inv_jacob_dampedR = transpose(JTR)*inv(JTR*transpose(JTR) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    pseudo_inv_jacob_dampedL = transpose(JTL)*inv(JTL*transpose(JTL) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    
    %nullspace descriptor function
    s_l_ = 0.0;
    s_barl = 0.0;
    Jsl = [];
    for i = 1:5
        tmpl = qL(i, 1) - j_mean_l(i);
        s_l_ = s_l_ + 1/2 * tmpl^2;
        s_barl = s_barl + tmpl; % s_ = (θ1 - θ1m) + (θ2 - θ2m)...
        Jsl = [Jsl tmpl];       % Js = ((θ1 - θ1m), (θ2 - θ2m), ...)
    end
    
    s_r_ = 0.0;
    s_barr = 0.0;
    Jsr = [];
    for i = 1:5
        tmpr = qR(i, 1) - j_mean_r(i);
        s_r_ = s_r_ + 1/2 * tmpr^2;
        s_barr = s_barr + tmpr; % s_ = (θ1 - θ1m) + (θ2 - θ2m)...
        Jsr = [Jsr tmpr];       % Js = ((θ1 - θ1m), (θ2 - θ2m), ...)
    end

    %nullspace calculation 
    PiR = (eye(5) - pseudo_inv_jacob_dampedR*JTR);
    pseudo_inv_null_dampedR = (Jsr*PiR)'*inv(Jsr*PiR * (Jsr*PiR)' + lambda); 
    GAIN_S = 1;
    nullspaceR = -pseudo_inv_null_dampedR * (GAIN_S*s_r_ - Jsr*pseudo_inv_jacob_dampedR*K*err_tR);
    
    PiL = (eye(5) - pseudo_inv_jacob_dampedL*JTL);
    pseudo_inv_null_dampedL = (Jsl*PiL)'*inv(Jsl*PiL * (Jsl*PiL)' + lambda); 
    GAIN_S = 1;
    nullspaceL = -pseudo_inv_null_dampedL * (GAIN_S*s_l_ - Jsl*pseudo_inv_jacob_dampedL*K*err_tL);  
    
    %Kinematic control law
    q_pontoR = pseudo_inv_jacob_dampedR*K*err_tR + 0.25*PiR*nullspaceR;
    qR = qR+q_pontoR;
    
    q_pontoL = pseudo_inv_jacob_dampedL*K*err_tL + 0.25*PiL*nullspaceL;
    qL = qL+q_pontoL;
    
    %Send joints values
    SetJoints(clientID, vrep, handles.jointsR, qR);
    SetJoints(clientID, vrep, handles.jointsL, qL);
end

    
    
    


    
    
    
