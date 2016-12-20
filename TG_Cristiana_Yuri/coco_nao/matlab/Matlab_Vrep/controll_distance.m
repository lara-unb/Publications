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

%Maximum tolerance
epsilon = 1e-3;

%Gain
K = 0.5;
err_dR = 1e3;
err_dL = 1e3;

joint_saveR = [];
joint_saveL = [];

err_savedR = [];
err_savedL = [];

while norm(abs(err_dR))>epsilon
    
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
    %Right Arm Jacobian
    JR = naoR.jacobian(qR);
    JTR = naoR.jacobp(JR, x_currR); %jacobian of the translation only 
    JDR = -2*(vec4(x_currL.translation - x_currR.translation))'*JTR; %jacobian of the distance only 
   
    %Left Arm Jacobian
    JL = naoL.jacobian(qL);
    JTL = naoL.jacobp(JL, x_currL);
    JDL = -2*(vec4(x_currR.translation - x_currL.translation))'*JTL;
    
    %Error
    err_dR = norm(vec4(x_currL.translation - x_currR.translation)) - 0.15;
    err_dL = norm(vec4(x_currR.translation - x_currL.translation)) - 0.15;

    err_savedR = [err_savedR err_dR];   
    err_savedL = [err_savedL err_dL]; 
    
    %damped pinv
    lambda = 10e-3;
    pseudo_inv_jacob_dampedR = transpose(JDR)*inv(JDR*transpose(JDR) + lambda*eye(1)); % JT'(JT*JT' + λI)^-1
    pseudo_inv_jacob_dampedL = transpose(JDL)*inv(JDL*transpose(JDL) + lambda*eye(1)); % JT'(JT*JT' + λI)^-1
    
    %nullspace descriptor function
    %Left Arm
    s_l_ = 0.0;
    s_barl = 0.0;
    Jsl = [];
    for i = 1:5
        tmpl = qL(i, 1) - j_mean_l(i);
        s_l_ = s_l_ + 1/2 * tmpl^2;
        s_barl = s_barl + tmpl; % s_ = (θ1 - θ1m) + (θ2 - θ2m)...
        Jsl = [Jsl tmpl];       % Js = ((θ1 - θ1m), (θ2 - θ2m), ...)
    end
    %Right Arm
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
    %Right Arm
    PiR = (eye(5) - pseudo_inv_jacob_dampedR*JDR);
    pseudo_inv_null_dampedR = (Jsr*PiR)'*inv(Jsr*PiR * (Jsr*PiR)' + lambda);
    GAIN_S = 1;
    nullspaceR = -pseudo_inv_null_dampedR * (GAIN_S*s_r_ - Jsr*pseudo_inv_jacob_dampedR*K*err_dR);
    
    %Left Arm
    PiL = (eye(5) - pseudo_inv_jacob_dampedL*JDL);
    pseudo_inv_null_dampedL = (Jsl*PiL)'*inv(Jsl*PiL * (Jsl*PiL)' + lambda); 
    GAIN_S = 1;
    nullspaceL = -pseudo_inv_null_dampedL * (GAIN_S*s_l_ - Jsl*pseudo_inv_jacob_dampedL*K*err_dL);
      
    %Kinematic control law
    q_pontoR = -pseudo_inv_jacob_dampedR*K*err_dR + 0.1*K*PiR*nullspaceR;
    qR = qR+q_pontoR;
    
    q_pontoL = -pseudo_inv_jacob_dampedL*K*err_dL + 0.1*K*PiL*nullspaceL;
    qL = qL+q_pontoL;
    
    %Send joints values
    SetJoints(clientID, vrep, handles.jointsR, qR);
    SetJoints(clientID, vrep, handles.jointsL, qL);
end

    
    
    


    
    
    
