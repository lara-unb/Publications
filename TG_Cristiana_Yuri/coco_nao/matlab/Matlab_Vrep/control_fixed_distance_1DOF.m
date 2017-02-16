%Separate control for both left and right arms
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

naoWR = BuildModelR;
naoWL = BuildModelL;

%Joint mean values
j_mean_l = [0 0.50615 0 -0.78975 0]; %Esquerdo
j_mean_r = [0 -0.50615 0 0.78975 0]; %Direito

%Set robot base to the current first joint
nao_pose = GetPose(clientID, vrep, handles.worldFrame, handles.nao);

naoR.set_base(nao_pose);
naoL.set_base(nao_pose);
naoWR.set_base(nao_pose);
naoWL.set_base(nao_pose);

% KINEMATIC CONTROL

%GO TO OBJECT AND GRAB

%Object's pose
obj_pose = GetPose(clientID, vrep, handles.worldFrame, handles.object);
qL = GetJoints(clientID, vrep, handles.jointsL);
qR = GetJoints(clientID, vrep, handles.jointsR);

obj_poseWR = naoR.fkm(qR);
obj_poseWL = naoL.fkm(qL);

%Shift the pose to fit the gripper on the object
t = DQ([0 0.0 0.0 0.0]);
r = DQ([cos(-pi/2) sin(-pi/2) 0 0]);
h = r+0.5*DQ.E*t*r;
x_refR = obj_pose;
x_refL = obj_pose*h;

%Maximum tolerance
epsilon = 1e-3;

%Gain
K=0.2;
K_ctrl = diag([1. 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05]);
K_null = 1;%diag([0 1 0 1 0 0 1 0 1 0]);
err = 1e3;

err_save = [];
joint_saveR = [];
joint_saveL = [];

e_save_d = [];
e_save_l = [];
e_save_r = [];

err_norm = [];

while norm(abs(err))>epsilon
    
    %Verify if V-rep is connected
    [connectionId]=vrep.simxGetConnectionId(clientID);
    if connectionId==-1
            disp('Server disconnected!');
            return;
    end  
    
    %Current joints
    qR = GetJoints(clientID, vrep, handles.jointsR);
    qL = GetJoints(clientID, vrep, handles.jointsL);
    qAug = [qL' qR']';
    
    joint_saveR = [joint_saveR qR];
    joint_saveL = [joint_saveL qL];
    
    %Current pose
    x_currR = naoR.fkm(qR);
    x_currL = naoL.fkm(qL);
    
    x_currR_conj = x_currR';
    x_rel = (x_currR_conj)*x_currL;
    
     %Jacobian
    JR = naoR.jacobian(qR);
    JL = naoL.jacobian(qL);

    c8 = diag([1 -1 -1 -1 1 -1 -1 -1]);
    JRel = [hamiplus8(x_currR_conj)*JL  haminus8(x_currL)*c8*JR];
    JRelT = naoR.jacobp(JRel, x_rel);
    JRelD = naoR.jacobd(JRel, x_rel);
    JRelD = 2*((vec4(x_rel.translation)))'*JRelT;
    JRelD(:, 5) = 0.0;
    JRelD(:, 10) = 0.0;
    
    JRR = haminus4(x_refR)*diag([1 -1 -1 -1])*JR(1:4, :);
    JRL = haminus4(x_refL)*diag([1 -1 -1 -1])*JL(1:4, :);
    
    %Concatenate
    
    J = [JRelD' [JRL zeros(4, 5) ]' [zeros(4 , 5) JRR]']';
    
    %Error    
    final_position = 0.3; %Converting absolute distance to relative distance
    err_rel = final_position - norm(vec4(x_rel.translation));

    
    err_rR = vec4(DQ(1) - (x_currR'*x_refR));
    err_rL = vec4(DQ(1) - (x_currL'*x_refL));
    
    err = [err_rel' err_rL' err_rR']';
    
    e_save_d = [e_save_d err_rel];
    e_save_l = [e_save_l err_rL];
    e_save_r = [e_save_r err_rR];
    
    err_save = [err_save err];
    err_norm = [err_norm norm(err)];

    %damped pinv  % JT'(JT*JT' + λI)^-1
    lambda = 10e-3;  
    pseudo_inv_jacob_damped = transpose(J)*inv(J*transpose(J) + lambda*eye(9));
    
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
    JsAug = [[Jsl zeros(1,5)]' [zeros(1,5) Jsr]']';
    s_aug_ = [s_l_ s_r_]';
    PiAug = (eye(10) - pseudo_inv_jacob_damped*J);
    pseudo_inv_null_dampedAug = (JsAug*PiAug)'*inv(JsAug*PiAug * (JsAug*PiAug)' + lambda);
    GAIN_S = 0.5;
    nullspaceAug = -pseudo_inv_null_dampedAug * (GAIN_S*s_aug_ - JsAug*pseudo_inv_jacob_damped*K_ctrl*err);
    
    %Kinematic control law  
    q_pontoAug = pseudo_inv_jacob_damped*K_ctrl*err + 0.0*K_null*PiAug*nullspaceAug;
    qAug = qAug+q_pontoAug;
    
    %Send joints values
    SetJoints(clientID, vrep, handles.jointsL, qAug(1:5));
    SetJoints(clientID, vrep, handles.jointsR, qAug(6:10));
end
  
    
    


    
    
    
