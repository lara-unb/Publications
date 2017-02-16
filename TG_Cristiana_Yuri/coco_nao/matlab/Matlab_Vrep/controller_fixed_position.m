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
x_ref = obj_pose;

qL = GetJoints(clientID, vrep, handles.jointsL);
qR = GetJoints(clientID, vrep, handles.jointsR);

obj_poseWR = naoR.fkm(qR);
obj_poseWL = naoL.fkm(qL);

%Shift the pose to fit the gripper on the object
t = DQ([0 0.0 0.0 0.0]); 

r = DQ([cos(pi/4) 0 0 sin(pi/4)]);
h = r+0.5*DQ.E*t*r;
x_refWR = obj_poseWR*h;

r = DQ([cos(-pi/4) 0 0 sin(-pi/4)]);
h = r+0.5*DQ.E*t*r;
x_refWL = obj_poseWL*h;
 
%Maximum tolerance
epsilon = 1e-3;

%Gain
K = 0.2;
err_tR = 1e3;
err_tL = 1e3;
err_rel = 1e3;
err_wR = 1e3;
err_wL = 1e3;

err_saveR = [];
err_saveL = [];

joint_saveR = [];
joint_saveL = [];


err_norm = [];
err_save = [];

while norm(abs(err_rel))>epsilon
    
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
    JTR = naoR.jacobp(JR, x_currR); %jacobian of the translation only 
    JTR_C = naoR.jacobp(JR, x_currR_conj);

    JL = naoL.jacobian(qL);
    JTL = naoL.jacobp(JL, x_currL);
   
    JWR = naoWR.jacobian(qR);
    JWL = naoWL.jacobian(qL);
    for i = 1:5
       for j = 1:8
          if(i ~= 5 || (j ~= 4 && j ~= 1))
             JWR(j, i) = 0.0; 
             JWL(j, i) = 0.0;
          end
       end
    end
   
    c8 = diag([1 -1 -1 -1 1 -1 -1 -1]);
    JRelT = [hamiplus8(x_currR_conj)*JL  haminus8(x_currL)*c8*JR];
    JRel = naoR.jacobp(JRelT, x_rel);
    JRel(:, 5) = 0.0;
    JRel(:, 10) = 0.0;
    
    %Concatenate
    
    J = [JRel' [JWL zeros(8, 5) ]' [zeros(8 , 5) JWR]']';
    J = JRel;
    
    %Error
    err_wR = vec8(x_refWR-x_currR);
    err_wL = vec8(x_refWL-x_currL);
    
    err_wR(2:3) = 0;
    err_wR(5:8) = 0;
    
    err_wL(2:3) = 0;
    err_wL(5:8) = 0;
    
    final_position = x_currR.P' * DQ([0.1 .0 .20 .2]) * x_currR.P; %Converting absolute distance to relative distance

    err_rel = vec4(final_position - x_rel.translation);
    
    err = [err_rel' err_wL' err_wR']';
    err = err_rel;
    
    err_saveR = [err_saveR err_wR];
    err_saveL = [err_saveL err_wL];
 
    err_norm = [err_norm norm(err)];
    err_save = [err_save err_rel];

    %damped pinv % JT'(JT*JT' + λI)^-1
    lambda = 10e-3;   
    pseudo_inv_jacob_damped = transpose(J)*inv(J*transpose(J) + lambda*eye(4));
    
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
    JsAug = [Jsl Jsr];
    s_aug_ = s_l_ + s_r_;
    PiAug = (eye(10) - pseudo_inv_jacob_damped*J);
    pseudo_inv_null_dampedAug = (JsAug*PiAug)'*inv(JsAug*PiAug * (JsAug*PiAug)' + lambda);
    GAIN_S = 1;
    nullspaceAug = -pseudo_inv_null_dampedAug * (GAIN_S*s_aug_ - JsAug*pseudo_inv_jacob_damped*K*err);
    nullspaceAug(5) = 0.0;
    nullspaceAug(10) = 0.0;
    
    %Kinematic control law    
    q_pontoAug = pseudo_inv_jacob_damped*K*err + 0.*K*PiAug*nullspaceAug;
    qAug = qAug+q_pontoAug;
    
    %Send joints values
    SetJoints(clientID, vrep, handles.jointsL, qAug(1:5));
    SetJoints(clientID, vrep, handles.jointsR, qAug(6:10));
end

    
    
    


    
    
    
