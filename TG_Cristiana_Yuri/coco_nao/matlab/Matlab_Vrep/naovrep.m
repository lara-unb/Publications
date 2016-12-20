%close all;
%clear all;
clc;
warning off;

curr_path = pwd;
DQ_lib_path = [curr_path '/DQ_Robotics']; 
addpath(DQ_lib_path);


% V-REP INITIALIZATION
[vrep clientID] = InitializeVREP;

% LOAD HANDLES
%Handles of scene objects
handles = GetHandlesR(clientID, vrep);

%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.joints, [0 0 0 0 0]);
fakeJoints = [0 0 0 0 0];

% ROBOT MODEL

%Model
nao = BuildModel;

%Joint mean values
%j_mean = [0 0.50615 0 -0.78975 0]; %Esquerdo
j_mean = [0 -0.50615 0 0.78975 0]; %Direito

%Set robot base to the chest
nao_pose = GetPose(clientID, vrep, handles.worldFrame, handles.nao);

nao.set_base(nao_pose);

% KINEMATIC CONTROL

%GO TO OBJECT AND GRAB

%Object's pose
obj_pose = GetPose(clientID, vrep, handles.worldFrame, handles.object);

%Shift the pose to fit the gripper on the object
x_ref = obj_pose;

%Maximum tolerance
epsilon = 1e-3;

%Gain

K = 0.5;
err_t = 1e10;

err_save = [];
joint_save = [];
err_norm = [];

while 1 %norm(abs(err_t))>epsilon
    
    %Verify if V-rep is connected
    [connectionId]=vrep.simxGetConnectionId(clientID);
    if connectionId==-1
            disp('Server disconnected!');
            return;
    end  
    
    %Current joints
    q = GetJoints(clientID, vrep, handles.joints);
    %q = fakeJoints';
    
    joint_save = [joint_save q];
    
    %Current pose
    x_curr = nao.fkm(q);
    
     %Jacobian
    J = nao.jacobian(q);
    JT = nao.jacobp(J, x_curr); %jacobian of the translation only 
   
    %Error
    err_t = vec4(x_ref.translation-x_curr.translation); %error of the translation only
    
    err_save = [err_save err_t];
    err_norm = [err_norm norm(err_t)];
    
    %damped pinv
    lambda = 10e-5;
    pseudo_inv_jacob_damped = transpose(JT)*inv(JT*transpose(JT) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    
    %nullspace descriptor function
    s_ = 0.0;
    s_bar = 0.0;
    Js = [];
    for i = 1:5
        tmp = q(i, 1) - j_mean(i);
        s_ = s_ + 1/2 * tmp^2;
        s_bar = s_bar + tmp; % s_ = (θ1 - θ1m) + (θ2 - θ2m)...
        Js = [Js tmp];       % Js = ((θ1 - θ1m), (θ2 - θ2m), ...)
    end
    
    %nullspace calculation
    
    Pi_null = (eye(5) - pseudo_inv_jacob_damped*JT);
    %Pi_null = (eye(5) - pinv(JT)*JT);
    pseudo_inv_null_damped = (Js*Pi_null)'*inv(Js*Pi_null * (Js*Pi_null)' + lambda);  
    GAIN_S = 0.5;
    nullspace = -pseudo_inv_null_damped * (GAIN_S*s_ - Js*pseudo_inv_jacob_damped*K*err_t);
    %nullspace = -pseudo_inv_null_damped * (GAIN_S*s_ - Js*pinv(JT)*K*err_t);
    
    eig((JT*Pi_null)*(JT*Pi_null)')

    %Kinematic control law
    q_ponto = pseudo_inv_jacob_damped*K*err_t + 0.1*Pi_null*nullspace;
    q = q+0.2*q_ponto;
    
    
    %fakeJoints = q';
    %Send joints values
    SetJoints(clientID, vrep, handles.joints, q);
end
    
    
