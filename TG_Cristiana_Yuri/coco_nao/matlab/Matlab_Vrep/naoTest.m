% close all;
% clc;
warning off;

curr_path = pwd;
DQ_lib_path = [curr_path '/DQ_Robotics']; 
addpath(DQ_lib_path);


% V-REP INITIALIZATION
[vrep clientID] = InitializeVREP;

% LOAD HANDLES
%Handles of scene objects
handles = GetHandles(clientID, vrep);

% ROBOT MODEL

%Model
nao = BuildModelR;
naoW = BuildModelR;

%Joint mean values
%j_mean = [0 0.50615 0 -0.78975 0]; %Esquerdo
j_mean = [0 -0.50615 0 0.78975 0]; %Direito

%Set robot base to the current first joint
pose_joints = GetPoseJoints(clientID, vrep, handles.worldFrame, handles.joints);
nao_pose = GetPose(clientID, vrep, handles.worldFrame, handles.nao);

x_rot = DQ([cos(pi/4) sin(pi/4) 0 0]);
nao.set_base(nao_pose);
naoW.set_base(nao_pose);
%nao.set_base(pose_joints(1));


%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.joints, [0 0 0 0 0]);
% return
% KINEMATIC CONTROL

%GO TO OBJECT AND GRAB

%Object's pose
obj_pose = GetPose(clientID, vrep, handles.worldFrame, handles.object);
q = GetJoints(clientID, vrep, handles.joints);
obj_poseW = nao.fkm(q);

%Shift the pose to fit the gripper on the object
t = DQ([0 0.0 0.0 0.0]); %t = DQ([0 0.0 0.05 0.0]); 
r1 = DQ([cos(-pi/4) sin(-pi/4) 0 0]);
r2 = DQ([cos(pi/4) 0 0 sin(pi/4)]);%r = DQ([cos(-pi/4) 0 -sin(-pi/4) 0]);
r = r2;
h = r+0.5*DQ.E*t*r;
x_ref = obj_pose;
x_refW = obj_poseW*h;
%x_ref = DQ([0.0243 0.8929 0.2171 0.3938 -0.0573454 0.0185763 -0.0387436 -0.0172157]); 
%Maximum tolerance
epsilon = 1e-2;

%Gain
%K = diag([1 0.1 0.1 0.5]);
K = 0.5;
err_t = 1e3;
err_w = 1e3;

count_t = 1;
cond_num = [];

err_savet = [];
joint_save = [];
err_normt = [];
err_savew = [];
err_normw = [];

while norm(abs(err_t))>epsilon && norm(abs(err_w)) > epsilon
    
    %Verify if V-rep is connected
    [connectionId]=vrep.simxGetConnectionId(clientID);
    if connectionId==-1
            disp('Server disconnected!');
            return;
    end  
    
    %Current joints
    q = GetJoints(clientID, vrep, handles.joints);
    
    joint_save = [joint_save q];
    
    %Current pose
    x_curr = nao.fkm(q);
    
     %Jacobian
    J = nao.jacobian(q);
    
    JW = naoW.jacobian(q);
    for i = 1:5
       for j = 1:8
          if(i ~= 5 || (j ~= 4 && j ~= 1))
             JW(j, i) = 0.0; 
          end
       end
    end
    JT = nao.jacobp(J, x_curr); %jacobian of the translation only 
    %JT(:,5) = 0.0;
    
    %Error
    err_t = vec4(x_ref.translation-x_curr.translation); %error of the translation only
    err_w = vec8(x_refW-x_curr);
    %err_r = vec4(x_ref.rotation_axis-x_curr.rotation_axis); %error of the rotation only
    
    err_savet = [err_savet err_t];
    err_normt = [err_normt norm(err_t)];
    err_savew = [err_savew err_w];
    err_normw = [err_normw norm(err_w)];
    
    %damped pinv
    lambda = 10e-3;
    pseudo_inv_jacob_damped = transpose(JT)*inv(JT*transpose(JT) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    pseudo_inv_jacob_wrist_damped = transpose(JW)*inv(JW*transpose(JW) + lambda*eye(8));
    
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
    
    Pi = (eye(5) - pseudo_inv_jacob_damped*JT);
    pseudo_inv_null_damped = (Js*Pi)'*inv(Js*Pi * (Js*Pi)' + lambda);  %pinv(Js*Pi) = 
%     nullspace = pseudo_inv_null_damped * (s_ - Js*pseudo_inv_jacob_damped*K*err_t);
    GAIN_S = 1;
    nullspace = -pseudo_inv_null_damped * (GAIN_S*s_ - Js*pseudo_inv_jacob_damped*K*err_t);
    nullspace(5) = 0.0;
   
%     theta = theta +  0.01875*taskNS.projector*zPT;
    
    
    %Kinematic control law
%     q_ponto = pseudo_inv_jacob_damped*K*err_t + nullspace;
    q_ponto = pseudo_inv_jacob_damped*K*0.0*err_t + 0.25*Pi*nullspace + pseudo_inv_jacob_wrist_damped*1.0*err_w;
    q = q+q_ponto;
    
    
    
    %Send joints values
    SetJoints(clientID, vrep, handles.joints, q);
%     svd_decr = svd(J);
%     cond_num(count_t) = svd_decr(1)/svd_decr(3);
%     count_t = count_t + 1;
%     if count_t > 2000
%         break;
%     end
%     disp (err_t);
end
    
%plot(cond_num);
    
    
    

