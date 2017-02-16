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
pose_jointsR = GetPoseJoints(clientID, vrep, handles.worldFrame, handles.jointsR);
pose_jointsL = GetPoseJoints(clientID, vrep, handles.worldFrame, handles.jointsL);

nao_pose = GetPose(clientID, vrep, handles.worldFrame, handles.nao);

%x_rot = DQ([cos(pi/4) sin(pi/4) 0 0]);
naoR.set_base(nao_pose);
naoL.set_base(nao_pose);
naoWR.set_base(nao_pose);
naoWL.set_base(nao_pose);

%nao.set_base(pose_joints(1));


%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.jointsR, [0 0 0 0 0]);
SetJoints(clientID, vrep, handles.jointsL, [0 0 0 0 0]);
% return
% KINEMATIC CONTROL

%GO TO OBJECT AND GRAB

%Object's pose
obj_pose = GetPose(clientID, vrep, handles.worldFrame, handles.object);
qL = GetJoints(clientID, vrep, handles.jointsL);
qR = GetJoints(clientID, vrep, handles.jointsR);

obj_poseWR = naoR.fkm(qR);
obj_poseWL = naoL.fkm(qL);
%obj_pose = nao.fkm(q);
qAug = [qL' qR']';

%Shift the pose to fit the gripper on the object
t = DQ([0 0.0 0.0 0.0]); %t = DQ([0 0.0 0.05 0.0]); 
r1 = DQ([cos(pi/4) 0 0 sin(pi/4)]);
r2 = DQ([cos(-pi/4) 0 0 sin(-pi/4)]);%r = DQ([cos(-pi/4) 0 -sin(-pi/4) 0]);
r = r1;
h = r+DQ.E*0.5*t*r;
x_ref = obj_pose;
x_refWR = obj_poseWR*h;
r = r2;
h = r+DQ.E*0.5*t*r;
x_refWL = obj_poseWL*h;
%x_ref = DQ([0.0243 0.8929 0.2171 0.3938 -0.0573454 0.0185763 -0.0387436 -0.0172157]); 
%Maximum tolerance
epsilon = 1e-3;

%Gain
%K = diag([1 0.1 0.1 0.5]);
K = 0.2;
err_tR = 1e3;
err_tL = 1e3;
err_rel = 1e3;
err_wR = 1e3;
err_wL = 1e3;
err_abs = 1e3;

count_t = 1;
cond_num = [];

err_saveR = [];
err_saveL = [];

err_saveAbs = [];
err_normAbs = [];

joint_saveR = [];
joint_saveL = [];

dq_saveL = [];
dq_saveR = [];
dq_saveA = [];

pos_saveL = [];
pos_saveR = [];
pos_saveA = [];

err_norm = [];

while norm(abs(err_abs))>epsilon
    
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
    
    %Porque não funciona?

    %x_rel_p = x_rel.P;
    %x_rel_p.q(1) = (x_rel_p.q(1))*0.5;
    
    %x_rel_t = (x_rel.translation)*0.5;
    %x_rel2 = x_rel_p + DQ.E*0.5*x_rel_t*x_rel_p; %TEM QUE SER NESSA FODENDA ORDEM. BY YURI

    x_rel2 =  exp(0.5*log(x_rel));
    
    
    x_abs = (x_currR*x_rel2);
    %x_abs = normalize(x_abs);

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
    
    %JRel = [hamiplus4(x_currR_conj)*JTL  haminus4(x_currL)*JTR_C];
    c8 = diag([1 -1 -1 -1 1 -1 -1 -1]);
    JRelT = [hamiplus8(x_currR_conj)*JL  haminus8(x_currL)*c8*JR];
    JRel = naoR.jacobp(JRelT, x_rel);

    %Jacobiana Absoluta
    
    JRel2_tmp1 = 0.5*( haminus4(x_rel2.P')*JRelT(1:4, :));
    JRel2_tmp2 = 0.25*( haminus4(x_rel2.P)*JRel + hamiplus4(x_rel.translation)*JRel2_tmp1);
    
    JRel2 = [JRel2_tmp1; JRel2_tmp2];
    
    J_x2_ext = [zeros(8, 5) JR];
    
    JAbs = haminus8(x_rel2)*J_x2_ext + hamiplus8(x_currR)*JRel2;
    
    JAbsT = naoR.jacobp(JAbs, x_abs);
    JAbsT(:, 5) = 0.0;
    JAbsT(:, 10) = 0.0;
    
    %Concatenated Jacobian
    
    J = [JAbsT']';% [JWL zeros(8, 5) ]' [zeros(8 , 5) JWR]']';
    
    %Error
    err_tR = vec4(x_ref.translation-x_currR.translation); %error of the translation only
    err_tL = vec4(x_ref.translation-x_currL.translation);
    err_wR = vec8(x_refWR-x_currR);
    err_wL = vec8(x_refWL-x_currL);
    
    err_wR(2:3) = 0;
    err_wR(5:8) = 0;
    
    err_wL(2:3) = 0;
    err_wL(5:8) = 0;
    
    final_position = x_currR.P' * DQ([0 .0 .20 .10]) * x_currR.P; %Converting absolute distance to relative distance

    err_rel = vec4(final_position - x_rel.translation);
    
    err_abs = vec4(x_ref.translation - x_abs.translation);
    
    err = [err_abs']';% err_wL' err_wR']';
    
    %err_rel = vec4(x_ref.translation - x_rel.translation);
    %err_r = vec4(x_ref.rotation_axis-x_curr.rotation_axis); %error of the rotation only
    
    err_saveR = [err_saveR err_tR];
    err_saveL = [err_saveL err_tL];
    
    err_normAbs = [err_normAbs norm(err_abs)];
    err_saveAbs = [err_saveAbs err_abs];

    err_norm = [err_norm norm(err)];
    
    pos_saveA = [pos_saveA vec4(x_abs.translation)];
    pos_saveL = [pos_saveL vec4(x_currL.translation)];
    pos_saveR = [pos_saveR vec4(x_currR.translation)];
    
    dq_saveA = [dq_saveA; x_abs];
    dq_saveL = [dq_saveL; x_currL];
    dq_saveR = [dq_saveR; x_currR];

    %damped pinv
    lambda = 10e-3;
    pseudo_inv_jacob_dampedR = transpose(JTR)*inv(JTR*transpose(JTR) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    pseudo_inv_jacob_dampedL = transpose(JTL)*inv(JTL*transpose(JTL) + lambda*eye(4)); % JT'(JT*JT' + λI)^-1
    
    pseudo_inv_jacob_dampedRel = transpose(JRel)*inv(JRel*transpose(JRel) + lambda*eye(4));
    pseudo_inv_jacob_wrist_dampedR = transpose(JWR)*inv(JWR*transpose(JWR) + lambda*eye(8));
    pseudo_inv_jacob_wrist_dampedL = transpose(JWL)*inv(JWL*transpose(JWL) + lambda*eye(8));
    
    pseudo_inv_jacob_dampedAbs = transpose(JAbsT)*inv(JAbsT*transpose(JAbsT) + lambda*eye(4));
    
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
    
    PiR = (eye(5) - pseudo_inv_jacob_dampedR*JTR);
    pseudo_inv_null_dampedR = (Jsr*PiR)'*inv(Jsr*PiR * (Jsr*PiR)' + lambda);  %pinv(Js*Pi) = 
%     nullspace = pseudo_inv_null_damped * (s_ - Js*pseudo_inv_jacob_damped*K*err_t);
    GAIN_S = 1;
    nullspaceR = -pseudo_inv_null_dampedR * (GAIN_S*s_r_ - Jsr*pseudo_inv_jacob_dampedR*K*err_tR);
    
    PiL = (eye(5) - pseudo_inv_jacob_dampedL*JTL);
    pseudo_inv_null_dampedL = (Jsl*PiL)'*inv(Jsl*PiL * (Jsl*PiL)' + lambda);  %pinv(Js*Pi) = 
%     nullspace = pseudo_inv_null_damped * (s_ - Js*pseudo_inv_jacob_damped*K*err_t);
    GAIN_S = 1;
    nullspaceL = -pseudo_inv_null_dampedL * (GAIN_S*s_l_ - Jsl*pseudo_inv_jacob_dampedL*K*err_tL);

    
    JsAug = [Jsl Jsr];
    s_aug_ = s_l_ + s_r_;
    PiAug = (eye(10) - pseudo_inv_jacob_dampedAbs*JAbsT);
    pseudo_inv_null_dampedAug = (JsAug*PiAug)'*inv(JsAug*PiAug * (JsAug*PiAug)' + lambda);
    GAIN_S = 1;
    nullspaceAug = -pseudo_inv_null_dampedAug * (GAIN_S*s_aug_ - JsAug*pseudo_inv_jacob_dampedAbs*K*err_rel);
    nullspace(5) = 0.0;
    nullspace(10) = 0.0;
%     theta = theta +  0.01875*taskNS.projector*zPT;

    JsAug = [Jsl Jsr];
    s_aug_ = s_l_ + s_r_;
    PiAug = (eye(10) - pseudo_inv_jacob_damped*J);
    pseudo_inv_null_dampedAug = (JsAug*PiAug)'*inv(JsAug*PiAug * (JsAug*PiAug)' + lambda);
    GAIN_S = 1;
    nullspaceAug = -pseudo_inv_null_dampedAug * (GAIN_S*s_aug_ - JsAug*pseudo_inv_jacob_damped*K*err);
    
    nullspaceAug(1) = 0.0;
    nullspaceAug(6) = 0.0;
    
    nullspaceAug(5) = 0.0;
    nullspaceAug(10) = 0.0;
    
    
    %Kinematic control law
%     q_ponto = pseudo_inv_jacob_damped*K*err_t + nullspace;
    %q_pontoR = pseudo_inv_jacob_dampedR*K*err_tR + 0.25*PiR*nullspaceR;
    %qR = qR+q_pontoR;
    
    %q_pontoL = pseudo_inv_jacob_dampedL*K*err_tL + 0.25*PiL*nullspaceL;
    %qL = qL+q_pontoL;
    
    q_wristR =  pseudo_inv_jacob_wrist_dampedR*1.0*err_wR;
    q_wristL =  pseudo_inv_jacob_wrist_dampedL*1.0*err_wL;
    
    q_pontoAug = pseudo_inv_jacob_damped*K*err + 0.1*K*PiAug*nullspaceAug;
    qAug = qAug+q_pontoAug;
    
    %Send joints values
    %SetJoints(clientID, vrep, handles.jointsR, qR);
    %SetJoints(clientID, vrep, handles.jointsL, qL);
    SetJoints(clientID, vrep, handles.jointsL, qAug(1:5));
    SetJoints(clientID, vrep, handles.jointsR, qAug(6:10));

%     svd_decr = svd(J);
%     cond_num(count_t) = svd_decr(1)/svd_decr(3);
%     count_t = count_t + 1;
%     if count_t > 2000
%         break;
%     end
%     disp (err_t);
end
    
%plot(cond_num);
    
    
    


    
    
    
