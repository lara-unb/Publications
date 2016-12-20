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
t2 = DQ([0 0.05 0.05 0.0]);
r = DQ([cos(-pi/2) sin(-pi/2) 0 0]);
h = r+0.5*DQ.E*t*r;

x_refD = 0.3;
x_refR = obj_pose;
x_refL = obj_pose*h;
x_refP = obj_pose;

%Maximum tolerance
epsilon = 1e-3;

%Gain
K=0.2;
K_ctrl = diag([0.4 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.4 0.4 0.4 0.4]);
K_null = 1;%diag([0 1 0 1 0 0 1 0 1 0]);
err = 1e3;

pose_saveL = [];
pose_saveR = [];
q_saveL = [];
q_saveR = [];

joint_saveR = [];
joint_saveL = [];

e_save_d = [];
e_save_l = [];
e_save_r = [];
e_save_p = [];

err_norm = [];
err_save = [];

time_save = [];

count = 1;

svd_ = [];
svd_l = [];
svd_r = [];
inv_svd = [];

rank_ = [];
inv_rank = [];

acao_ctrl = [];

x_ref_save = [];
i = 1;

initialTime = vrep.simxGetLastCmdTime(clientID);
while norm(abs(err))>epsilon
    count = count + 1;
    if count >= 500
        h2 = r+0.5*DQ.E*t2*r;
        x_refP = obj_pose*h2;
    end
    
    
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
    
    pose_saveL = [pose_saveL [vec4(x_currL.P)' vec4(x_currL.translation)']'];
    pose_saveR = [pose_saveR [vec4(x_currR.P)' vec4(x_currR.translation)']'];
    
    q_saveL = [q_saveL; x_currL];
    q_saveR = [q_saveR; x_currR];
      
    x_currR_conj = x_currR';
    x_rel = (x_currR_conj)*x_currL;
    
    x_rel2 =  exp(0.5*log(x_rel));
    x_abs = (x_currR*x_rel2);
    
     %Jacobiana Distancia
    JR = naoR.jacobian(qR);
    JL = naoL.jacobian(qL);

    c8 = diag([1 -1 -1 -1 1 -1 -1 -1]);
    JRel = [hamiplus8(x_currR_conj)*JL  haminus8(x_currL)*c8*JR];
    JRelT = naoR.jacobp(JRel, x_rel);
    JRelD = naoR.jacobd(JRel, x_rel);
    JRelD = 2*((vec4(x_rel.translation)))'*JRelT;
    JRelD(:, 5) = 0.0;
    JRelD(:, 10) = 0.0;
    
    %Jacobiana rotacao de cada braco
    JRR = haminus4(x_refR)*diag([1 -1 -1 -1])*JR(1:4, :);
    JRL = haminus4(x_refL)*diag([1 -1 -1 -1])*JL(1:4, :);
    
    %Jacobiana Absoluta 
    JRel2_tmp1 = 0.5*( haminus4(x_rel2.P')*JRel(1:4, :));
    JRel2_tmp2 = 0.25*( haminus4(x_rel2.P)*JRelT + hamiplus4(x_rel.translation)*JRel2_tmp1);
   
    JRel2 = [JRel2_tmp1; JRel2_tmp2];
    
    J_x2_ext = [zeros(8, 5) JR];
    
    JAbs = haminus8(x_rel2)*J_x2_ext + hamiplus8(x_currR)*JRel2;
    
    JAbsT = naoR.jacobp(JAbs, x_abs);
    JAbsT(:, 5) = 0.0;
    JAbsT(:, 10) = 0.0;    
    
    %Concatenate
    
    J = [JRelD' [JRL zeros(4, 5) ]' [zeros(4 , 5) JRR]' JAbsT']';
    [U,S,V] = svd(J);
    s_temp = diag(S);
    rank_ = [rank_ nnz(s_temp)];
    svd_l = [svd_l svd(JL)];
    svd_r = [svd_r svd(JR)];
    
    %Erro Distancia
    err_rel = x_refD - norm(vec4(x_rel.translation));
    
    %Erro rotacao de cada braco
    err_rR = vec4(DQ(1) - (x_currR'*x_refR));
    err_rL = vec4(DQ(1) - (x_currL'*x_refL));
    
    %Erro posicao absoluta
    err_abs = vec4(x_refP.translation - x_abs.translation);
    
    err = [err_rel' err_rL' err_rR' err_abs']';
    
    saveRef = [x_refD x_refL x_refR x_refP];
    
    x_ref_save = [x_ref_save saveRef];
    
    e_save_d = [e_save_d err_rel];
    e_save_l = [e_save_l err_rL];
    e_save_r = [e_save_r err_rR];
    e_save_p = [e_save_p err_abs];
    
    err_save = [err_save err];
    err_norm = [err_norm norm(err)];
    
    curr_t = vrep.simxGetLastCmdTime(clientID);
    
    time_save = [time_save (curr_t - initialTime)];

    %damped pinv  % JT'(JT*JT' + λI)^-1
    lambda = 10e-3;  
    pseudo_inv_jacob_damped = transpose(J)*inv(J*transpose(J) + lambda*eye(13));
    
    [U,S,V] = svd(pseudo_inv_jacob_damped);
    s_temp = diag(S);
    inv_rank = [inv_rank nnz(s_temp)];
    inv_svd = [inv_svd svd(pseudo_inv_jacob_damped)];
    
    
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
    
    acao_ctrl = [acao_ctrl q_pontoAug];
    
    %Send joints values
    if (i < 495 || i > 505)
        SetJoints(clientID, vrep, handles.jointsL, qAug(1:5));
        SetJoints(clientID, vrep, handles.jointsR, qAug(6:10));
    end
    
    i = i+1;
end

figure;
plot(e_save_d);
xlabel('Iterations');
ylabel('Error (m)');

  
    
    


    
    
    
