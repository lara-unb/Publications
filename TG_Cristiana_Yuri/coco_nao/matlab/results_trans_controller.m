close all;
clear all;
clc;
warning off;

curr_path = pwd;
DQ_lib_path = [curr_path '/DQ_Robotics']; 
addpath(DQ_lib_path);


file = fopen('resultados_translation_controller1.txt');

data = textscan(file, '%f %f %f %f %f %f %f %f %d');

obj_pose = normalize(DQ([data{1}(1) data{2}(1) data{3}(1) data{4}(1) data{5}(1) data{6}(1) data{7}(1) data{8}(1)]));
time_obj = data{9}(1);

pose = [];
time = [];
diff = [];
norma = [];

for i = 2:71
    tmp = normalize(DQ([data{1}(i) data{2}(i) data{3}(i) data{4}(i) data{5}(i) data{6}(i) data{7}(i) data{8}(i)]));
    pose = [pose tmp];
    tmp2 = vec4(tmp.translation - obj_pose.translation);
    
    diff = [diff; tmp2'];
    norma = [norma norm(abs(tmp2))];
    time = [time data{9}(i)];
end

    