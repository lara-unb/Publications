%%TIME DIFFERENCE%%
%Get the average time offset between the joint recorded data

clc;
clear all;

%load File
filename = 'resultados.txt';
m = csvread(filename);

%transpose data
mt = m';

%joints_diff_matrix;

for i = 0:4
    %joint data
    mult = 1;
    if(i > 0)
        mult = -1;
    end
    j1 = [mt(11, :); mt(1+i,:)];
    j2 = [mt(11, :); mt(6+i,:)*mult];

    diff_vector = [];
    erro1 = j2(2, 30:330);

    for j = 1:50
        erro2 = j1(2, 30+j:330+j);
        diff = erro1 - erro2;
        diff_vector = [diff_vector norm(diff)]; 
    end
    
    figure;
    
    plot(j1(1,:), j1(2,:), '-b');
    hold on;
    plot(j2(1,:), j2(2,:), '-g');
 
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    title(strcat('Joint ', num2str(i)));
    %plot2svg(strcat('Joint ', num2str(i)));
    
    [a, b] = min(diff_vector);
    diff_mean_time = j2(1, 1:end-b) - j1(1, b+1:end);
    diff_mean = j2(2, 1:end-b) - j1(2, b+1:end);
    joints_diff_matrix{i+1} = diff_mean_time';
    joints_diff_{i+1} = diff_mean';
end

            