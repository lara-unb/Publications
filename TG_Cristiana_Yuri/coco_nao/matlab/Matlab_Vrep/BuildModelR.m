function robot = BuildModelR
    %BUILDMODEL

    %Os dados de distancia entre os links se referem aos valores do Nao
    %simulado no VREP. Braço Direito
    d =     [0.0814 -.0977 0        0.105   0         0.05595   0       ];%0.05775]; %meters
    theta = [0      0      pi/2     0       0         0         -pi/2   ]; %rad
    a =     [-.0026 0      -0.015   0       0         0         0       ]; %meters
    alpha = [-pi/2  pi/2   pi/2     -pi/2   pi/2      -pi/2     0       ]; %rad
    dummy = [1      0      0        0       0         0         1       ];
      
    
    robot_matrix = [theta;
                   d;
                   a;
                   alpha;
                   dummy];
     robot = DQ_kinematics(robot_matrix,'standard');
%     robot = DQ_kinematics(robot_matrix,'modified');
end  
   
