function robot = BuildModel
    %BUILDMODEL

    %LEFT ARM MODEL
%     theta = [pi/2   0       pi/2    0       0       0 ]; %rad
%     d =     [0.0815 0       0.105   0       0       0.05595 ];%0.05775]; %meters
%     a =     [0.098  0       0       0.015   0       0 ]; %meters
%     alpha = [0      -pi/2   pi/2    pi/2    -pi/2   pi/2 ]; %rad
%     dummy = [1      0       0       0       0       0 ];
    

%                 theta = [0       pi/2    0       0       0 ]; %rad
%                 d =     [0       0       0.105   0       0.05595 ];%0.05775]; %meters
%                 a =     [0       0      -0.015    0    0 ]; %meters
%                 alpha = [-pi/2   pi/2    pi/2    -pi/2   pi/2 ]; %rad
%                 dummy = [0       0       0       0       0 ];


%     theta = [0     pi/2  0       0    0 ]; %rad
%     d =     [0     0.105 0       0    0.05595 ]%0.05775]; %meters
%     a =     [0     0     0.015   0    0 ]; %meters
%     alpha = [-pi/2 pi/2  pi/2   -pi/2 pi/2 ]; %rad
%     dummy = [0 0 0 0 0 ];
% 
%     d =     [0      0       0.105    0         0.1061];%0.05775]; %meters
%     theta = [0      pi/2     pi    -2*pi/180   93.7*180/pi]; %rad
%     a =     [0      0.015    0       0         0.0252 ]; %meters
%     alpha = [pi/2   pi/2    pi/2   -pi/2       89.4*180/pi]; %rad
%     dummy = [0      0        0       0         0      ];


%     d =     [0      0.015    0       0     0      0.0020    0       0.0420     0];%0.05775]; %meters
%     theta = [0      pi/2    -pi/2    0    -pi/2   pi/2     -pi/2    0          -pi/2]; %rad
%     a =     [0      0.105    0       0     0      0.05595   0      -0.0022     0 ]; %meters
%     alpha = [pi/2   pi/2     0     -pi/2   pi/2   pi/2      0      -pi/2       pi/2]; %rad
%     dummy = [0      1        0       1      0     1         0       1          0      ];
%   
    %Os dados de distancia entre os links se referem aos valores do Nao
    %simulado no VREP. Braço Esquerdo
    d =     [0.0814 0.0983 0        0.105   0         0.05595];%0.05775]; %meters
    theta = [0      0      pi/2     0       0         0      ]; %rad
    a =     [-.0026 0      0.015    0       0         0      ]; %meters
    alpha = [-pi/2  pi/2   pi/2     -pi/2   pi/2      0      ]; %rad
    dummy = [1      0      0        0       0         0      ];
    
    %Os dados de distancia entre os links se referem aos valores do Nao
    %simulado no VREP. Braço Direito
    d =     [0.0814 -.0977 0        0.105   0         0.05595];%0.05775]; %meters
    theta = [0      0      pi/2     0       0         0      ]; %rad
    a =     [-.0026 0      -0.015   0       0         0      ]; %meters
    alpha = [-pi/2  pi/2   pi/2     -pi/2   pi/2      0      ]; %rad
    dummy = [1      0      0        0       0         0      ];
    
    %d =     [0      0       0.105    0         0.05595];%0.05775]; %meters
    %theta = [0      pi/2     0       0         0      ]; %rad
    %a =     [0      0.015    0       0         0      ]; %meters
    %alpha = [pi/2   pi/2   -pi/2    pi/2       0      ]; %rad
    %dummy = [0      0        0       0         0      ];
    
 
%     d =     [0      0       0.105    0         0.05595   0    ];%0.05775]; %meters
%     theta = [0      pi/2     0       0         0         pi/2 ]; %rad
%     a =     [0      0.015    0       0         0         0    ]; %meters
%     alpha = [pi/2   pi/2   -pi/2    pi/2       0         pi/2]; %rad
%     dummy = [0      0        0       0         0         1    ];    
      
    
    robot_matrix = [theta;
                   d;
                   a;
                   alpha;
                   dummy];
     robot = DQ_kinematics(robot_matrix,'standard');
%     robot = DQ_kinematics(robot_matrix,'modified');
end     