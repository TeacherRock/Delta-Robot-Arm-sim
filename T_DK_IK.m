clc,clear,close all;

%% Parameter
[DH, MDH] = DH_MDH();

%% main
Angle_DH = [   0.1,    0.2,    0.3,    0.4,    0.5,    0.6];
T1 = T_DH(Angle_DH);  
x1 = [T1(1:3, 4)];

Angle_MDH = [ 0,    0.1,    0.2,    0.3,    0.4,    0.5,    0.6];
T2 = T_MDH(Angle_MDH);  
x2 = [T2(1:3, 4)];

Euler = Joint2Euler(T1(1:3, 1:3));
T3 = T_Euler(Euler);


T4 = T_Euler([0, 0, 180]*pi/180);

EulerAngle = [];
Position = [];
for i = 1 : 360
    EulerAngle = [EulerAngle; 0, 0, pi;];
    Position = [Position; 30 + 5*cos(i*pi/180), 5*sin(i*pi/180), 30;];
end


theta = Inverse_Kinematic(EulerAngle, Position);

p = [];
for i = 1 : 360
    P = T_DH(theta(i, :)');
    p = [p; P(1, 4), P(2, 4), P(3, 4);];
    plot3(p(:, 1), p(:, 2), p(:, 3),'r')
    xlim([24, 36]); ylim([-6, 6])
    axis("equal")
    pause(0.01)
end

%%
% tic
% Draw([0, 0, 0, 0, 0, 0, 0]);
% toc
%% Transformation matrix (Forward Kinematic)
function Output = T_DH(Angle)
    % DH = [theta_DH', d_DH', a_DH', alpha_DH'];
    [DH, MDH] = DH_MDH();
    theta_DH = DH(:, 1);
    d_DH =     DH(:, 2);
    a_DH =     DH(:, 3);
    alpha_DH = DH(:, 4);
    Axis = length(Angle);
    T = eye(4);
    for i = 1 : Axis
        Ti = [cos(Angle(i) + theta_DH(i)), -sin(Angle(i) + theta_DH(i))*cos(alpha_DH(i)),  sin(Angle(i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*cos(Angle(i) + theta_DH(i));
              sin(Angle(i) + theta_DH(i)),  cos(Angle(i) + theta_DH(i))*cos(alpha_DH(i)), -cos(Angle(i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*sin(Angle(i) + theta_DH(i));
                                        0,                              sin(alpha_DH(i)),                              cos(alpha_DH(i)),                             d_DH(i); 
                                        0,                                             0,                                             0,                                   1;];
        T = T * Ti;
    end    
    Output = T;
end

function Output = T_MDH(Angle, MDH)
    % MDH = [theta_MDH', d_MDH', b_MDH', alpha_MDH'];
    [DH, MDH] = DH_MDH();
    theta_MDH = MDH(:, 1);
    d_MDH =     MDH(:, 2);
    b_MDH =     MDH(:, 3);
    alpha_MDH = MDH(:, 4);
    Axis = length(Angle) - 1;
    T = eye(4);
    for i = 1 : Axis
        Ti = [                   cos(Angle(i+1) + theta_MDH(i+1)),                   -sin(Angle(i+1) + theta_MDH(i+1)),                  0,                      b_MDH(i);
               sin(Angle(i+1) + theta_MDH(i+1))*cos(alpha_MDH(i)),  cos(Angle(i+1) + theta_MDH(i+1))*cos(alpha_MDH(i)), -sin(alpha_MDH(i)), -d_MDH(i+1)*sin(alpha_MDH(i));
               sin(Angle(i+1) + theta_MDH(i+1))*sin(alpha_MDH(i)),  cos(Angle(i+1) + theta_MDH(i+1))*sin(alpha_MDH(i)),  cos(alpha_MDH(i)),  d_MDH(i+1)*cos(alpha_MDH(i)); 
                                                                0,                                                   0,                  0,                             1;];
        T = T * Ti;  
    end    
    Output = T;
end

% Joint to Euler
function Output = Joint2Euler(R)
    % Euler angle (Z-Y-X) -----更好詮釋末端的姿勢
    alpha = atan2(R(2, 1), R(1, 1)); 
    beta = atan2(-R(3, 1), sqrt(R(3, 2)^2 + R(3, 3)^2));
    gamma = atan2(R(3, 2), R(3, 3));
    
    Output = [alpha, beta, gamma];
end

% Euler transformation matrix
function Output = T_Euler(Euler)
    alpha = Euler(:, 1);
    beta = Euler(:, 2); 
    gamma = Euler(:, 3);
    Output = [ cos(alpha).*cos(beta), cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
               sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
               sin(alpha).*cos(beta), sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
              -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
              -sin(beta),    cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];

end

function Output = T_i(Angle, i)
    % DH = [theta_DH', d_DH', a_DH', alpha_DH'];
    [DH, MDH] = DH_MDH();
    theta_MDH = MDH(:, 1);
    d_MDH =     MDH(:, 2);
    b_MDH =     MDH(:, 3);
    alpha_MDH = MDH(:, 4);
    T = eye(4);
    for i = 1 : i
        Ti = [                   cos(Angle(i+1) + theta_MDH(i+1)),                   -sin(Angle(i+1) + theta_MDH(i+1)),                  0,                      b_MDH(i);
               sin(Angle(i+1) + theta_MDH(i+1))*cos(alpha_MDH(i)),  cos(Angle(i+1) + theta_MDH(i+1))*cos(alpha_MDH(i)), -sin(alpha_MDH(i)), -d_MDH(i+1)*sin(alpha_MDH(i));
               sin(Angle(i+1) + theta_MDH(i+1))*sin(alpha_MDH(i)),  cos(Angle(i+1) + theta_MDH(i+1))*sin(alpha_MDH(i)),  cos(alpha_MDH(i)),  d_MDH(i+1)*cos(alpha_MDH(i)); 
                                                                0,                                                   0,                  0,                             1;];
        T = T * Ti;  
    end    
    Output = T;
end
%% Inverse Kinematic
function Output = Inverse_Kinematic(EulerAngle, Position)
    [DH, MDH] = DH_MDH();
    theta_DH = DH(:, 1);
    d_DH =     DH(:, 2);
    a_DH =     DH(:, 3);
    alpha_DH = DH(:, 4);

    n = length(EulerAngle(:, 1));   
    theta = zeros(n, 6);
    
    T = T_Euler(EulerAngle);
    t11 = T(1:n, 1);        t12 = T(1:n, 2);        t13 = T(1:n, 3);
    t21 = T(n+1:2*n, 1);    t22 = T(n+1:2*n, 2);    t23 = T(n+1:2*n, 3); 
    t31 = T(2*n+1:end, 1);  t32 = T(2*n+1:end, 2);  t33 = T(2*n+1:end, 3); 
    % -------------------------------------- theta 1 ----------------------------------------------
    % 奇異點 (J5y, J5x) = (0, 0)
    J5y = Position(:, 2) - d_DH(6)*t23;
    J5x = Position(:, 1) - d_DH(6)*t13;
    J5z = Position(:, 3) - d_DH(6)*t33;

    % 左手臂解
    theta(:, 1) = atan2(J5y, J5x);
    % 右手臂解
%     theta(:, 1) = atan2(J5y, J5x) + pi;
    
    % -------------------------------------- theta 3 ----------------------------------------------
    % 奇異點 cos(theta(:, 3) + atan2(d_DH(4), d_DH(3))) > 1 、theta(:, 3) = -atan2(d_DH(4), d_DH(3)) 表手臂完全伸直
    
    % 手臂上解
    theta(:, 3) = acos( ( (J5x./cos(theta(:, 1)) - a_DH(1)).^2 + (J5z - d_DH(1)).^2 - a_DH(2)^2 - a_DH(3)^2 - d_DH(4)^2) ...
                    / (sqrt((2*a_DH(2)*a_DH(3))^2 + (2*a_DH(2)*d_DH(4))^2))) ...
             - atan2(d_DH(4), a_DH(3));
    % 手臂下解
 %     theta(:, 3) = -acos( ( (J5x./cos(theta(:, 1)) - a_DH(1)).^2 + (J5z - d_DH(1)).^2 - a_DH(2)^2 - a_DH(3)^2 - d_DH(4)^2) ...
%                     / (sqrt((2*a_DH(2)*a_DH(3))^2 + (2*a_DH(2)*d_DH(4))^2))) ...
%              - atan2(d_DH(4), a_DH(3));

    % -------------------------------------- theta 2 ----------------------------------------------
    C2 = ( (J5z - d_DH(1)).*(-d_DH(4)*cos(theta(:, 3)) - a_DH(3)*sin(theta(:, 3))) ...
         - (J5x./cos(theta(:, 1)) - a_DH(1)).*(-a_DH(2) - a_DH(3)*cos(theta(:, 3)) + d_DH(4)*sin(theta(:, 3)))) ...
     ./   ( (-a_DH(3)*sin(theta(:, 3)) - d_DH(4)*cos(theta(:, 3))).^2 ...
        +  (-a_DH(2) - a_DH(3)*cos(theta(:, 3)) + d_DH(4)*sin(theta(:, 3))).^2);
    S2 = ( J5x./cos(theta(:, 1)) - a_DH(1) - (a_DH(2) + a_DH(3)*cos(theta(:, 3)) - d_DH(4)*sin(theta(:, 3))).*C2 ) ...
     ./   (-d_DH(4)*cos(theta(:, 3)) - a_DH(3)*sin(theta(:, 3)));

    theta(:, 2) = atan2(S2, C2);

    % -------------------------------------- theta 5 ----------------------------------------------
    % 手腕上解
    theta(:, 5) = acos( -t13.*cos(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
                    - t23.*sin(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ... 
                    - t33.*cos(theta(:, 2) + theta(:, 3)));
    % 手腕下解
%     theta(:, 5) = -acos( -t13.*cos(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
%                      - t23.*sin(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ... 
%                      - t33.*cos(theta(:, 2) + theta(:, 3)));

    % -------------------------------------- theta 4 ----------------------------------------------
    % 奇異點 theta(:, 5) = 0  =>> sin(theta(:, 5)) = 0  =>> theta(:, 4) 無限多解
    theta(:, 4) = atan2( (t13.*sin(theta(:, 1)) - t23.*cos(theta(:, 1))) ./ -sin(theta(:, 5)), ...
                      (t13.*cos(theta(:, 1)).*cos(theta(:, 2) + theta(:, 3)) ...
                           + t23.*sin(theta(:, 1)).*cos(theta(:, 2) + theta(:, 3)) ...
                           - t33.*sin(theta(:, 2) + theta(:, 3))) ./ -sin(theta(:, 5)));

    % -------------------------------------- theta 6 ----------------------------------------------
    % 奇異點 theta(:, 5) = 0  =>> sin(theta(:, 5)) = 0  =>> theta(:, 6) 無限多解
    theta(:, 6) = atan2( (-t12.*cos(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
                           - t22.*sin(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
                           - t32.*cos(theta(:, 2) + theta(:, 3))) ./ -sin(theta(:, 5)), ...
                      (-t11.*cos(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
                           - t21.*sin(theta(:, 1)).*sin(theta(:, 2) + theta(:, 3)) ...
                           - t31.*cos(theta(:, 2) + theta(:, 3))) ./  sin(theta(:, 5)));

    Output = theta - theta_DH';

end

%% draw
function Draw(JointAngle)
    temp = 0;
    for link = 0 : temp
        TR = stlread('simplify\' + string(link) + '.stl');
        Point.orgin = TR.Points*0.1;
        patch('Vertices', Point.orgin,'Faces', TR.ConnectivityList, 'FaceVertexCData', [255, 0, 0], 'FaceColor', 'flat')
        view(3); axis vis3d; axis("equal")
        hold on 
    end
end
