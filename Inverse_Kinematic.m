function Output = Inverse_Kinematic(EulerAngle, Position)
    DH = DH_MDH('DH');
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