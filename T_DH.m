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