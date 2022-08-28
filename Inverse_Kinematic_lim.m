function [JointAngle, SingularFlag] = Inverse_Kinematic_lim(EulerAngle, Position)
% git測試用註解
    DH = DH_MDH('DH');
    theta_DH = DH(:, 1);
    d_DH =     DH(:, 2);
    a_DH =     DH(:, 3);
    alpha_DH = DH(:, 4);

    Delta_Limitation = Delta_Constraint();

    n = length(EulerAngle(:, 1));   
    theta = zeros(n, 6);
    JointAngle = zeros(n,8,6);
    
    T = T_Euler(EulerAngle);
    t11 = T(1:n, 1);        t12 = T(1:n, 2);        t13 = T(1:n, 3);
    t21 = T(n+1:2*n, 1);    t22 = T(n+1:2*n, 2);    t23 = T(n+1:2*n, 3); 
    t31 = T(2*n+1:end, 1);  t32 = T(2*n+1:end, 2);  t33 = T(2*n+1:end, 3); 

    J5y = Position(:, 2) - d_DH(6)*t23;
    J5x = Position(:, 1) - d_DH(6)*t13;
    J5z = Position(:, 3) - d_DH(6)*t33;
    
    % 奇異點 (J5y, J5x) = (0, 0)
    SingularFlag = ~isempty(find((J5y == 0 & J5x == 0), 1));

    if SingularFlag ~= 1
        % -------------------------------------- Joint 1 ----------------------------------------------
        % 左手臂解
        J1(:, 1) = wrapToPi(atan2(J5y, J5x));    % wrapToPi() : Wrap angle in radians to [−pi pi]
        % 右手臂解
        J1(:, 2) = wrapToPi(J1(:, 1) + pi);
        
        % -------------------------------------- Joint 3 ----------------------------------------------
        [J3(:, 2), J3(:, 1), s3_up_left, s3_low_left] = calculateJ3(J1(:, 1));
        [J3(:, 4), J3(:, 3), s3_up_right, s3_low_right] = calculateJ3(J1(:, 2));
    
        % ------------------------ Joint 2 & Joint 5 & Joint 4 & Joint 6-------------------------------
        if s3_up_left ~= 1
            J2(:, 2) = calculateJ2(J1(:, 1), J3(:, 2));
            [J5(:, 4), J5(:, 3), s5_up_left_flip, s5_up_left_nonflip ] ...
                                                            = calculateJ5(J1(:, 1), J2(:, 2), J3(:, 2));
            [J4(:, 4), J6(:, 4)] = calculateJ4J6(J1(:, 1), J2(:, 2), J3(:, 2), J5(:, 4), s5_up_left_flip);
            [J4(:, 3), J6(:, 3)] = calculateJ4J6(J1(:, 1), J2(:, 2), J3(:, 2), J5(:, 3), s5_up_left_nonflip);
        end

        if s3_low_left ~= 1
            J2(:, 1) = calculateJ2(J1(:, 1), J3(:, 1));
            [J5(:, 2), J5(:, 1), s5_low_left_flip, s5_low_left_nonflip ] ...
                                                            = calculateJ5(J1(:, 1), J2(:, 1), J3(:, 1));
            [J4(:, 2), J6(:, 2)] = calculateJ4J6(J1(:, 1), J2(:, 1), J3(:, 1), J5(:, 2), s5_low_left_flip);
            [J4(:, 1), J6(:, 1)] = calculateJ4J6(J1(:, 1), J2(:, 1), J3(:, 1), J5(:, 1), s5_low_left_nonflip);
        end

        if s3_up_right ~= 1
            J2(:, 4) = calculateJ2(J1(:, 2), J3(:, 4));
            [J5(:, 8), J5(:, 7), s5_up_right_flip, s5_up_right_nonflip ] ...
                                                            = calculateJ5(J1(:, 2), J2(:, 4), J3(:, 4));
            [J4(:, 8), J6(:, 8)] = calculateJ4J6(J1(:, 2), J2(:, 4), J3(:, 4), J5(:, 8), s5_up_right_flip);
            [J4(:, 7), J6(:, 7)] = calculateJ4J6(J1(:, 2), J2(:, 4), J3(:, 4), J5(:, 7), s5_up_right_nonflip);
        end

        if s3_low_right ~= 1
            J2(:, 3) = calculateJ2(J1(:, 2), J3(:, 3));
            [J5(:, 6), J5(:, 5), s5_low_right_flip, s5_low_right_nonflip ] ...
                                                            = calculateJ5(J1(:, 2), J2(:, 3), J3(:, 3));
            [J4(:, 6), J6(:, 6)] = calculateJ4J6(J1(:, 2), J2(:, 3), J3(:, 3), J5(:, 6), s5_low_right_flip);
            [J4(:, 5), J6(:, 5)] = calculateJ4J6(J1(:, 2), J2(:, 3), J3(:, 3), J5(:, 5), s5_low_right_nonflip);
        end

        JointAngle(:, 1:4, 1) = repmat(checklimit(J1(:, 1), Delta_Limitation.Joint.Pos(1, :)), 1, 4);
        JointAngle(:, 5:8, 1) = repmat(checklimit(J1(:, 2), Delta_Limitation.Joint.Pos(1, :)), 1, 4);

        JointAngle(:, 1:2, 2) = repmat(checklimit(J2(:, 1), Delta_Limitation.Joint.Pos(2, :)), 1, 2);
        JointAngle(:, 3:4, 2) = repmat(checklimit(J2(:, 2), Delta_Limitation.Joint.Pos(2, :)), 1, 2);
        JointAngle(:, 5:6, 2) = repmat(checklimit(J2(:, 3), Delta_Limitation.Joint.Pos(2, :)), 1, 2);
        JointAngle(:, 7:8, 2) = repmat(checklimit(J2(:, 4), Delta_Limitation.Joint.Pos(2, :)), 1, 2);

        JointAngle(:, 1:2, 3) = repmat(checklimit(J3(:, 1), Delta_Limitation.Joint.Pos(3, :)), 1, 2);
        JointAngle(:, 3:4, 3) = repmat(checklimit(J3(:, 2), Delta_Limitation.Joint.Pos(3, :)), 1, 2);
        JointAngle(:, 5:6, 3) = repmat(checklimit(J3(:, 3), Delta_Limitation.Joint.Pos(3, :)), 1, 2);
        JointAngle(:, 7:8, 3) = repmat(checklimit(J3(:, 4), Delta_Limitation.Joint.Pos(3, :)), 1, 2);

        for k =1 : 8
            JointAngle(:, k, 4) = checklimit(J4(:, k), Delta_Limitation.Joint.Pos(4, :));
            JointAngle(:, k, 5) = checklimit(J5(:, k), Delta_Limitation.Joint.Pos(5, :));
            JointAngle(:, k, 6) = checklimit(J6(:, k), Delta_Limitation.Joint.Pos(6, :));
        end

%         JointAngle(:, 1:4, 1) = repmat(J1(:, 1), 1, 4);
%         JointAngle(:, 5:8, 1) = repmat(J1(:, 2), 1, 4);
% 
%         JointAngle(:, 1:2, 2) = repmat(J2(:, 1), 1, 2);
%         JointAngle(:, 3:4, 2) = repmat(J2(:, 2), 1, 2);
%         JointAngle(:, 5:6, 2) = repmat(J2(:, 3), 1, 2);
%         JointAngle(:, 7:8, 2) = repmat(J2(:, 4), 1, 2);
% 
%         JointAngle(:, 1:2, 3) = repmat(J3(:, 1), 1, 2);
%         JointAngle(:, 3:4, 3) = repmat(J3(:, 2), 1, 2);
%         JointAngle(:, 5:6, 3) = repmat(J3(:, 3), 1, 2);
%         JointAngle(:, 7:8, 3) = repmat(J3(:, 4), 1, 2);
% 
%         JointAngle(:, :, 4) = J4;
%         JointAngle(:, :, 5) = J5;
%         JointAngle(:, :, 6) = J6;


    else
        JointAngle = ones(n,8,6) * Inf;
    end

    function J2 = calculateJ2(J1 ,J3)
        C2 = ((J5z - d_DH(1)).*(-d_DH(4)*cos(J3) - a_DH(3)*sin(J3)) ...
             - (J5x./cos(J1) - a_DH(1)).*(-a_DH(2) - a_DH(3)*cos(J3) + d_DH(4)*sin(J3))) ...
         ./   ((-a_DH(3)*sin(J3) - d_DH(4)*cos(J3)).^2 + (-a_DH(2) - a_DH(3)*cos(J3) + d_DH(4)*sin(J3)).^2);

        S2 = ( J5x./cos(J1) - a_DH(1) - (a_DH(2) + a_DH(3)*cos(J3) - d_DH(4)*sin(J3)).*C2 ) ...
         ./   (-d_DH(4)*cos(J3) - a_DH(3)*sin(J3));
    
        J2 = wrapToPi(atan2(S2, C2) - theta_DH(2));
    end

    function [J3_up, J3_low, s3_up, s3_low] = calculateJ3(J1)
        A = ( (J5x./cos(J1) - a_DH(1)).^2 + (J5z - d_DH(1)).^2 - a_DH(2)^2 - a_DH(3)^2 - d_DH(4)^2) ...
                        / (sqrt((2*a_DH(2)*a_DH(3))^2 + (2*a_DH(2)*d_DH(4))^2));
        
        J3_up  = wrapToPi( acos(A) - atan2(d_DH(4), a_DH(3)));
        J3_low = wrapToPi(-acos(A) - atan2(d_DH(4), a_DH(3)));
        
        [J3_up, s3_up] = checkJ3singular(J3_up);
        [J3_low, s3_low] = checkJ3singular(J3_low);

        function [J3, s3] = checkJ3singular(J3)
            % 奇異點 cos(J3 + atan2(d_DH(4), d_DH(3))) > 1 、 J3 = -atan2(d_DH(4), a_DH(3)) 表手臂完全伸直
            % J3 無解
            if ~isempty(find((A > 1 | A < -1) == true, 1))
                s3 = true;
                J3 = ones(size(A), 1) * Inf;
            else
                s3 = false;
            end

            % J3 手臂伸直狀態
            if ~isempty(find((abs(J3 + atan2(d_DH(4), a_DH(3))) < 0.01) == true, 1))
                s3 = true;
                J3 = ones(size(A)) * Inf;
            else
                s3 = false;
            end

        end
    end

    function [J5_flip, J5_nonflip, s5_flip, s5_nonflip ] = calculateJ5(J1, J2, J3)
        J2 = J2 + theta_DH(2);
        J5_flip = wrapToPi(acos( -t13.*cos(J1).*sin(J2 + J3) - t23.*sin(J1).*sin(J2 + J3) - t33.*cos(J2 + J3)));
        J5_nonflip = -J5_flip;

        [J5_flip, s5_flip] = checkJ5singuar(J5_flip);
        [J5_nonflip, s5_nonflip] = checkJ5singuar(J5_nonflip);

        function [J5, s5] = checkJ5singuar(J5)
            if ~isempty(find(abs(J5) < 0.001, 1))
                s5 = 1;
                J5 = ones(length(J5), 1)*Inf;
            else
                s5 = 0;
            end
        end
    end

    function [J4, J6] = calculateJ4J6(J1, J2, J3, J5, singular)
        J2 = J2 + theta_DH(2);

        if singular == 1
           J4 = ones(length(J1), 1)*Inf;
           J6 = J4;
        else
            J4 = wrapToPi(atan2((t13.*sin(J1) - t23.*cos(J1)) ./ -sin(J5), ...
                                (t13.*cos(J1).*cos(J2 + J3) + t23.*sin(J1).*cos(J2 + J3) - t33.*sin(J2 + J3)) ./ -sin(J5)));
    
            J6 = wrapToPi(atan2((-t12.*cos(J1).*sin(J2 + J3) - t22.*sin(J1).*sin(J2 + J3) - t32.*cos(J2 + J3)) ./ -sin(J5), ...
                                (-t11.*cos(J1).*sin(J2 + J3) - t21.*sin(J1).*sin(J2 + J3) - t31.*cos(J2 + J3)) ./  sin(J5)));
        end

    end

    function Output = checklimit(Joint, limit)
        if min(min(Joint)) < limit(1) || max(max(Joint)) > limit(2)
            Output = ones(size(Joint))*Inf;
        else 
            Output = Joint;
        end
    end

end