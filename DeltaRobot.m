classdef DeltaRobot< handle
    properties
        Link_num = 7;
        Link_orgin
        Link_rot
        Joint
        Endeffector
        constant = 1;
        VIEW = [ 13 , 22 ]
    end

    methods
        % 讀取建模資料
        function this = DeltaRobot(path)
            for i = 1 : this.Link_num
                tpath = path + "\" + string(i-1) + ".stl";
                TR = stlread(tpath);
                this.Link_orgin{i}.Points = TR.Points*0.1*this.constant;
                this.Link_orgin{i}.Connect = TR.ConnectivityList;
                this.Link_rot{i}.Points = this.Link_orgin{i}.Points;
                this.Link_rot{i}.Connect = this.Link_orgin{i}.Connect;
                this.Endeffector = [];
            end
        end

        % 對初始建模做平移與旋轉
        function Link_RT(this, R, T, link)
            link = link + 1;
            alpha = deg2rad(R(:, 1));
            beta =  deg2rad(R(:, 2)); 
            gamma = deg2rad(R(:, 3));
            Rmatirx = [ cos(alpha).*cos(beta),   cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
                                                          sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
                        sin(alpha).*cos(beta),   sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
                                                         -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
                       -sin(beta),               cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];
            this.Link_rot{link}.Points = (Rmatirx*this.Link_orgin{link}.Points')';
            this.Link_rot{link}.Points = this.Link_rot{link}.Points + T;
        end

        % 順項運動學
        function Forward_Kinematic(this, Angle, Axis, DH)
            theta_DH = DH(:, 1);
            d_DH =     DH(:, 2);
            a_DH =     DH(:, 3);
            alpha_DH = DH(:, 4);
            Angle = deg2rad(Angle);
            T = eye(4);
            for i = 1 : Axis
                Ti = [cos(Angle(:, :, i) + theta_DH(i)), -sin(Angle(:, :, i) + theta_DH(i))*cos(alpha_DH(i)),  sin(Angle(:, :, i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*cos(Angle(:, :, i) + theta_DH(i));
                      sin(Angle(:, :, i) + theta_DH(i)),  cos(Angle(:, :, i) + theta_DH(i))*cos(alpha_DH(i)), -cos(Angle(:, :, i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*sin(Angle(:, :, i) + theta_DH(i));
                                                0,                              sin(alpha_DH(i)),                              cos(alpha_DH(i)),                             d_DH(i); 
                                                0,                                             0,                                             0,                                   1;];
                T = T * Ti;
            end    
            this.Joint.Pos{Axis} = T(1:3, 4);
            this.Joint.Dir{Axis} = T(1:3, 1:3);
        end

        % 回傳尤拉角旋轉矩陣
        function Output = Euler(Euler)
            alpha = Euler(:, 1);
            beta = Euler(:, 2); 
            gamma = Euler(:, 3);
            Output = [ cos(alpha).*cos(beta), cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
                       sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
                       sin(alpha).*cos(beta), sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
                      -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
                      -sin(beta),    cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];

        end

        % 暫時畫圖工具
        function temp_Draw(this, link, color)
            link = link + 1;
            patch('Vertices', this.Link_rot{link}.Points, ...
                  'Faces', this.Link_rot{link}.Connect, ...
                  'FaceVertexCData', color, ...
                  'FaceColor', 'flat')
            hold on 
        end

        % 對旋轉軸旋轉
        function Rot(this, Axis, Angle, direction, origin)

            this.Link_rot{Axis+1}.Points = MyRotate(this.Link_rot{Axis+1}.Points, direction, Angle, origin);
            
            function  newPoints = MyRotate(Points, direction, Angle, origin)
                % 罗德里格旋转公式
                v = Points - origin';
                direction = direction/norm(direction);
                k = [ones(1, length(v(:, 1)))*direction(1); ...
                     ones(1, length(v(:, 1)))*direction(2); ...
                     ones(1, length(v(:, 1)))*direction(3)];
                v_rot = v*cos(Angle) + (cross(k, v').*sin(Angle))' + (k.*dot(k, v')*(1 - cos(Angle)))';
                newPoints = origin' + v_rot; 
            end
        end

        % 正式畫圖
        function Draw(this, Angle)
            DH = DH_MDH('DH');
            % 取得各軸位置與座標方向
            for i = 1 : 6
                this.Forward_Kinematic(Angle, i, DH);
            end

            theta_DH = DH(:, 1);
            Angle = pagetranspose(deg2rad(Angle));

            % 畫出各軸位置與座標方向
            plot3([0, 20], [0, 0], [0, 0], 'g', 'linewidth', 2); hold on
            plot3([0, 0], [0, 20], [0, 0], 'b', 'linewidth', 2); hold on
            plot3([0, 0], [0, 0], [0, 20], 'r', 'linewidth', 2); hold on
            view(3); axis vis3d; axis("equal"); grid on;
            xlabel('x (cm)'); ylabel('y (cm)'); zlabel('z (cm)'); view(this.VIEW)
            xlim([-50 50]); ylim([-30 30]); zlim([-10 80])
            for i = 1 : 6
                xbase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 1)];
                ybase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 2)];
                zbase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 3)];
                plot3(xbase(1, :), xbase(2, :), xbase(3, :), 'k', 'linewidth', 2); hold on
                plot3(ybase(1, :), ybase(2, :), ybase(3, :), 'k', 'linewidth', 2); hold on
                plot3(zbase(1, :), zbase(2, :), zbase(3, :), 'k', 'linewidth', 2); hold on
            %  gbr
            end

            plot3([0, this.Joint.Pos{1}(1)], ...
                  [0, this.Joint.Pos{1}(2)], ...
                  [0, this.Joint.Pos{1}(3)], 'k', 'linewidth', 2); hold on
            for i = 1 : 5
                plot3([this.Joint.Pos{i}(1), this.Joint.Pos{i+1}(1)], ...
                      [this.Joint.Pos{i}(2), this.Joint.Pos{i+1}(2)], ...
                      [this.Joint.Pos{i}(3), this.Joint.Pos{i+1}(3)], 'k', 'linewidth', 2); hold on
            end

            this.Endeffector = [this.Endeffector; this.Joint.Pos{6}'];
            plot3(this.Endeffector(:, 1), this.Endeffector(:, 2), this.Endeffector(:, 3), 'r', 'linewidth', 2);


            % R : ZYX
            % link 0 
            this.Link_RT([-90, 0, 0], [-3.7500, 0, 11.61], 0);
            this.temp_Draw(0, [255, 0, 0]);
            plot3(0, 0, max(this.Link_rot{1}.Points(:, 3)), '-k*','MarkerSize',12)
            
            % link 1
            this.Link_RT([180, 0, 0], [0.4842, 0, 33.4482], 1);
            this.Rot(1, Angle(1), [0, 0, 1], [0, 0, 0]')
            this.temp_Draw(1, [0, 255, 0]);
            
            % link 2
            this.Link_RT([90, 0, 90], [2.2166, 0, 54.4517], 2);
            this.Rot(2, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(2, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.temp_Draw(2, [255, 0, 0]);
            
            % R : ZXY
            % link 3
            this.Link_RT([180, -90, 90], [6.0214, 0, 73.5457], 3);
            this.Rot(3, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(3, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(3, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.temp_Draw(3, [0, 255, 0]);
            
            % link 4
            this.Link_RT([90, 0, 90], [28.8959, 0, 76.1113], 4);
            this.Rot(4, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(4, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(4, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.Rot(4, Angle(4), this.Joint.Dir{3}(:, 3), this.Joint.Pos{3});
            this.temp_Draw(4, [255, 0, 0]);
            
            % R : ZYX
            % link 5
            this.Link_RT([90, 180, 90], [38.4049, -0.5000, 75.4671], 5);
            this.Rot(5, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(5, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(5, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.Rot(5, Angle(4), this.Joint.Dir{3}(:, 3), this.Joint.Pos{3});
            this.Rot(5, Angle(5), this.Joint.Dir{4}(:, 3), this.Joint.Pos{4});
            this.temp_Draw(5, [0, 255, 0]);
            
            % R : ZXY
            % link 6
            this.Link_RT([0, -90, 180], [43.7008, 0, 75.5090], 6);
            this.Rot(6, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(6, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(6, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.Rot(6, Angle(4), this.Joint.Dir{3}(:, 3), this.Joint.Pos{3});
            this.Rot(6, Angle(5), this.Joint.Dir{4}(:, 3), this.Joint.Pos{4});
            this.Rot(6, Angle(6), this.Joint.Dir{5}(:, 3), this.Joint.Pos{5});
            this.temp_Draw(6, [255, 0, 0]);

            hold off
        end


        % 逆向運動學
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


    end




end