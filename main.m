clc,clear,close all;

path = "D:\成大\碩一\新訓\我的\8_台達機械手臂\simplify";

robot = DeltaRobot(path);

EulerAngle = [];
Position = [];
for i = 1 : 360
    EulerAngle = [EulerAngle; 0, 0, pi;];
    Position = [Position; 30 + 5*cos(i*pi/180), 5*sin(i*pi/180), 30;];
end

theta = Inverse_Kinematic(EulerAngle, Position);
theta_lim = Inverse_Kinematic_lim(EulerAngle, Position);
%%
thetadeg(:, 1, :) = rad2deg(theta);
thetadeg_lim = rad2deg(theta_lim);

% for i = 1 : 6
%     error(:, i) = thetadeg(:, i) - thetadeg_lim(:, 4, i);
% end

%%
for j = 1 : 8
    p = [];
    pic_num = 1;
    figure(j)
    for i = 1 : 20 : 360
        robot.Draw(thetadeg_lim(i, j, :));
%         robot.Draw(thetadeg(i, 1, :));
    
        drawnow;
        F = getframe(gcf);
        I = frame2im(F);
        [I,map]=rgb2ind(I,256);
        
        if pic_num == 1
            imwrite(I, map, "8pose/test" + string(j) +".gif", 'gif', 'Loopcount', inf, 'DelayTime', 0.2);
        else
            imwrite(I, map, "8pose/test" + string(j) +".gif", 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
        end
        
        pic_num = pic_num + 1;
    
        pause(0.001)
    end
end
%%
% thetadeg(1, 1, :) = [0, 0, 0, 0, 0, 0];
% robot.Draw(thetadeg(1, 1, :));
