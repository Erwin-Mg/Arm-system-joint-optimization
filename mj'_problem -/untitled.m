% L = 17;
% II =2;%小正方体内点数

    relative_jacob13=cell(1,myLength(coop13));
    MM_relative_jacob13 = zeros(1,myLength(coop13));
    if ~isempty(coop13)
        for i=1:myLength(coop13)
            if length(J13_3{i})==joint_number
                %计算两末端点向量p
                p = coop13recover(i,:)-coop13(i,:);
                matix1 = - [ eye(3) , -skew(p); zeros(3,3) , eye(3)  ];
                %计算基座臂末端到基座臂基座的旋转矩阵R1
                r_BA = R_BA1{matching_row_indices13(i)};
                %计算两条臂当前关节角下的雅各比矩阵
                JA = J13_1{i};
                JB = J13_3{i};
                matix2 = [ r_BA, zeros(3,3); zeros(3,3), r_BA ];
                %计算基座臂末端到操作臂基座的旋转矩阵
                R_AB = base1'*base3;
                R2 = r_BA\R_AB;
                matix3 = [ R2, zeros(3,3); zeros(3,3), R2 ];
                relative_jacob13{i} = [ matix1*matix2*JA , matix3*JB];
                MM_relative_jacob13(i) = sqrt(det(relative_jacob13{i}*relative_jacob13{i}'));
            else
                MM_relative_jacob13(i)=0;
            end
        end
        MM_relative_jacob(2)=max(MM_relative_jacob13);
    else
        MM_relative_jacob(2)=0;
    end
%% 
% % 设置球体的直径
% sphere_diameter = 845;
% sphere_radius = sphere_diameter / 2;
% 
% % 创建球体
% [x, y, z] = sphere;  % 生成单位球的坐标
% x = x * sphere_radius;  % 缩放球体到指定直径
% y = y * sphere_radius;
% z = z * sphere_radius;
% 
% % 设置正方体的边长
% cube_side = 17;
% cube_half_side = cube_side / 2;
% 
% % 定义正方体的顶点坐标 (中心在原点)
% vertices = [
%     -cube_half_side, -cube_half_side, -cube_half_side;
%      cube_half_side, -cube_half_side, -cube_half_side;
%      cube_half_side,  cube_half_side, -cube_half_side;
%     -cube_half_side,  cube_half_side, -cube_half_side;
%     -cube_half_side, -cube_half_side,  cube_half_side;
%      cube_half_side, -cube_half_side,  cube_half_side;
%      cube_half_side,  cube_half_side,  cube_half_side;
%     -cube_half_side,  cube_half_side,  cube_half_side;
% ];
% 
% % 定义正方体的面
% faces = [
%     1, 2, 3, 4;
%     5, 6, 7, 8;
%     1, 2, 6, 5;
%     2, 3, 7, 6;
%     3, 4, 8, 7;
%     4, 1, 5, 8;
% ];
% 
% % 绘制球体
% figure;
% hold on;
% surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');  % 半透明球体
% colormap([0 0.5 1]);  % 球体颜色
% axis equal;
% 
% % 绘制正方体
% patch('Vertices', vertices, 'Faces', faces, 'FaceColor', [1 0 0], 'FaceAlpha', 0.5);
% 
% % 设置视图
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('球体和正方体');
% view(3);
% grid on;
% hold off;
% 


function rowCount = myLength(A)
    % myLength 函数返回矩阵 A 的行数

    % 获取矩阵 A 的行数
    rowCount = size(A, 1);
end