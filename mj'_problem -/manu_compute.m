function [mu_ba] = manu_compute(robot, theta, constraint)
%MANU 此处显示有关此函数的摘要
%   此处显示详细说明
num_steps = 1000;
mu_ba = 0;
theta = theta*pi/180;
switch constraint
    case 0 % 位置+姿态约束
        q_plus   = theta;
        q_minus  = theta;
        mu_plus  = zeros(1, num_steps); % 初始化 mu_plus
        mu_minus = zeros(1, num_steps); % 初始化 mu_minus

        % 可视化零空间运动并验证末端位置
        for i = 1:num_steps
            if (any(abs(q_plus) >= pi)) || (any(abs(q_minus) >= pi))
%                 disp('A joint angle reached its limit.');
                break;
            end

            % 计算当前雅可比矩阵和伪逆
            J0_plus = robot.jacob0(q_plus);
            J0_minus = robot.jacob0(q_minus);

            % 计算零空间
            null_space_plus = null(J0_plus);
            null_space_minus = null(J0_minus);

            % 更新关节角度
            q_plus = q_plus + (null_space_plus * 0.001 * randn(size(null_space_plus, 2), 1))';
            q_minus = q_minus - (null_space_minus * 0.001 * randn(size(null_space_minus, 2), 1))';

            % 计算操控性
            mu_plus(i) = sqrt(det(J0_plus * J0_plus'));
            mu_minus(i) = sqrt(det(J0_minus * J0_minus'));
        end
        mu_ba = (mean(mu_plus) + mean(mu_minus)) / 2;

    case 1 % 姿态约束
        % 初始化关节角度
        q_plus = theta;
        q_minus = theta;
        mu_plus = zeros(1, num_steps); % 初始化 mu_plus
        mu_minus = zeros(1, num_steps); % 初始化 mu_minus

        % 可视化零空间运动并验证末端位置
        for i = 1:num_steps
            if (any(abs(q_plus) >= pi)) || (any(abs(q_minus) >= pi))
%                 disp('A joint angle reached its limit.');
                break;
            end

            % 计算当前雅可比矩阵和伪逆
            J0_plus = robot.jacob0(q_plus);
            J0_minus = robot.jacob0(q_minus);

            % 计算零空间
            null_space_plus = null(J0_plus(1:3,:));
            null_space_minus = null(J0_minus(1:3,:));

            % 更新关节角度
            q_plus = q_plus + (null_space_plus * 0.001 * randn(size(null_space_plus, 2), 1))';
            q_minus = q_minus - (null_space_minus * 0.001 * randn(size(null_space_plus, 2), 1))';

            % 计算操控性
            mu_plus(i) = sqrt(det(J0_plus * J0_plus'));
            mu_minus(i) = sqrt(det(J0_minus * J0_minus'));
        end
        mu_ba = (mean(mu_plus) + mean(mu_minus)) / 2;
end
end



