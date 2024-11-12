function Popobj = nullspace_new(D, joint_number, robot, joint_angel, ia)
    % nullspace_obj 计算零空间相关的指标
    % 
    % 输入参数:
    %   D            - 点数据矩阵
    %   joint_number - 关节数量
    %   robot        - 机器人对象
    %   joint_angel  - 关节角度矩阵
    %   ia           - 索引数组
    %
    % 输出参数:
    %   Popobj       - 计算出的目标值向量 [1/longest_x, 1/longest_y, 1/longest_z, X_ma, Y_ma, Z_ma]

    % 计算每个维度上的长度和索引
    [DX, ~, longest_x, indice_x] = compute_length(D, 1);
    [DY, ~, longest_y, indice_y] = compute_length(D, 2);
    [DZ, ~, longest_z, indice_z] = compute_length(D, 3);
    
    % 若自由度大于等于6，则存在零空间
    if joint_number >= 6
        % 计算 X 方向的最大可操作性
        if longest_x ~= 0
            X_traj = DX(indice_x(1):indice_x(end), :);
            X_ma_null = zeros(longest_x, 1);
            for i = 1:size(X_traj, 1)
                [~, row_indice] = ismember(X_traj(i, :), D, "rows");
                row_indice = ia(row_indice);
                X_ma_null(i) = manu_compute(robot, joint_angel(row_indice, :), 0);
            end
            X_ma = max(X_ma_null);
        else
            X_ma = 1 / 100000;
        end
        
        % 计算 Y 方向的最大可操作性
        if longest_y ~= 0
            Y_traj = DY(indice_y(1):indice_y(end), :);
            Y_ma_null = zeros(longest_y, 1);
            for i = 1:size(Y_traj, 1)
                [~, row_indice] = ismember(Y_traj(i, :), D, "rows");
                row_indice = ia(row_indice);
                Y_ma_null(i) = manu_compute(robot, joint_angel(row_indice, :), 0);
            end
            Y_ma = max(Y_ma_null);
        else
            Y_ma = 1 / 100000;
        end
        
        % 计算 Z 方向的最大可操作性
        if longest_z ~= 0
            Z_traj = DZ(indice_z(1):indice_z(end), :);
            Z_ma_null = zeros(longest_z, 1);
            for i = 1:size(Z_traj, 1)
                [~, row_indice] = ismember(Z_traj(i, :), D, "rows");
                row_indice = ia(row_indice);
                Z_ma_null(i) = manu_compute(robot, joint_angel(row_indice, :), 0);
            end
            Z_ma = max(Z_ma_null);
        else
            Z_ma = 1 / 100000;
        end
        
        % 如果最长路径为0, 则设置为较小值
        if longest_x == 0
            longest_x = 1 / 50000;
        end
        if longest_y == 0
            longest_y = 1 / 50000;
        end
        if longest_z == 0
            longest_z = 1 / 50000;
        end
        
        % 计算目标值向量
        Popobj = [1 / longest_x, 1 / longest_y, 1 / longest_z, X_ma, Y_ma, Z_ma];
    else
        Popobj = 50000*ones(1,6);% 若自由度小于6，不存在零空间
    end
end
