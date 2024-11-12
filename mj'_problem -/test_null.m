
% Code for calculating the dexterity of three continuum robots combining
% concentric tube mechanisms and cable driven mechanisms.
%
% Please Refer to:
%
% Liao Wu, Ross Crawford, Jonathan Roberts. Dexterity analysis of three 
% 6-DOF continuum robots combining concentric tube mechanisms and cable 
% driven mechanisms. IEEE Robotics and Automation Letters. 2017, 2(2): 
% 514-521.
Popdec = rand(1,6);
Popobj = 50000*ones(size(Popdec,1),3);
for k = 1:size(Popdec,1)
    flag = false;
    X = Popdec(k,:);
    %% set up parameters for simulation
    N       = 50000;        % number of samples, for quick run
    % N       = 100000000;    % number of samples, used in the paper
    Ntheta  = 60;           % number of intervals for theta
    Nh      = 30;           % number of intervals for h
    % deltaX  = 10;            % interval of position X
    % deltaZ  = 10;            % interval of position Z

    TT = binary_to_string(X); % 转换为字符串
    %[0-0.25]为T模块 [0.25-0.5]代码为R模块 [0.5-0.75]代码为I模块 [0.75-0.1]为I模块
    params_array = assign_params_to_pairs(TT);
    %% D-H自动建模
    j=1;
    a2=0;
    length_robot=0; %机械臂长度
    for i = 1:length(TT)
        %起始模块
        if TT(i) == 'T' && i == 1
            A(j,:) = [0 0 0 0 0];
            A(j+1,:) = [0 0 0 pi/2 0];
            a2 = 27;
            j = j+2;
            length_robot = length_robot + 97 + 27;
        elseif  TT(i) == 'R' && i ==1
            A(j,:) = [0 0 0 pi/2 0];
            j = j+1;
            a2 = 37;
            length_robot = length_robot + 37 + 37;
        elseif i==1
            if TT(i) =='I' || TT(i) == 'i'
                Popobj(k,1:3) = 50000*ones(1,3);
                flag = true;  %如果起始模块为连杆模块，flag为true，
                break
            end
        end
        %后续模块
        if TT(i) == 'T' && i~=1
            A(j,:) = [0 97+a2 0 pi/2 0];
            A(j+1,:) = [0 0 0 pi/2 0];
            a2 = 27;
            j = j+2;
            length_robot = length_robot + 27 + 97;
        elseif TT(i) == 'R' && i~=1
            A(j,:) = [0 0 37+a2 pi/2 0];
            a2 = 37;
            j = j+1;
            length_robot = length_robot + 37 + 37;
        elseif TT(i) == 'i'
            a2 = a2 + 275;
            length_robot = length_robot + 275;
        elseif TT(i) == 'I'
            a2 = a2 + 380;
            length_robot = length_robot + 380;
        end
    end
    %如果起始模块为连杆模块，直接跳过此次计算（淘汰此个体）
    if flag == true;
        break;
    end

    for u = 1 : size (A,1)
        S(u) = Link([A(u,:)],'modified');
    end
    deltaX  = length_robot/5;            % interval of position X
    deltaZ  = length_robot/5;            % interval of position Z
    robot = SerialLink(S);
    % robot1.teach;
    % robot.teach


    % 在关节空间内随机采样
    joint_number = size(A,1);
    joint_angel = random ('Uniform',0,1,N,joint_number) * pi;


    if joint_number <6   %自由度小于6则排除
        Popobj(k,1:3) = 50000*ones(1,3);
        continue
    end
    %% calculate the statistics of the configurations of all the samples
    C=zeros(N,5);


    % 根据机器人根部位姿乘转换矩阵 R
    % robot.base = T;

    for p=1:N
        C(p,:)=config1(robot,joint_angel(p,:),Ntheta,Nh,deltaX,deltaZ);
    end

    [D,ia, ~] = unique(C, 'rows');
    % D_indice = [D, ia];                 %保留索引信息
    % joint_angel = joint_angel(ia,:);
    % D=unique(C,'rows');

    % D1 = sortrows(D, [4 5 1 3]);

    [~,~]=unique(D(:,1:3),'rows'); %此处可忽略


    [DX,~,longest_x,indice_x] = compute_length(D,1);
    [DY,~,longest_y,indice_y] = compute_length(D,2);
    [DZ,~,longest_z,indice_z] = compute_length(D,3);
    % 若自由度大于等于6, 则存在零空间
    if joint_number >=6
        %     if longest_x~=0
        %         X_traj = DX(indice_x(1):indice_x(end),:);
        %         X_ma_null = zeros(longest_x,1);
        %         for i=1:size(X_traj,1)
        %             [~,row_indice] = ismember (X_traj(i,:),D,"rows");
        %             row_indice = ia(row_indice);
        %             X_ma_null(i) = manu_compute(robot,joint_angel(row_indice,:),0);
        %         end
        %         X_ma=max(X_ma_null);
        %     else
        %         X_ma = 1/100000;
        %     end
        %
        %     if longest_y~=0
        %         Y_traj = DY(indice_y(1):indice_y(end),:);
        %         Y_ma_null = zeros(longest_y,1);
        %         for i=1:size(Y_traj,1)
        %             [~,row_indice] = ismember (Y_traj(i,:),D,"rows");
        %             row_indice = ia(row_indice);
        %             Y_ma_null(i) = manu_compute(robot,joint_angel(row_indice,:),0);
        %         end
        %         Y_ma=max(Y_ma_null);
        %     else
        %         Y_ma = 1/100000;
        %     end
        %
        %
        %     if longest_z~=0
        %         Z_traj = DZ(indice_z(1):indice_z(end),:);
        %         Z_ma_null = zeros(longest_z,1);
        %         for i=1:size(Z_traj,1)
        %             [~,row_indice] = ismember (Z_traj(i,:),D,"rows");
        %             row_indice = ia(row_indice);
        %             Z_ma_null(i) = manu_compute(robot,joint_angel(row_indice,:),0);
        %         end
        %
        %         Z_ma=max(Z_ma_null);
        %     else
        %         Z_ma = 1/100000;
        %     end

        Popobj(k,:) = [1/longest_x,1/longest_y,1/longest_z];
    else  %若自由度小于6，自由度太少不考虑，跳出当前循环
        Popobj(k,:) = 50000*ones(1,3);
        %[longest_x,longest_y,longest_z,X_ma,Y_ma,Z_ma] = deal(0);
        continue
    end


end



%% 
% function result = binary_to_string(binary_code)
%     % 初始化空字符串
%     result = '';
%     
%     % 遍历二进制数组，每次取两个比特进行映射
%     for o = 1:2:length(binary_code)
%         pair = binary_code(o:o+1);  % 取出每两个比特
%         
%         % 判断二进制对，并根据规则转换为相应的字符
%         if isequal(pair, [1 1])
%             result = [result 'T'];
%         elseif isequal(pair, [0 0])
%             result = [result 'R'];
%         elseif isequal(pair, [1 0])
%             result = [result 'i'];
%         elseif isequal(pair, [0 1])
%             result = [result 'I'];
%         else
%             error('无效的二进制对: [%d %d]', pair(1), pair(2));
%         end
%     end 
% end
function result = binary_to_string(real_code)
    % 初始化空字符串
    result = '';
    
    % 遍历输入数组，对每个实数值进行判断并映射到相应的字符
    for o = 1:length(real_code)
        value = real_code(o);  % 取出每个值
        
        % 判断实数范围并转换为相应的字符
        if value >= 0 && value < 0.25
            result = [result 'T'];
        elseif value >= 0.25 && value < 0.5
            result = [result 'R'];
        elseif value >= 0.5 && value < 0.75
            result = [result 'I'];
        elseif value >= 0.75 && value <= 1
            result = [result 'i'];
        else
            error('无效的数值: %f', value);
        end
    end 
end


function params_array = assign_params_to_pairs(pairs)
    % 初始化结构体数组
    params_array = struct();
    
    % 遍历每个字符，并为每个字符赋值对应的参数
    for h = 1:length(pairs)
        char = pairs(h);  % 取出当前的字符
        
        switch char
            case 'T'
                params_array(h).a = [97,27];
                params_array(h).alpha = pi/2;
            case 'R'
                params_array(h).a = [37, 37];
                params_array(h).alpha = pi/2;
            case 'i'
                params_array(h).a = 275;
                params_array(h).alpha = 0;
            case 'I'
                params_array(h).a = 380;
                params_array(h).alpha = pi/2;
            otherwise
                error('未定义的字符: %s', char);
        end
    end
end





