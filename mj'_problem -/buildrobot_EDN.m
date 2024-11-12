function [robot_type,robotArm,joint_number,length_robot] = buildrobot_EDN(X)
%BUILDROBOT_EDN 此处显示有关此函数的摘要
%   此处显示详细说明
TT = binary_to_string(X); % 转换为字符串
flag = false;
j = 1;
a2 = 0;
length_robot = 0; %机械臂长度
joint_number = 0;
robot_type = true;  % 初始化robot_type为true

for i = 1:length(TT)
    % 检查是否有两个连续相同的字母
    if i > 1 && TT(i) == TT(i-1)
        robot_type = false;
%         disp('出现连续相同的模块，机器人构建失败。');
        break;
    end
    
    % 起始模块
    if TT(i) == 'T' && i == 1
        A(j,:) = [0 0 0 0 0];
        A(j+1,:) = [0 0 0 pi/2 0];
        a2 = 27;
        j = j+2;
        length_robot = length_robot + 97 + 27;
    elseif TT(i) == 'R' && i == 1
        A(j,:) = [0 0 0 pi/2 0];
        j = j+1;
        a2 = 37;
        length_robot = length_robot + 37 + 37;
    elseif i == 1
        if TT(i) == 'I' || TT(i) == 'i'
            flag = true;  %如果起始模块为连杆模块，flag为true，
            break
        end
    end
    
    % 后续模块
    if TT(i) == 'T' && i ~= 1
        A(j,:) = [0 97 + a2 0 pi/2 0];
        A(j+1,:) = [0 0 0 pi/2 0];
        a2 = 27;
        j = j + 2;
        length_robot = length_robot + 27 + 97;
    elseif TT(i) == 'R' && i ~= 1
        A(j,:) = [0 0 37 + a2 pi/2 0];
        a2 = 37;
        j = j + 1;
        length_robot = length_robot + 37 + 37;
    elseif TT(i) == 'i'
        a2 = a2 + 275;
        length_robot = length_robot + 275;
    elseif TT(i) == 'I'
        a2 = a2 + 380;
        length_robot = length_robot + 380;
    end
end

% 构建机器人
if ~flag && robot_type && size(A,1) >= 6
    for u = 1 : size(A,1)
        S(u) = Link([A(u,:)], 'modified');
    end
    robotArm1 = SerialLink(S);
    robotArm2 = SerialLink(S);
    robotArm3 = SerialLink(S);
    robotArm4 = SerialLink(S);
    robotArm  = {robotArm1, robotArm2, robotArm3, robotArm4}; 
    joint_number = size(A, 1);
else
    robot_type = false;
    robotArm = 0;
end


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

end
