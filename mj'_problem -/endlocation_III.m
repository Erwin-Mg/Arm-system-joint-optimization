% 确定机械臂参数
% 定义D-H参数
function [R_BA,Jacob,endlocation_1,g]=endlocation_III(robot,joint_number,W)
% N=10000; %随机采样数

% robotArm2 = robotArm{2};
% robotArm3 = robotArm{3};
% robotArm4 = robotArm{4};


q.qlim=[-180,180]/180*pi;

j=joint_number;%j=关节数
% Q_1={N,j+3};
endlocation_1=zeros(W,j+3);
qs=zeros(1,j);
Q_1=cell(W,1);
Jacob=cell(W,1);
R_BA=cell(W,1);
for i=1:W
    for u=1:j
        qs(u)=q.qlim(1)+rand*(q.qlim(2)-q.qlim(1));
    end
    Q_1{i}=robot.fkine(qs);
    R_BA{i} = [Q_1{i}.n,Q_1{i}.o,Q_1{i}.a];
    Jacob{i}= robot.jacob0(qs);
%     MF{i}=robot.inertia(qs);
    endlocation_1(i,1:3)=Q_1{i}.t;
    endlocation_1(i,4:j+3)=qs;
    g{i} = Q_1{i}.T;
end

% %% 计算位姿球相关
% N = 1000000;  % 采样次数
% N       = 100000;        % number of samples, for quick run
% % N       = 100000000;    % number of samples, used in the paper
% Ntheta  = 60;           % number of intervals for theta
% Nh      = 30;           % number of intervals for h
% deltaX  = 10;            % interval of position X
% deltaZ  = 10;            % interval of position Z
% joint_number = 7;       % 机器人旋转关节数量
% % 在关节空间内随机采样
% joint_angel = random ('Uniform',0,1,N,joint_number) * pi;
% C=zeros(N,5);
% 
% 
% % 根据机器人根部位姿乘转换矩阵 R
% % robot.base = T;
% tic;
% for i=1:N
%     fk=robotArm1.fkine(joint_angel(i,:));
%     C{i}=fk.T;
% end
% toc;
end



