function Popobj = all_obj(obj,Popdec)
% tic;
% Popdec = rand(2,30);
% Popobj = zeros(size(Popdec,1),10);
for i = 1:size(Popdec)
    X = Popdec(i,:);
% X = rand(1,30);
%% D-H自动建模
% j=1;
% a2=0;
N = 10000;
% length_robot=0; %机械臂长度
Ntheta  = 60;           % number of intervals for theta
Nh      = 30;

[robot_type,robotArm,joint_number,length_robot] = buildrobot_EDN(X(1:obj.D-24));
%% 函数值计算

if robot_type
    deltaX  = length_robot/5;            % interval of position X
    deltaZ  = length_robot/5;            % interval of position Z
    robot = robotArm{1};
    joint_angel = random ('Uniform',0,1,N,joint_number) * pi;
    [R_BA1,Jacob,endlocation,g]=endlocation_III(robot,joint_number,N);
    Popobj_initial = obj_multiarm(X(obj.D-23:end),length_robot,joint_number,robotArm,R_BA1,Jacob,endlocation,N);  % 计算适应度函数值
   
    for p=1:N
        C(p,:)=config1(g{p},Ntheta,Nh,deltaX,deltaZ);
    end
    [D,ia, ~] = unique(C, 'rows');
    Popobj_nullspace = nullspace_new(D, joint_number, robot, joint_angel, ia);
else
    Popobj_initial  = 50000*ones(1,6);
    Popobj_nullspace = 50000*ones(1,2);
end
Popobj(i,:) = [Popobj_nullspace   Popobj_initial];
% toc;
end
% toc;
end

