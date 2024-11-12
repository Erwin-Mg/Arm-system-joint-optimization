classdef MJ_Problem < PROBLEM
% <many><real><expensive>
%
%   This problem provides a general framework, whose details can be defined
%   by the inputs of the constructor.
%
% All the acceptable properties:
%   encoding  	<string>            encoding scheme of each decision variable (1.real 2.integer 3.label 4.binary 5.permutation)
%   lower    	<vector>            lower bound of each decision variable
%   upper      	<vector>            upper bound of each decision variable
%   initFcn     <function handle>   function for initializing solutions
%   evalFcn     <function handle>   function for evaluating solutions
%   decFcn      <function handle>   function for repairing invalid solutions
%   objFcn     	<function handle>   objective functions
%   conFcn     	<function handle>   constraint functions
%   gradFcn     <function handle>   function for calculating the gradients of objectives and constraints
%   data        <any>               data of the problem

%------------------------------- Copyright --------------------------------
% Copyright (c) 2024 BIMK Group. You are free to use the PlatEMO for
% research purposes. All publications which use this platform or any code
% in the platform should acknowledge the use of "PlatEMO" and reference "Ye
% Tian, Ran Cheng, Xingyi Zhang, and Yaochu Jin, PlatEMO: A MATLAB platform
% for evolutionary multi-objective optimization [educational forum], IEEE
% Computational Intelligence Magazine, 2017, 12(4): 73-87".
%--------------------------------------------------------------------------

    methods
        %% Constructor
        % 定义问题基本参数
        function Setting(obj)
            obj.M = 8;  %定义目标函数数量
            obj.D = 32;  %随机定义模块数量
%           if isempty(obj.D); obj.D = 5; end
            obj.lower = zeros(1,obj.D) ;
            obj.upper = ones(1,obj.D) ;            
            obj.encoding = ones(1,obj.D);   %1（实数）2（整数）、3（标签）、4（二进制数）或5（序列编号   
            obj.maxFE = 1000;
        end
        function PopObj = CalObj(obj,PopDec) 
            PopObj = all_obj(obj,PopDec);   % 
        end
        function PopCon = CalCon(obj,PopDec)
            for i = 1:size(PopDec)
                X = PopDec(i,:);
                [robot_type,~,~,~] = buildrobot_EDN(X(1:obj.D-24));
                if robot_type
                    PopCon(i,:) = 0;
                else
                    PopCon(i,:) = 1;%违反约束
                end
            end
        end
        
        function PopDec = CalDec(obj,PopDec)  %解的修复函数
            for i = 1:size(PopDec)
                X = PopDec(i,:);
                [robot_type,~,~,~] = buildrobot_EDN(X(1:obj.D-24));
                if ~robot_type
                    X = repair(X(1:obj.D-24));%违反约束
                    PopDec(i,1:obj.D-24) = X;
                end
            end
        end
    end
end
