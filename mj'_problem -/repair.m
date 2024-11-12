function X_repaired = repair(X)
    % repair - 将实数数组 X 转换为字符数组，修复相邻重复字符，最终转换回实数数组
    %
    % 输入:
    %   X - 实数数组，值范围在 [0, 1]
    % 输出:
    %   X_repaired - 修复后的实数数组，与原始 X 具有相同大小
    
    % 将实数数组 X 转换为字符数组
    X_char = binary_to_string(X);

    % 如果第一个字符是 'I' 或 'i'，将其随机改为 'R' 或 'T'
    if X_char(1) == 'I' || X_char(1) == 'i'
        X_char(1) = randsample(['R', 'T'], 1);
    end

    % 字符选项集，用于替换字符（假设只包含 'T', 'R', 'I', 'i' 四种）
    chars = 'TRIi';

    % 检查并替换相邻相同字符，直到满足条件
    while true
        has_adjacent_duplicates = false;

        % 循环遍历字符数组，检查相邻重复字符
        for i = 1:length(X_char) - 1
            if X_char(i) == X_char(i + 1)
                % 如果发现相邻字符相同，则随机替换一个
                has_adjacent_duplicates = true;

                % 随机选择一个不同的字符来替换 X_char(i + 1)
                new_char = X_char(i);
                while new_char == X_char(i)
                    new_char = chars(randi(length(chars)));  % 随机选择一个不同字符
                end
                X_char(i + 1) = new_char;  % 替换相邻重复字符
            end
        end

        % 如果不存在相邻重复字符，退出循环
        if ~has_adjacent_duplicates
            break;
        end
    end

    % 将修复后的字符数组转换回实数数组
    X_repaired = string_to_binary(X_char);
end


function result = binary_to_string(real_code)
    % binary_to_string - 将实数数组转换为对应的字符数组
    % 输入: 实数数组 real_code，范围在 [0, 1]
    % 输出: 字符串 result

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

function real_code = string_to_binary(X_char)
    % string_to_binary - 将字符数组转换为实数数组
    % 输入: 字符数组 X_char
    % 输出: 实数数组 real_code，值范围在 [0, 1]

    % 初始化空的实数数组
    real_code = zeros(1, length(X_char));

    % 根据字符映射回对应的实数区间
    for o = 1:length(X_char)
        if X_char(o) == 'T'
            real_code(o) = rand() * 0.25;  % 映射到 [0, 0.25)
        elseif X_char(o) == 'R'
            real_code(o) = 0.25 + rand() * 0.25;  % 映射到 [0.25, 0.5)
        elseif X_char(o) == 'I'
            real_code(o) = 0.5 + rand() * 0.25;  % 映射到 [0.5, 0.75)
        elseif X_char(o) == 'i'
            real_code(o) = 0.75 + rand() * 0.25;  % 映射到 [0.75, 1]
        else
            error('无效的字符: %s', X_char(o));
        end
    end
end
