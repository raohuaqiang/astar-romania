%% 罗马尼亚A*算法 - 简化版
clear; clc;

fprintf('========== 罗马尼亚A*算法 ==========\n');
fprintf('起点：Arad, 终点：Bucharest\n\n');

%% 1. 邻接表（城市之间的实际距离）
adj = struct();
adj.Arad = {'Zerind',75; 'Sibiu',140; 'Timisoara',118};
adj.Zerind = {'Arad',75; 'Oradea',71};
adj.Oradea = {'Zerind',71; 'Sibiu',151};
adj.Sibiu = {'Arad',140; 'Oradea',151; 'Fagaras',99; 'Rimnicu_Vilcea',80};
adj.Timisoara = {'Arad',118; 'Lugoj',111};
adj.Lugoj = {'Timisoara',111; 'Mehadia',70};
adj.Mehadiag = {'Lugoj',70; 'Dobreta',75};
adj.Dobreta = {'Mehadia',75; 'Craiova',120};
adj.Craiova = {'Dobreta',120; 'Rimnicu_Vilcea',146; 'Pitesti',138};
adj.Rimnicu_Vilcea = {'Sibiu',80; 'Craiova',146; 'Pitesti',97};
adj.Fagaras = {'Sibiu',99; 'Bucharest',211};
adj.Pitesti = {'Rimnicu_Vilcea',97; 'Craiova',138; 'Bucharest',101};
adj.Bucharest = {'Fagaras',211; 'Pitesti',101; 'Giurgiu',90; 'Urziceni',85};
adj.Giurgiu = {'Bucharest',90};
adj.Urziceni = {'Bucharest',85; 'Hirsova',98; 'Vaslui',142};
adj.Hirsova = {'Urziceni',98; 'Eforie',86};
adj.Eforie = {'Hirsova',86};
adj.Vaslui = {'Urziceni',142; 'Iasi',92};
adj.Iasi = {'Vaslui',92; 'Neamt',87};
adj.Neamt = {'Iasi',87};

%% 2. 启发式函数h(n)：到Bucharest的直线距离
h = struct();
h.Arad = 366; h.Zerind = 374; h.Oradea = 380; h.Sibiu = 253;
h.Timisoara = 329; h.Lugoj = 244; h.Mehadiag = 241; h.Dobreta = 242;
h.Craiova = 160; h.Rimnicu_Vilcea = 193; h.Fagaras = 178;
h.Pitesti = 193; h.Bucharest = 0; h.Giurgiu = 77; h.Urziceni = 80;
h.Hirsova = 151; h.Eforie = 161; h.Vaslui = 199; h.Iasi = 226; h.Neamt = 234;

%% 3. 调用A*算法
[start, goal] = deal('Arad', 'Bucharest');
[path, cost] = a_star_simple(adj, h, start, goal);

%% 4. 显示结果
fprintf('========== 搜索结果 ==========\n');
fprintf('最优路径：');
for i = 1:length(path)
    if i < length(path)
        fprintf('%s → ', path{i});
    else
        fprintf('%s\n', path{i});
    end
end
fprintf('\n总距离：%d km\n', cost);

fprintf('\n========== 路径详情 ==========\n');
total = 0;
for i = 1:length(path)-1
    from_city = path{i};
    to_city = path{i+1};
    
    % 查找两城市间的距离
    if isfield(adj, from_city)
        neighbors = adj.(from_city);
        for j = 1:size(neighbors, 1)
            if strcmp(neighbors{j,1}, to_city)
                distance = neighbors{j,2};
                total = total + distance;
                fprintf('%s → %s: %d km (累计: %d km)\n', from_city, to_city, distance, total);
                break;
            end
        end
    end
end
fprintf('验证总距离：%d km\n', total);
fprintf('================================\n');

%% A*算法核心函数
function [path, cost] = a_star_simple(adj, h, start, goal)
    % Open List: 待考察节点
    open_list = {start};                     % 城市名列表
    g_score = containers.Map();              % 实际代价 g(n)
    f_score = containers.Map();              %评估代价 f(n) = g(n) + h(n)
    came_from = containers.Map();            % 记录父节点
    
    % 初始化起点
    g_score(start) = 0;
    f_score(start) = h.(start);
    
    while ~isempty(open_list)
        % 找到f值最小的节点
        [~, idx] = min(cellfun(@(x) f_score(x), open_list));
        current = open_list{idx};
        
        % 如果到达终点
        if strcmp(current, goal)
            % 回溯路径
            path = {current};
            cost = g_score(current);
            while isKey(came_from, current)
                current = came_from(current);
                path = [{current}, path];
            end
            return;
        end
        
        % 从open_list中移除当前节点
        open_list(idx) = [];
        
        % 遍历所有邻居
        if isfield(adj, current)
            neighbors = adj.(current);
            for i = 1:size(neighbors, 1)
                neighbor = neighbors{i,1};
                move_cost = neighbors{i,2};
                
                % 计算新的g值
                tentative_g = g_score(current) + move_cost;
                
                % 如果邻居不在g_score中，或者找到更短路径
                if ~isKey(g_score, neighbor) || tentative_g < g_score(neighbor)
                    % 更新父节点
                    came_from(neighbor) = current;
                    
                    % 更新g值和f值
                    g_score(neighbor) = tentative_g;
                    f_score(neighbor) = tentative_g + h.(neighbor);
                    
                    % 如果邻居不在open_list中，添加进去
                    if ~ismember(neighbor, open_list)
                        open_list{end+1} = neighbor;
                    end
                end
            end
        end
    end
    
    % 如果找不到路径
    path = {};
    cost = Inf;
    fprintf('警告：未找到从 %s 到 %s 的路径！\n', start, goal);
end