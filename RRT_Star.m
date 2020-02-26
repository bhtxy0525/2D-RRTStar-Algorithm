%%平面二维RRT*算法，搜索无碰撞路径
%%qinit:起始点
%%qrand:随机采样点
%%qnearest:距离qnearest最近的一个节点
%%qgoal:终止点
clear all;close all;clc
%% 自由建图
map=~zeros(500,500);
map(230:270,50:120) = 0;
map(230:270,200:450) = 0;
map(20:100,350:400) = 0;
map(170:450,350:400) = 0;
map(350:400,20:60) = 0;
imshow(map);
[m,n] = size(map);
%% 定义初始参数,以左上角为原点计数，矩阵第一个数表示y，第二个数表示x
x_init = 20;y_init = 100;  %起始点坐标
x_goal = 480; y_goal = 400;%目标点坐标
stepsize = 20;             %搜索步长
threshold = 20;            %阈值,当qnearst和qgoal间的距离在阈值之内时，代表随机树到达目标点,与stepsize保持同步
RadiusForNer = 50;         %rewire的范围，半径r
MaxIterations = 2000;       %最大迭代次数
display = 1;               %显示上述数据为真
P_handleList = [];
L_handleList = [];
resHandleList= [];
%% 建树并初始化
RRTStarTree.node(1).coord =[x_init y_init];  %树中第一个节点即为起始点
RRTStarTree.node(1).Parent = [x_init y_init] %初始节点的父节点是本身，每个节点都有一个且只有一个父节点
RRTStarTree.node(1).Cost = 0;                %代价为0
RRTStarTree.node(1).ParIndex = 0;            %起始节点的父节点索引，设为0
%% 判断起始点和目标点是否符合要求
if feasiblePoint(RRTStarTree.node(1).coord,map) ==0
	error('起始点不符合地图要求');%%error后面的程序不会被执行
end

if feasiblePoint([x_goal,y_goal],map) ==0
	error('目标点不符合地图要求');%%error后面的程序不会被执行
end
%% 显示地图
if display
	imshow(map);
    rectangle('position',[1 1 size(map)-1],'LineWidth', 2,'edgecolor','k');
    hold on;
    scatter(RRTStarTree.node(1).coord(2),RRTStarTree.node(1).coord(1),100,'sm','filled');
    hold on;
    scatter(y_goal,x_goal,100,'sb','filled');
end
tic; %%程序开始，计算时间
counter = 1;       %%节点数
pathFound = 0;     %%到达目标点附近标志
NumIterations = 0  %%显示迭代次数
for i = 1:MaxIterations
    NumIterations = NumIterations+1
    %step1：在地图中随机采样一个x_rand
    x_rand = rand(1,2).*size(map);    %%rand(m,n) 产生m行n列均匀分布在（0,1）内的伪随机数
    
    %step2：选择离x_rand最近的节点
    for i=1:length(RRTStarTree.node)
        for j=1:2
            ExistNode(i,j) =  RRTStarTree.node(i).coord(1,j);
        end
    end
    [A,MinIndex] = min(distanceCost(ExistNode,x_rand),[],1); %%A代表当前存在的节点与随机点的最近距离，I代表最小距离所在行数
    closestNode = RRTStarTree.node(MinIndex).coord; %%选出来RRT树中的qnearst节点，每次都更新，并不累加
    temp_parent = MinIndex;  %%临时父节点的索引
    temp_Cost = stepsize + RRTStarTree.node(MinIndex).Cost; %%临时累计代价
    
    %step3：扩展得到x_new节点
    theta = atan2(x_rand(1)-closestNode(1),x_rand(2)-closestNode(2)); %向qrand方向扩展一段距离
    x_new = double(int32(closestNode(1:2)+stepsize*[sin(theta) cos(theta)])); %%沿着qneaest到qrand所在直线步进了一段距离,取整了
    %检查新节点是否与周围环境发生碰撞
    if ~checkPath(closestNode(1:2),x_new,map) %%判断树中最近节点扩展到新节点路径中是否与障碍物相撞
        %%failedAttempts = failedAttempts+1 %如果相撞，执行
        continue; %不执行循环体余下部分，直接开始下次循环，即如果新节点遇障，重新赋值搜索，当前次不会加入随机树列
    end
    
    %step4：以x_new为圆心,半径为R的圆内搜索符合条件的节点 
    Dis_NearToNewList= [];          %%每次循环要把之前保存的值清空
    Index_NearToNewList = [];
    for i = 1:counter
        Dis_NearToNew = distanceCost(x_new,RRTStarTree.node(i).coord);
        if (Dis_NearToNew<RadiusForNer)
            Dis_NearToNewList = [Dis_NearToNewList Dis_NearToNew]; %%距离列表
            Index_NearToNewList =[Index_NearToNewList i];          %%索引列表
        end
    end
    
    %step5：重新选择x_new的父节点 
    for i = 1:length(Index_NearToNewList)
        Cost_InitToNew = Dis_NearToNewList(i) + RRTStarTree.node(Index_NearToNewList(i)).Cost;  %%总代价=xnew到xnear的距离+当前xnear到xinit的距离
        if (Cost_InitToNew<temp_Cost)
            if ~checkPath(RRTStarTree.node(Index_NearToNewList(i)).coord,x_new,map) %%判断树中最近节点扩展到新节点路径中是否与障碍物相撞
                continue; %不执行循环体余下部分，直接开始下次循环，即如果新节点遇障，重新赋值搜索，当前次不会加入随机树列
            end
            temp_Cost = Cost_InitToNew;
            temp_parent = Index_NearToNewList(i); %%节点索引号
        end
    end
    
    %step6：将x_new插入到随机树中
    counter = counter+1;
    RRTStarTree.node(counter).coord = x_new;
    RRTStarTree.node(counter).Parent = RRTStarTree.node(temp_parent).coord;
    RRTStarTree.node(counter).Cost = temp_Cost;
    RRTStarTree.node(counter).ParIndex = temp_parent;
    P_handle = plot(x_new(2), x_new(1), 'o', 'MarkerSize', 6, 'MarkerFaceColor','k');
    L_handle = plot([RRTStarTree.node(counter).Parent(2), x_new(2)], [RRTStarTree.node(counter).Parent(1), x_new(1)], 'g', 'Linewidth', 3);
    P_handleList = [P_handleList P_handle];
    L_handleList = [L_handleList L_handle];
    disp('重新选择父节点完成');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%至此，RRT*算法的第一部分（重新选择父节点）过程结束%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   %step7:重新布线
    for i = 1:length(Index_NearToNewList)
        if (Index_NearToNewList(i) ~= temp_parent) %%父节点不参与qnear运算
            Dis_NewToNear = temp_Cost + Dis_NearToNewList(i);
            if (Dis_NewToNear<RRTStarTree.node(Index_NearToNewList(i)).Cost) %%满足重新布线条件
                NearPoint = RRTStarTree.node(Index_NearToNewList(i)).coord;
                if ~checkPath(NearPoint,x_new,map) %%判断树中最近节点扩展到新节点路径中是否与障碍物相撞
                    continue; %不执行循环体余下部分，直接开始下次循环，即如果新节点遇障，重新赋值搜索，当前次不会加入随机树列
                end
                RRTStarTree.node(Index_NearToNewList(i)).Parent = x_new;     %%更新父节点
                RRTStarTree.node(Index_NearToNewList(i)).Cost = Dis_NewToNear;  %%更新代价
                RRTStarTree.node(Index_NearToNewList(i)).ParIndex = counter;    %%x_new的索引号
                lHandleList(Index_NearToNewList(i)) = plot([RRTStarTree.node(Index_NearToNewList(i)).coord(2), x_new(2)], [RRTStarTree.node(Index_NearToNewList(i)).coord(1), x_new(1)], 'r', 'Linewidth', 3);%%新线替换旧线
            end
        end
    end
    %Step 8:检查x_new是否到达目标点附近 
    if (distanceCost(x_new,[x_goal y_goal]) < threshold && ~pathFound) %只进入一次，往后只是在扩展其他未探索到的点
        pathFound = 1;
        counter = counter + 1;
        Goal_Index = counter;
        RRTStarTree.node(counter).coord = [x_goal y_goal];
        RRTStarTree.node(counter).Partent = x_new;
        RRTStarTree.node(counter).ParIndex = counter-1;  %%目标节点的父节点的索引号
    end
    %step9:在规定迭代次数内寻找最优路径
    if (pathFound == 1)
            disp('找到路径');
            path.pos(1).x = x_goal;
            path.pos(1).y = y_goal;
            pathIndex = RRTStarTree.node(Goal_Index).ParIndex;
            j =2;
            while 1
                path.pos(j).x = RRTStarTree.node(pathIndex).coord(1);
                path.pos(j).y = RRTStarTree.node(pathIndex).coord(2);
                pathIndex = RRTStarTree.node(pathIndex).ParIndex;
                if pathIndex ==0
                    break; %%退出本次while循环
                end
                j = j+1;
            end
            for delete_index = 1:length(resHandleList)
                delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).y; path.pos(j-1).y;], [path.pos(j).x; path.pos(j-1).x], 'b', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
    end
    pause(0.01)
end
toc;
title('2D RRTStar Algorithm');
