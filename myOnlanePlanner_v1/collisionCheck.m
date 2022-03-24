function [final_trajectory,is_brake]=collisionCheck(cost_index,curve_set_position,curve_set_heading,curve_set_kappa,speed_profile,obstacle_info,is_brake)
% 该函数用于对轨迹进行碰撞检测和筛选，轨迹和障碍物信息均为预测时域内的离散状态信息序列


if isempty(obstacle_info)
    trajectory=generateTrajectory(curve_set_position((cost_index(1)-1)*3+1:cost_index(1)*3,:),curve_set_heading(cost_index(1),:),...
                curve_set_kappa(cost_index(1),:),speed_profile);
    final_trajectory=trajectory;
    return;
end

% predictTime=int(speed_profile(3,end)); 
quadratureTime=speed_profile(3,2)-speed_profile(3,1);
totalCheckPointNumber=size(speed_profile,2);
obstCount=size(obstacle_info,1)/3;
distToCollision=zeros(obstCount,totalCheckPointNumber);

for i=1:length(cost_index)
    trajectory=generateTrajectory(curve_set_position((cost_index(i)-1)*3+1:cost_index(i)*3,:),curve_set_heading(cost_index(i),:),...
                                    curve_set_kappa(cost_index(i),:),speed_profile);
    for k=1:obstCount
        for j=1:totalCheckPointNumber
            count=j;
            distToCollision(k,j)=sqrt((trajectory(1,count)-obstacle_info(1+3*(k-1),count))^2+(trajectory(2,count)-obstacle_info(2+3*(k-1),count))^2);
        end
    end
    if min(min(distToCollision))>=2.6
        final_trajectory=trajectory;
        is_brake=[0,0];
        return;
    end
end
% error('all trajectories failed')
if is_brake(2)
    is_brake=[1,0];
else
    is_brake=[0,1];
end
final_trajectory=[];
% print('deceleration')
end