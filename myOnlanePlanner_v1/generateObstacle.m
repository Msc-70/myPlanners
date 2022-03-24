function obstacle_info=generateObstacle(is_static,initial_position,reference_line,total_time,quadrature_time,obst_dyna)
% 该函数用于生成静/动态障碍物轨迹，数据格式为[3,:]的[x;y;time]
% is_static=[true/false, dynamic_obstacle_index];

if initial_position(1)==-100
    obstacle_info=[];
    return;
end

count=size(initial_position,2);
obstacle_info=zeros(3*count,total_time/quadrature_time+1);

if is_static(1)
    for j=1:count
        for i=1:total_time/quadrature_time+1
            obstacle_info(1+3*(j-1),i)=initial_position(1,j);
            obstacle_info(2+3*(j-1),i)=initial_position(2,j);
            obstacle_info(3+3*(j-1),i)=(i-1)*quadrature_time;
        end
    end
else
    for j=1:count
        for i=1:total_time/quadrature_time+1
            obstacle_info(1+3*(j-1),i)=reference_line(1,is_static(j+1)+obst_dyna*(i-1));
            obstacle_info(2+3*(j-1),i)=reference_line(2,is_static(j+1)+obst_dyna*(i-1));
            obstacle_info(3+3*(j-1),i)=(i-1)*quadrature_time;
        end
        obstacle_info(1+3*(j-1),end)=reference_line(1,is_static(j+1)+obst_dyna*i);
        obstacle_info(2+3*(j-1),end)=reference_line(2,is_static(j+1)+obst_dyna*i);
    end
end

end