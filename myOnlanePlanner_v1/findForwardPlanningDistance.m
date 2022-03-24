function forwardPlanningDistance=findForwardPlanningDistance(max_acc,max_speed,current_speed,time_interval,is_brake)
% 该函数用于粗略确定所规划轨迹向前延申的距离

if is_brake(2)
   forwardPlanningDistance=time_interval*current_speed;
   return;
end

if is_brake(1)
    t_hinge=current_speed/max_acc(2);
    if t_hinge>=time_interval
        forwardPlanningDistance=0.5*max_acc(2)*time_interval^2+(current_speed-max_acc(2)*time_interval)*time_interval+3;
    else
        forwardPlanningDistance=0.5*max_acc(2)*t_hinge^2+3;
    end
    return;
end

if current_speed>max_speed+1
    error('speed>max')
elseif current_speed>max_speed
    current_speed=max_speed;
end

t_hinge=(max_speed-current_speed)/max_acc(1);

if t_hinge>=time_interval
    forwardPlanningDistance=0.5*max_acc(1)*time_interval^2;
elseif t_hinge<=0
    forwardPlanningDistance=max_speed*time_interval;
else
    forwardPlanningDistance=0.5*max_acc(1)*time_interval^2+current_speed*t_hinge+max_speed*(time_interval-t_hinge);
end

end