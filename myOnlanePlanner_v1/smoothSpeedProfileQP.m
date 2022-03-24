function speed_profile=smoothSpeedProfileQP(max_acc,max_speed,current_speed,total_time,quadrature_time,is_brake)
% 该函数利用三次参数B样条生成平滑的速度参考，数据格式为[speed,station,time]'
% 需要指定最大加速度、最大速度、预测时间和积分小区间时间

coarse_speed=zeros(2,total_time/quadrature_time+1);
a0=(current_speed(2)-current_speed(1))/quadrature_time/2;

if is_brake(1)
    coarse_speed(1,1)=current_speed(2);
    for i=1:total_time/quadrature_time
        if coarse_speed(1,i)>=quadrature_time*max_acc(2)
            coarse_speed(1,i+1)=coarse_speed(1,i)-quadrature_time*max_acc(2);
        end
        coarse_speed(2,i+1)=coarse_speed(2,i)+quadrature_time;
    end
    speed_profile=speedQPSmooth(max_speed,max_acc(2),5,coarse_speed,current_speed(2),a0,0,quadrature_time);
    return;
end

if is_brake(2)
    coarse_speed(1,:)=current_speed(2);
    for i=1:total_time/quadrature_time
        coarse_speed(2,i+1)=coarse_speed(2,i)+quadrature_time;
    end
    speed_profile=speedQPSmooth(max_speed,max_acc(1),5,coarse_speed,current_speed(2),a0,0,quadrature_time);
    return;
end

t_hinge=(max_speed-current_speed(2))/max_acc(1);
if current_speed(2)>=max_speed
    speed_profile=zeros(3,total_time/quadrature_time+1);
    for i=1:total_time/quadrature_time
       speed_profile(1,i+1)=max_speed;
       speed_profile(2,i+1)=speed_profile(2,i)+max_speed*quadrature_time;
       speed_profile(3,i+1)=speed_profile(3,i)+quadrature_time;
    end
    speed_profile(1,1)=max_speed;
   
elseif t_hinge>=total_time
    coarse_speed(1,1)=current_speed(2);
    for i=1:total_time/quadrature_time
       coarse_speed(1,i+1)=coarse_speed(1,i)+0.9*max_acc(1)*quadrature_time;
       coarse_speed(2,i+1)=coarse_speed(2,i)+quadrature_time;
    end
    speed_profile=speedQPSmooth(max_speed,max_acc(1),5,coarse_speed,current_speed(2),a0,0,quadrature_time);
else
    count=floor(t_hinge/quadrature_time);
    coarse_speed(1,1:count)=linspace(current_speed(2),max_speed,count);
    coarse_speed(1,count+1:end)=max_speed*ones(1,total_time/quadrature_time+1-count);
    for i=1:total_time/quadrature_time
       coarse_speed(2,i+1)=coarse_speed(2,i)+quadrature_time;
    end
    speed_profile=speedQPSmooth(max_speed,max_acc(1),5,coarse_speed,current_speed(2),a0,0,quadrature_time);
end

end