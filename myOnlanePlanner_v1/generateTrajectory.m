function trajectory=generateTrajectory(curve_position,curve_heading,curve_kappa,speed_profile)
% 该函数用于结合路径曲线和参考速度形成带时间戳的参考轨迹
% 时间戳由参考速度给出，根据累计里程找到对应的参考点坐标

trajectory=zeros(5,size(speed_profile,2)); 
index=1; station=0;

for i=1:size(speed_profile,2)
    for j=index:size(curve_position,2)
        if station-speed_profile(2,i)>=-0.0001
            index=j;
            trajectory(1,i)=curve_position(1,index);
            trajectory(2,i)=curve_position(2,index);
            trajectory(3,i)=curve_heading(index);
            trajectory(4,i)=curve_kappa(index);
            trajectory(5,i)=speed_profile(3,i);
            break;
        end
        station=station+curve_position(3,j);
    end
end
if trajectory(1,end-3)==0
   trajectory(:,end-3)=trajectory(:,end-4); 
end
if trajectory(1,end-2)==0
   trajectory(:,end-2)=trajectory(:,end-3); 
end
if trajectory(1,end-1)==0
   trajectory(:,end-1)=trajectory(:,end-2); 
end
if trajectory(1,end)==0
   trajectory(:,end)=trajectory(:,end-1); 
end

end