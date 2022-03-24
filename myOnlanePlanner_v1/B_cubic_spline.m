function [refined_point_set,refined_heading_set,refined_kappa_set]=B_cubic_spline(original_waypoint_set,headingAngle,tailAngle,point_number,flag)
% 这个函数用于生成密集的道路参考线离散点，利用三次B样条线插值模型，输入参数为稀疏的离散航路点集[2,:]、始末位置切向角deg
% 输出参数为点位置坐标及其间隔距离，点朝向角，点曲率，均为离散序列
% 两个辅助点的设置可能需要一定的调参，或者舍弃辅助点
% pointNumber用于指定每段点的数量，flag指明是否需要计算角度和曲率，以及辅助点间隔大小

controlPoints=original_waypoint_set;headingAngle=deg2rad(headingAngle);tailAngle=deg2rad(tailAngle);
subPointSetting_head=flag(3);subPointSetting_tail=flag(3);
sub_controlPoint1=[controlPoints(1,1)-subPointSetting_head*cos(headingAngle);controlPoints(2,1)-subPointSetting_head*sin(headingAngle)];
sub_controlPoint2=[controlPoints(1,1)+subPointSetting_head*cos(headingAngle);controlPoints(2,1)+subPointSetting_head*sin(headingAngle)];
sub_controlPoint3=[controlPoints(1,end)-subPointSetting_tail*cos(tailAngle);controlPoints(2,end)-subPointSetting_tail*sin(tailAngle)];
sub_controlPoint4=[controlPoints(1,end)+subPointSetting_tail*cos(tailAngle);controlPoints(2,end)+subPointSetting_tail*sin(tailAngle)];
controlPoints=[sub_controlPoint1,controlPoints(:,1),sub_controlPoint2,controlPoints(:,2:end-1),...
                sub_controlPoint3,controlPoints(:,end),sub_controlPoint4];

% 每个分段间的插值点数量
M=point_number;
s=0:1/M:1-1/M;

N=length(controlPoints);
refined_point_set=zeros(2,M*(N-3)); count=1;
for i=1:N-3
    for j=1:length(s)
    % 四个基函数
    f1s=1/6*(1-s(j))^3;
    f2s=1/2*s(j)^3-s(j)^2+2/3;
    f3s=-1/2*s(j)^3+1/2*s(j)^2+1/2*s(j)+1/6;
    f4s=1/6*s(j)^3;
    refined_point_set(:,count)=controlPoints(:,i)*f1s+controlPoints(:,i+1)*f2s+controlPoints(:,i+2)*f3s+controlPoints(:,i+3)*f4s;
    count=count+1;
    end
end
refined_point_set=[refined_point_set,original_waypoint_set(:,end)];
dx=diff(refined_point_set(1,:)); dy=diff(refined_point_set(2,:));
dx_pre = [dx(1),dx]; dx_after = [dx,dx(end)]; dx_final = (dx_pre + dx_after)/2;
dy_pre = [dy(1),dy]; dy_after = [dy,dy(end)]; dy_final = (dy_pre + dy_after)/2;
ds_final = sqrt(dx_final.^2 + dy_final.^2);
refined_point_set=[refined_point_set;ds_final];

% 利用中点欧拉法+atan2四象限反正切计算参考路径朝向，尽可能避免角度多值问题
% refined_heading_set=zeros(2,M*(N-3));
if flag(1)==1
    refined_heading_set = atan2(dy_final,dx_final);
else
    refined_heading_set=0;
end

% refined_kappa_set=zeros(2,M*(N-3));
if flag(2)==1
    dheading = diff(refined_heading_set); dheading_pre = [dheading(1),dheading];
    dheading_after = [dheading,dheading(end)]; dheading_final = (dheading_pre + dheading_after)/2;
    %为了防止dheading出现多一个2pi的错误，假设真实的dheading较小，用sin(dheading)近似dheading
    refined_kappa_set = sin(dheading_final) ./ ds_final;
    refined_kappa_set=[refined_kappa_set,refined_kappa_set(end)];
else
    refined_kappa_set=0;
end

end