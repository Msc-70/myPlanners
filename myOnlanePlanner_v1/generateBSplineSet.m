function [curve_set_position,curve_set_heading,curve_set_kappa]=generateBSplineSet(initial_pose,goal_pose,intermediate_pose1,intermediate_pose2)
% 此函数用于生成B样条曲线簇，控制点和始末角度均由外部直接给定，其中末状态为沿着参考线横向撒点的结果
% 初始位姿、中间位姿均包含[x,y,theta]'，末位姿相同且为3*N矩阵

position=[];heading=[];kappa=[];
for i=1:size(goal_pose,2)
    controlPoints=[initial_pose(1:2),intermediate_pose1(:,i),intermediate_pose2(:,i),goal_pose(1:2,i)];
    [positionTemp,headingTemp,kappaTemp]=B_cubic_spline(controlPoints,rad2deg(initial_pose(3)),rad2deg(goal_pose(3,i)),50,[1,1,1]);
    position=[position;positionTemp];
    heading=[heading;headingTemp];
    kappa=[kappa;kappaTemp];
end

curve_set_position=position;
curve_set_heading=heading;
curve_set_kappa=kappa;

end