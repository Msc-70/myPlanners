function [goal_pose_set,intermediate_pose,refStartIndex]=findGoalPose(referenceLine,forwardPredictionDistance,current_pose,refStartIndex,lateral_offset)
% 该函数用于确定轨迹的始末状态，末状态根据前瞻距离从参考线上选择
% lateral_offset=[max_lateral_distance,offset_number]

comparisonTemp=100; goal_pose_set=[]; intermediate_pose=[]; intermediate_pose2=[];
for i=refStartIndex:size(referenceLine,2)
    minDist=sqrt((referenceLine(1,i)-current_pose(1))^2+(referenceLine(2,i)-current_pose(2))^2);
    if comparisonTemp>minDist
       comparisonTemp=minDist; 
    else
        refStartIndex=i;
%         currentPoseOnRef=referenceLine(:,i);
        accumulateDist=0; j=i;
        while(accumulateDist<=forwardPredictionDistance)
            accumulateDist=accumulateDist+referenceLine(3,j);
            j=j+1;
        end
        goalPoseStd=[referenceLine(1,j-1);referenceLine(2,j-1);referenceLine(4,j-1)];
        midPose1=[referenceLine(1,floor((j+i)/2)-25);referenceLine(2,floor((j+i)/2)-25);referenceLine(4,floor((j+i)/2)-25)];
        midPose2=[referenceLine(1,floor((j+i)/2)+25);referenceLine(2,floor((j+i)/2)+25);referenceLine(4,floor((j+i)/2)+25)];
        for k=0:lateral_offset(2)
            newX=goalPoseStd(1)-lateral_offset(1)*k/lateral_offset(2)*sin(goalPoseStd(3));
            newY=goalPoseStd(2)+lateral_offset(1)*k/lateral_offset(2)*cos(goalPoseStd(3));
            goal_pose_set=[goal_pose_set,[newX;newY;goalPoseStd(3)]];
            midX=midPose2(1)-0.9*lateral_offset(1)*k/lateral_offset(2)*sin(midPose2(3));
            midY=midPose2(2)+0.9*lateral_offset(1)*k/lateral_offset(2)*cos(midPose2(3));
            intermediate_pose=[intermediate_pose,[midX;midY]];
            midX2=midPose1(1)-0.6*lateral_offset(1)*k/lateral_offset(2)*sin(midPose1(3));
            midY2=midPose1(2)+0.6*lateral_offset(1)*k/lateral_offset(2)*cos(midPose1(3));
            intermediate_pose2=[intermediate_pose2,[midX2;midY2]];
        end
        intermediate_pose=[intermediate_pose2,intermediate_pose];
        return;
    end
end
error('goal_pose generation failed')

end