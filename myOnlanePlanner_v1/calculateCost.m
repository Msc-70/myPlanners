function candidate_index=calculateCost(curve_set_heading,curve_set_kappa,weights)
% 该函数用于计算备选曲线簇的代价值，目前有三项cost，分别为偏离中心线程度、角度变化和曲率变化
% 权重weight的维度需要与cost项数量一致

count=size(curve_set_heading,1);
cost_deviation=zeros(1,count);
cost_dHeading=zeros(1,count);
cost_dKappa=zeros(1,count);

for i=1:count
   cost_deviation(i)=weights(1)*i; %abs(floor(count/2)-i+1+0.1);
end

for i=1:count
   cost_dHeading(i)=weights(2)*sum(abs(diff(curve_set_heading(i,:))));
end

for i=1:count
   cost_dKappa(i)=weights(3)*sum(abs(diff(curve_set_kappa(i,:))));
end

candidate_cost=cost_deviation+cost_dHeading+cost_dKappa;
[~,candidate_index]=sort(candidate_cost);

end