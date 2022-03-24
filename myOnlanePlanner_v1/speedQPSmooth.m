function [smoothedSpeed]=speedQPSmooth(v_lim,a_lim,j_lim,v_ref,v0,a0,j0,delta_t)

N=size(v_ref,2); % number of discrete points of speed to be optimizaed
w_speed=1; w_acc=1; w_jerk=1; % weights for speed follow and comfort
vf=v_ref(1,end); 
af=(v_ref(1,end)-v_ref(1,end-1))/delta_t;
if a0>=1.2*a_lim
   a0=1.2*a_lim; 
end

H_speed=eye(N,N); % cost func matrix for speed follow

H_acc=zeros(N,N); % cost func matrix for acceleration
for i=1:N-1
   H_acc(i,i)=2;
   H_acc(i+1,i)=-1;
   H_acc(i,i+1)=-1;
end
H_acc(1,1)=1; H_acc(N,N)=1;

H_jerk=zeros(N,N); % cost func matrix for jerk
for i=1:N-2
   H_jerk(i,i)=6;
   H_jerk(i+1,i)=-4;
   H_jerk(i,i+1)=-4;
   H_jerk(i+2,i)=1;
   H_jerk(i,i+2)=1;
end
H_jerk(1,1)=1; H_jerk(N,N)=1;
H_jerk(2,2)=5; H_jerk(N-1,N-1)=5;
H_jerk(1,2)=-2; H_jerk(N,N-1)=-2;
H_jerk(2,1)=-2; H_jerk(N-1,N)=-2;

H_total=w_speed*H_speed+w_acc/(delta_t^2)*H_acc+w_jerk/(delta_t^4)*H_jerk;
H_total=H_total*2; % total cost matrix

f=-2*v_ref(1,:); % the f term of QP problem

A_eq=zeros(6,N); % equality constraint matrix for endpoints
A_eq(1,1)=1; A_eq(2,1)=-1; A_eq(2,2)=1;
A_eq(3,1)=1; A_eq(3,2)=-2; A_eq(3,3)=1;
A_eq(4,N-2)=1; A_eq(4,N-1)=-2; A_eq(4,N)=1;
A_eq(5,N-1)=-1; A_eq(5,N)=1; A_eq(6,N)=1;
b_eq=[v0; a0*delta_t; j0*delta_t^2; 0; af*delta_t; vf];

A_ineq_jerk=zeros(N-2,N); % inequality constraint matrix for intermediate variables
for i=1:N-2
   A_ineq_jerk(i,i)=1;
   A_ineq_jerk(i,i+1)=-2;
   A_ineq_jerk(i,i+2)=1;
end
b_ineq_jerk=j_lim*delta_t^2*ones(N-2,1);

A_ineq_acc=zeros(N-1,N);
for i=1:N-1
   A_ineq_acc(i,i)=-1;
   A_ineq_acc(i,i+1)=1;
end
b_ineq_acc=1.2*a_lim*delta_t*ones(N-1,1);

A_ineq=[A_ineq_acc;A_ineq_jerk];
b_ineq=[b_ineq_acc;b_ineq_jerk];

lb=zeros(N,1); ub=1.1*v_lim*ones(N,1); % bounds for variables

sol=quadprog(H_total,f,A_ineq,b_ineq,A_eq,b_eq,lb,ub); % solve the QP
station=zeros(1,N);
for i=1:N-1
    station(i+1)=station(i)+0.5*(sol(i)+sol(i+1))*delta_t;
end
smoothedSpeed=[sol';station;v_ref(2,:)];

end