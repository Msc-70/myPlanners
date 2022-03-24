function [x,y,v,theta,phy] = kinematic_bicycle_model(x,y,v,theta,phy,a,w,delta_t,wheelbase)

    x=x+delta_t*v*cos(theta);
    y=y+delta_t*v*sin(theta);
    v=v+delta_t*a;
    theta=theta+delta_t*v*tan(phy)/wheelbase;
    phy=phy+delta_t*w;
    
end