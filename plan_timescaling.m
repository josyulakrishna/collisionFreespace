function [outputArg1,outputArg2] = plan_timescaling(inputArg1,inputArg2)
%TIMESCALING_PLANNER Summary of this function goes here
%   Detailed explanation goes here

ahorizon = end_t; % Total end time of robots simulation (depands on scaling)
aprevscale = 0.5; % Previous scale
acurrentscale = 1; % Current scale

atime = cur_t;
while(t<=endt)
    aprevscale = acurrentscale;
    
    
    if(t<15)
        tempscale =1;
    elseif(15<=t && t<33)
        tempscale = 0.5;
    else
        tempscale = 0.7;
    end
    acurrentscale = tempscale;
    figure(1);
    cla;
    ahorizon = t + ((ahorizon-t)/(acurrentscale/aprevscale));
    adelt = delt*acurrentscale;
    atime = atime + adelt;
    [~,~,velax,velay,~,~,omegaa,~]=get_robo(x_weights_robo,tan_weights_robo,yo_cr(1),to,atime,tf);
    plot_mod_velx(ai,:) = velax;
    [xo_cr(ai+1,:),yo_cr(ai+1,:),thetar(ai+1,:)]=get_circles(xo_cr(ai,:),yo_cr(ai,:),velax,velay,thetar(ai),omegaa,delt,acurrentscale);
    fill(double(subs(plot_rectx,[xcentre,ycentre,director],[xo_cr(ai+1,:),yo_cr(ai+1,:),thetar(ai+1,:)])),double(subs(plot_recty,[xcentre,ycentre,director],[xo_cr(ai+1,:),yo_cr(ai+1,:),thetar(ai+1,:)])),'y');
    robot_plot_with_sensor(xo_cr(ai+1,:),yo_cr(ai+1,:),radiusrobo,0);
    plot(xo_cr(1:ai+1,:),yo_cr(1:ai+1,:),'r');
    ai=ai+1;
    t = t + delt;
    pause(0.01);
    endt = ahorizon;
end


end

