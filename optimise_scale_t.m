function [scale] = optimise_scale_t(t_win, vel, t)
%OPTIMISE_SCALE Summary of this function goes here
%   Detailed explanation goes here

vx = vel.vx;
vy = vel.vy;
v_min = vel.vmin;
v_max = vel.vmax;
v = sqrt(vx.^2 + vy.^2);
% cvx_begin quiet
%     variable a;
%     minimize((1/(2*a*(t-t_win(2)+1))-1)^2);
%     subject to
%         (1/(2*a*(t-t_win(1))+1))*t_win(1) <= t;
%         t <= t_win(2)*(1/(2*a*(t-t_win(2))+1))*t_win(2);
%         v_min <= (1/(2*a*(t-t_win(1))+1))*v;
%         v*(1/(2*a*(t-t_win(2))+1)) <= v_max;
%          (1/(2*a*(t-t_win(1))+1)) >= 0;
% cvx_end

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    a0=1.0;
    a=a0;
    [a,fval]=fmincon(@(a)cost(a,t,t_win),a0,A,b,Aeq,beq,lb,ub,@(x)nonlincon(a,t,t_win,v_min,v_max,v));
    scale = double(1/(2*a*(t-t_win(2))+1));
    disp(scale);
end

function c1= cost(a,t,t_win)
c1=((1/(2*a*(t-t_win(2)+1))-1)^2);
end
function [c,ceq]= nonlincon(a,t,t_win,v_min,v_max,v)
    constr1=    log(2*a*(t-t_win(1))+1)*t_win(1) -t 
    constr2=    -1*( -t+ log(2*a*(t-t_win(1))+1)*t_win(2))
    constr3=    -1*(-v_min+ log(2*a*(t-t_win(1))+1)*v)
    constr4=    v*log(2*a*(t-t_win(1))+1) - v_max
    constr5=    -1*log(2*a*(t-t_win(1))+1)
    c=[constr1, constr2, constr3, constr4,constr5];
    ceq = []; 
end