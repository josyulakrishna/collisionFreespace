function [scale] = optimise_scale(t_win, vel, t)
%OPTIMISE_SCALE Summary of this function goes here
%   Detailed explanation goes here

vx = vel.vx;
vy = vel.vy;
v_min = vel.vmin;
v_max = vel.vmax;
v = sqrt(vx.^2 + vy.^2)

cvx_begin quiet
    variable s;
    minimize((s-1)^2);
    subject to
        s*t_win(1) <= t;
        t <= t_win(2)*s;
        v_min <= s*v;
        s*v <= v_max;
        s >= 0;
cvx_end

scale = double(s);
disp(scale);

end

