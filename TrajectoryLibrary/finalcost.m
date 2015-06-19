function [h,dh] = finalcost(t,x)
c = 1;
h = c*t;
dh = [c,zeros(1,size(x,1))];
end
