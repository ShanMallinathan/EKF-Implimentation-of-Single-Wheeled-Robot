function [Control_Mat] = G(theta)
%Function to Compute control matrix

dt = 0.1;
Control_Mat = [cos(theta) * dt, 0;
               sin(theta) * dt, 0;
               0,               dt;];
end