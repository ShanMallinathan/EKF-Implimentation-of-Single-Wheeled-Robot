%function to return the landmark's relative location

function [h_landmark] = h_l(x)

j=1;
measure_data = load('my_measurements.mat');

for i=1:length(measure_data.l)
    r = sqrt((measure_data.l(i, 1) - x(1))^2 + (measure_data.l(i, 2) - x(2))^2);
    theta = wrapToPi(atan2((measure_data.l(i, 2) - x(2)), (measure_data.l(i, 1) - x(1)))) - x(3);
    h_landmark(j:j+1) = [r; theta];
    j = j+2;
end
