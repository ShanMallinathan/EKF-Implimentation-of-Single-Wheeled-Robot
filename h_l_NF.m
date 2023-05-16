%function to return the landmark's relative location for single sensor

function [h_landmark] = h_l_NF(x, sensor_index)

%get landmark values
measure_data = load('my_measurements.mat');

%calculate r and theta
r = sqrt((measure_data.l(sensor_index, 1) - x(1))^2 + (measure_data.l(sensor_index, 2) - x(2))^2);
theta = wrapToPi(atan2((measure_data.l(sensor_index, 2) - x(2)), (measure_data.l(sensor_index, 1) - x(1)))) - x(3);
h_landmark = [r; theta;];

