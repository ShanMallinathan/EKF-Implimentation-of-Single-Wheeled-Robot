%Fucntion to compute measurement matrix for independednt sensors
function [Measure_H] = H_NF(xp, i, sensor_index)

     d= 0; %Distance between lidar and vehicle's geometric center
     
     %Getting the measuremnt data
     measure_data = load('my_measurements.mat');
     theta = xp(3);
     
    
    %Computing the jacobian from the state
    X = measure_data.r(i, sensor_index) .* cos(theta + measure_data.b(i, sensor_index)); %x intercept of r
    Y = measure_data.r(i, sensor_index) .* sin(theta + measure_data.b(i, sensor_index));%y intercept of r
    h11 = -X ./ (measure_data.r(i, sensor_index));
    h12 = -Y ./ (measure_data.r(i, sensor_index));
    h21 = Y ./ (measure_data.r(i, sensor_index) .^ 2);
    h22 = -X ./ (measure_data.r(i, sensor_index) .^ 2);

    Measure_H = [h11, h12, 0;
                 h21, h22, -1;];
  