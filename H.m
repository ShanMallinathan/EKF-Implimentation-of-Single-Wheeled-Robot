%Fucntion to compute measurement matrix
function [Measure_H] = H(xp, i)

     d= 0; %Distance between lidar and vehicle's geometric center
     
     %Getting the measuremnt data
     measure_data = load('my_measurements.mat');
     theta = xp(3);
     
    
    %Computing the jacobian from the state
    X = measure_data.r(i, :) .* cos(theta + measure_data.b(i, :)); %x intercept of r
    Y = measure_data.r(i, :) .* sin(theta + measure_data.b(i, :));%y intercept of r
    h11 = -X ./ (measure_data.r(i, :));
    h12 = -Y ./ (measure_data.r(i, :));
    h21 = Y ./ (measure_data.r(i, :) .^ 2);
    h22 = -X ./ (measure_data.r(i, :) .^ 2);

    %Combining all the elements of jacobian into 12X3 matrix
    j = 1;
    for o = 1:size(measure_data.l)
        Measure_H(j:j+1, :) = [h11(o), h12(o), 0;
                               h21(o), h22(o), -1];
        j = j + 2;
    end
end