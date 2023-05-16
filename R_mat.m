%Function to compute measurement covariance matrix
function [Meas_Covar] = R_mat

%Creating a diagonal matrix across all the landmarks
Meas_Covar = diag([0.01, 0.25, 0.01, 0.25, 0.01, 0.25, 0.09, 0.25, 0.09, 0.25, 0.09, 0.25]);

end

