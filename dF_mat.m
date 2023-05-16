%Function to compute df jacobian
function [F] = dF_mat(v, theta)

dt = 0.1;
F = [1, 0, -v *sin(theta) *dt;
     0, 1, v * cos(theta) *dt;
     0, 0, 1];
end