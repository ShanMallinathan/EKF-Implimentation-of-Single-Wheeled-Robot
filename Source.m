%EKF implimentation for a autnomous wheeled robot using matlab
%Code by Shanthinath Mallinathan
clc
clearvars
disp("Kalman filter for single wheeled robot by Shanthinath Mallinathan");
disp("Processing please wait...");

% clock initialisation
t1 = clock;

%reading the data
input_data = load('my_input.mat');
measure_data = load('my_measurements.mat');

disp("Initiating sensor fusion approach...");
%variable handling
x = zeros(3, length(input_data.t)); %initial state
x(:, 1) = [1, 1, 0];
dt = input_data.t(2) - input_data.t(1);
F = diag([1, 1, 1]);

R = measure_data.r;
B = measure_data.b;
wrapToPi(B);

%Computing process variance matrix
Q = 0.1 * diag([0.01, 0.25]);

%To compute initial covariance matrix
P = zeros(3,3, length(input_data.t)); 
P(:, :, 1) = diag([1, 1, 1]) * 10^(-5);

%%
%%Incorporatng Sensor Fusion

% For Plot
Chistat_SF = zeros(1, length(input_data.t));
r_b_SF = zeros(12, length(input_data.v));

%Extended Kalman Filter Algo
for i = 2:length(input_data.t)
    %Prediction steps
    %X Predict
    dF = dF_mat(input_data.v(i-1), x(3, i-1));
    xp = F * x(:, i-1) + G(x(3, i-1)) * [input_data.v(i-1) + 0.01; input_data.om(i-1) + 0.25]; 
    %Covariance prediction
    Pp = dF * P(:, :, i-1) * dF' + G(x(3, i-1)) * Q * G(x(3, i-1))';
    %Correction step
    %Kalman Gain
    h = (H(xp, i-1));
    dr = (h * Pp * transpose(h) + R_mat);
    Dr = dr ^(-1);
    K = (Pp * transpose(h)) * Dr;
    %innovation
    %Vectorising measurements across waypoints
    Z = [R(i-1, 1); B(i-1, 1); R(i-1, 2); B(i-1, 2); R(i-1, 3); B(i-1, 3); R(i-1, 4); B(i-1, 4); R(i-1, 5); B(i-1, 5); R(i-1, 6); B(i-1, 6);];
    innovation = Z - h_l(xp)'; 
    %State correction
    x(:, i) = xp + K * innovation;
    %Covariance correction
    P(:, : , i) = (eye(3, 3) - K * h) * Pp;
    Chistat_SF(i) = innovation' * Dr * innovation./6;
    r_b_SF(:, i-1) = h_l(x(:, i)); 
end

t2 = clock; %final time for SF
disp("Sensor fusion approach complete.")
disp("Time taken for processing sensor fusion: " + (t2(6) - t1(6)) + "Seconds")

%%
% Computing with respect to single landmark 

disp("Initiating non sensor fusion approach...");
t3 = clock; % clock initalisation

landmark_id = 1;  %Change this to change the landmark 

x_NF = zeros(3, length(input_data.t)); %initial state
x_NF(:, 1) = [1, 1, 0];

P_NF = zeros(3,3, length(input_data.t)); 
P_NF(:, :, 1) = diag([1, 1, 1]) * 10^(-5);

%for R amtrix
sigma_theta = 0.25;
if (landmark_id >= 1 && landmark_id <= 3)
    sigma_r = 0.01;
else
    sigma_r = 0.09;
end

% For Plot
Chistat_NF = zeros(1, length(input_data.t));
r_b_NF = zeros(2, length(input_data.v));

%Extended Kalman Filter Algo
for i = 2:length(input_data.t)
    %Prediction steps
    %X Predict
    dF = dF_mat(input_data.v(i-1), x_NF(3, i-1));
    xp_NF = F * x_NF(:, i-1) + G(x_NF(3, i-1)) * [input_data.v(i-1) + 0.01; input_data.om(i-1) + 0.25]; 
    %Covariance prediction
    Pp_NF = dF * P_NF(:, :, i-1) * dF' + G(x_NF(3, i-1)) * Q * G(x_NF(3, i-1))';
    %Correction step
    %Kalman Gain
    R_NF = diag([sigma_r, sigma_theta]);
    h = (H_NF(xp_NF, i-1, landmark_id));
    dr = (h * Pp_NF * transpose(h) + R_NF);
    Dr = dr ^(-1);
    K = (Pp_NF * transpose(h)) * Dr;
    %innovation
    %Vectorising measurements across waypoints
   
    Z = [R(i-1, landmark_id); B(i-1, landmark_id);] ;
    innovation = (Z - h_l_NF(xp_NF, landmark_id)) ;
    %State correction
    x_NF(:, i) = xp_NF + K * innovation;
    %Covariance correction
    P_NF(:, : , i) = (eye(3, 3) - K * h) * Pp_NF;
    Chistat_NF(i) = innovation' * Dr * innovation;
    r_b_NF(:, i-1) = h_l_NF(x_NF(:, i), landmark_id); 
end

t4 = clock; %clock final
disp("Non sensor fusion approach complete.");
disp("Time taken for processing sensor fusion: " + (t4(6) - t3(6)) + "Seconds");

%%
% Plotting 
disp("Results estimated successfully!! Plotting results please wait...");
%Plotting the values
% Position of the robot and Landmarks
figure
hold on
plot(x(1, :), x(2, :));
plot(x_NF(1, :), x_NF(2, :));
xlabel('X coordinate')
ylabel('Y coordinate')
title('Robot Path')%plotting the landmarks
plot([measure_data.l(:, 1)], [measure_data.l(:, 2)], '.', 'markersize', 10);
axis equal
legend('Robot path with fusion', 'Robot path without fusion', 'Landmarks')
hold off

%Chistat Plot
figure
hold on
plot(input_data.t, Chistat_SF)
plot(input_data.t, Chistat_NF)
xlabel('Time [s]')
ylabel('\chi^2')
title('\chi^2 statistics about innovations')
legend("With sensor fusion", "Without sensor fusion")

%Measured and predicted range
radial = r_b_SF(1:2:end, :);
radial_NF = r_b_NF(1:2:end, :);
for i = 1:length(measure_data.l)
figure
hold on
title("Measured vs predicted r : Landmark " + i)
plot(input_data.t(1, 2:end), measure_data.r(:, i), Color='r');
plot(input_data.t(1, 2:end), radial(i, :), Color='b');
if (i==landmark_id)
    plot(input_data.t(1, 2:end), radial_NF(1, :), Color='g');
end
if (i==landmark_id)
    legend("Measured with fusion", "Predicted", "Measured without fusion");
else
    legend("Measured with fusion", "Predicted");
end
hold off
end

%Measured and predicted range
bearing = r_b_SF(2:2:end, :);
bearing_NF = r_b_NF(2:2:end, :);
for i = 1:length(measure_data.l)
figure
hold on
title("Measured vs predicted bearing : Landmark " + i)
plot(input_data.t(1, 2:end), measure_data.b(:, i), Color='r');
plot(input_data.t(1, 2:end), bearing(i, :), Color='b');
if (i==landmark_id)
    plot(input_data.t(1, 2:end), bearing_NF(1, :), Color='g');
end
if (i==landmark_id)
    legend("Measured with fusion", "Predicted", "Measured without fusion");
else
    legend("Measured with fusion", "Predicted");
end
hold off
end
disp("Results plotted successfully");

