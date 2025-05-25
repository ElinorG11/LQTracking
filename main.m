% A New Type of Model Predictive Optimal-Control Method,
% Based on Potryagin’s Minimum Principle.

% Basic example: ideal storage device, with a quadratic cost function.

% HERE: the input is based on real electric-vehicle data.
% The signal is NOT KNOWN, and is estimated based on regression.
% The control loop is working with the estimated Laplace transform, not the
% real one.
% Note: I am using the same signal to learn the signal statistics, and
% for testing. These are not two separate signals (as it should be).

clear; clc;

% set latex interpreter for plotting
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% control parameters
Emax = 1e6;
a = 0.01; % this is alpha in my theoretical analysis
% 1/a is the control loop time constant.

% load vehicle data
datafile = 'vehicle_data.mat';
if ~exist('vehicle_data')
    load(datafile);
end
% number of entries in data file
DataLen = length(vehicle_data);
% choose data
DataEntry = 45;  % choose between 1 and DataLen

t0 = vehicle_data{DataEntry}.Time; % time
V0 = vehicle_data{DataEntry}.Battery_Voltage; % voltage
I0 = vehicle_data{DataEntry}.Battery_Current; % current
P0 = V0.*I0; % power
u0 = P0;

% Simulink parameters
T = t0(end) - 5/a; % final time.
MaxStep = min(diff(t0)) / 2;
RelTol = 1e-4;

% Create input signal
dt = MaxStep;
t1 = 0:dt:t0(end);
u1 =  interp1(t0,u0,t1,'linear');

% Compute the Laplace transform numerically
disp('computing Laplace transform.');
disp('please wait ...');
Lap_Length = max(find(t1<T));
Lap = NaN*zeros(Lap_Length,1);
for k=1:Lap_Length
    Utemp = u1(k:end);
    y = MyLaplace(Utemp,dt,a);
    Lap(k) = y;
end
Lap(end-1) = Lap(end-2);
Lap(end) = Lap(end-2); % 'Lap' is the Laplace transform
Lap_time = (0:(Lap_Length-1))*dt; % 'Lap_time' is the time vector associated with Lap
Lap_time = Lap_time(:);
disp('done computing Laplace transform.');

% Use regression to predict the result of the Laplace transform
% I am using the input signal 'u1' to predict the Laplace transform 'Lap'
disp('building regression matrices.');
disp('please wait ...');
ss = ceil((3/a)/dt);  % number of samples of u1 used for prediction
MM = Lap_Length - ss + 1;
H = zeros(MM,ss+1);
yy = zeros(MM,1);
for kk=1:MM,
    H(kk,:) = [u1(kk:(kk+ss-1)), 1];
    yy(kk) = Lap(kk+ss-1);
end
disp('done building regression matrices');

% Train Machine Learning Model
% disp('Training Machine Learning model...');
% Here goes the model of your choice, for instance: ML_model = fitrsvm(H, yy, 'KernelFunction', 'gaussian', 'Standardize', true);
% disp('Training completed.');

% Predict the Laplace transform using the trained ML model
% disp('Predicting Laplace transform with trained model...');
% Lap_est = predict(ML_model, H);
% Lap_est = [ones(ss-1,1) * Lap_est(1); Lap_est]; % Match length of Lap and Lap_est
% disp('Prediction completed.');

% Reference with standard statistical method: Linear Regression. Comment if you train the ML estimator.
disp('computing regression coefficients and signal.');
rr = H \ yy; % the operator '\' provides a least-squares solution to the system H*rr = yy
Lap_est = H*rr; % this is the estimated Laplace transform
Lap_est = [ones(ss-1,1)*Lap_est(1) ; Lap_est]; % match the length of the vectors Lap and Lap_est
disp('done computing regression coefficients and signal.');

% Simulink
disp('running Simulink.');
disp('please wait ...');
main_sim; % open simulink
sim(bdroot); % run Simulink
disp('done running Simulink.');

figure(1);
numplots = 3;
curplot = 1;

time_limits = [0 , T];

subplot(numplots,1,curplot);
plot(ts,El,'k--','linewidth',2);
hold on;
plot(ts,El+Emax,'k--','linewidth',2);
plot(ts,Eg,'color',[0.4660 0.6740 0.1880],'linestyle','-','linewidth',2);
hold off;
xlim(time_limits);
ylabel('$E_l,\ E_g$','FontSize', 25);
yticklabels(strrep(yticklabels,'-','$-$'));
ax = gca;
ax.FontSize = 30;  % Font Size of 15
lgd = legend('$E_l$', '$E_l+E_{\max}$', '$E_g$');
set(lgd, 'Interpreter', 'latex', 'FontSize', 30);
grid on;
curplot = curplot+1;

subplot(numplots,1,curplot);
plot(ts,0*ts.^0,'color',[0.6350 0.0780 0.1840],'linestyle','--','linewidth',2);
hold on;
plot(ts,Emax*ts.^0,'color',[0.6350 0.0780 0.1840],'linestyle','--','linewidth',2);
plot(ts,E,'k-','linewidth',2);
hold off;
xlim(time_limits);
ylabel('$E$','FontSize', 25);
yticklabels(strrep(yticklabels,'-','$-$'));
ax = gca;
ax.FontSize = 30;  % Font Size of 15
grid on;
curplot = curplot+1;

subplot(numplots,1,curplot);
plot(ts,pl,'k--','linewidth',2);
hold on;
plot(ts,pg,'color',[0 0.4470 0.7410],'linestyle','-','linewidth',2);
hold off;
xlim(time_limits);
ylabel('$p_l,\ pg$','FontSize', 25);
yticklabels(strrep(yticklabels,'-','$-$'));
lgd = legend('$p_l$', '$p_g$');
set(lgd, 'Interpreter', 'latex', 'FontSize', 30);
grid on;
curplot = curplot+1;

xlabel('Time','FontSize', 25);
set(gcf,'Color','white');
ax = gca;
ax.FontSize = 30;  % Font Size of 15

% Get original positions
p1 = get(ax1, 'Position');
p2 = get(ax2, 'Position');
p3 = get(ax3, 'Position');

% Define new height (e.g., increase by 30%)
new_height = p2(4) * 1.3;

% Set new positions with larger height and adjusted vertical placement
set(ax3, 'Position', [p3(1), p3(2), p3(3), new_height]);
set(ax2, 'Position', [p2(1), p3(2)+new_height+0.03, p2(3), new_height]);
set(ax1, 'Position', [p1(1), p2(2)+new_height+0.03, p1(3), new_height]);



figure(2);
plot(Lap_time,Lap,'linewidth',2);
hold on;
plot(Lap_time,Lap_est,'r-','linewidth',2);
hold off;
xlabel('Time','FontSize', 30);
%title('the real and estimated Laplace transform signals.')
set(gcf,'Color','white');
ax = gca;
ax.FontSize = 30;  % Font Size of 15



beep;
fprintf('done\n');

% dir_name = strcat("Base_Emax_",num2str(Emax),"_a_",num2str(a),"_dt_",num2str(dt),"_T_",num2str(T));
% if ~exist(dir_name)
%     mkdir(strcat("results/",dir_name));
% end
% writematrix(El,strcat("results/",dir_name,"/El_entry",char(num2str(DataEntry)),".xlsx"));
% writematrix(Eg,strcat("results/",dir_name,"/Eg_entry",char(num2str(DataEntry)),".xlsx"));
% writematrix(E,strcat("results/",dir_name,"/E_entry",char(num2str(DataEntry)),".xlsx"));
% writematrix(pl,strcat("results/",dir_name,"/pl_entry",char(num2str(DataEntry)),".xlsx"));
% writematrix(pg,strcat("results/",dir_name,"/pg_entry",char(num2str(DataEntry)),".xlsx"));
% writematrix(ts,strcat("results/",dir_name,"/ts_entry",char(num2str(DataEntry)),".xlsx"));
