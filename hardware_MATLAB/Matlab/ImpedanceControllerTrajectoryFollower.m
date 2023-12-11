% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE
% Bezier curve control points
% The constant point (0.0, 0.078), works for both left and right arms to
% test
const_point = [0.0; -0.0863]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
pts_foot = repmat(const_point,1,8);
       
%pts_foot = []; % YOUR BEZIER PTS HERE
        
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
% Combo of angle1= 0.79/-.79, and angle2=2.25/-2.25 works for initializing
% at setpoint
% set these for how we want angles viewed 0 at right x-axis, counter-clockwise increasiing
angle1_init = 0.838; %pi/2;
angle2_init = 2.13; %pi/2; 
%% DON'T EDIT THESE, these are adjusted for computation/control
angle1_init = -angle1_init;
angle2_init = -angle2_init;
% angles used in calculations 
%angle1_init = 0;
%angle2_init = -angle2_init;
% Total experiment time is buffer,trajectory,buffer
traj_time         = 10;
pre_buffer_time   = 0; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;
% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 500;
gains.K_yy = 250;
gains.K_xy = -10;
gains.D_xx = 2;
gains.D_yy = 1;
gains.D_xy = 0;
% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 1;
%% Run Experiment
[output_data] = RunTrajectoryExperiment(angle1_init, angle2_init, pts_foot,...
                                        traj_time, pre_buffer_time, post_buffer_time,...
                                        gains, duty_max);
%% Extract data
t = output_data(:,1);
x = output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
dy = -output_data(:,15);
xdes = output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydes = output_data(:,17); % desired foot position in Y
K_out = output_data(:,20);
%Low pass filter on dy to visualize what's mbed seeing with low pass filter
alpha = 0.7; % smoothing factor
filtered_dy = zeros(size(dy)); % preallocate for efficiency
filtered_dy(1) = dy( 1); % initialize the first element
% Apply the low-pass filter
for i = 2:length(dy)
    filtered_dy(i) = alpha * dy(i) + (1 - alpha) * filtered_dy(i - 1);
end
%% Plot foot vs desired
figure(3); clf;
subplot(211); hold on
plot(t,xdes,'r-'); plot(t,x);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});
subplot(212); hold on
plot(t,ydes,'r-'); plot(t,y);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});
figure(4); clf; hold on
plot(xdes,ydes,'r-'); plot(x,y,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
figure(5); clf; hold on;
plot(t,-y.*10)
plot(t, dy, 'k-'); % Plot original data
xlabel('t(s)'); ylabel('speed (m/s)');
plot(t, filtered_dy, 'r-'); % Plot filtered data in red
legend({'pos','Actual', 'Filtered'}); % Update the legend
hold off;
figure(6); clf;
plot(t,K_out);
xlabel('Time (s)'); ylabel('K_yy'); 
ylim([0 500])