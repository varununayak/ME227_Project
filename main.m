%% ME227 Project Main Simulation Loops
% TEAM FORCE INDIA
% Authors:  Kaushik Ram Sadagopan, Pranav Keni and Varun Nayak

clear all; clc; close all;


%CHOOSE LATERAL CONTROLLER HERE
 %Mode = 1; % lookahead feedback only
 Mode = 2; %lookahead feedback + feedforward
 %Mode = 3; % PID 
%Mode = 4; % Extended LQR
 
 process_noise = true;
 sensor_noise = true;
 parameter_errors = false; %imports setupniki2 instead
 use_straight_path = false;
 
%%%CHOOSE EITHER STRAIGHT PATH OR PROJECT PATH%% 

% STRAIGHT path information
if(use_straight_path)
    init = [0;0;0];
    s = [0     249];
    k = [0     0];
    % integrate s and k to get path
    path = integrate_path(s,k,init);
else
    
    load('project_data.mat') %load the path file
end

%load desired speed and acceleration info
load('desired.mat');
path.UxDes = Ux_des;
path.axDes = ax_des;



g = 9.81;                   	% gravity acceleration, meters/sec^2

% vehicle parameters
if(parameter_errors)
  setup_niki2;
else
   setup_niki;
end

% simulation time
t_final = 36.2;
dt = 0.005;
t_s = 0:dt:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps    = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps     = zeros(N,1);
dpsi_rad   = zeros(N,1);
s_m         = zeros(N,1);
e_m        = zeros(N,1);
delta_rad   = zeros(N,1);

ux_des_plot = zeros(N,1);

delta_plot = zeros(N,1);
Fx_plot = zeros(N,1);


% set initial conditions
e_m(1) = 0.0; %starting lateral error
ux_mps(1) = 0; %starting speed



for idx = 1:N

    % look up K
    s_m(idx);
    K = interp1(path.s_m, path.k_1pm, s_m(idx));
    
    % current states
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);

    ux_des_plot(idx) = interp1(path.s_m, path.UxDes, s);

    %Here Call the me227_controller function to calculate inputs to the
    %vehicle!
    
    %noisy sensor model option;
    if (sensor_noise)
        [ s_noisy, e_noisy, dpsi_noisy, ux_noisy, uy_noisy, r_noisy ] = add_sensor_noise( s, e, dpsi, ux, uy, r);
        [ delta, Fx ] = me227_controller( s_noisy, e_noisy, dpsi_noisy, ux_noisy, uy_noisy, r_noisy, Mode, path);
    else
        [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path);
    end
    
    delta_plot(idx) = delta;
    Fx_plot(idx) = Fx;
    
    %Calculate the Dynamics with the Nonlinear Bike Model
    [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r, process_noise, parameter_errors  );
        
            
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = r_radps(idx) + dt*r_dot;
        uy_mps(idx+1)= uy_mps(idx) + dt*uy_dot;
        ux_mps(idx+1) = ux_mps(idx) + dt*ux_dot;
        dpsi_rad(idx+1) = dpsi_rad(idx) + dt*dpsi_dot;
        s_m(idx+1) = s_m(idx) + dt*s_dot;
        e_m(idx+1) = e_m(idx) + dt*e_dot;
        
                     
            
    end
end

figure();
hold on;
 

subplot(3,1,1);
plot(t_s, e_m)
hold on; grid on;
xlabel('Time [s]')
ylabel('e [m]')
title('Lateral Error e ')


subplot(3,1,2);
 plot(t_s, dpsi_rad)
 hold on; grid on;
 xlabel('Time [s]')
 ylabel('\Delta\psi [rad]')
 title('\Delta\psi');

subplot(3,1,3);
plot(t_s,ux_des_plot, t_s, ux_mps )
hold on; grid on;
title("Desired Speed Tracking");
xlabel('Time [s]')
ylabel('U_x [m/s]')  
legend("U_xDes", "U_x");
ylim([0 15]);

%Plot accelerations and limits

figure()
hold on;
ay_1 = r_radps.*ux_mps;
ay = (uy_mps(2:end)-uy_mps(1:end-1))/dt + ay_1(2:end);
ay_limit = 4*ones(length(ay),1);
subplot(3,1,1);
plot(t_s(1:end-1), ay, t_s(1:end-1), ay_limit,'r--')
hold on; grid on;
xlabel('Time [s]')
ylabel('a_y [m/s^2]')
title('Lateral Acceleration ')
legend('Actual','Limit');



ax_1 = r_radps.*uy_mps;
ax = (ux_mps(2:end)-ux_mps(1:end-1))/dt - ax_1(2:end);
ax_ulimit = 3*ones(length(ax),1);
ax_llimit = -4*ones(length(ax),1);
subplot(3,1,2);
 plot(t_s(1:end-1), ax,t_s(1:end-1), ax_llimit,'r--',t_s(1:end-1), ax_ulimit,'r--')
 hold on; grid on;
 xlabel('Time [s]')
 ylabel('a_x [m/s^2]')
 title('Longitudinal Acceleration');
legend('Actual','LLimit','ULimit');
ylim([-5,5]);
 
 

 subplot(3,1,3);
a_total = sqrt(ax.^2+ay.^2);
a_total_limit = 4*ones(length(a_total),1);
plot(t_s(1:end-1),a_total,t_s(1:end-1),a_total_limit,'r--' )
hold on; grid on;
title("Total Acceleration");
xlabel('Time [s]')
ylabel('a_t[m/s2]')  
legend('Actual','Limit');

figure()
subplot(2,1,1);
plot(t_s, delta_plot);
title("Steering Angle \delta");
subplot(2,1,2);
plot(t_s, Fx_plot);
title("Tractive Force F_x");

% Animation
path.s = path.s_m;
path.k = path.k_1pm;
path.posE = path.posE_m;
path.posN = path.posN_m;
path.psi = path.psi_rad;
animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%STUDENT FUNCTIONS BELOW
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate Forces with the Fiala Nonlinear Tire Model
function Fy = fiala_model(alpha, tire)
%   Calculate tire forces with the fiala model

%%%%% STUDENT CODE HERE %%%%%

    Fz = tire.Fz;
   alpha_sl = abs(3*tire.mu*Fz/tire.Ca);
 
    if(abs(alpha)<alpha_sl)
        Fy = -tire.Ca*tan(alpha) + tire.Ca^2/(3*tire.mu*Fz)*(2-tire.mu_s/tire.mu)*abs(tan(alpha))*tan(alpha)...
            - tire.Ca^3/(9*tire.mu^2*Fz^2)*(tan(alpha))^3*(1-2*tire.mu_s/(3*tire.mu));
    else
        Fy = -tire.mu_s*Fz*sign(alpha);
    end
%%%%% END STUDENT CODE %%%%%
end

%Calculate the Nonlinear Bicycle Model Dynamics
function [ r_dot, Uy_dot, Ux_dot, s_dot, e_dot, dpsi_dot] = ...
    nonlinear_bicycle_model( r, Uy, Ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r, process_noise ,parameter_errors )
%KINEMATIC_MODEL
%   Calculate state derivatives for the kinematic vehicle model

%%%%% STUDENT CODE HERE %%%%%

%longitudinal terms:
frr = 0.015 + parameter_errors*((0.004).*rand(1) -0.002) ; %rolling friction constant
C_DA = 0.594 + parameter_errors*((0.05).*rand(1) -0.025); %coefficient of drag X surface area
rho = 1.225; %density of air
g = 9.81;


% slip angles
    [alpha_f, alpha_r] = slip_angles( r, Uy, Ux, delta, veh);
% lateral tire forces
    Fyf = fiala_model(alpha_f, tire_f);
    Fyr = fiala_model(alpha_r, tire_r);
%Split longitudinal force based on drive and brakedistribution
if Fx > 0
    Fxf = Fx;   %because the GTI is a front wheel drive car
    Fxr = 0;
else
    Fxf = Fx/2;
    Fxr = Fx/2; %equally distributed brake force
end

% dynamics
Ux_dot = ( Fxr + Fxf*cos(delta) - Fyf*sin(delta) + veh.m*Uy*r )/veh.m;
Ux_dot = Ux_dot + ( -frr*veh.m*g - 0.5*rho*C_DA*(Ux^2) )/veh.m;    %added the rolling friction and the drag terms
Uy_dot = ( Fyf*cos(delta) + Fyr + Fxf*sin(delta) - veh.m*r*Ux ) / veh.m;
r_dot = ( veh.a*Fyf*cos(delta) + veh.a*Fxf*sin(delta) - veh.b*Fyr) / veh.Iz;
s_dot = (1/(1-e*K))*(Ux*cos(dpsi) - Uy*sin(dpsi) );
e_dot = Uy*cos(dpsi) + Ux*sin(dpsi);
dpsi_dot = r - K*s_dot;

if(process_noise)
    Ux_dot = Ux_dot + normrnd(0,0.01);
    Uy_dot = Uy_dot + normrnd(0,0.01);
    r_dot = r_dot + normrnd(0,0.01);
    e_dot = e_dot + normrnd(0,0.01);
    dpsi_dot = dpsi_dot + normrnd(0,0.01);
end

%%%%% END STUDENT CODE %%%%%
end

%Calculate the Slip Angles Here:
function [alpha_f, alpha_r] = slip_angles( r, Uy, Ux, delta, veh)
%slip_angles
%   calculate the tire slip angles 

%%%%% STUDENT CODE HERE %%%%%
    alpha_f = atan2(Uy+veh.a*r, Ux) - delta ;
    alpha_r = atan2(Uy-veh.b*r,Ux);
%%%%% END STUDENT CODE %%%%%
end

%Use standard Euler Integration
function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state

%%%%% STUDENT CODE HERE %%%%%
    x1  = x0 + x0_dot*dt;
%%%%% END STUDENT CODE %%%%%
end

%adds gaussian white noise to the state variables
function [ s_noisy, e_noisy, dpsi_noisy, ux_noisy, uy_noisy, r_noisy ] = add_sensor_noise( s, e, dpsi, ux, uy, r)
    
    s_noisy = s;
    e_noisy = e + normrnd(0.00,0.01);
    dpsi_noisy = dpsi + normrnd(0.00,0.01);
    ux_noisy = ux + normrnd(0,0.01);
    uy_noisy = uy + normrnd(0,0.01);
    r_noisy = r + normrnd(0,0.01);
    
end
