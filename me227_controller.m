function [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
    
    % ME227 Controller:
    % Spring 2019
    % Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
    % 
    % Here you should use the inputs from above to calculate the inputs to the
    % vehicle. These will be delta and Fx and will be determined based upon
    % your control laws below. 
    %
    % For the project you wil use this same input output structure and in this
    % homework you will use this control structure for defining the control
    % inputs in your simulation. 
     % Define your vehicle parameters here (you must define all of your
    % parameters here, you may not reference and outside function in order to 
    % function on the GTI)
    Caf_lin = 80000;
    Car_lin = 120000;
    b = 1.367;
    a = 1.264;
    m = 1926.2; 
    L = a + b;
    K_steer = m/L*(b*Car_lin - a*Caf_lin)/(Caf_lin*Car_lin);
    Iz = 2763.49;

    %Controller saturation values
    Fx_MAX = 15000;
    delta_MAX = 50*(3.14159/180);

    % Define your controller parameters here:
    %lookahead controller (feedback only)
    K_la = 4000; %default is 3500
    x_la = 15; %default 15

    %drive (longitudinal) controller gain
    K_drive = 1500; %default is 1890
    frr = 0.015; %rolling friction constant
    C_DA = 0.594; %coefficient of drag X surface area
    rho = 1.225; %density of air

    g = 9.81;

    %PID Control Terms
    persistent e_prev ;
    persistent dpsi_prev;
    persistent e_integrated;
    persistent dpsi_integrated;
    
    if(isempty(e_prev))
        e_prev = 0;
        dpsi_prev = 0;
        e_integrated = 0;
        dpsi_integrated = 0;
    end
    
    Kp_e = 0.27;
    Kp_dpsi = 0.3;
    Kd_e = 0.05; Kd_dpsi = 0.01;
    Ki_e = 0.00; Ki_dpsi = 0.0;

    dt = 0.005; %controller operates at 200Hz
    
        
    %%%%%----------END STUDENT CODE----------%%%%%%

    %Find Uxdesired for the current distance along the path via interpolation
    uxdes = interp1(path.s_m, path.UxDes, s);
    axdes = interp1(path.s_m, path.axDes, s);

    %Find Curvature for the current distance along the path via interpolation
    K = interp1(path.s_m, path.k_1pm, s);
    
    %find derivative
    %e_deriv = (e - e_prev)/dt; %numerical derivative
    e_deriv = ux*sin(dpsi) + uy*cos(dpsi); %using model
    
    %dpsi_deriv = (dpsi-dpsi_prev)/dt; %using numerical derivative
    s_dot = (1/1-e*K)*(ux*cos(dpsi) - uy*sin(dpsi));
    dpsi_deriv = r - K*s_dot; %using model

    %%%%%----------START STUDENT CODE----------%%%%%%

    % Use the Lateral Control Law to Calculate Delta
    if Mode == 1
        % Calculate the feedback steering command with lateral error and heading error
        delta = -K_la*(e + x_la*dpsi)/Caf_lin;

    elseif Mode == 2
       
        dpsi_ss = K*(m*a*ux^2/L/(Car_lin)-b);
        deltaff = K_la*x_la/(Caf_lin)*dpsi_ss + K*(L+K_steer*ux^2);
        delta = -K_la*(e + x_la*dpsi)/(Caf_lin) + deltaff;

    elseif Mode == 3        
        %sum of errors (integral term)
        e_integrated = e_integrated + e;
        dpsi_integrated = dpsi_integrated + dpsi;
    
        %anti-windup
        if(abs(e_integrated) > 15)
            e_integrated = 15*sign(e_integrated);
        end
        if(abs(dpsi_integrated) > 3)
            dpsi_integrated = 3*sign(dpsi_integrated);
        end
        
        %feedforward
        dpsi_ss = K*(m*a*ux^2/L/Car_lin-b);
        deltaff = K_la*x_la/Caf_lin*dpsi_ss + K*(L+K_steer*ux^2);
        
        delta_p = -[Kp_e Kp_dpsi]*[e;dpsi];
        delta_d =  -[Kd_e Kd_dpsi]*[e_deriv;dpsi_deriv];
        delta_i =   - [Ki_e Ki_dpsi]*[e_integrated;dpsi_integrated]
        delta = delta_p + delta_d + delta_i   + deltaff;

        %store as previous value before going into next call
        e_prev = e;
        dpsi_prev = dpsi;
    elseif Mode ==4
        x_lqr = [e;e_deriv;dpsi;dpsi_deriv];
        K_lqr = [2.0 ux/30 ux/3 ux/50];
        delta = -K_lqr*x_lqr;
     
    else
        dpsi_ss = K*(m*a*ux^2/L/(Car_lin)-b);
        deltaff = K_la*x_la/(Caf_lin)*dpsi_ss + K*(L+K_steer*ux^2);
        delta = -K_la*(e + x_la*dpsi)/(Caf_lin) + deltaff;
        
        %sum of errors (integral term)
        e_integrated = e_integrated + e;
        dpsi_integrated = dpsi_integrated + dpsi;
    
        %anti-windup
        if(abs(e_integrated) > 15)
            e_integrated = 15*sign(e_integrated);
        end
        if(abs(dpsi_integrated) > 3)
            dpsi_integrated = 3*sign(dpsi_integrated);
        end
              
        
        delta_i =   - [Ki_e Ki_dpsi]*[e_integrated;dpsi_integrated];
        delta = delta + delta_i 

        %store as previous value before going into next call
        e_prev = e;
        dpsi_prev = dpsi;
        
    end

    
    % Use the Longitudinal Control Law to Calcuate Fx

    Fx = m*axdes + frr*m*g + 0.5*C_DA*(ux^2) + K_drive*(uxdes - ux); %with drag and rollfriction compensation   
  
        
    %saturate the control inputs takes care of Inf
    if(abs(Fx)>Fx_MAX)
            Fx = Fx_MAX*sign(Fx);
    end
    
    if(abs(delta)>delta_MAX)
        delta = delta_MAX*sign(delta);
    end
    
    %isnan check
    if(isnan(delta))
        delta = 0;
    end
    if(isnan(Fx))
        Fx = 0;
    end

    
  
%%%%%----------END STUDENT CODE----------%%%%%%
end

