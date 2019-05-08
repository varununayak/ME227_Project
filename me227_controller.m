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
    
    if(s<0 || isnan(s)) %remove this for actual car
        s = 0;
    end

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
    Fx_MAX = 10000;
    delta_MAX = 50*(3.14159/180);

    % Define your controller parameters here:
    %lookahead controller (feedback only)
    K_la = 3500; %default is 3500
    x_la = 18; %default 15

    %drive (longitudinal) controller gain
    K_drive = m*0.1*9.81/1;
    frr = 0.015; %rolling friction constant
    C_DA = 0.594; %coefficient of drag X surface area
    rho = 1.225; %density of air

    g = 9.81;

    %PID Control Terms
    persistent e_prev; e_prev = 0;
    persistent dpsi_prev; dpsi_prev = 0;
    persistent e_integrated; e_integrated = 0;
    persistent dpsi_integrated; dpsi_integrated = 0;
    
    persistent delta_prev; delta_prev = 0;
    persistent Fx_prev; Fx_prev = 0;

    Kp_e = K_la/Caf_lin; Kp_dpsi = K_la*x_la/Caf_lin;
    Kd_e = 0.01; Kd_dpsi = 0.01;
    Ki_e = 0.05; Ki_dpsi = 0.05;

    dt = 0.005; %controller operates at 200Hz
    
    
    %%%%%----------END STUDENT CODE----------%%%%%%

    %Find Uxdesired for the current distance along the path via interpolation
    uxdes = interp1(path.s_m, path.UxDes, s);
    axdes = interp1(path.s_m, path.axDes, s);

    %Find Curvature for the current distance along the path via interpolation
    K = interp1(path.s_m, path.k_1pm, s);

    %%%%%----------START STUDENT CODE----------%%%%%%

    % Use the Lateral Control Law to Calculate Delta
    if Mode == 1
        % Calculate the feedback steering command with lateral error and heading error
        delta = -K_la*(e + x_la*dpsi)/Caf_lin;

    elseif Mode == 2
        dpsi_ss = K*(m*a*ux^2/L/Car_lin-b);
        deltaff = K_la*x_la/Caf_lin*dpsi_ss + K*(L+K_steer*ux^2);
        delta = -K_la*(e + x_la*dpsi)/Caf_lin + deltaff;

    elseif Mode == 3

        %find derivative 
        e_deriv = (e-e_prev)/dt;
        dpsi_deriv = (dpsi-dpsi_prev)/dt;

        %sum of errors (integral term)
        e_integrated = e_integrated + e;
        dpsi_integrated = dpsi_integrated + dpsi;
    
        %anti-windup
        e_integrated = sat(e_integrated,100);
        dpsi_integrated = sat(dpsi_integrated,100);

        %feedforward
        dpsi_ss = K*(m*a*ux^2/L/Car_lin-b);
        deltaff = K_la*x_la/Caf_lin*dpsi_ss + K*(L+K_steer*ux^2);

        delta = -[Kp_e Kp_dpsi]*[e;dpsi] -[Kd_e Kd_dpsi]*[e_deriv;dpsi_deriv] ...
            - [Ki_e Ki_dpsi]*[e_integrated;dpsi_integrated] + deltaff;

        %store as previous value before going into next call
        e_prev = e;
        dpsi_prev = dpsi;
        
    else %LQR
        e_deriv = (e-e_prev)/dt;
        dpsi_deriv = (dpsi-dpsi_prev)/dt;        
        x_lqr = [e;e_deriv;dpsi;dpsi_deriv];
        K_lqr = get_K_lqr();
        delta = -K_lqr*x_lqr;
        
    end

    %put low pass on delta
    low_pass = [0.7 0.96 0.8 0.02];
    delta = delta_prev + low_pass(Mode)*( delta - delta_prev ); %low pass
    delta_prev = delta;
    
    % Use the Longitudinal Control Law to Calcuate Fx

    %Fx = K_drive*(uxdes - ux); %without drag and rollfriction compensation
    Fx = m*axdes + frr*m*g + 0.5*C_DA*(ux^2) + K_drive*(uxdes - ux); %with drag and rollfriction compensation   
        
    
    
    %saturate the control inputs
    Fx = sat(Fx,Fx_MAX);
    delta = sat(delta,delta_MAX);
    
    
    if(isnan(delta))
        delta = 0;
    end
    if(isnan(Fx))
        Fx = 0;
    end

    %saturationg function
    function y = sat(x,c)
        if(abs(x)>c)
            y = c*sign(x);
        else
            y = x;
        end
    end

    function K_lqr = get_K_lqr()
       
        A = [0 1 0 0;...
            0 -(Caf_lin+Car_lin)/m/ux (Caf_lin+Car_lin)/m (-a*Caf_lin+b*Car_lin)/m/ux;...
            0 0 0 1;...
            0 (b*Car_lin - a*Caf_lin)/m/Iz (a*Caf_lin-b*Car_lin)/Iz -(a^2*Caf_lin + b^2*Car_lin)/Iz/ux];
        B = ones(4,1);
        
        Q = diag([1 10 1 10]);
        R = 1;
        N = 0;
        
        A(isinf(A)) = 10000;
        A(isnan(A)) = 0;
        %K_lqr = [1 0.8 2.12 0.1]; %mean LQR
        K_lqr = [1 ux/10 ux ux/40];    %extended LQR
        %K_lqr = lqr(A,B,Q,R,N);
        
    end


%%%%%----------END STUDENT CODE----------%%%%%%
end

