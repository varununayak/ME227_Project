%% Script to find the gains for LQR incase it doesn't compile

Caf_lin = 80000;
Car_lin = 120000;
b = 1.367;
a = 1.264;
m = 1926.2; 
L = a + b;
K_steer = m/L*(b*Car_lin - a*Caf_lin)/(Caf_lin*Car_lin);
Iz = 2763.49;

B = ones(4,1);

Q = diag([1 100 100 100]);
R = 30;
N = 0;
        


Num = 200;
K_lqr_store = cell(N,1)

ux_store = linspace(0,20,Num);

K_1 = zeros(Num,1);
K_2 = zeros(Num,1);
K_3 = zeros(Num,1);
K_4 = zeros(Num,1);

for i = 1:Num
    
    ux = ux_store(i)
    
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
    
            
    K_lqr = lqr(A,B,Q,R,N);
    
    K_1(i)  = K_lqr(1);
    K_2(i)  = K_lqr(2);
    K_3(i)  = K_lqr(3);
    K_4(i)  = K_lqr(4);
    
end

figure();


subplot(4,1,1);
plot(ux_store,K_1);
title("LQR Gain vs U_x");

ylim([-20 20])
subplot(4,1,2);
plot(ux_store,K_2);
ylim([-20 20])

subplot(4,1,3);
plot(ux_store,K_3);
ylim([-20 20])

subplot(4,1,4);
plot(ux_store,K_4);
ylim([-20 20])

xlabel("U_x [m/s]");


save('lqr_gains.mat','K_lqr_store')