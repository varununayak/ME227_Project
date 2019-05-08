clc
ay_max = 4;
a_max = 4;
c = 0.0099;
Kmax1 = 0.1154;
Kmax2 = 0.1159;
Ux_min1 = (ay_max/Kmax1)^0.5;
Ux_min2 = (ay_max/Kmax2)^0.5;
Ux_des = zeros(1009,1);
ax_des = zeros(1009,1);
ds = 0.25;
Ux_exit_last = sqrt(2*a_max*9);

Ux_des(1:19) = 0.5*(6*path.s_m(1:19)).^(2/3);
ax_des(1:19) = (6*path.s_m(1:19)).^(1/3);

Ux_des(20:49) = sqrt(4.5^2 + 2*3*(path.s_m(20:49)-4.5));
ax_des(20:49) = 3;

[Ux_des(50:96) , ax_des(50:96)] = bound_clothoid_speed( 50 , 96 , Ux_des ,ax_des, path , Ux_des(49), Ux_min1, a_max , c , ds, 0);
Ux_des(97:158) = Ux_des(96);
[Ux_des(159:205), ax_des(159:205)] = clothoid_speed( 159 , 205 , Ux_des , ax_des, path , Ux_min1, a_max , c , ds ,1);

[Ux_des(302:348) , ax_des(302:348)] = clothoid_speed( 348 , 302 , Ux_des ,ax_des, path , Ux_min2, a_max , c , ds, 0);
dist_x1 = ((Ux_des(302)^2 - Ux_des(205)^2)+8*24.1)/14;

Ux_des(206:262) = sqrt(Ux_des(205)^2 + 2*3*(path.s_m(206:262)-51));
ax_des(206:262) = 3;
Ux_des(263:301) = sqrt(Ux_des(262)^2 - 2*4*(path.s_m(263:301)-65.25));
ax_des(263:301) = -4;

Ux_des(349:409) = Ux_des(348);
[Ux_des(410:456) , ax_des(410:456)] = clothoid_speed( 410 , 456 , Ux_des ,ax_des, path , Ux_min2, a_max , c , ds ,1);

Ux_des(457:513) = sqrt(Ux_des(456)^2 + 2*3*(path.s_m(457:513)-113.75));
ax_des(457:513) = 3;
Ux_des(514:552) = sqrt(Ux_des(513)^2 - 2*4*(path.s_m(514:552)-128));
ax_des(514:552) = -4;

[Ux_des(553:599) , ax_des(553:599)] = clothoid_speed( 599 , 553 , Ux_des ,ax_des, path , Ux_min1, a_max , c , ds, 0);
Ux_des(600:661) = Ux_des(599);
[Ux_des(662:708) , ax_des(662:708)] = clothoid_speed( 662 , 708 , Ux_des ,ax_des, path , Ux_min1, a_max , c , ds ,1);

Ux_des(709:765) = sqrt(Ux_des(708)^2 + 2*3*(path.s_m(709:765)-176.75));
ax_des(709:765) = 3;
Ux_des(766:805) = sqrt(Ux_des(765)^2 - 2*4*(path.s_m(766:805)-191));
ax_des(766:805) = -4;

[Ux_des(806:852), ax_des(806:852)] = clothoid_speed( 852 , 806 , Ux_des , ax_des, path , Ux_min2, a_max , c , ds, 0);
Ux_des(853:913) = Ux_des(852);
[Ux_des(914:960) , ax_des(914:960)] = bound_clothoid_speed( 914 , 960 , Ux_des ,ax_des, path , Ux_exit_last, Ux_min2, a_max , c , ds ,1);

Ux_des(961:995) = sqrt(Ux_exit_last^2 - 2*4*(path.s_m(961:995)-239.75));
ax_des(961:1009) = -a_max;
function [U1 , a1] = clothoid_speed( init , final , Ux , ax, path , Ux_min , a_max , c , ds ,dir) 
    Ux(init) = Ux_min;
    if(dir == 0)
        for idx = init:-1:final
            u_dot = (1/Ux(idx))*sqrt(a_max^2 - (c*(path.s_m(idx)-path.s_m(final))*(Ux(idx))^2)^2);
            ax(idx) = -u_dot*Ux(idx);
            Ux(idx-1) = integrate_euler( Ux(idx), u_dot, ds );
        end
    else
        for idx = init:final
            u_dot = (1/Ux(idx))*sqrt(a_max^2 - (c*(path.s_m(idx)-path.s_m(final))*(Ux(idx))^2)^2);
            ax(idx) = u_dot*Ux(idx);
            if (ax(idx)<=3)
                Ux(idx+1) = integrate_euler( Ux(idx), u_dot, ds );
            else
                Ux(idx+1) = sqrt((Ux(idx))^2 + 2*3*ds);
                ax(idx) = 3;
            end
        end
    end
    i1 = min(init,final);
    i2 = max(init,final);
    U1 = Ux(i1:i2);
    a1 = ax(i1:i2);
end

function [U1 , a1] = bound_clothoid_speed( init , final , Ux , ax, path , Ux_bound ,Ux_min, a_max , c , ds ,dir) 
    if(dir == 0)
        Ux(init) = Ux_bound;
        for idx = init:1:final
            if(Ux(idx) > Ux_min)
                u_dot = -(1/Ux(idx))*sqrt(a_max^2 - (c*(path.s_m(idx)-path.s_m(init))*(Ux(idx))^2)^2);
                ax(idx) = u_dot*Ux(idx);
                Ux(idx+1) = integrate_euler( Ux(idx), u_dot, ds );
            else
                Ux(idx + 1) = Ux(idx);
                ax(idx) = 0;
            end
        end
    else
        Ux(init) = Ux_min;
        for idx = init:final
            if(Ux(idx) < Ux_bound)
                u_dot = (1/Ux(idx))*sqrt(a_max^2 - (c*(path.s_m(idx)-path.s_m(final))*(Ux(idx))^2)^2);
                ax(idx) = u_dot*Ux(idx);
                if (ax(idx)<=3)
                    Ux(idx+1) = integrate_euler( Ux(idx), u_dot, ds );
                else
                    Ux(idx+1) = sqrt((Ux(idx))^2 + 2*3*ds);
                    ax(idx) = 3;
                end
            else
                Ux(idx+1) = Ux_bound;
                ax(idx) = 0;
            end
        end
    end
    i1 = min(init,final);
    i2 = max(init,final);
    U1 = Ux(i1:i2);
    a1 = ax(i1:i2);
end

%Use standard Euler Integration
function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state

%%%%% STUDENT CODE HERE %%%%%
x1 = x0 + x0_dot*dt;
%%%%% END STUDENT CODE %%%%%
end
