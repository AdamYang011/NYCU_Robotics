% A to B is 0.5 sec
% B to C is 0.5 sec
% tacc is 0.2 sec
% sampling time is 0.002 sec
Tacc = 0.2;
T = 0.5;

A = [0 0 -1 10;
     -1 0 0 20;
     0 1 0 30;
     0 0 0 1;];

B = [1 0 0 30;
     0 1 0 20;
     0 0 1 10;
     0 0 0 1;];

C = [0 -1 0 -10;
     0 0 1 -20;
     -1 0 0 -30;
     0 0 0 1;];

thetaA = []; thetaB = []; thetaC = [];

%calculate ?,£c,£r of A,B,C
A_phi = atan2(A(2,3), A(1,3));
A_theta = atan2((cos(A_phi)*A(1,3)+sin(A_phi)*A(2,3)), A(3,3));
A_psi = atan2((-sin(A_phi)*A(1,1)+cos(A_phi)*A(2,1)), (-sin(A_phi)*A(1,2)+cos(A_phi)*A(2,2)));
thetaA = [A(1,4),A(2,4),A(3,4),A_phi,A_theta,A_psi];

B_phi = atan2(B(2,3), B(1,3));
B_theta = atan2((cos(B_phi)*B(1,3)+sin(B_phi)*B(2,3)), B(3,3));
B_psi = atan2((-sin(B_phi)*B(1,1)+cos(B_phi)*B(2,1)), (-sin(B_phi)*B(1,2)+cos(B_phi)*B(2,2)));
thetaB = [B(1,4),B(2,4),B(3,4),B_phi,B_theta,B_psi];

C_phi = atan2(C(2,3), C(1,3));
C_theta = atan2((cos(C_phi)*C(1,3)+sin(C_phi)*C(2,3)), C(3,3));
C_psi = atan2((-sin(C_phi)*C(1,1)+cos(C_phi)*C(2,1)), (-sin(C_phi)*C(1,2)+cos(C_phi)*C(2,2)));
thetaC = [C(1,4),C(2,4),C(3,4),C_phi,C_theta,C_psi];

deltaC = thetaC - thetaB;
deltaB = thetaB - thetaA;

theta_p = [];
endpointA = [];endpointB = [];

position = [];
ini_p = [];
end_p = [];
ini_px = [];ini_py = [];ini_pz = [];
end_px = [];end_py = [];end_pz = [];
v_px = [];v_py = [];v_pz = [];
a_px = [];a_py = [];a_pz = [];

v_joint1 = [];v_joint2 = [];v_joint3 = [];
v_joint4 = [];v_joint5 = [];v_joint6 = [];

a_joint1 = [];a_joint2 = [];a_joint3 = [];
a_joint4 = [];a_joint5 = [];a_joint6 = [];



%According to time to calculate the position, velocity, and acceleration
for t = -0.5:0.002:0.5
    if t < -0.2
        t = t + 0.5;
        linear_h = t/T;
        theta_p = deltaB*linear_h + thetaA;
        endpointA = theta_p;
        ini_px = [ini_px theta_p(1)];
        ini_py = [ini_py theta_p(2)];
        ini_pz = [ini_pz theta_p(3)];
        end_px = [end_px theta_p(4)+A(1,3)*2];
        end_py = [end_py theta_p(5)+A(2,3)*2];
        end_pz = [end_pz theta_p(6)+A(3,3)*2];

        theta_v = deltaB/T;
        v_px = [v_px theta_v(1)];
        v_py = [v_py theta_v(2)];
        v_pz = [v_pz theta_v(3)];

        theta_a = zeros(6,1);
        a_px = [a_px theta_a(1)];
        a_py = [a_py theta_a(2)];
        a_pz = [a_pz theta_a(3)];

    elseif t<=0.2 && t>=-0.2
        deltaB = endpointA - thetaB;
        h = (t + Tacc) / (2 * Tacc);
        theta_p = ((deltaC * (Tacc/T) + deltaB)*(2-h)*(h^2) - (2 * deltaB)) * h + endpointA;
        endpointB = theta_p;
        ini_px = [ini_px theta_p(1)];
        ini_py = [ini_py theta_p(2)];
        ini_pz = [ini_pz theta_p(3)];
        end_px = [end_px theta_p(4)+A(1,3)*2];
        end_py = [end_py theta_p(5)+A(2,3)*2];
        end_pz = [end_pz theta_p(6)+A(3,3)*2];

        theta_v = ((deltaC * (Tacc/T) + deltaB)*(1.5-h)*2*(h^2) - deltaB)/Tacc;
        v_px = [v_px theta_v(1)];
        v_py = [v_py theta_v(2)];
        v_pz = [v_pz theta_v(3)];
        
        theta_a = ((deltaC * (Tacc/T) + deltaB)*(1-h))*(3*h)/(Tacc^2);
        a_px = [a_px theta_a(1)];
        a_py = [a_py theta_a(2)];
        a_pz = [a_pz theta_a(3)];

    elseif t > 0.2
        t = (t-0.2);
        linear_h = t/T;
        theta_p = deltaC*linear_h + endpointB;
        ini_px = [ini_px theta_p(1)];
        ini_py = [ini_py theta_p(2)];
        ini_pz = [ini_pz theta_p(3)];
        end_px = [end_px theta_p(4)+A(1,3)*2];
        end_py = [end_py theta_p(5)+A(2,3)*2];
        end_pz = [end_pz theta_p(6)+A(3,3)*2];
        
        theta_v = deltaC/T;
        v_px = [v_px theta_v(1)];
        v_py = [v_py theta_v(2)];
        v_pz = [v_pz theta_v(3)];

        theta_a = zeros(6,1);
        a_px = [a_px theta_a(1)];
        a_py = [a_py theta_a(2)];
        a_pz = [a_pz theta_a(3)];
    end
end

%plot position of x y z
figure
subplot(3,1,1);
t = 0:0.002:1;
y = ini_px;
plot(t,y)
title('Position of x');
xticks(0:0.1:1);
yticks(-10:10:30)
ylim([-10 30])

subplot(3,1,2);
t = 0:0.002:1;
y = ini_py;
plot(t,y)
title('Position of y');
xticks(0:0.1:1);
ylabel('Position(cm)')
yticks(-20:10:20)
ylim([-20 20])

subplot(3,1,3);
t = 0:0.002:1;
y = ini_pz;
plot(t,y)
title('Position of z');
xticks(0:0.1:1);
xlabel('time(s)')
yticks(-30:10:30)
ylim([-30 30])

%plot velocity of x y z
figure
subplot(3,1,1);
t = 0:0.002:1;
y = v_px;
plot(t,y)
title('Velocity of x');
xticks(0:0.1:1);
yticks(-80:20:40)
ylim([-80 40])

subplot(3,1,2);
t = 0:0.002:1;
y = v_py;
plot(t,y)
title('Velocity of y');
xticks(0:0.1:1);
ylabel('Velocity(cm/s)')
yticks(-80:20:0)
ylim([-80 0])

subplot(3,1,3);
t = 0:0.002:1;
y = v_pz;
plot(t,y)
title('Velocity of z');
xticks(0:0.1:1);
xlabel('time(s)')
yticks(-80:10:-40)
ylim([-80 -40])

%plot Acceleration of x y z
figure
subplot(3,1,1);
t = 0:0.002:1;
y = a_px;
plot(t,y)
title('Acceleration of x');
xticks(0:0.1:1);
yticks(-500:100:0)
ylim([-500 0])

subplot(3,1,2);
t = 0:0.002:1;
y = a_py;
plot(t,y)
title('Acceleration of y');
xticks(0:0.1:1);
ylabel('Acceleration(cm/s^2)')
yticks(-300:50:0)
ylim([-300 0])

subplot(3,1,3);
t = 0:0.002:1;
y = a_pz;
plot(t,y)
title('Acceleration of z');
xticks(0:0.1:1);
xlabel('time(s)')
yticks(-150:50:0)
ylim([-150 0])

%plot position motion
figure
plot3(ini_px, ini_py, ini_pz)
xlabel('x(cm)')
ylabel('y(cm)')
zlabel('z(cm)')
grid
hold on
%A dot
x = 10; y = 20; z = 30;
plot3(x,y,z,'r*')
text(x,y,z,'A (10,20,30)')
%B dot
x = 30; y = 20; z = 10;
plot3(x,y,z,'r*')
text(x,y,z,'B (30,20,10)')
%C dot
x = -10; y = -20; z=-30;
plot3(x,y,z,'r*')
text(x,y,z,'C (-10,-20,-30)')
%plot the line from data point to the direction of acceleration
quiver3(ini_px, ini_py, ini_pz, end_px, end_py, end_pz, "c")