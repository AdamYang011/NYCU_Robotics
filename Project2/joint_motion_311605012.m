% A to B is 0.5 sec
% B to C is 0.5 sec
% tacc is 0.2 sec
% sampling time is 0.002 sec

Tacc = 0.2;
T = 0.5;

%result of calculating inverse kinemetic
thetaA = [-70.166696084446997 -27.204412794324707 13.280020368960283 -107.785254677590729 81.076847119486033 64.194498589472474];
thetaB = [7.004109132849219 72.754870654120566 13.280020368960281 -0.000000000000000 -72.754870654120566 -7.004109132849219];
thetaC = [109.8332 27.2045 -13.2801 -22.0744 64.5293 9.8929];

deltaC = thetaC - thetaB;
deltaB = thetaB - thetaA;

theta_p = [];
joint1 = [];joint2 = [];joint3 = [];
joint4 = [];joint5 = [];joint6 = [];
endpointA = [];endpointB = [];

position = [];
ini_p = [];
end_p = [];
ini_px = [];ini_py = [];ini_pz = [];
end_px = [];end_py = [];end_pz = [];

v_joint1 = [];v_joint2 = [];v_joint3 = [];
v_joint4 = [];v_joint5 = [];v_joint6 = [];

a_joint1 = [];a_joint2 = [];a_joint3 = [];
a_joint4 = [];a_joint5 = [];a_joint6 = [];




for t = -0.5:0.002:0.5
    if t < -0.2
        t = t + 0.5;
        linear_h = t/T;
        theta_p = deltaB*linear_h + thetaA;
        endpointA = theta_p;
        [temp1, temp2, temp3, temp4, temp5, temp6] = forward(theta_p);
        ini_px = [ini_px temp1* 2.54];
        ini_py = [ini_py temp2* 2.54];
        ini_pz = [ini_pz temp3* 2.54];
        end_px = [end_px temp4* 2.54];
        end_py = [end_py temp5* 2.54];
        end_pz = [end_pz temp6* 2.54];

        joint1 = [joint1 theta_p(1)];
        joint2 = [joint2 theta_p(2)];
        joint3 = [joint3 theta_p(3)* 2.54];
        joint4 = [joint4 theta_p(4)];
        joint5 = [joint5 theta_p(5)];
        joint6 = [joint6 theta_p(6)];

        theta_v = deltaB/T;
        v_joint1 = [v_joint1 theta_v(1)];
        v_joint2 = [v_joint2 theta_v(2)];
        v_joint3 = [v_joint3 theta_v(3)* 2.54];
        v_joint4 = [v_joint4 theta_v(4)];
        v_joint5 = [v_joint5 theta_v(5)];
        v_joint6 = [v_joint6 theta_v(6)];

        theta_a = zeros(6,1);
        a_joint1 = [a_joint1 theta_a(1)];
        a_joint2 = [a_joint2 theta_a(2)];
        a_joint3 = [a_joint3 theta_a(3)* 2.54];
        a_joint4 = [a_joint4 theta_a(4)];
        a_joint5 = [a_joint5 theta_a(5)];
        a_joint6 = [a_joint6 theta_a(6)];
    elseif t<=0.2 && t>=-0.2
        deltaB = endpointA - thetaB;
        h = (t + Tacc) / (2 * Tacc);
        theta_p = ((deltaC * (Tacc/T) + deltaB)*(2-h)*(h^2) - (2 * deltaB)) * h + endpointA;
        endpointB = theta_p;
        [temp1, temp2, temp3, temp4, temp5, temp6] = forward(theta_p);
        ini_px = [ini_px temp1* 2.54];
        ini_py = [ini_py temp2* 2.54];
        ini_pz = [ini_pz temp3* 2.54];
        end_px = [end_px temp4* 2.54];
        end_py = [end_py temp5* 2.54];
        end_pz = [end_pz temp6* 2.54];

        joint1 = [joint1 theta_p(1)];
        joint2 = [joint2 theta_p(2)];
        joint3 = [joint3 theta_p(3)* 2.54];
        joint4 = [joint4 theta_p(4)];
        joint5 = [joint5 theta_p(5)];
        joint6 = [joint6 theta_p(6)];

        theta_v = ((deltaC * (Tacc/T) + deltaB)*(1.5-h)*2*(h^2) - deltaB)/Tacc;
        v_joint1 = [v_joint1 theta_v(1)];
        v_joint2 = [v_joint2 theta_v(2)];
        v_joint3 = [v_joint3 theta_v(3)* 2.54];
        v_joint4 = [v_joint4 theta_v(4)];
        v_joint5 = [v_joint5 theta_v(5)];
        v_joint6 = [v_joint6 theta_v(6)];
        
        theta_a = ((deltaC * (Tacc/T) + deltaB)*(1-h))*(3*h)/(Tacc^2);
        a_joint1 = [a_joint1 theta_a(1)];
        a_joint2 = [a_joint2 theta_a(2)];
        a_joint3 = [a_joint3 theta_a(3)* 2.54];
        a_joint4 = [a_joint4 theta_a(4)];
        a_joint5 = [a_joint5 theta_a(5)];
        a_joint6 = [a_joint6 theta_a(6)];

    elseif t > 0.2
        t = (t-0.2);
        linear_h = t/T;
        theta_p = deltaC*linear_h + endpointB;
        [temp1, temp2, temp3, temp4, temp5, temp6] = forward(theta_p);
        ini_px = [ini_px temp1* 2.54];
        ini_py = [ini_py temp2* 2.54];
        ini_pz = [ini_pz temp3* 2.54];
        end_px = [end_px temp4* 2.54];
        end_py = [end_py temp5* 2.54];
        end_pz = [end_pz temp6* 2.54];
       
        joint1 = [joint1 theta_p(1)];
        joint2 = [joint2 theta_p(2)];
        joint3 = [joint3 theta_p(3)* 2.54];
        joint4 = [joint4 theta_p(4)];
        joint5 = [joint5 theta_p(5)];
        joint6 = [joint6 theta_p(6)];

        theta_v = deltaC/T;
        v_joint1 = [v_joint1 theta_v(1)];
        v_joint2 = [v_joint2 theta_v(2)];
        v_joint3 = [v_joint3 theta_v(3)* 2.54];
        v_joint4 = [v_joint4 theta_v(4)];
        v_joint5 = [v_joint5 theta_v(5)];
        v_joint6 = [v_joint6 theta_v(6)];

        theta_a = zeros(6,1);
        a_joint1 = [a_joint1 theta_a(1)];
        a_joint2 = [a_joint2 theta_a(2)];
        a_joint3 = [a_joint3 theta_a(3)* 2.54];
        a_joint4 = [a_joint4 theta_a(4)];
        a_joint5 = [a_joint5 theta_a(5)];
        a_joint6 = [a_joint6 theta_a(6)];
    end
end

%plot angle
figure
subplot(3,2,1);
t = 0:0.002:1;
y = joint1;
plot(t,y)
title('joint1');
xticks(0:0.1:1);
yticks(-100:50:150)
ylim([-100 150])

subplot(3,2,2);
t = 0:0.002:1;
y = joint2;
plot(t,y)
title('joint2')
xticks(0:0.1:1)
yticks(-40:20:80)
ylim([-40 80])

subplot(3,2,3);
t = 0:0.002:1;
y = joint3;
plot(t,y)
title('joint3')
ylabel('angle(degree)')
xticks(0:0.1:1)
yticks(-40:20:40);

subplot(3,2,4);
t = 0:0.002:1;
y = joint4;
plot(t,y)
title('joint4')
xticks(0:0.1:1)
yticks(-120:20:0);
ylim([-120 0])

subplot(3,2,5);
t = 0:0.002:1;
y = joint5;
plot(t,y)
title('joint5')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(-50:50:100);

subplot(3,2,6);
t = 0:0.002:1;
y = joint6;
plot(t,y)
title('joint6')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(0:20:80);
ylim([0 80])

%plot velocity
figure
subplot(3,2,1);
t = 0:0.002:1;
y = v_joint1;
plot(t,y)
title('joint1');
xticks(0:0.1:1);
yticks(150:10:210)
ylim([150 210])

subplot(3,2,2);
t = 0:0.002:1;
y = v_joint2;
plot(t,y)
title('joint2')
xticks(0:0.1:1)
yticks(-100:50:200)
ylim([-100 200])

subplot(3,2,3);
t = 0:0.002:1;
y = v_joint3;
plot(t,y)
title('joint3')
ylabel('angularvelocity(degree)')
xticks(0:0.1:1)
yticks(-150:50:0);

subplot(3,2,4);
t = 0:0.002:1;
y = v_joint4;
plot(t,y)
title('joint4')
xticks(0:0.1:1)
yticks(-50:50:250);
ylim([-50 250])

subplot(3,2,5);
t = 0:0.002:1;
y = v_joint5;
plot(t,y)
title('joint5')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(-400:200:400);
ylim([-400 400])

subplot(3,2,6);
t = 0:0.002:1;
y = v_joint6;
plot(t,y)
title('joint6')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(-150:50:50);
ylim([-150 50])

%plot angular acceleration
figure
subplot(3,2,1);
t = 0:0.002:1;
y = a_joint1;
plot(t,y)
title('joint1');
xticks(0:0.1:1);
yticks(0:50:200)
ylim([0 200])

subplot(3,2,2);
t = 0:0.002:1;
y = a_joint2;
plot(t,y)
title('joint2')
xticks(0:0.1:1)
yticks(-1200:200:0)
ylim([-1200 0])

subplot(3,2,3);
t = 0:0.002:1;
y = a_joint3;
plot(t,y)
title('joint3')
ylabel('angularvelocity(degree)')
xticks(0:0.1:1)
yticks(-600:100:0);

subplot(3,2,4);
t = 0:0.002:1;
y = a_joint4;
plot(t,y)
title('joint4')
xticks(0:0.1:1)
yticks(-1000:200:0);
ylim([-1000 0])

subplot(3,2,5);
t = 0:0.002:1;
y = a_joint5;
plot(t,y)
title('joint5')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(0:500:2500);
ylim([0 2500])

subplot(3,2,6);
t = 0:0.002:1;
y = a_joint6;
plot(t,y)
title('joint6')
xlabel('time(s)')
xticks(0:0.1:1)
yticks(0:200:800);
ylim([0 800])

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

%forward kinemetic
function [ini_x,ini_y,ini_z,end_x,end_y,end_z] = forward(theta)
    theta1 = theta(1);
    theta2 = theta(2);
    d3 = theta(3);
    theta4 = theta(4);
    theta5 = theta(5);
    theta6 = theta(6);
    
    %kinecmatic parameter
    d1 = 0;
    d2 = 6.375;
    d4 = 0;
    d5 = 0;
    d6 = 0;
    alpha1 = -90;
    alpha2 = 90;
    alpha3 = 0;
    alpha4 = -90;
    alpha5 = 90;
    alpha6 = 0;
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    theta3 = 0;

    A1 = [ cosd(theta1)    -sind(theta1)*cosd(alpha1)    sind(theta1)*sind(alpha1)    a1*cosd(theta1);
           sind(theta1)    cosd(theta1)*cosd(alpha1)     -cosd(theta1)*sind(alpha1)   a1*sind(theta1);
                     0         sind(alpha1)                 cosd(alpha1)                d1;
                     0              0                             0                       1  ];             
    A2 = [ cosd(theta2)    -sind(theta2)*cosd(alpha2)    sind(theta2)*sind(alpha2)    a2*cosd(theta2);
           sind(theta2)    cosd(theta2)*cosd(alpha2)     -cosd(theta2)*sind(alpha2)   a2*sind(theta2);
                     0         sind(alpha2)                  cosd(alpha2)                 d2;
                     0              0                             0                        1  ];
    A3 = [ cosd(theta3)    -sind(theta3)*cosd(alpha3)    sind(theta3)*sind(alpha3)    a3*cosd(theta3);
           sind(theta3)    cosd(theta3)*cosd(alpha3)     -cosd(theta3)*sind(alpha3)   a3*sind(theta3);
                     0         sind(alpha3)                  cos(alpha3)                 d3;
                     0              0                             0                        1  ];           
    A4 = [ cosd(theta4)    -sind(theta4)*cosd(alpha4)    sind(theta4)*sind(alpha4)    a4*cosd(theta4);
           sind(theta4)    cosd(theta4)*cosd(alpha4)     -cosd(theta4)*sind(alpha4)   a4*sind(theta4);
                     0         sind(alpha4)                  cosd(alpha4)                d4;
                     0              0                             0                        1  ];            
    A5 = [ cosd(theta5)    -sind(theta5)*cosd(alpha5)    sind(theta5)*sind(alpha5)    a5*cosd(theta5);
           sind(theta5)    cosd(theta5)*cosd(alpha5)     -cosd(theta5)*sind(alpha5)   a5*sind(theta5);
                     0         sind(alpha5)                  cosd(alpha5)                 d5;
                     0              0                             0                        1  ];             
    A6 = [ cosd(theta6)    -sind(theta6)*cosd(alpha6)    sind(theta6)*sind(alpha6)    a6*cosd(theta6);
           sind(theta6)    cosd(theta6)*cosd(alpha6)     -cosd(theta6)*sind(alpha6)   a6*sind(theta6);
                     0         sind(alpha6)                  cosd(alpha6)                 d6;
                     0              0                             0                        1  ];

    T6 = A1*A2*A3*A4*A5*A6;
    ini_x = T6(13);
    ini_y = T6(14);
    ini_z = T6(15);
    end_x = ini_x + T6(9) * 2;
    end_y = ini_y + T6(10) * 2;
    end_z = ini_z + T6(11) * 2;
end