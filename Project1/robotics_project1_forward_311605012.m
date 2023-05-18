function robotics_project1_forward_311605012
%define output format
format long

fprintf("----------------------------INPUT----------------------------\n\n");

%  enter the joint variables
input_msg = "Please enter the joint variable of theta1 between ";
error_msg = "is out of range";
error_msg2 = " ,please type it again.\n";
range1 = [-160,160];
range2 = [-125,125];
range3 = [-30,30];
range4 = [-140,140];
range5 = [-100,100];
range6 = [-260,260];

%check the input whether is out of the range
%If the input is out of the range, calling the user to type the value again
while 1
    theta1 = input(input_msg + num2str(range1(1)) + " ~ " + num2str(range1(2)) + " :\n");
    if theta1 <= range1(2) && theta1 >= range1(1)
        break;
    end
    fprintf(2,"£c1 " + error_msg + error_msg2);
end
while 1
    theta2 = input(input_msg + num2str(range2(1)) + " ~ " + num2str(range2(2)) + " :\n");
    if theta2 <= range2(2) && theta2 >= range2(1)
        break;
    end
    fprintf(2,"£c2 " + error_msg + error_msg2);
end
while 1
    d3 = input(input_msg + num2str(range3(1)) + " ~ " + num2str(range3(2)) + " :\n");
    if d3 <= range3(2) && d3 >= range3(1)
        break;
    end
    fprintf(2,"£c3 " + error_msg + error_msg2);
end
while 1
    theta4 = input(input_msg + num2str(range4(1)) + " ~ " + num2str(range4(2)) + " :\n");
    if theta4 <= range4(2) && theta4 >= range4(1)
        break;
    end
    fprintf(2,"£c4 " + error_msg + error_msg2);
end
while 1
    theta5 = input(input_msg + num2str(range5(1)) + " ~ " + num2str(range5(2)) + " :\n");
    if theta5 <= range5(2) && theta5 >= range5(1)
        break;
    end
    fprintf(2,"£c5 " + error_msg + error_msg2);
end
while 1
    theta6 = input(input_msg + num2str(range6(1)) + " ~ " + num2str(range6(2)) + " :\n");
    if theta6 <= range6(2) && theta6 >= range6(1)
        break;
    end
    fprintf(2,"£c6 " + error_msg + error_msg2);
end

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
x = T6(13);
y = T6(14);
z = T6(15);
phi = atan2(T6(2,3),T6(1,3));
phi = phi*180/pi;
theta = atan2((cosd(phi)*T6(9) + sind(phi)*T6(10)),T6(11));
theta = theta*180/pi;
psi = atan2((-sind(phi)*T6(1) + cosd(phi)*T6(2)),(-sind(phi)*T6(5) + cosd(phi)*T6(6)));
psi = psi*180/pi;

fprintf("----------------------------OUTPUT----------------------------\n\n");
fprintf("(n, o, a, p):\n");
disp(T6);
fprintf("Cartesian point (x, y, z, £r, £c, £Z):\n");
fprintf('%f ', x , y , z ,  phi,  theta,  psi);
fprintf("\n");

end