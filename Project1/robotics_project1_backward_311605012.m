function robotics_project1_backward_311605012
%define output format
format long

%define the parameter that already know
d2 = 6.375;
range1 = [-160,160];
range2 = [-125,125];
range3 = [-30,30];
range4 = [-140,140];
range5 = [-100,100];
range6 = [-260,260];

fprintf("----------------------------INPUT----------------------------\n\n");

%enter the Cartesian point
T=input('Please enter the Cartesian point:\n');

     nx = T(1,1);
     ny = T(2,1);
     nz = T(3,1);
     ox = T(1,2);
     oy = T(2,2);
     oz = T(3,2);
     ax = T(1,3);
     ay = T(2,3);
     az = T(3,3);
     px = T(1,4);
     py = T(2,4);
     pz = T(3,4);

fprintf("----------------------------OUTPUT----------------------------\n\n");

%theta1_1, d3_1, theta5_1
theta1_1=atan2d(py,px)-atan2d(d2,sqrt(px^2+py^2-d2^2));
d3_1 = sqrt((cosd(theta1_1)*px+sind(theta1_1)*py)^2 + pz^2);
c2=(-(cosd(theta1_1)*px+sind(theta1_1)*py)^2+d3_1^2+pz^2)/(2*d3_1*pz);
s2=c2*(cosd(theta1_1)*px+sind(theta1_1)*py)/pz;
theta2=atan2d(s2,c2);
theta5_1=atan2d(sqrt((cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_1)*ax+cosd(theta1_1)*ay)^2),(cosd(theta1_1)*sind(theta2)*ax+sind(theta1_1)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_1)*ax+cosd(theta1_1)*ay)/sind(theta5_1),(cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_1));
theta6= atan2d((cosd(theta1_1)*sind(theta2)*ox+sind(theta1_1)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_1),(cosd(theta1_1)*sind(theta2)*nx+sind(theta1_1)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_1)));
ans1=[theta1_1 theta2 d3_1 theta4 theta5_1 theta6];
fprintf('corresponding variables ans1 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_1 || theta1_1 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_1 || d3_1 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_1 || theta5_1 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans1);
fprintf("\n");

%theta1_1, d3_1, theta5_2
theta5_2=atan2d(-sqrt((cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_1)*ax+cosd(theta1_1)*ay)^2),(cosd(theta1_1)*sind(theta2)*ax+sind(theta1_1)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_1)*ax+cosd(theta1_1)*ay)/sind(theta5_2),(cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_2));
theta6= atan2d((cosd(theta1_1)*sind(theta2)*ox+sind(theta1_1)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_2),(cosd(theta1_1)*sind(theta2)*nx+sind(theta1_1)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_2)));
ans2=[theta1_1 theta2 d3_1 theta4 theta5_2 theta6];
fprintf('corresponding variables ans2 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_1 || theta1_1 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_1 || d3_1 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_2 || theta5_2 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans2);
fprintf("\n");

%theta1_1, d3_2, theta5_1
d3_2 =- sqrt((cosd(theta1_1)*px+sind(theta1_1)*py)^2 + pz^2);
c2=(-(cosd(theta1_1)*px+sind(theta1_1)*py)^2+d3_2^2+pz^2)/(2*d3_2*pz);
s2=c2*(cosd(theta1_1)*px+sind(theta1_1)*py)/pz;
theta2=atan2d(s2,c2);
theta5_1=atan2d(sqrt((cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_1)*ax+cosd(theta1_1)*ay)^2),(cosd(theta1_1)*sind(theta2)*ax+sind(theta1_1)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_1)*ax+cosd(theta1_1)*ay)/sind(theta5_1),(cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_1));
theta6= atan2d((cosd(theta1_1)*sind(theta2)*ox+sind(theta1_1)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_1),(cosd(theta1_1)*sind(theta2)*nx+sind(theta1_1)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_1)));
ans3=[theta1_1 theta2 d3_2 theta4 theta5_1 theta6];
fprintf('corresponding variables ans3 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_1 || theta1_1 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_2 || d3_2 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_1 || theta5_1 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans3);
fprintf("\n");

%theta1_1, d3_2, theta5_2
theta5_2=atan2d(-sqrt((cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_1)*ax+cosd(theta1_1)*ay)^2),(cosd(theta1_1)*sind(theta2)*ax+sind(theta1_1)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_1)*ax+cosd(theta1_1)*ay)/sind(theta5_2),(cosd(theta1_1)*cosd(theta2)*ax+sind(theta1_1)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_2));
theta6= atan2d((cosd(theta1_1)*sind(theta2)*ox+sind(theta1_1)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_2),(cosd(theta1_1)*sind(theta2)*nx+sind(theta1_1)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_2)));
ans4=[theta1_1 theta2 d3_2 theta4 theta5_2 theta6];
fprintf(' corresponding variables ans4 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_1 || theta1_1 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_2 || d3_2 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_2 || theta5_2 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans4);
fprintf("\n");

%theta1_2, d3_1, theta5_1
theta1_2=atan2d(py,px)-atan2d(d2,-sqrt(px^2+py^2-d2^2));
d3_1 = sqrt((cosd(theta1_2)*px+sind(theta1_2)*py)^2 + pz^2);
c2=(-(cosd(theta1_2)*px+sind(theta1_2)*py)^2+d3_1^2+pz^2)/(2*d3_1*pz);
s2=c2*(cosd(theta1_2)*px+sind(theta1_2)*py)/pz;
theta2=atan2d(s2,c2);
theta5_1=atan2d(sqrt((cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_2)*ax+cosd(theta1_2)*ay)^2),(cosd(theta1_2)*sind(theta2)*ax+sind(theta1_2)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_2)*ax+cosd(theta1_2)*ay)/sind(theta5_1),(cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_1));
theta6= atan2d((cosd(theta1_2)*sind(theta2)*ox+sind(theta1_2)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_1),(cosd(theta1_2)*sind(theta2)*nx+sind(theta1_2)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_1)));
ans5=[theta1_2 theta2 d3_1 theta4 theta5_1 theta6];
fprintf('corresponding variables ans5 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_2 || theta1_2 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_1 || d3_1 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_1 || theta5_1 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans5);
fprintf("\n");

%theta1_2, d3_1, theta5_2
theta5_2=atan2d(-sqrt((cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_2)*ax+cosd(theta1_2)*ay)^2),(cosd(theta1_2)*sind(theta2)*ax+sind(theta1_2)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_2)*ax+cosd(theta1_2)*ay)/sind(theta5_2),(cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_2));
theta6= atan2d((cosd(theta1_2)*sind(theta2)*ox+sind(theta1_2)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_2),(cosd(theta1_2)*sind(theta2)*nx+sind(theta1_2)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_2)));
ans6=[theta1_2 theta2 d3_1 theta4 theta5_2 theta6];
fprintf('corresponding variables ans6 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_2 || theta1_2 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_1 || d3_1 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_2 || theta5_2 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans6);
fprintf("\n");

%theta1_2, d3_2, theta5_1
d3_2 = -sqrt((cosd(theta1_2)*px+sind(theta1_2)*py)^2 + pz^2);
c2=(-(cosd(theta1_2)*px+sind(theta1_2)*py)^2+d3_2^2+pz^2)/(2*d3_2*pz);
s2=c2*(cosd(theta1_2)*px+sind(theta1_2)*py)/pz;
theta2=atan2d(s2,c2);
theta5_1=atan2d(sqrt((cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_2)*ax+cosd(theta1_2)*ay)^2),(cosd(theta1_2)*sind(theta2)*ax+sind(theta1_2)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_2)*ax+cosd(theta1_2)*ay)/sind(theta5_1),(cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_1));
theta6= atan2d((cosd(theta1_2)*sind(theta2)*ox+sind(theta1_2)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_1),(cosd(theta1_2)*sind(theta2)*nx+sind(theta1_2)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_1)));
ans7=[theta1_2 theta2 d3_2 theta4 theta5_1 theta6];
fprintf('corresponding variables ans7 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_2 || theta1_2 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_2 || d3_2 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_1 || theta5_1 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans7);
fprintf("\n");

%theta1_2, d3_1, theta5_2
theta5_2=atan2d(-sqrt((cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)^2+(-sind(theta1_2)*ax+cosd(theta1_2)*ay)^2),(cosd(theta1_2)*sind(theta2)*ax+sind(theta1_2)*sind(theta2)*ay+cosd(theta2)*az));
theta4= atan2d((-sind(theta1_2)*ax+cosd(theta1_2)*ay)/sind(theta5_2),(cosd(theta1_2)*cosd(theta2)*ax+sind(theta1_2)*cosd(theta2)*ay-sind(theta2)*az)/sind(theta5_2));
theta6= atan2d((cosd(theta1_2)*sind(theta2)*ox+sind(theta1_2)*sind(theta2)*oy+cosd(theta2)*oz)/sind(theta5_2),(cosd(theta1_2)*sind(theta2)*nx+sind(theta1_2)*sind(theta2)*ny+cosd(theta2)*nz)/(-sind(theta5_2)));
ans8=[theta1_2 theta2 d3_2 theta4 theta5_2 theta6];
fprintf('corresponding variables ans8 (£c1, £c2, £c3, £c4, £c5, £c6):\n');

%check the input whether is out of the range
if range1(1) >= theta1_2 || theta1_2 >= range1(2)
    disp('£c1 is out of range!'); 
end
if range2(1) >= theta2 || theta2 >= range2(2)
	disp('£c2 is out of range!');
end    
if range3(1) >= d3_2 || d3_2 >= range3(2)
    disp('d3 is out of range!');
end
if range4(1) >= theta4 || theta4 >= range4(2)
	disp('£c4 is out of range!');
end
if range5(1) >= theta5_2 || theta5_2 >= range5(2)
    disp('£c5 is out of range!');
end
if range6(1) >= theta6 || theta6 >= range6(2)
    disp('£c6 is out of range!');
end
fprintf("%.15f  ",ans8);
fprintf("\n");


end