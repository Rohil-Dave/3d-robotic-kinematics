%Test 4 01881518
clear all; %Clear all vars
clc; %Clear screen


%%
%--Question 1
% My choice is 2
% See written explanation or MATLAB code below
% Determinant of rotation matrix is 1
% Therefore size of R*v = size of v
% so ratio of sizes v'/v = 1

% therefor choice 2 is the answer


%%
%--Question 2
% My choice is 1
% See written explanation or MATLAB code below
% Z Y Z is valid since this is a moving frame rotation
% in moving axes the new z axis is a new axis that is linearly
% independent of the previous axes used for rotations and thus it is a
% valid rotation

% therefore choice 1 (yes) is the answer





%%
%--Question 3
% My choice is 1
% See written explanation or MATLAB code below
% vector v is linearly dependent on Y' and Z axes and since
% these axes were already used for rotation thus this is not a 
% valid Euler rotation

% Euler states that "Any rotation can be described by three successive rotations around
% linearly independent axes", vector v does not satisfy this

% therefore the choice 1 (no) is the answer



%%
%--Question 4
% My choice is 2
% See written explanation or MATLAB code below

% choice 2 is a condition for singularity as column 3 is linearly dependent
% on column 1 as it is column scaled
% choice 3 makes the two columns perpendicular to each and does not make
% the jacobian matrix singular so choice 3 can be eliminated
% choice 1 shows that dot product of the columns (inner product) is not 1
% so the columns are linearly independent making the jacobian non singular

% therefore choice 2 is the answer



%%
%--Question 5
% My choice is 1
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen

f1 = [414.9; -264.9; -324.7]; % force vector 1

f2 = [-324.7; -264.9; 414.9]; % force vector 1

J = [0.2 0.1 0.5;... % jacobian matrix
    0.15 0.05 0.01;...
    0.01 0.01 0.6];

tau1 = J'*f1  % computes torques experienced
tau2 = J'*f2

% f2 results in torque value for joint 3 that is well above limit
% f1 results in torque value that is acceptable by max values

% therefore choice 1 is the answer


% question asks for force vector when torques are half max
% tau
tau = [20;12.5;5]; % torque half of max

f = inv(J')*tau
% f results in halved values of choice 1, confirming it is the answer
% as f is closest to choice 1 compared to choice 2


%%
%--Question 6
% My choice is 3
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen
syms th d a alpha real; %Define symbolic variables as real variables

%Rotation matrix around z_i axis
rotation_z = [cos(th) -sin(th) 0;...
              sin(th) cos(th) 0;...
              0 0 1];
%Translation along z_i axis
translation_z = [0 0 d]';

%The screw on z_i axis. A Screw is a transformation matrix where rotation
%and translation happen on just one axis.
screw_z = [ rotation_z translation_z;...
            0 0 0 1];
        
%Rotation matrix around x_(i-1) axis     
rotation_x = [1 0 0;...
              0 cos(alpha) -sin(alpha);...
              0 sin(alpha) cos(alpha)];
%Translation along x_(i-1) axis         
translation_x = [a 0 0]';

%The screw on x_(i-1) axis
screw_x = [ rotation_x translation_x;...
            0 0 0 1];
        
%The homogeneous transformation matrix
T = screw_x*screw_z

% angle between x axis ith frame and z axis (i-1)frame is determined by
% element in 3rd row, 1st column
element_in_question = T(3,1) % gives sin(alpha)*sin(th)

% define relevant parameters
th = pi/6;
alpha = pi/3;

ang = acosd(sin(alpha)*sin(th))

% angle is 64.34 deg therefore choice 3 is the answer

%%
%--Question 7
% My choice is 2
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen

syms L1 L2 L3 L4 theta1 theta2 theta3 real; %Define symbolic variables as real variables

%     ai alphai    di thetai
DH = [L1 sym(pi)/3      L2 theta1;...   % Joint 1
      0 sym(pi)/6 L3 theta2;...   % Joint 2
      0 0  L4 theta3];     % Joint 3  

 %Separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end

T1 = vpa(T(1).A,3); % transformation matrix frame 1 to base
T2 = T(2).A; % transformation matrix from frame 2 to frame 1
T3 = T(3).A; % transformation matrix from frame 3 to frame 2

Tr3to1 = vpa(simplify(T2*T3),2) % computes transformation matrix from frame 3 to frame 1, to 2 decimals

% therefore choice 2 is the answer


%%
%--Question 8
% My choice is 4
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen

syms L1 L2 L3 L4 theta1 theta2 theta3 real; %Define symbolic variables as real variables

%     ai alphai    di thetai
DH = [L1 sym(pi)/3      L2 theta1;...   % Joint 1
      0 sym(pi)/6 L3 theta2;...   % Joint 2
      0 0  L4 theta3];     % Joint 3  

 %Separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end

T1 = vpa(T(1).A,3); % transformation matrix frame 1 to base
T2 = T(2).A; % transformation matrix from frame 2 to frame 1
T3 = T(3).A; % transformation matrix from frame 3 to frame 2

Tr3to1 = vpa(simplify(T2*T3),2) % computes transformation matrix from frame 3 to frame 1, to 2 decimal

% from Tr3tobase we know last column is position vector
P = vpa(Tr3to1(1:3,4),2);

dist = simplify(sqrt(P'*P))  % compute distance between origin 3rd frame to 1st frame

% dist simplifies to L3+L4 as check = 1
check = 18446744075499707273^(1/2)/4294967296

%therefore choice 4 is the answer

%%
%--Question 9
% My choice is 3
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen

R = [-0.595 -0.677 0.433;...
    -0.097 -0.4742 -0.875;...
    0.7978 -0.5626 0.2165];

% solving for Euler parameters
ep4 = 0.5*sqrt(1 + R(1,1) + R(2,2) + R(3,3));
theta = 2*acos(ep4);
ep1 = (R(3,2)-R(2,3))/(4*ep4);
ep2 = (R(1,3)-R(3,1))/(4*ep4);
ep3 = (R(2,1)-R(1,2))/(4*ep4);

% compute values for i j k based on calculated Euler parameters
c1 = ep1/sin(theta/2)
c2 = ep2/sin(theta/2)
c3 = ep3/sin(theta/2)

% c1 = 0.4147
% c2 = -0.4843
% c3 = 0.7699

unitsum = c1^2 + c2^2 + c3^2 % check; equals 1 because unit vector

% therefore choice 3 is the answer


%%
%--Question 10
% My choice is 2
% See written explanation or MATLAB code below
clear all; %Clear all vars
clc; %Clear screen

syms L1 L2 L3 L4 theta1 theta2 theta3 real; %Define symbolic variables as real variables

%     ai alphai    di thetai
DH = [L1 sym(pi)/3      L2 theta1;...   % Joint 1
      0 sym(pi)/6 L3 theta2;...   % Joint 2
      0 0  L4 theta3];     % Joint 3  

 %Separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end

T1 = vpa(T(1).A,3); % transformation matrix frame 1 to base
T2 = T(2).A; % transformation matrix from frame 2 to frame 1
T3 = T(3).A; % transformation matrix from frame 3 to frame 2

Tr3tobase = vpa(simplify(T1*T2*T3),2) % computes transformation matrix from frame 3 to base frame

% from Tr3tobase we know last column is position vector
P = vpa(Tr3tobase(1:3,4),2)

J = jacobian(P, [theta1, theta2, theta3]) % jacobian matrix

% therefore choice 2 is the answer

