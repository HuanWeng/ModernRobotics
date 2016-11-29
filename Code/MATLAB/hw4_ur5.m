clear;clc;
%This program will simulate the ur5 robot falls under the gravity only.
%There is no for apply on the end-effector tip, and not joint torque and
%force.
thetalist = [0.0,0.0,0.0,0.0,0.0,0.0];
dthetalist = [0.0,0.0,0.0,0.0,0.0,0.0];
taumat = zeros(1000,6);
Ftipmat = zeros(1000,6);
g = [0,0,-9.81];

G1 = [[0.010267495893,0.,0.,0.,0.,0.];[0.,0.010267495893,0.,0.,0.,0.];[0.,0.,0.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
G2 = [[0.22689067591,0.,0.,0.,0.,0.];[0.,0.22689067591,0.,0.,0.,0.];[0.,0.,0.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
G3 = [[0.049443313556,0.,0.,0.,0.,0.];[0.,0.049443313556,0.,0.,0.,0.];[0.,0.,0.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];
G4 = [[0.111172755531,0.,0.,0.,0.,0.];[0.,0.111172755531,0.,0.,0.,0.];[0.,0.,0.21942,0.,0.,0.];[0.,0.,0.,1.219,0.,0.];[0.,0.,0.,0.,1.219,0.];[0.,0.,0.,0.,0.,1.219]];
G5 = [[0.111172755531,0.,0.,0.,0.,0.];[0.,0.111172755531,0.,0.,0.,0.];[0.,0.,0.21942,0.,0.,0.];[0.,0.,0.,1.219,0.,0.];[0.,0.,0.,0.,1.219,0.];[0.,0.,0.,0.,0.,1.219]];
G6 = [[0.0171364731454,0.,0.,0.,0.,0.];[0.,0.0171364731454,0.,0.,0.,0.];[0.,0.,0.033822,0.,0.,0.];[0.,0.,0.,0.1879,0.,0.];[0.,0.,0.,0.,0.1879,0.];[0.,0.,0.,0.,0.,0.1879]];
Glist = [G1;G2;G3;G4;G5;G6];

M01 = [[1.,0.,0.,0.];
       [0.,1.,0.,0.];
       [0.,0.,1.,0.];
       [0.,0.,.089159,1.]]';
M12 = [[0.,0.,-1.,0.];
       [0.,1.,0.,0.];
       [1.,0.,0.,0.];
       [0.28,.13585,0.,1.]]';
M23 = [[1.,0.,0.,0.];
       [0.,1.,0.,0.];
       [0.,0.,1.,0.];
       [0.,-.1197,.395,1]]';
M34 = [[0.,0.,-1.,0.];
       [0.,1.,0.,0.];
       [1.,0.,0.,0.];
       [0.,0.,.14225,1]]';
M45 = [[1.,0.,0.,0.];
       [0.,1.,0.,0.];
       [0.,0.,1.,0.];
       [0.,.093,0.,1]]';
M56 = [[1.,0.,0.,0.];
       [0.,1.,0.,0.];
       [0.,0.,1.,0.];
       [0.,0.,.09465,1]]';
M67 = [[ 1., 0., 0., 0.];
       [0., 0., 1., 0.0823];
       [0., -1., 0., 0.];
       [0., 0., 0., 1.]];   
   
Mlist = [M01;M12;M23;M34;M45;M56;M67];

Slist = [[0,0,1,0,0,0]; [0,1,0,-0.089159,0,0]; [0,1,0,-0.089159,0,0.425];[0,1,0,-0.089159,0,0.81725]; [0,0,-1,-0.10915,0.81725,0]; [0,1,0,0.005491,0,0.81725]];

dt = 0.01;
intRes = 10;

[thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt,intRes);
csvwrite('/Users/51ibm/Dropbox/ur5robot.csv',thetamat)

