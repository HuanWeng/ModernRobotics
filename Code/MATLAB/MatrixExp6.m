%*** CHAPTER 3: RIGID-BODY MOTIONS ***

function T = MatrixExp6(expmat)
% Takes a 6-vector of exponential coordinates (S*theta) 
% Returns a T matrix SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I
% Rodriguez R = I + sin(theta)*omg + (1-cos(theta))*omg^2
% Example Input:
%{
  clear;clc;
  expmat = [ 0,      0,       0,      0;
             0,      0, -1.5708, 2.3562;
             0, 1.5708,       0, 2.3562;
             0,      0,       0,      0]
  T = MatrixExp6(expmat)
%}
% Output:
% T =
%    1.0000         0         0         0
%         0    0.0000   -1.0000   -0.0000
%         0    1.0000    0.0000    3.0000
%         0         0         0    1.0000 
omgtheta = so3ToVec(expmat(1:3,1:3));
if Nearzero(norm(omgtheta))
    T = [eye(3), expmat(1:3,4); 0, 0, 0, 1];
else
    [omghat,theta] = AxisAng3(omgtheta);
    omgmat = expmat(1:3,1:3) / theta;
    term3 = (theta - sin(theta)) * omgmat * omgmat
    G = eye(3) * theta + (1 - cos(theta)) * omgmat + term3;
    termv =  G * expmat(1:3,4) / theta;
    T = [MatrixExp3(expmat(1:3,1:3)), termv; 0, 0, 0, 1];
end
end

