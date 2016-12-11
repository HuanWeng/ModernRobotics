%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function  [R,p]= TransToRp(T)
% Take transformation matrix T SE(3) 
% Returns R the corresponding rotation matrix,
% and p the corresponding position vector
% Example Input:
%{
  clear;clc;
  T = [[1,0,0,0]; [0,0,-1,0]; [0,1,0,3]; [0,0,0,1]];
  [R,p]= TransToRp(T)
%}
% Output:
% R =
%     1     0     0
%     0     0    -1
%     0     1     0
% p =
%     0
%     0
%     3

R=T(1:3,1:3);
p=T(1:3,4);
end

