%*** BASIC HELPER FUNCTIONS ***

function judge = Nearzero(near)
% Takes a scalar.
% Checks if the scalar is small enough to be neglected.
% Example Input:
%{ 
  clear;clc;
  near = -1e-6;
  judge = Nearzero(near)
%} 
% Output:
% judge =
%     1
if norm(near) < 1e-5
    judge = true;
else
    judge = false;
end
end