%**********************************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
%**********************************************************************************************

function angvmat = MatrixLog3(R)
% Takes R (rotation matrix)
% Returns the corresponding 3-vector of exponential coordinates (expc3 = omghat*theta)
% Example Input:
%{
  clear;clc;
  R = [[0, 0,1];[1, 0, 0];[0, 1, 0]];
  expc3 = MatrixLog3(R)
%} 
% Output:
% angvmat =
%         0   -1.2092    1.2092
%    1.2092         0   -1.2092
%   -1.2092    1.2092         0
if Nearzero(norm(R-eye(3)))
	angvmat=zeros(3);
elseif Nearzero(trace(R)+1)
	if ~Nearzero(1+R(3,3))
        omg=(1/(2*(1+R(3,3)))^0.5)*[R(1,3);R(2,3);1+R(3,3)];
    elseif ~Nearzero(1+R(2,2))
        omg=(1/(2*(1+R(2,2)))^0.5)*[R(1,2);1+R(2,2);R(3,2)];
    else
        omg=(1/(2*(1+R(1,1)))^0.5)*[1+R(1,1);R(2,1);R(3,1)];
    end
	angvmat=VecToso3(pi*omg);
else
	acosinput = (trace(R)-1)/2;
	if acosinput > 1
        acosinput = 1;
    elseif acosinput < -1
        acosinput = -1;
    end
	theta=acos(acosinput);
	omgmat=(1/(2*sin(theta)))*(R-R');
	angvmat=theta*omgmat;
end

end

