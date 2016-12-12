%**************************************************************************
%****************************  CHAPTER 3: RIGID-BODY MOTIONS  *************
%**************************************************************************

function expmat = MatrixLog6(T)
% Takes a transformation matrix T SE(3)
% Returns the corresponding 6-vector of exponential coordinates S*theta
% Example Input:
%{
  clear;clc;
  T = [[1,0,0,0]; [0,0,-1,0]; [0,1,0,3]; [0,0,0,1]];
  expmat = MatrixLog6(T)
%} 
% Output:
% expc6 =
%    1.5708
%         0
%         0
%         0
%    2.3562
%    2.3562

[R,p]=TransToRp(T);
if Nearzero(norm(R-eye(3)))
    expmat = [zeros(3),T(1:3,4);0,0,0,1];
    
    
    elseif norm(R(1,1)+R(2,2)+R(3,3)+1)<1e-5
        theta=pi;
        omg=MatrixLog3(R);
        v=(eye(3)/theta-0.5*VecToso3(omg)+(1/theta-0.5*cot(theta/2))*VecToso3(omg)*VecToso3(omg))*p;
    else
        acosinput = (R(1,1)+R(2,2)+R(3,3)-1)/2;
        if acosinput > 1
            acosinput = 1;
        elseif acosinput < -1
            acosinput = -1;
        end
        theta=acos(acosinput);
        w1=1/(2*sin(theta))*(R-R');
        omg=so3ToVec(w1);
        v=(eye(3)/theta-0.5*VecToso3(omg)+(1/theta-0.5*cot(theta/2))*VecToso3(omg)*VecToso3(omg))*p;
    end
    expmat=theta*[omg;v];
end

