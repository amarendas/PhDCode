function [C,Ceq]=nonlincon1(y)
C1=y(4)-(y(3)/2)*(1-y(1)/y(2))+20;
C2=sqrt((y(3)/2)^2+y(1)^2)-100;
C=[C1;C2];
Ceq=[];