function ph0 = phi(ph,xr,yr,R,L)
           % Clculates phi for pure persuit
           ph0 = (xr-R*cos(ph))^2+(yr-R*sin(ph))^2-L^2;
 