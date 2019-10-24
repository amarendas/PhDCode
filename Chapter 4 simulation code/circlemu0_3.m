
clear all
close all

del_t=0.1;
t=0:del_t:60;
maxN=max(size(t));
OneV=ones(1,maxN);
b=beta(t);
R=5;
mu=.3;
mass=70;
x=R.*cos(b);
y=R.*sin(b);
omega=beta_dot(t);
F_cent=mass*R*(omega.^2);
F_fri=mass*mu*OneV;
ux(1)=0; del_R(1)=0;
for i=1:maxN
   
    FNet(i)=F_cent(i)-mass*mu;
    if FNet(i)<0
        FNet(i)=0;
    end
   
    if i>1
        ax(i)=FNet(i-1)/mass;
        ux(i)=ux(i-1)+ ax(i)*del_t;
%         del_R(i)=del_R(i-1)+ux(i-1)*del_t;
    end
end
x1=(R+ux).*cos(b);
y1=(R+ux).*sin(b);

nR=sqrt(4.893^2+4.083^2); 
x2=(nR).*cos(b);
y2=(nR).*sin(b);

%------------Result manipulation ----------------
TrimN=499;
t1=linspace(0,60,TrimN);
x1Trim=x1(1:TrimN);y1Trim=y1(1:TrimN);
x1Int=interp1(t1,x1(1:TrimN),t,'spline');
y1Int=interp1(t1,y1(1:TrimN),t,'spline');
xerror=x-x1Int;
yerror=y-x1Int;
delR=sqrt(xerror.^2+yerror.^2);
s=zeros(maxN,1);
s1=s;
for i=2:maxN
    s(i)=sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2)+s(i-1);
    s1(i)=sqrt((x1Int(i)-x1Int(i-1))^2+(y1Int(i)-y1Int(i-1))^2)+s1(i-1);
end
Serror=s-s1;
figure; plot(t,s,t,s1,t,Serror);grid;title 'Simulation with with wheel slip (mu=0.3)'; xlabel('time'),ylabel('Circumferential distance m');
legend('Given Path', 'Traveled Parth', 'Error in distance traveled')

figure; plot3(x,y,t,x1Int,y1Int,t);grid;title ' 3-D plot of robots path'; xlabel 'x'; ylabel 'y'; zlabel 'time';
figure;plot(x,y, 'r',x1Int,y1Int,'--');axis square;legend('Given Path','Path Traced');grid; title('Path traced by RARS (\mu=0.3) '), xlabel('x (m)'),ylabel('y (m)');
figure; plot (t,x,t,x1Int, t,x-x1Int); grid; title ' X co-ordiinate Plot'; xlabel ' time' ;legend('Given Path x Coord','Path Traced x Coord', 'Error in x ')
figure; plot (t,y,t,y1Int, t,y-y1Int); grid; title ' Y co-ordiinate Plot'; xlabel ' time' ;legend('Given Path y Coord','Path Traced y Coord', 'Error in y ')
figure; plot (5-sqrt(x1Int.^2+y1Int.^2));grid ; title ' error radial '
figure;
plot(t,F_fri,'-r',t, F_cent,'-.g',t,FNet,'-b');grid;
xlabel('Time (s)'), ylabel('Force (N)');legend('Friction','Centrifugal','Net')
title ('Lateral Force acting on RARS');


