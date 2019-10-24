
clear all
close all

del_t=0.1
t=0:del_t:60;
max=max(size(t));
OneV=ones(1,601)
b=beta(t);
R=5;
mu=.3;
mass=70
x=R.*cos(b);
y=R.*sin(b);
omega=beta_dot(t);
F_cent=mass*R*(omega.^2);
F_fri=mass*mu*OneV;
ux(1)=0; del_R(1)=0;
for i=1:601
   
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

nR=sqrt(4.893^2+4.083^2)
x2=(nR).*cos(b);
y2=(nR).*sin(b);
figure;
plot(x,y,x1(1:499),y1(1:499),'+');
%plot(x,y,x1(1:450),y1(1:450),x2,y2,'-');
axis square;legend('Given Path','Path Traced');
grid; title('Path traced by RARS (\mu=0.3) '), xlabel('x (m)'),ylabel('y (m)');

figure;
plot(t,F_fri,'-r',t, F_cent,'-.g',t,FNet,'-b');grid;
xlabel('Time (s)'), ylabel('Force (N)');legend('Friction','Centrifugal','Net')
title ('Lateral Force acting on RARS');


