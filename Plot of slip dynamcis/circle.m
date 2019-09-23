
clear all
del_t=0.1
t=0:del_t:60;
b=beta(t);
R=5;
mass=30
x=R.*cos(b);
y=R.*sin(b);
omega=beta_dot(t);
F_cent=mass*R*(omega.^2);
ux(1)=0; del_R(1)=0;
for i=1:601
   
    FNet(i)=F_cent(i)-mass*.16;
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
plot(x,y.'.',x1(1:500),y1(1:500));axis square
figure;plot(t,FNet,t, omega*R)
