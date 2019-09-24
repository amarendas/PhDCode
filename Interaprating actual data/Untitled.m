clear 
M=csvread('Testdata_24_09_2018(2).txt')
N=max(size(t1));
dif=zeros(N,1);
dif1=zeros(N,1);

for i=1:N-1
    if t1(i+1)>t1(i);
        dif(i)=t1(i+1) -t1(i);
    else
        dif(i)=1000000-t1(i)+t1(i+1);
    end
    %dif(i)=t1(i+1) -t1(i);
end
t=1:N;
plot(t,dif);
t_mean=mean(dif(10:N-5));
t_std=std(dif(10:N-5));
