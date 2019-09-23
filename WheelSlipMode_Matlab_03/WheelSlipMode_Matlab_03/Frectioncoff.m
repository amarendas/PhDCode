function mu=Frectioncoff(lam)
% lam: is slip defined as (actual linear displacement - wheel rotation *
% radius of wheel) / max of the above two quantaties
mu_P=0.1;% peak coefficient of friction
%lam
if ((-0.15<lam)&&(lam<0.15))
    mu=(lam-0.15)*mu_P/0.15+mu_P;
end
if (0.15<=lam)&&(lam<=1)
    mu=-(lam-0.15)*0.34*mu_P/0.85+mu_P;
end
if (-0.15>=lam)&&(lam>=-1)
    mu=-(lam+0.15)*0.34*mu_P/0.85-mu_P;
end
