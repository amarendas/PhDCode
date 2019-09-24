clear all
syms t x(t)
% some constants
syms m k b f
syms THE_X THE_XD THE_XDD
HOLDER_list = [THE_X THE_XD THE_XDD]
actual_list = [ x diff(x,t) diff(x,t,2)]
% apply physics
v=diff(x,t)
KE= 0.5 *m *v^2
PE=0.5*k*x^2
L=KE-PE
L_new=subs(L,actual_list, HOLDER_list)
dLdx=diff (L_new,THE_X)
dLdxdot=diff(L_new,THE_XD)
dLdxdot=subs(dLdxdot,HOLDER_list, actual_list)
dt_of_dLdxdot=diff(dLdxdot,t)
our_EOM_LHS=dt_of_dLdxdot-dLdx
our_EOM_LHS=subs(our_EOM_LHS,HOLDER_list,actual_list)
