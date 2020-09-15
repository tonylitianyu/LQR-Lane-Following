% syms n x
% 
% 
% 
% eqn = 2*n*x^2 - 3*(1-n)*x + 2 == 0
% 
% sol = simplify(solve(eqn,x))
% sol = sol(2)
% 
% lim = double(subs(sol,n,0.0001))
% 
% xa = linspace(0.0001,0.25,50);
% ya = (atan(double(subs(sol,n,xa))).*180)./pi;
% 
% 
% plot(xa,ya)
% grid on
% title('Anthony Li - Homework #1 - Problem 1')
% xlabel('\eta')
% ylabel('\beta_{c}')
syms b sigma theta

xaa = linspace(0,6,60)
yaa = ((atan(2*xaa)/(-2))+(pi/2))*(180/pi)
plot(xaa,yaa)
grid on
title('Anthony Li - Homework #1 - Problem 2')
xlabel('\beta')
ylabel('\theta^*')