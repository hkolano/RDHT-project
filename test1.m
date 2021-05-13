
%%% code for syms equation 
clear
syms theta1 x1 dtheta1 dx1 ddtheta1 ddx1 theta2 x2 dtheta2 dx2 ddtheta2 ddx2 
syms Tin bp r kp Ip Mpd A1 a1 A2 a2 Tout Mw1 Mw2 b k F1 F2


y1=A1*x1/a1+10;
y2=A2*x2/a2+10;
dy1=A1*dx1/a1;
dy2=A2*dx2/a2;
ddy1=A1*ddx1/a2;
ddy2=A2*ddx2/a2;


f(1)=Tin-bp*(dx1/r-dtheta1)-kp*(x1/r-theta1)-Ip*ddtheta1==0;

f_2a=bp*(dx1-dtheta1*r)+kp*(x1-theta1*r)-F1-Mpd*ddx1==0;
f_2b=F1*a1/A1-b*dy1-k*(y2-y1)-Mw1*ddy1==0;
f_2_solve=solve(f_2a,F1);
f(2)=subs(f_2b,F1,f_2_solve);
f(4)=-bp*(dx2/r-dtheta2)-kp*(x2/r-theta2)-Ip*ddtheta2-Tout==0;

f_3a=bp*(dx2-dtheta2*r)+kp*(x2-theta2*r)+F2-Mpd*ddx2==0;
f_3b=-(F2*a2/A2)-(b*dy2)+(k*(y2-y1))-Mw2*ddy2==0;

f_3_solve=solve(f_3a,F2);
f(3)=subs(f_3b,F2,f_3_solve);

[A,B]=equationsToMatrix(f,[theta1 dtheta1 x1 dx1 x2 dx2 theta2 dtheta2])
