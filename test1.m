
%%% code for syms equation 
clear
syms theta1 x1 dtheta1 dx1 ddtheta1 ddx1 theta2 x2 dtheta2 dx2 ddtheta2 ddx2 
syms Tin bp r kp Ip Mpd A a Tout Mw1 Mw2 b k F


y1=A*x1/a+10;
y2=A*x2/a+10;
dy1=A*dx1/a;
dy2=A*dx2/a;
ddy1=A*ddx1/a;
ddy2=A*ddx2/a;


f(1)=Tin-bp*(dx1/r-dtheta1)-kp*(x1/r-theta1)-Ip*ddtheta1==0;

f_2a=bp*(dx1-dtheta1*r)+kp*(x1-theta1*r)-F-Mpd*ddx1==0;
f_2b=F*a/A-b*dy1-k*(y2-y1)-Mw1*ddy1==0;

f(3)=bp*(dx2/r-dtheta2)-kp*(x2/r-theta2)-Ip*ddtheta2-Tout==0;

f_4a=bp*(dx2-dtheta2*r)+kp*(x2-theta2*r)-F-Mpd*ddx2==0;
f_4b=-(F*a/A)-(b*dy2)-(k*(y2-y1))-Mw2*ddy2==0;

f(2)=f_2a+f_2b;
f(4)=f_4a+f_4b;

[A,B]=equationsToMatrix(f,[theta1 dtheta1 x1 dx1 theta2 dtheta2 x2 dx2])
