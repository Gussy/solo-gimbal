function angMeas = calcMagAng(decl,gPhi,gPsi,gTheta,magX,magY,magZ,q0,q1,q2,q3)
%CALCMAGANG
%    ANGMEAS = CALCMAGANG(DECL,GPHI,GPSI,GTHETA,MAGX,MAGY,MAGZ,Q0,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    14-Jan-2015 16:51:18

t2 = cos(gPhi);
t3 = cos(gTheta);
t4 = sin(gPhi);
t5 = sin(gTheta);
t6 = q0.^2;
t7 = q1.^2;
t8 = q2.^2;
t9 = q3.^2;
t10 = t6+t7-t8-t9;
t11 = sin(gPsi);
t12 = cos(gPsi);
t13 = q0.*q2.*2.0;
t14 = q1.*q3.*2.0;
t15 = t13+t14;
t16 = q0.*q3.*2.0;
t18 = q1.*q2.*2.0;
t17 = t16-t18;
t19 = t3.*t11;
t20 = t4.*t5.*t12;
t21 = t19+t20;
t22 = t16+t18;
t23 = t5.*t11;
t24 = t23-t3.*t4.*t12;
t25 = q0.*q1.*2.0;
t31 = q2.*q3.*2.0;
t26 = t25-t31;
t27 = t6-t7+t8-t9;
t28 = t5.*t12;
t29 = t3.*t4.*t11;
t30 = t28+t29;
t32 = t3.*t12;
t33 = t32-t4.*t5.*t11;
angMeas = -decl-tan((-magY.*(t21.*t22-t24.*t26+t2.*t12.*t27)+magX.*(-t22.*t33+t26.*t30+t2.*t11.*t27)+magZ.*(-t4.*t27+t2.*t5.*t22+t2.*t3.*t26))./(magY.*(t10.*t21+t15.*t24-t2.*t12.*t17)+magX.*(t10.*t33+t15.*t30+t2.*t11.*t17)-magZ.*(t4.*t17+t2.*t5.*t10-t2.*t3.*t15)));
