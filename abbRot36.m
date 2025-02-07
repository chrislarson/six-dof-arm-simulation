function R = abbRot36(th1, th2, th3, T06)

% DH Parameters
a1 = 0;
a2 = 0.27;
a3 = 0.07;

alpha1 = -pi/2;
alpha2 = 0;
alpha3 = -pi/2;

d1 = 0.290;
d2 = 0;
d3 = 0;

T01 = dhTransform(a1, d1, alpha1, th1);
T12 = dhTransform(a2, d2, alpha2, th2 - pi/2);
T23 = dhTransform(a3, d3, alpha3, th3);
T03 = T01 * T12 * T23;

R03 = rotFromTransform(T03);
R06 = rotFromTransform(T06);

R = R03\R06;

end

