%{
    * The observation results of the disturbance observer are: Unknown model + Disturbance error.
    * So the disturbance should be superimposed into the model.
%}
function Out = ESO_main(GData, Z_Pre, y, u, omg)

b = GData.UAV_Model.para_b;
Ts = GData.Ts;

z1_1 = Z_Pre(1);
z2_1 = Z_Pre(2);
z3_1 = Z_Pre(3);

e = z1_1 - y;

a1 = 0.279;
a2 = -10.6597;

eta1 = 3*omg;
eta2 = 3*omg*omg;
eta3 = omg*omg*omg;

z1 = z1_1 + Ts*(z2_1 + 0.1721*z1_1 - eta1*e);
z2 = z2_1 + Ts*(z3_1 - eta2*e + a1*z2_1 + a2*z1_1 + 0.0622*u);
z3 = z3_1 - Ts*eta3*e;


Out = [z1, z2, z3];

end