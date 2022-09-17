function GData = Reference_Model(GData)

Ts = GData.Ts;
A = GData.UAV_Model.A; 
B = GData.UAV_Model.B; 
C = GData.UAV_Model.C; 
D = GData.UAV_Model.D;

a1 = -GData.Ref_Model.a1;
a2 = -GData.Ref_Model.a2;

Am = [A(1,1), a1, a2;
           1,  0,  0; 
           0,  1,  0];
Cm = [0,0,1];
Dm = 0;
Bm = B*inv(-Cm*inv(Am)*B);
[Amd, Bmd, Cmd, Dmd] = c2dm(Am, Bm, Cm, Dm, Ts);

F = inv(B'*B) * B' * (A - Am);
Br = inv(B'*B) * B' * Bm;

Xm = zeros(3,1);

for i = 1:1:300
    t(i) = i*Ts;
    if i >= 50
        ori_ref(i) = 1;
    else
        ori_ref(i) = 0;
    end
    Xm = Amd*Xm + Bmd*ori_ref(i);
    Y(i) = Cmd*Xm+Dmd;
end

figure('Name','Reference Model Validation','NumberTitle','off')
plot(t,ori_ref,'r',t,Y,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude(Rad)');
legend('Ori Ref','RefModel');
legend('boxoff');

GData.Ref_Model.Am = Am;
GData.Ref_Model.Bm = Bm;
GData.Ref_Model.Cm = Cm;
GData.Ref_Model.Dm = Dm;

end