function GData = UAV_Model(GData)

Ts = GData.Ts;
srt = GData.Ori_Data.StartTime;
spt = GData.Ori_Data.StopTime;
cRoll = GData.Ori_Data.Roll(srt/Ts:spt/Ts);
cRollRate = GData.Ori_Data.RollRate(srt/Ts:spt/Ts);
cRef_RollRate = GData.Ori_Data.Ref_RollRate(srt/Ts:spt/Ts);

K = 0.0058;
T = 0.0335;
Zeta = 0.8307;

para_b  = K/T^2;

A = [-2*Zeta/T -1/T^2  0;
             1      0  0;
             0      1  0];
B = [K/T^2, 0, 0]';
C = [0 1 0
     0 0 1];
D = [0, 0]';

[Ad, Bd, Cd, Dd] = c2dm(A, B, C, D, Ts);

[ny, nx] = size(C);
K = zeros(nx,ny); 
X = zeros(nx,1);
Ident_model = idss(Ad, Bd, Cd, Dd, K, X, GData.Ts);

LRoll = dtrend(cRollRate*200);
Y = idsim(Ident_model, LRoll);

t = (1:1:length(LRoll))*Ts;

figure('Name','UAV Model Validation ','NumberTitle','off')
subplot(2,1,1)
plot(t,Y(:,1),'r',t,cRollRate,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude Rate(Rad/s)');
legend('Fitted Model','Ori Date');
legend('boxoff');
subplot(2,1,2)
plot(t,Y(:,2),'r',t,cRoll,'b');grid;
xlabel('Time(s)');ylabel('Attitude(Rad)');
legend('Fitted Model','Ori Date');
legend('boxoff');

GData.UAV_Model.para_b  = para_b;
GData.UAV_Model.A  = A;
GData.UAV_Model.B  = B;
GData.UAV_Model.C  = C;
GData.UAV_Model.D  = D;

GData.UAV_Model.cRoll = cRoll;
GData.UAV_Model.cRollRate = cRollRate;
GData.UAV_Model.cRef_RollRate = cRef_RollRate;
GData.UAV_Model.LRoll = LRoll;

end

