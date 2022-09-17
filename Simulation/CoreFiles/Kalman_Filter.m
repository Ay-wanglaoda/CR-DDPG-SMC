function GData = Kalman_Filter(GData)

Ts = GData.Ts;

LRoll = GData.UAV_Model.LRoll;
Roll = GData.UAV_Model.cRoll;
RollRate = GData.UAV_Model.cRollRate;

A = GData.UAV_Model.A; 
B = GData.UAV_Model.B; 
C = GData.UAV_Model.C; 
D = GData.UAV_Model.D;
[Ad, Bd, Cd, Dd] = c2dm(A, B, C, D, Ts);

[~, nx] = size(A);
E = eye(nx);   
 
sys = dss(Ad, [Bd Bd], Cd, [Dd,Dd], E, Ts);     
 
[Kest, ~, ~] = kalman(sys, GData.Kalman.Q, GData.Kalman.R);
[Akd, Bkd, Ckd, Dkd] = dssdata(Kest);

[ny, nx] = size(Ckd);
K = zeros(nx,ny); 
X = zeros(nx,1);
Kalman_model = idss(Akd, Bkd, Ckd, Dkd, K, X, Ts);

U = [LRoll, RollRate, Roll]; 
t = (1:1:length(LRoll))*Ts;
Y  = idsim(Kalman_model, U);

figure('Name','Kalman Validation ','NumberTitle','off')
subplot(2,1,1)
plot(t,Y(:,4),'r',t,RollRate,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude Rate(Rad/s)');
legend('Kal','Ori');
legend('boxoff');
subplot(2,1,2)
plot(t,Y(:,5),'r',t,Roll,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude(Rad)');
legend('Kal','Ori');
legend('boxoff');

GData.Kalman.Ak = Akd;
GData.Kalman.Bk = Bkd;
[my, ~] = size(A);
GData.Kalman.Ck = Ckd((ny - my + 1):ny , :);
GData.Kalman.Dk = Dkd((ny - my + 1):ny , :);

end