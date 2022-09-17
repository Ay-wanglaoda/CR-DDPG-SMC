function GData = Ori_ExpDataView(GData)

load('Roll.mat');
load('RollRate.mat');
load('Ref_RollRate.mat');

t = (1:1:length(Roll))*GData.Ts;

figure('Name','Original Date of UAV','NumberTitle','off')
subplot(2,1,1)
plot(t,Ref_RollRate,'r',t,RollRate,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude Rate(Rad/s)');
legend('Ref','FB');
legend('boxoff');
subplot(2,1,2)
plot(t,Roll,'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Attitude Rate(Rad/s)');

GData.Ori_Data.Roll = Roll;
GData.Ori_Data.RollRate = RollRate;
GData.Ori_Data.Ref_RollRate = Ref_RollRate;

end

