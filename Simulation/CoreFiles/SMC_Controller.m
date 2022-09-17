function GData = SMC_Controller(GData, Type)

Ts = GData.Ts;
A = GData.UAV_Model.A; 
B = GData.UAV_Model.B; 
C = GData.UAV_Model.C; 
D = GData.UAV_Model.D;
Am = GData.Ref_Model.Am;
Bm = GData.Ref_Model.Bm;
Cm = GData.Ref_Model.Cm;
Dm = GData.Ref_Model.Dm;

[Amd, Bmd, Cmd, Dmd] = c2dm(Am,Bm,Cm,Dm,Ts);
[Ad, Bd, Cd, Dd] = c2dm(A, B, C(2,:), D(2,:), Ts);

Q = diag([1 100 1000]);
R = 1;
S = lqr(Am,B,Q,R);
S = [0.2960, 12.0149, 65.7611];

K1 = inv(-Cm*inv(Am)*B);
K2 = pinv(B) * (Am - A);
Ke = -inv(S*B)*(S*Am) - inv(S*B)*S*22;

switch Type
    case 1
        % Jayakrishnan H J. Position and attitude control of a quadrotor UAV using super twisting sliding mode[J]. IFAC-PapersOnLine, 2016, 49(1): 284-289.
        k_ST1 = 5.2;
        k_ST2 = 4.6;
        
        inte_sf = 0;
        
    case 2
        % Lei W, Li C. An Adaptive Quasi-Sliding Mode Control Without Chattering[C]//2018 37th Chinese Control Conference (CCC). IEEE, 2018: 1018-1023
        alpha = 0.2;  % > 0
        beta = 0.1;  % > 0
        adapt_gain = 0;
        logistic_f = 0;
        
    case 3
        % Improved continuous fast terminal sliding mode control with extended state observer for speed regulation of PMSM drive system. IEEE Transactions on Vehicular Technology, 2019
        addpath('CoreFiles\ESO_Related');
        mu1 = -48; mu2 = -100;
        k1 = 28; k2 = 12;
        tao1 = 1.2; tao2 = 3; tao3 = 0.8;
        
        Z_Pre = zeros(3,1);
        integ_s = 0;
        
    case 4
        % Wang W, Ma H, Xia M, et al. Attitude and altitude controller design for quad-rotor type MAVs[J]. Mathematical Problems in Engineering, 2013, 2013
        Ksw = -13.1;   
        
    case 5
        RL_w1 = [-0.18867311,  0.23862985,  0.35373363, -1.3140799;
                 -0.27189627, -0.24557325, -0.22583362,  0.7492207;
                  0.0342381 ,  0.02285562, -0.4790898,   0.7627703;
                 -0.04600874, -2.1629233,   0.09920471,  1.8975701;
                 -0.08541179,  0.25031388,  0.11860853, -0.22551149;
                  0.306677,    0.5115195,   0.27771783, -0.47571173;
                 -0.09601015,  0.2452123,   0.62851256, -0.20100474;
                  0.06050915, -1.932745,    0.16893318,  1.6309489];
        RL_w2 = [-1.28122613e-01, -1.16419561e-01,  5.29267967e-01,  5.31909525e-01,  -2.25663617e-01, -4.54075411e-02,  2.17842340e-01, -7.72076696e-02;
                  2.19069749e-01,  3.59739453e-01, -3.26179899e-02, -9.16155159e-01,   3.12300138e-02, -8.71139884e-01,  9.30157661e-01,  3.90844762e-01;
                 -7.78552234e-01,  3.98956120e-01, -1.54555008e-01,  1.45986354e+00,   3.23942095e-01,  1.53794006e-01, -3.76101166e-01,  1.57034218e+00;
                 -2.59486675e-01, -1.02539845e-01, -3.46352315e+00,  4.51373726e-01,  -1.79284006e-01, -1.62547016e+00,  7.12312087e-02,  7.50592470e-01;
                 -1.07227697e-03, -7.69205689e-02, -1.42617032e-01,  5.35305440e-02,  -1.35661468e-01,  8.72405991e-02, -2.17442051e-01, -2.96871603e-01;
                  5.36025055e-02, -6.37495741e-02,  4.05702710e-01,  5.78896642e-01,   5.18364075e-04, -1.45312992e-03, -4.67699207e-02, -1.78571299e-01;
                 -1.90361932e-01, -3.44265550e-02, -5.26629090e-02,  1.67425111e-01,  -2.00125247e-01,  1.15778781e-01, -6.86993226e-02, -1.55622140e-01;
                 -9.04430002e-02, -5.09341918e-02,  8.67535233e-01,  7.43093342e-02,   1.31306397e-02,  7.83211589e-02,  8.13976228e-02, -1.77992538e-01];
        RL_w3 = [0.15683709, -0.9271344,   6.3579426,  -2.0787654,  -0.05395385,  0.222573,   0.06584771,  0.13229522];
        RL_b1 = [0.24917296, -0.32038918, -0.23084621, -0.12315981, -0.18189543, -0.03214272,  0.11225598,  0.04353074]';
        RL_b2 = [0.25142217, -0.2700297,  -0.05467508,  0.01475302, -0.07981405,  0.39992827, -0.2933577,   0.31287763]';
        RL_b3 = -0.16573577;
        
    case 6
        Ksw = -13.1;  
end

% Initialization
X =  zeros(3,1);
Xm = zeros(3,1);
e =  zeros(3,1);
U = 0;
delay_Hz = 15;
input_U = zeros(delay_Hz,1);
slide_surface = 0;

for i = 1:1:400
    t(i) = i*Ts;

    ref = 0;
    if(i > 50)
        ref = 0.3;
    end
    
    
    switch Type
        case 1
            slide_surface = S*e;
            inte_sf = inte_sf + sign(slide_surface) * Ts;
            
            ueq = Ke * e + K2 * X + K1 * ref;
            usw = -k_ST1*sqrt(abs(slide_surface)) * sign(slide_surface) - k_ST2*inte_sf;
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = ueq + input_U(delay_Hz);
            
        case 2
            slide_surface = S*e;
            logistic_f = (1 - power(2.7183,-alpha*slide_surface))/(1 + power(2.7183,-alpha*slide_surface));
            adapt_gain = adapt_gain + (1/beta*slide_surface*logistic_f);

            usw = 20*logistic_f*adapt_gain;
            ueq = Ke * e + K2 * X + K1 * ref;
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = ueq + input_U(delay_Hz);
            
        case 3
            slide_surface = e(2) + mu1*power(abs(e(2)), tao1)*sign(e(2)) + mu2*power(abs(e(3)), tao2)*sign(e(3));
            
            Z = ESO_main(GData, Z_Pre, X(2), U, 12);
            Z_Pre = Z;
            
            integ_s = integ_s + (k1*slide_surface + k2*sign(slide_surface)*power(abs(slide_surface), tao3)) * Ts;
            
            ueq = integ_s;
            usw = Xm(2) + mu1*power(abs(e(2)), tao1)*sign(e(2)) + mu2*power(abs(e(3)), tao2)*sign(e(3));
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = GData.UAV_Model.para_b*(ueq + input_U(delay_Hz) - Z(3));
            
        case 4
            slide_surface = S*e;
            f_SF = slide_surface/(abs(slide_surface) + 0.25);
            
            ueq = Ke * e + K2 * X + K1 * ref;
            usw = Ksw*f_SF;   
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = ueq + input_U(delay_Hz);
            
        case 5
            input_X = [Xm(2), Xm(3), X(2), X(3)]';
            % First layer
            temp = RL_w1*input_X;
            temp1 = RL_Relu(temp + RL_b1);
            
            % Second layer
            temp = RL_w2*temp1;
            temp2 = RL_Relu(temp + RL_b2);
            
            % Output layer
            temp = RL_w3*temp2;
            temp3 = RL_tanh(temp + RL_b3);
            
            ueq = Ke * e + K2 * X + K1 * ref;
            usw = 12*temp3; 
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = ueq + input_U(delay_Hz);
            
        case 6
            slide_surface = S*e;
            f_SF = sign(slide_surface);
            
            ueq = Ke * e + K2 * X + K1 * ref;
            usw = Ksw*f_SF;   
            
            for ii = delay_Hz:-1:2
                input_U(ii) = input_U(ii - 1);
            end
            
            input_U(1) = usw;
            
            U = ueq + input_U(delay_Hz);
    end
    
    
    dX = Ad*X + Bd*U;
    dXm = Amd*Xm + Bmd*ref;
    
    X = dX;
    Xm = dXm;
    e = X - Xm;
    
    X_Ap(:,i) = X;
    Xm_Ap(:,i) = Xm;
    U_Ap(i) = U;
end

figure('Name','Controller Output','NumberTitle','off')
subplot(311)
plot(t,U_Ap,'r');title('U');grid;
xlabel('Time(s)');ylabel('Control Input');
subplot(312)
plot(t,Xm_Ap(2,:),'r',t,X_Ap(2,:),'b',t,Xm_Ap(3,:),'r--',t,X_Ap(3,:),'b--','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Tar Tracking');
legend('Ref Rate','Rate','Ref Att','Att');
legend('boxoff')
subplot(313)
plot(t,Xm_Ap(2,:) - X_Ap(2,:),'r',t,Xm_Ap(3,:) - X_Ap(3,:),'b','LineWidth',1.2);grid;
xlabel('Time(s)');ylabel('Tracking Err');
legend('Rate','Att');
legend('boxoff')

end

