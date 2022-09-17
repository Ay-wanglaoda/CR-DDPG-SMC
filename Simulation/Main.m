%{
*   UAV Attitude Controller Design Process
*   Auther: Wang Qi
*   Date: 2022.8.12
*   E-mail: wangs_2602@163.com

*   GlobalData Structure
*   Ori_Data:   Data collection under the stabilize mode of the UAV.
*   UAV_Model:  The UAV model fitted from the original data by using ident tool.
*   Kalman:     The design and model parameters of Kalman filter.
*   Ref_Model:  The data of the reference model.
%}

clear;clc;
close all;

%   Adding path and load the global data.
addpath('Experiment_Data','CoreFiles');
load('.\Experiment_Data\GlobalData.mat');

%   Setting figure plot.
PlotSet();

%   Running Frequency
GlobalData.Ts = 0.02;

%   Data view
GlobalData = Ori_ExpDataView(GlobalData);

%   Fit the model of the UAV
GlobalData.Ori_Data.StartTime = 30;
GlobalData.Ori_Data.StopTime  = 47;
GlobalData = UAV_Model(GlobalData);

%   Kalman filter Design
GlobalData.Kalman.Q = 1000000;
GlobalData.Kalman.R  = diag([100 1]);
GlobalData = Kalman_Filter(GlobalData);

%   Reference Model Design
GlobalData.Ref_Model.a1 = 320;
GlobalData.Ref_Model.a2 = 700;
GlobalData = Reference_Model(GlobalData);


%   Reference Model Design
Type = 1;   % 1:ST-Based; 2:Fuzzy-Based; 3:ESO-Based; 4:SF-Based; 5:RL-Based; 6:Traditional;
GlobalData = SMC_Controller(GlobalData, Type);

save('.\Experiment_Data\GlobalData.mat','GlobalData');