clear 
close all
clc

filename = "./../inputs/trainingdata/data_to_run";
% data_to_run.csv
% data_to_train_0.csv
% data_to_train_1.csv
% data_to_train_2.csv
% data_to_train_3.csv
% data_to_train_4.csv
% data_to_train_5.csv
% data_to_train_6.csv
% data_to_train_7.csv
% data_to_train_8.csv
% data_to_train_9.csv
% data_to_train_10.csv
% data_to_train_11.csv
% data_to_train_12.csv
% data_to_train_13.csv

data = readtable(filename + ".csv");
run('VehParams.m');


sample_Ti = 1.e-3;                                              % Time Step
tiSer = 0 : sample_Ti : (length(data.vx_mps)-1)*0.008;      % Time Series
tend = tiSer(end);                                              % Stop Time
tiearly = 0 : 0.008 : (length(data.vx_mps)-1)*0.008;      % Time Series

vx_mps_TiSer = timeseries(interp1(tiearly, data.vx_mps, tiSer), tiSer);                  % Speed vx Data
vy_mps_TiSer = timeseries(interp1(tiearly, data.vy_mps, tiSer), tiSer);                  % Speed vy Data
dpsi_radps_TiSer = timeseries(interp1(tiearly, data.dpsi_radps, tiSer), tiSer);          % Speed yaw rate Data
ax_mps_TiSer = timeseries(interp1(tiearly, data.ax_mps2, tiSer), tiSer);                 % Acceleration ax Data
ay_mps_TiSer = timeseries(interp1(tiearly, data.ay_mps2, tiSer), tiSer);                 % Acceleration ay Data
DeltaWheel_rad_TiSer = timeseries(interp1(tiearly, data.deltawheel_rad, tiSer)', tiSer');  % Steering Wheel angle Data
TwheelRL_Nm_TiSer = timeseries(interp1(tiearly, data.TwheelRL_Nm, tiSer)', tiSer');        % Torque Rear Left Data
TwheelRR_Nm_TiSer = timeseries(interp1(tiearly, data.TwheelRR_Nm, tiSer)', tiSer');        % Torque Rear Right Data
TwheelFL_Nm_TiSer = timeseries(interp1(tiearly, data.TwheelRL_Nm*0, tiSer)', tiSer');      % Torque Front Left Data 
TwheelFR_Nm_TiSer = timeseries(interp1(tiearly, data.TwheelRR_Nm*0, tiSer)', tiSer');      % Torque Front Right Data
pBrakeF_bar_TiSer = timeseries(interp1(tiearly, data.pBrakeF_bar/2, tiSer)', tiSer');      % Brake pressure Front
pBrakeR_bar_TiSer = timeseries(interp1(tiearly, data.pBrakeR_bar/2, tiSer)', tiSer');      % Brake pressure Rear
vx_INIT = data.vx_mps(1);
vy_INIT = data.vy_mps(1);
dpsi_rad_INIT = data.dpsi_radps(1);
omega_rad_INIT = data.vx_mps(1)/tireFL__MFSIMPLE__r_tire_m;
[lambda_perc_INIT, alpha_rad_INIT] = calcWheelSlips(ones(4,1)*omega_rad_INIT, [vx_INIT; vy_INIT; dpsi_rad_INIT],...
  data.deltawheel_rad(1), VEH__VehicleData__w_TrackF_m, VEH__VehicleData__w_TrackR_m, VEH__VehicleData__l_WheelbaseF_m, VEH__VehicleData__l_WheelbaseTotal_m - VEH__VehicleData__l_WheelbaseF_m, tireFL__MFSIMPLE__r_tire_m, tireFL__MFSIMPLE__r_tire_m);
%%
out = sim('VehDyn_nstm_model.slx');
%%
vx_mps = out.SimRealState.vx_mps.Data;
vy_mps = out.SimRealState.vy_mps.Data; 
dpsi_radps = out.SimRealState.dPsi_radps.Data; 
ax_mps2 = out.SimRealState.ax_mps2.Data; 
ay_mps2 = out.SimRealState.ay_mps2.Data; 
deltawheel_rad = DeltaWheel_rad_TiSer.Data; 
TwheelRL_Nm = TwheelRL_Nm_TiSer.Data; 
TwheelRR_Nm = TwheelRR_Nm_TiSer.Data;
pBrakeF_bar = pBrakeF_bar_TiSer.Data; 
pBrakeR_bar = pBrakeR_bar_TiSer.Data;
ax_mps2(1) = data.ax_mps2(1); % correct at first time step due to numerical intergation ODE4
ay_mps2(1) = data.ay_mps2(1);
VehDynData_Table = table(vx_mps, vy_mps, dpsi_radps, ax_mps2, ay_mps2, deltawheel_rad, TwheelRL_Nm, TwheelRR_Nm, pBrakeF_bar, pBrakeR_bar);
writetable(VehDynData_Table, sprintf(filename + '.csv' ));