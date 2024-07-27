filename_tirecoeff = "./PINN_results/TireCoeffEstimationResults/";
filename_vehStates = "./PINN_results/TireCoeffEstimationResults/";

% tirecoeff = readtable(filename_vehStates + 'data_to_run_for slips' + ".csv");

tirecoeff_0 = readtable(filename_tirecoeff + 'layer11_activations_0' + ".csv");
tirecoeff_1 = readtable(filename_tirecoeff + 'layer11_activations_1' + ".csv");
tirecoeff_2 = readtable(filename_tirecoeff + 'layer11_activations_2' + ".csv");
tirecoeff_3 = readtable(filename_tirecoeff + 'layer11_activations_3' + ".csv");
tirecoeff_4 = readtable(filename_tirecoeff + 'layer11_activations_4' + ".csv");
tirecoeff_5 = readtable(filename_tirecoeff + 'layer11_activations_5' + ".csv");
% tirecoeff_6 = readtable(filename_tirecoeff + 'layer11_activations_6' + ".csv");
% tirecoeff_7 = readtable(filename_tirecoeff + 'layer11_activations_7' + ".csv");
% tirecoeff_8 = readtable(filename_tirecoeff + 'layer21_activations_8' + ".csv");
% tirecoeff_9 = readtable(filename_tirecoeff + 'layer21_activations_9' + ".csv");
% tirecoeff_10 = readtable(filename_tirecoeff + 'layer21_activations_10' + ".csv");
% tirecoeff_11 = readtable(filename_tirecoeff + 'layer21_activations_11' + ".csv");
% tirecoeff_12 = readtable(filename_tirecoeff + 'layer21_activations_12' + ".csv");
% tirecoeff_13 = readtable(filename_tirecoeff + 'layer21_activations_13' + ".csv");
% tirecoeff_14 = readtable(filename_tirecoeff + 'layer21_activations_14' + ".csv");
% tirecoeff_15 = readtable(filename_tirecoeff + 'layer21_activations_15' + ".csv");
% tirecoeff_16 = readtable(filename_tirecoeff + 'layer21_activations_16' + ".csv");
% tirecoeff_17 = readtable(filename_tirecoeff + 'layer21_activations_17' + ".csv");
% tirecoeff_18 = readtable(filename_tirecoeff + 'layer21_activations_18' + ".csv");
% tirecoeff_19 = readtable(filename_tirecoeff + 'layer21_activations_19' + ".csv");


vehStates_0 = readtable(filename_vehStates + 'output_state_activations_0' + ".csv");
vehStates_1 = readtable(filename_vehStates + 'output_state_activations_1' + ".csv");
vehStates_2 = readtable(filename_vehStates + 'output_state_activations_2' + ".csv");
vehStates_3 = readtable(filename_vehStates + 'output_state_activations_3' + ".csv");
vehStates_4 = readtable(filename_vehStates + 'output_state_activations_4' + ".csv");
vehStates_5 = readtable(filename_vehStates + 'output_state_activations_5' + ".csv");
% vehStates_6 = readtable(filename_vehStates + 'output_state_activations_6' + ".csv");
% vehStates_7 = readtable(filename_vehStates + 'output_state_activations_7' + ".csv");
% vehStates_8 = readtable(filename_vehStates + 'output_state_activations_8' + ".csv");
% vehStates_9 = readtable(filename_vehStates + 'output_state_activations_9' + ".csv");
% vehStates_10 = readtable(filename_vehStates + 'output_state_activations_10' + ".csv");
% vehStates_11 = readtable(filename_vehStates + 'output_state_activations_11' + ".csv");
% vehStates_12 = readtable(filename_vehStates + 'output_state_activations_12' + ".csv");
% vehStates_13 = readtable(filename_vehStates + 'output_state_activations_13' + ".csv");
% vehStates_14 = readtable(filename_vehStates + 'output_state_activations_14' + ".csv");
% vehStates_15 = readtable(filename_vehStates + 'output_state_activations_15' + ".csv");
% vehStates_16 = readtable(filename_vehStates + 'output_state_activations_16' + ".csv");
% vehStates_17 = readtable(filename_vehStates + 'output_state_activations_17' + ".csv");
% vehStates_18 = readtable(filename_vehStates + 'output_state_activations_18' + ".csv");
% vehStates_19 = readtable(filename_vehStates + 'output_state_activations_19' + ".csv");

tirecoeff_PINN = [tirecoeff_0;tirecoeff_1;tirecoeff_2;tirecoeff_3;tirecoeff_4;tirecoeff_5;];%tirecoeff_6;tirecoeff_7];%;tirecoeff_8;tirecoeff_9; tirecoeff_10; tirecoeff_11; tirecoeff_12; tirecoeff_13; tirecoeff_14; tirecoeff_15; tirecoeff_16; tirecoeff_17;];% tirecoeff_18; tirecoeff_19];
vehStates_PINN = [vehStates_0;vehStates_1;vehStates_2;vehStates_3;vehStates_4;vehStates_5;];%vehStates_6;vehStates_7];%;vehStates_8; vehStates_9; vehStates_10; vehStates_11; vehStates_12; vehStates_13; vehStates_14; vehStates_15; vehStates_16; vehStates_17;];% vehStates_18; vehStates_19];

tiPINN = 0 : sample_Ti : (length(tirecoeff_PINN.Var1)-1)*0.001;      % Time Series
tiPINN_vehstates = 0 : sample_Ti : (length(vehStates_PINN.Var1)-1)*0.001;      % Time Series

B_long_PINN = timeseries(tirecoeff_PINN.Var1 , tiPINN); 
C_long_PINN = timeseries(tirecoeff_PINN.Var2 , tiPINN); 
D_long_PINN = timeseries(tirecoeff_PINN.Var3 , tiPINN); 
E_long_PINN = timeseries(tirecoeff_PINN.Var4 , tiPINN); 
F_long_PINN = timeseries(tirecoeff_PINN.Var5 , tiPINN); 
eps_long_PINN = timeseries(tirecoeff_PINN.Var6 , tiPINN); 
B_lat_PINN = timeseries(tirecoeff_PINN.Var7 , tiPINN); 
C_lat_PINN = timeseries(tirecoeff_PINN.Var8 , tiPINN); 
D_lat_PINN = timeseries(tirecoeff_PINN.Var9 , tiPINN); 
E_lat_PINN = timeseries(tirecoeff_PINN.Var10 , tiPINN); 
F_lat_PINN = timeseries(tirecoeff_PINN.Var11 , tiPINN); 
eps_lat_PINN = timeseries(tirecoeff_PINN.Var12, tiPINN); 

vx_PINN = timeseries(vehStates_PINN.Var1 , tiPINN_vehstates); 
vy_PINN = timeseries(vehStates_PINN.Var2 , tiPINN_vehstates); 
yawrate_PINN = timeseries(vehStates_PINN.Var3 , tiPINN_vehstates); 
