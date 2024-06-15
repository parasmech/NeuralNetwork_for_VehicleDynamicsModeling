filename_slip_tirecoeff = "./PINN_results/slip_Tirecoeffs_1GRU_2D/";
filename_vehStates = "./PINN_results/veh_states_1GRU_2D/";

slip_tirecoeff_0 = readtable(filename_slip_tirecoeff + 'layer5_activations_0' + ".csv");
slip_tirecoeff_1 = readtable(filename_slip_tirecoeff + 'layer5_activations_1' + ".csv");
slip_tirecoeff_2 = readtable(filename_slip_tirecoeff + 'layer5_activations_2' + ".csv");
slip_tirecoeff_3 = readtable(filename_slip_tirecoeff + 'layer5_activations_3' + ".csv");
slip_tirecoeff_4 = readtable(filename_slip_tirecoeff + 'layer5_activations_4' + ".csv");
slip_tirecoeff_5 = readtable(filename_slip_tirecoeff + 'layer5_activations_5' + ".csv");
slip_tirecoeff_6 = readtable(filename_slip_tirecoeff + 'layer5_activations_6' + ".csv");
slip_tirecoeff_7 = readtable(filename_slip_tirecoeff + 'layer5_activations_7' + ".csv");
slip_tirecoeff_8 = readtable(filename_slip_tirecoeff + 'layer5_activations_8' + ".csv");
slip_tirecoeff_9 = readtable(filename_slip_tirecoeff + 'layer5_activations_9' + ".csv");

vehStates_0 = readtable(filename_vehStates + 'output_state_activations_0' + ".csv");
vehStates_1 = readtable(filename_vehStates + 'output_state_activations_1' + ".csv");
vehStates_2 = readtable(filename_vehStates + 'output_state_activations_2' + ".csv");
vehStates_3 = readtable(filename_vehStates + 'output_state_activations_3' + ".csv");
vehStates_4 = readtable(filename_vehStates + 'output_state_activations_4' + ".csv");
vehStates_5 = readtable(filename_vehStates + 'output_state_activations_5' + ".csv");
vehStates_6 = readtable(filename_vehStates + 'output_state_activations_6' + ".csv");
vehStates_7 = readtable(filename_vehStates + 'output_state_activations_7' + ".csv");
vehStates_8 = readtable(filename_vehStates + 'output_state_activations_8' + ".csv");
vehStates_9 = readtable(filename_vehStates + 'output_state_activations_9' + ".csv");

slip_tirecoeff_PINN = [slip_tirecoeff_0;slip_tirecoeff_1;slip_tirecoeff_2;slip_tirecoeff_3;slip_tirecoeff_4;slip_tirecoeff_5;slip_tirecoeff_6;slip_tirecoeff_7;slip_tirecoeff_8;slip_tirecoeff_9];
vehStates_PINN = [vehStates_0;vehStates_1;vehStates_2;vehStates_3;vehStates_4;vehStates_5;vehStates_6;vehStates_7;vehStates_8;vehStates_9];

tiPINN = 0 : sample_Ti : (length(slip_tirecoeff_PINN.Var1)-1)*0.001;      % Time Series
tiPINN_vehstates = 0 : sample_Ti : (length(vehStates_PINN.Var1)-1)*0.001;      % Time Series

long_slip_front_PINN = timeseries(slip_tirecoeff_PINN.Var1 , tiPINN); 
long_slip_rear_PINN = timeseries(slip_tirecoeff_PINN.Var2 , tiPINN); 
lat_slip_front_PINN = timeseries(slip_tirecoeff_PINN.Var3 , tiPINN); 
lat_slip_rear_PINN = timeseries(slip_tirecoeff_PINN.Var4 , tiPINN); 
B_long_PINN = timeseries(slip_tirecoeff_PINN.Var5 , tiPINN); 
C_long_PINN = timeseries(slip_tirecoeff_PINN.Var6 , tiPINN); 
D_long_PINN = timeseries(slip_tirecoeff_PINN.Var7 , tiPINN); 
E_long_PINN = timeseries(slip_tirecoeff_PINN.Var8 , tiPINN); 
F_long_PINN = timeseries(slip_tirecoeff_PINN.Var9 , tiPINN); 
eps_long_PINN = timeseries(slip_tirecoeff_PINN.Var10 , tiPINN); 
B_lat_PINN = timeseries(slip_tirecoeff_PINN.Var11 , tiPINN); 
C_lat_PINN = timeseries(slip_tirecoeff_PINN.Var12 , tiPINN); 
D_lat_PINN = timeseries(slip_tirecoeff_PINN.Var13 , tiPINN); 
E_lat_PINN = timeseries(slip_tirecoeff_PINN.Var14 , tiPINN); 
F_lat_PINN = timeseries(slip_tirecoeff_PINN.Var15 , tiPINN); 
eps_lat_PINN = timeseries(slip_tirecoeff_PINN.Var16 , tiPINN); 

vx_PINN = timeseries(vehStates_PINN.Var1 , tiPINN_vehstates); 
vy_PINN = timeseries(vehStates_PINN.Var2 , tiPINN_vehstates); 
yawrate_PINN = timeseries(vehStates_PINN.Var3 , tiPINN_vehstates); 
