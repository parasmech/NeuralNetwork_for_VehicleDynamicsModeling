import pandas as pd
from src.run_neuralnetwork import VehicleDynamics_model, VehicleDynamics_model_every
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    file_path_data = Path('data_to_run_for slips.csv')
    inputdata = pd.read_csv(file_path_data, skiprows=1,header=None,dtype=float)
    inputdata = inputdata.values

    # file_path_slips = Path('generate_data\PINN_results\slip_Tirecoeffs_FF4L_60_oldlims_sliploss_LongSlipWt1k\layer11_activations_0.csv')
    # slip_coeffs = pd.read_csv(file_path_slips, skiprows=1,header=None,dtype=float)
    # slip_coeffs = slip_coeffs.values

    # all_files_path_slips = list(file_path_slips.glob("*.csv"))

    # dataframes = []
    # for file in all_files_path_slips:
    #     df = pd.read_csv(file, skiprows=1,header=None,dtype=float)
    #     dataframes.append(df)
    # combined_df = pd.concat(dataframes, axis=0)
    # combined_df.reset_index(drop=True, inplace=True)
    # slip_coeffs = combined_df.values

    next_state, Fxr, Fyr = VehicleDynamics_model(inputdata[0:1450,0:12])
    #next_state_every, Fxr_every, Fyr_every = VehicleDynamics_model_every(inputdata[4: len(slip_coeffs)+4], slip_coeffs)
    
    time = np.arange(0, (1450+1)*1.e-3, 1.e-3)
    
    plt.figure(figsize=(8, 6))
    plt.plot(time, next_state[0,:], 'b-')
    # plt.plot(time, next_state_every[:,0], 'c-')
    plt.xlabel('time [s]', fontsize=10)
    plt.ylabel('vx [m/s]', fontsize=10)
    plt.show()

    # plt.figure(figsize=(8, 6))
    # plt.plot(time[0:1494], Fxr, 'b-')
    # plt.xlabel('time [s]', fontsize=10)
    # plt.ylabel('Fxr [N]', fontsize=10)
    # plt.show()

    # plt.figure(figsize=(8, 6))
    # plt.plot(time[0:1494], Fyr, 'b-')
    # plt.xlabel('time [s]', fontsize=10)
    # plt.ylabel('Fyr [N]', fontsize=10)
    # plt.show()
    # plt.figure(figsize=(8, 6))
    # plt.plot(time, long_sr1, 'b-')
    # plt.plot(time, long_sr, 'k-')
    # plt.xlabel('time [s]', fontsize=10)
    # plt.ylabel('vx [m/s]', fontsize=10)
    # plt.show()

    # plt.figure(figsize=(8, 6))
    # plt.plot(time, lat_sr1, 'b-')
    # plt.plot(time, lat_sr, 'k-')
    # plt.xlabel('time [s]', fontsize=10)
    # plt.ylabel('vx [m/s]', fontsize=10)
    # plt.show()