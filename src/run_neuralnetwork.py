import numpy as np
import sys
from tqdm import tqdm
import os.path
from tensorflow import keras

# custom modules
import src

"""
Created by: Rainer Trauth
Created on: 01.04.2020
"""


def VehicleDynamics_model(inputs):
    # Unpack inputs
    vx, vy, yaw_rate,a1,a2, steering, Trl, Trr, pbarF, pbarR, wF, wR,  = np.hsplit(inputs, 12)
    
    # Uncomment below when taking values from lambda layer
    # (long_sf, long_sr, lat_sf, lat_sr, long_pac1, long_pac2, long_pac3, long_pac4, long_pac5, long_pac6,
    #  lat_pac1, lat_pac2, lat_pac3, lat_pac4, lat_pac5, lat_pac6) = np.hsplit(slip_coeffs, 16)
    
    # see from the data only initial state
    vx = vx[4][0]
    vy = vy[4][0]
    yaw_rate = yaw_rate[4][0]

    #np.print("Slip coefficients: ", slip_coefficients)

    # Unpack slip coefficients
    # (long_sf, long_sr, lat_sf, lat_sr, long_pac1, long_pac2, long_pac3, long_pac4, long_pac5, long_pac6,
    #  lat_pac1, lat_pac2, lat_pac3, lat_pac4, lat_pac5, lat_pac6) = np.split(slip_coefficients, num_or_size_splits=16, axis=-1)

    lat_pac1 = 10
    lat_pac2 = 1.603
    lat_pac3 = 1.654
    lat_pac4 = 1
    lat_pac5 = 3114
    lat_pac6 = -0.1783
    long_pac1 = 18
    long_pac2 = 2
    long_pac3 = 1.7168
    long_pac4 = 1
    long_pac5 = 3114
    long_pac6 = -0.289

    # Constants
    m = 750
    g = 9.81
    lf = 1.724
    lr = 1.247
    rho = 1.22
    Cwr = 1.034
    Cwf = 0.522
    A = 1
    Izz = 1000
    cf = 20
    cd = 0.725
    Ts = 1.e-3


    vx_new_values = []
    vy_new_values = []
    yaw_rate_new_values = []
    Fxr_values = []
    Fyr_values = []

    vx_new_values.append(vx)
    vy_new_values.append(vy)
    yaw_rate_new_values.append(vy)
    
    # Calculate forces and dynamics
    for i in range(len(inputs)):

        vx_fr = vx
        vx_fl = vx
        vx_rr = vx
        vx_rl = vx
        vy_fr = vy + (yaw_rate*lf)
        vy_fl = vy + (yaw_rate*lf)
        vy_rr = vy - (yaw_rate*lr)
        vy_rl = vy - (yaw_rate*lr)


        R = np.array([[np.cos(-steering[i]), -np.sin(-steering[i])],               
                      [np.sin(-steering[i]), np.cos(-steering[i])]])

        vxT_fl, vyT_fl = np.squeeze(R) @ np.array([vx_fl, vy_fl])
        vxT_fr, vyT_fr = np.squeeze(R) @ np.array([vx_fr, vy_fr])

        # calculate tire slip angles 
        alpha_rad = np.zeros(4)
        alpha_rad = np.array([np.arctan2(-vyT_fl, vxT_fl), np.arctan2(-vyT_fr, vxT_fr), np.arctan2(-vy_rl, vx_rl), np.arctan2(-vy_rr, vx_rr)])

        v_ref = np.array([vxT_fl, vxT_fr, vx_rl, vx_rr])
        r_tire_m = 0.3118
        v_wheel = np.zeros(4)
        v_wheel[0:2] = r_tire_m * wF[i]
        v_wheel[2:4] = r_tire_m * wR[i]

        long_slips = (v_wheel-v_ref)/v_ref

        long_sf, long_sf, long_sr, long_sr = long_slips
        lat_sf, lat_sf, lat_sr,  lat_sr = alpha_rad

        Fzrl = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
        Fzrr = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
        Fzfl = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)
        Fzfr = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)

        Fxfl = Fzfl * (long_pac3 + long_pac6 * (Fzfl - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - np.arctan(long_pac1 * long_sf))))
        Fxfr = Fzfr * (long_pac3 + long_pac6 * (Fzfr - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - np.arctan(long_pac1 * long_sf))))
        Fxrl = Fzrl * (long_pac3 + long_pac6 * (Fzrl - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - np.arctan(long_pac1 * long_sr))))
        Fxrr = Fzrr * (long_pac3 + long_pac6 * (Fzrr - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - np.arctan(long_pac1 * long_sr))))

        Fyfl = Fzfl * (lat_pac3 + lat_pac6 * (Fzfl - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - np.arctan(lat_pac1 * lat_sf))))
        Fyfr = Fzfr * (lat_pac3 + lat_pac6 * (Fzfr - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - np.arctan(lat_pac1 * lat_sf))))
        Fyrl = Fzrl * (lat_pac3 + lat_pac6 * (Fzrl - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - np.arctan(lat_pac1 * lat_sr))))
        Fyrr = Fzrr * (lat_pac3 + lat_pac6 * (Fzrr - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - np.arctan(lat_pac1 * lat_sr))))

        Fxf = Fxfl + Fxfr
        Fxr = Fxrl + Fxrr
        Fyf = Fyfl + Fyfr
        Fyr = Fyrl + Fyrr

        Res = cf * vx + (0.5 * rho * cd * A * vx ** 2)
        ax = ((1 / m) * (Fxf * np.cos(steering[i]) - Fyf * np.sin(steering[i]) + Fxr - Res))
        ay = ((1 / m) * (Fxf * np.sin(steering[i]) + Fyf * np.cos(steering[i]) + Fyr))
        yaw_acc = ((1 / Izz) * (lf * ((Fyf * np.cos(steering[i]) + Fxf * np.sin(steering[i]))) - Fyr * lr))
        vx_new = vx + (ax + (yaw_rate * vy)) * Ts
        vy_new = vy + (ay - (yaw_rate * vx)) * Ts
        yaw_rate_new = yaw_rate + yaw_acc * Ts
        vx = vx_new
        vy = vy_new
        yaw_rate = yaw_rate_new

        vx_new_values.append(vx_new[0])
        vy_new_values.append(vy_new[0])
        yaw_rate_new_values.append(yaw_rate_new[0])

        Fxr_values.append(Fxr)
        Fyr_values.append(Fyr)

    vx_new_values = np.array(vx_new_values)
    vy_new_values = np.array(vy_new_values)
    yaw_rate_new_values = np.array(yaw_rate_new_values)
    Fxr_values = np.array(Fxr_values)
    Fyr_values = np.array(Fyr_values)

    return np.vstack((vx_new_values, vy_new_values, yaw_rate_new_values)), Fxr_values, Fyr_values


def VehicleDynamics_model_every(inputs, slip_coeffs):
    # Unpack inputs
    vx, vy, yaw_rate,a1,a2, steering, Trl, Trr, pbarF, pbarR,wF, wR, long_sf, long_sr, lat_sf, lat_sr = np.hsplit(inputs, 16)
    # (long_sf, long_sr, lat_sf, lat_sr, long_pac1, long_pac2, long_pac3, long_pac4, long_pac5, long_pac6,
    #  lat_pac1, lat_pac2, lat_pac3, lat_pac4, lat_pac5, lat_pac6) = np.hsplit(slip_coeffs, 16)
    
    lat_pac1 = 10
    lat_pac2 = 1.603
    lat_pac3 = 1.654
    lat_pac4 = 1
    lat_pac5 = 3114
    lat_pac6 = -0.1783
    long_pac1 = 18
    long_pac2 = 2
    long_pac3 = 1.7168
    long_pac4 = 1
    long_pac5 = 3114
    long_pac6 = -0.289

    # Constants
    m = 750
    g = 9.81
    lf = 1.724
    lr = 1.247
    rho = 1.22
    Cwr = 1.034
    Cwf = 0.522
    A = 1
    Izz = 1000
    cf = 20
    cd = 0.725
    Ts = 1.e-3


    # Calculate forces and dynamics
    Fzrl = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
    Fzrr = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
    Fzfl = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)
    Fzfr = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)

    Fxfl = Fzfl * (long_pac3 + long_pac6 * (Fzfl - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - np.arctan(long_pac1 * long_sf))))
    Fxfr = Fzfr * (long_pac3 + long_pac6 * (Fzfr - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - np.arctan(long_pac1 * long_sf))))
    Fxrl = Fzrl * (long_pac3 + long_pac6 * (Fzrl - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - np.arctan(long_pac1 * long_sr))))
    Fxrr = Fzrr * (long_pac3 + long_pac6 * (Fzrr - long_pac5) / long_pac5) * np.sin(long_pac2 * np.arctan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - np.arctan(long_pac1 * long_sr))))

    Fyfl = Fzfl * (lat_pac3 + lat_pac6 * (Fzfl - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - np.arctan(lat_pac1 * lat_sf))))
    Fyfr = Fzfr * (lat_pac3 + lat_pac6 * (Fzfr - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - np.arctan(lat_pac1 * lat_sf))))
    Fyrl = Fzrl * (lat_pac3 + lat_pac6 * (Fzrl - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - np.arctan(lat_pac1 * lat_sr))))
    Fyrr = Fzrr * (lat_pac3 + lat_pac6 * (Fzrr - lat_pac5) / lat_pac5) * np.sin(lat_pac2 * np.arctan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - np.arctan(lat_pac1 * lat_sr))))

    Fxf = Fxfl + Fxfr
    Fxr = Fxrl + Fxrr
    Fyf = Fyfl + Fyfr
    Fyr = Fyrl + Fyrr

    Res = cf * vx + (0.5 * rho * cd * A * vx ** 2)
    ax = ((1 / m) * (Fxf * np.cos(steering) - Fyf * np.sin(steering) + Fxr - Res))
    ay = ((1 / m) * (Fxf * np.sin(steering) + Fyf * np.cos(steering) + Fyr))
    yaw_acc = ((1 / Izz) * (lf * ((Fyf * np.cos(steering) + Fxf * np.sin(steering))) - Fyr * lr))
    vx_new = vx + (ax + (yaw_rate * vy)) * Ts
    vy_new = vy + (ay - (yaw_rate * vx)) * Ts
    yaw_rate_new = yaw_rate + yaw_acc * Ts


    return np.hstack((np.vstack((vx[0], vx_new)), np.vstack((vy[0], vy_new)), np.vstack((yaw_rate[0], yaw_rate_new)))), Fxr, Fyr


# SET FLOATING POINT PRECISION
np.set_printoptions(formatter={'float': lambda x: "{0:0.16f}".format(x)})


def run_nn(path_dict: dict,
           params_dict: dict,
           startpoint: float,
           counter: int,
           nn_mode: str):
    """Runs the recurrent neural network to test its predictions against actual vehicle data.

    :param path_dict:           dictionary which contains paths to all relevant folders and files of this module
    :type path_dict: dict
    :param params_dict:         dictionary which contains all parameters necessary to run this module
    :type params_dict: dict
    :param startpoint:          row index where to start using provided test data
    :type startpoint: float
    :param counter:             number of current testing loop (only used for naming of output files)
    :type counter: int
    """

    if not nn_mode == "feedforward" or not nn_mode == "recurrent":
        ValueError('unknown "neural network mode"; must be either "feedforard" or "recurrent"')

    # if no model was trained, load existing model in inputs folder /inputs/trained_models
    if params_dict['NeuralNetwork_Settings']['model_mode'] == 0:
        path2scaler = path_dict['filepath2scaler_load']

        if nn_mode == "feedforward":
            path2model = path_dict['filepath2inputs_trainedmodel_ff']
        elif nn_mode == "recurrent":
            path2model = path_dict['filepath2inputs_trainedmodel_recurr']

    else:
        path2scaler = path_dict['filepath2scaler_save']

        if nn_mode == "feedforward":
            path2model = path_dict['filepath2results_trainedmodel_ff']
        elif nn_mode == "recurrent":
            path2model = path_dict['filepath2results_trainedmodel_recurr']

    with open(path_dict['filepath2inputs_testdata'] + '.csv', 'r') as fh:
        data = np.loadtxt(fh, delimiter=',')

    if startpoint + params_dict['Test']['run_timespan'] > data.shape[0]:
        sys.exit("test dataset fully covered -> exit main script")

    input_shape = params_dict['NeuralNetwork_Settings']['input_shape']
    output_shape = params_dict['NeuralNetwork_Settings']['output_shape']
    input_timesteps = params_dict['NeuralNetwork_Settings']['input_timesteps']

    # scale dataset the vanish effects of different input data quantities
    data = src.prepare_data.scaler_run(path2scaler=path2scaler,
                                       params_dict=params_dict,
                                       dataset=data)

    initial, steeringangle_rad, torqueRL_Nm, torqueRR_Nm, brakepresF_bar, brakepresR_bar = \
        src.prepare_data.create_dataset_separation_run(data, params_dict, startpoint,
                                                       params_dict['Test']['run_timespan'], nn_mode)

    # load neural network model
    model = keras.models.load_model(path2model)

    results = np.zeros((len(torqueRR_Nm) + input_timesteps, input_shape))

    if nn_mode == "feedforward":
        new_input = np.zeros((1, input_shape * input_timesteps))

        for m in range(0, input_timesteps):
            results[m, 0:output_shape] = initial[:, m * input_shape:m * input_shape + output_shape]

    elif nn_mode == "recurrent":
        new_input = np.zeros((1, input_timesteps, input_shape))

        results[0:input_timesteps, :] = initial[0, :, :]

    for i_count in tqdm(range(0, len(torqueRR_Nm))):

        if i_count == 0:
            data_convert = initial

        else:
            data_convert = new_input

        result_process = model.predict(data_convert)
        results[i_count + input_timesteps, 0:output_shape] = result_process

        # convert test data
        if nn_mode == "feedforward":
            temp = np.zeros((1, input_shape * input_timesteps))
            temp[:, 0:input_shape * (input_timesteps - 1)] = data_convert[0, input_shape:input_shape * input_timesteps]

            temp[:, input_shape * (input_timesteps - 1):input_shape * (input_timesteps - 1) + output_shape] \
                = result_process

            temp[:, input_shape * (input_timesteps - 1) + output_shape] = steeringangle_rad[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 1] = torqueRL_Nm[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 2] = torqueRR_Nm[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 3] = brakepresF_bar[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 4] = brakepresR_bar[i_count]

        elif nn_mode == "recurrent":
            temp = np.zeros((1, input_timesteps, input_shape))
            temp[0, 0:input_timesteps - 1, :] = data_convert[0, 1:input_timesteps, :]

            temp[0, input_timesteps - 1, 0:output_shape] = result_process
            temp[0, input_timesteps - 1, output_shape] = steeringangle_rad[i_count]
            temp[0, input_timesteps - 1, output_shape + 1] = torqueRL_Nm[i_count]
            temp[0, input_timesteps - 1, output_shape + 2] = torqueRR_Nm[i_count]
            temp[0, input_timesteps - 1, output_shape + 3] = brakepresF_bar[i_count]
            temp[0, input_timesteps - 1, output_shape + 4] = brakepresR_bar[i_count]

        new_input = temp

    results[:, output_shape:input_shape] = data[startpoint:startpoint + len(steeringangle_rad) + input_timesteps,
                                                output_shape:input_shape]

    results = src.prepare_data.scaler_reverse(path2scaler=path2scaler,
                                              params_dict=params_dict,
                                              dataset=results)

    np.savetxt(os.path.join(path_dict['path2results_matfiles'], 'prediction_result_' + nn_mode + str(counter) + '.csv'),
               results)
