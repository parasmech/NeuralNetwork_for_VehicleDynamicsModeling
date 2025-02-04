import numpy as np
import sys
from tqdm import tqdm
import os.path
from tensorflow import keras
import tensorflow as tf
from tensorflow.keras.models import load_model, Model
from tensorflow.keras.layers import Lambda, Input, Dense, LeakyReLU, Dropout,Layer
# custom modules
import src
import pickle
from sklearn.preprocessing import StandardScaler
from joblib import dump, load
import keras
keras.config.enable_unsafe_deserialization()

"""
Created by: Rainer Trauth
Created on: 01.04.2020
"""
@keras.saving.register_keras_serializable()
def slicing_op(inputs):
    return tf.concat([inputs[:,0:10], inputs[:,12:22], inputs[:,24:34], inputs[:,36:46], inputs[:,48:58]], axis=1)

# Register the custom layer
#SlicingOpLambda = Lambda(slicing_op)

@keras.saving.register_keras_serializable()
def vehicle_dynamics_function(inputs):
    vehicle_inputs, bounded_slip_coefficients = inputs
    return VehicleDynamics_model(vehicle_inputs, bounded_slip_coefficients)

class InverseTransformLayer(Layer):
    def __init__(self, path2scaler, **kwargs):
        super(InverseTransformLayer, self).__init__(**kwargs)
        with open(path2scaler, 'rb') as f:
            self.scaler = load(f)

    def call(self, inputs):
        # Extract the scaler parameters
        mean_ = self.scaler.mean_
        scale_ = self.scaler.scale_

        # Perform the inverse transformation
        outputs = inputs * scale_ + mean_
        return outputs

@keras.saving.register_keras_serializable()
def custom_weighted_loss(y_true, y_pred):
    weights = tf.constant([1.0, 1.0, 1.0], dtype=tf.float32)
    # Ensure y_true and y_pred are the same shape
    y_true = tf.cast(y_true, dtype=tf.float32)
    y_pred = tf.cast(y_pred, dtype=tf.float32)

    # Compute the mean squared error for each neuron
    mse = K.square(y_true - y_pred)

    # Apply the weights to the respective neurons
    weighted_mse = mse * weights

    # Return the mean of the weighted MSE
    return K.mean(weighted_mse, axis=-1)

@keras.saving.register_keras_serializable()
def VehicleDynamics_model(inputs, slip_coefficients):
    # Unpack inputs
    vx, vy, yaw_rate, a1, a2, steering, Trl, Trr, pbarF, pbarR, wF, wR = tf.split(inputs, num_or_size_splits=12, axis=-1)
    
    # Unpack slip coefficients
    (long_pac1, long_pac2, long_pac3, long_pac4, long_pac5, long_pac6, 
        lat_pac1, lat_pac2, lat_pac3, lat_pac4, lat_pac5, lat_pac6) = tf.split(slip_coefficients, num_or_size_splits=12, axis=-1)

    # Constants
    m = 750.0
    g = 9.81
    lf = 1.724
    lr = 1.247
    rho = 1.22
    Cwr = 1.034
    Cwf = 0.522
    A = 1.0
    Izz = 1000.0
    cf = 20.0
    cd = 0.725
    Ts = 1.e-3

    vx_fr = vx
    vx_fl = vx
    vx_rr = vx
    vx_rl = vx
    vy_fr = vy + (yaw_rate * lf)
    vy_fl = vy + (yaw_rate * lf)
    vy_rr = vy - (yaw_rate * lr)
    vy_rl = vy - (yaw_rate * lr)

    cos_steering = tf.cos(-steering)
    sin_steering = tf.sin(-steering)

    vxT_fl = cos_steering * vx_fl - sin_steering * vy_fl
    vyT_fl = sin_steering * vx_fl + cos_steering * vy_fl
    vxT_fr = cos_steering * vx_fr - sin_steering * vy_fr
    vyT_fr = sin_steering * vx_fr + cos_steering * vy_fr

    #ipdb.set_trace(context=6)
    # Calculate tire slip angles
    alpha_rad = tf.concat([
        tf.atan2(-vyT_fl, vxT_fl), 
        tf.atan2(-vyT_fr, vxT_fr), 
        tf.atan2(-vy_rl, vx_rl), 
        tf.atan2(-vy_rr, vx_rr)
    ], axis=1)

    v_ref = tf.concat([vxT_fl, vxT_fr, vx_rl, vx_rr], axis=1)
    r_tire_m = 0.3118
    v_wheel = tf.concat([r_tire_m * wF,r_tire_m * wF, r_tire_m * wR,r_tire_m * wR], axis=1)

    # Ensure compatible shapes for v_wheel and v_ref
   # v_wheel = tf.expand_dims(v_wheel, axis=-1)  # Adjust shape to [batch_size, 2, 1]
    #v_wheel = tf.tile(v_wheel, [1, 1, 4])  # Tile to [batch_size, 2, 4]

    long_slips = (v_wheel - v_ref) / v_ref

    long_sf, long_sf, long_sr, long_sr = tf.split(long_slips, num_or_size_splits=4, axis=-1)
    lat_sf, lat_sf, lat_sr, lat_sr = tf.split(alpha_rad, num_or_size_splits=4, axis=-1)

    # Calculate forces and dynamics
    Fzrl = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
    Fzrr = ((m * g * lf) / (2 * (lf + lr))) + (0.25 * rho * Cwr * A * vx ** 2)
    Fzfl = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)
    Fzfr = ((m * g * lr) / (2 * (lf + lr))) + (0.25 * rho * Cwf * A * vx ** 2)

    Fxfl = Fzfl * (long_pac3 + long_pac6 * (Fzfl - long_pac5) / long_pac5) * tf.sin(long_pac2 * tf.atan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - tf.atan(long_pac1 * long_sf))))
    Fxfr = Fzfr * (long_pac3 + long_pac6 * (Fzfr - long_pac5) / long_pac5) * tf.sin(long_pac2 * tf.atan(long_pac1 * long_sf - long_pac4 * (long_pac1 * long_sf - tf.atan(long_pac1 * long_sf))))
    Fxrl = Fzrl * (long_pac3 + long_pac6 * (Fzrl - long_pac5) / long_pac5) * tf.sin(long_pac2 * tf.atan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - tf.atan(long_pac1 * long_sr))))
    Fxrr = Fzrr * (long_pac3 + long_pac6 * (Fzrr - long_pac5) / long_pac5) * tf.sin(long_pac2 * tf.atan(long_pac1 * long_sr - long_pac4 * (long_pac1 * long_sr - tf.atan(long_pac1 * long_sr))))

    Fyfl = Fzfl * (lat_pac3 + lat_pac6 * (Fzfl - lat_pac5) / lat_pac5) * tf.sin(lat_pac2 * tf.atan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - tf.atan(lat_pac1 * lat_sf))))
    Fyfr = Fzfr * (lat_pac3 + lat_pac6 * (Fzfr - lat_pac5) / lat_pac5) * tf.sin(lat_pac2 * tf.atan(lat_pac1 * lat_sf - lat_pac4 * (lat_pac1 * lat_sf - tf.atan(lat_pac1 * lat_sf))))
    Fyrl = Fzrl * (lat_pac3 + lat_pac6 * (Fzrl - lat_pac5) / lat_pac5) * tf.sin(lat_pac2 * tf.atan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - tf.atan(lat_pac1 * lat_sr))))
    Fyrr = Fzrr * (lat_pac3 + lat_pac6 * (Fzrr - lat_pac5) / lat_pac5) * tf.sin(lat_pac2 * tf.atan(lat_pac1 * lat_sr - lat_pac4 * (lat_pac1 * lat_sr - tf.atan(lat_pac1 * lat_sr))))

    Fxf = Fxfl + Fxfr
    Fxr = Fxrl + Fxrr
    Fyf = Fyfl + Fyfr
    Fyr = Fyrl + Fyrr

    Res = cf * vx + (0.5 * rho * cd * A * vx ** 2)
    ax = ((1 / m) * (Fxf * tf.cos(steering) - Fyf * tf.sin(steering) + Fxr - Res))
    ay = ((1 / m) * (Fxf * tf.sin(steering) + Fyf * tf.cos(steering) + Fyr))
    yaw_acc = ((1 / Izz) * (lf * ((Fyf * tf.cos(steering) + Fxf * tf.sin(steering))) - Fyr * lr))
    vx_new = vx + (ax + (yaw_rate * vy)) * Ts
    vy_new = vy + (ay - (yaw_rate * vx)) * Ts
    yaw_rate_new = yaw_rate + yaw_acc * Ts
    #ipdb.set_trace(context=6)
    return tf.concat([vx_new, vy_new, yaw_rate_new], axis=-1)

# SET FLOATING POINT PRECISION
np.set_printoptions(formatter={'float': lambda x: "{0:0.16f}".format(x)})

@keras.saving.register_keras_serializable()
def apply_bounds(slip_coefficients):
    min_bounds = tf.constant([5.0, 0.5, 0.1, 0.0, 1000.0, -1.0, 5.0, 0.5, 0.1, 0.0, 1000.0, -1.0], dtype=tf.float32)
    max_bounds = tf.constant([30.0, 2.0, 2.0, 1.0, 5000.0, 0.0, 30.0, 2.0, 2.0, 1.0, 5000.0, 0.0], dtype=tf.float32)
    return min_bounds + slip_coefficients * (max_bounds - min_bounds)

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
    print(data.shape)
    data = data[:,0:12]
    data2 = data[:,12:]
    if startpoint + params_dict['Test']['run_timespan'] > data.shape[0]:
        sys.exit("test dataset fully covered -> exit main script")

    
    input_shape = params_dict['NeuralNetwork_Settings']['input_shape']
    output_shape = params_dict['NeuralNetwork_Settings']['output_shape']
    input_timesteps = params_dict['NeuralNetwork_Settings']['input_timesteps']

    # scale dataset the vanish effects of different input data quantities
    data = src.prepare_data.scaler_run(path_dict=path_dict,
                                       path2scaler=path2scaler,
                                       params_dict=params_dict,
                                       dataset=data)

   # initial, steeringangle_rad, torqueRL_Nm, torqueRR_Nm, brakepresF_bar, brakepresR_bar
    initial, vx, vy, yaw_rate, acc_x, acc_y, steeringangle_rad, torqueRL_Nm, torqueRR_Nm, brakepresF_bar, brakepresR_bar,wf,wr = \
        src.prepare_data.create_dataset_separation_run(data, params_dict, startpoint,
                                                       params_dict['Test']['run_timespan'], nn_mode)

    # load neural network model
    
    model = keras.models.load_model(path_dict['filepath2inputs_trainedmodel_ff'], custom_objects={'VehicleDynamics_model': VehicleDynamics_model, 'custom_weighted_loss': custom_weighted_loss, 'slicing_op': slicing_op, 'vehicle_dynamics_function':vehicle_dynamics_function})
    
    results = np.zeros((len(torqueRR_Nm) + input_timesteps, input_shape))

    if nn_mode == "feedforward":
        new_input = np.zeros((1, input_shape * input_timesteps))

        for m in range(0, input_timesteps):
            results[m, 0:output_shape] = initial[:, m * input_shape:m * input_shape + output_shape]

    elif nn_mode == "recurrent":
        new_input = np.zeros((1, input_timesteps, input_shape))

        results[0:input_timesteps, :] = initial[0, :, :]
    all_activations = []
    all_output_states = []
    for i_count in tqdm(range(0, len(torqueRR_Nm))):

        if i_count == 0:
            data_convert = initial
            #print("data 1st time step: \n", data_convert)

        else:
            data_convert = new_input
            #print("data next time step: \n", data_convert)
        result_process = model.predict(data_convert)
        
        results[i_count + input_timesteps, 0:output_shape] = result_process[:,0:3]
        
        # convert test data
        if nn_mode == "feedforward":
            temp = np.zeros((1, input_shape * input_timesteps))
            temp[:, 0:input_shape * (input_timesteps - 1)] = data_convert[0, input_shape:input_shape * input_timesteps]

            temp[:, input_shape * (input_timesteps - 1):input_shape * (input_timesteps - 1) + output_shape] \
                = result_process[:,0:3]
            # temp[:, input_shape * (input_timesteps - 1) + 0] = vx[i_count]
            # temp[:, input_shape * (input_timesteps - 1) + 1] = vy[i_count]
            # temp[:, input_shape * (input_timesteps - 1) + 2] = yaw_rate[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape] = acc_x[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape+1] = acc_y[i_count]
            
            temp[:, input_shape * (input_timesteps - 1) + output_shape+2] = steeringangle_rad[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 3] = torqueRL_Nm[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 4] = torqueRR_Nm[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 5] = brakepresF_bar[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 6] = brakepresR_bar[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 7] = wf[i_count]
            temp[:, input_shape * (input_timesteps - 1) + output_shape + 8] = wr[i_count]

        elif nn_mode == "recurrent":
            temp = np.zeros((1, input_timesteps, input_shape))
            temp[0, 0:input_timesteps - 1, :] = data_convert[0, 1:input_timesteps, :]

            temp[0, input_timesteps - 1, 0:output_shape] = result_process
            temp[0, input_timesteps - 1, output_shape] = steeringangle_rad[i_count]
            temp[0, input_timesteps - 1, output_shape + 1] = torqueRL_Nm[i_count]
            temp[0, input_timesteps - 1, output_shape + 2] = torqueRR_Nm[i_count]
            temp[0, input_timesteps - 1, output_shape + 3] = brakepresF_bar[i_count]
            temp[0, input_timesteps - 1, output_shape + 4] = brakepresR_bar[i_count]

        activation_model = Model(inputs=model.input, outputs=model.layers[10].output)
        activations = activation_model.predict(data_convert)
        all_activations.append(activations)

        final_model = Model(inputs=model.input, outputs=model.layers[11].output)
        output_states = final_model.predict(data_convert)
        all_output_states.append(output_states)
        new_input = temp

   
    results[:, output_shape:input_shape] = data[startpoint:startpoint + len(steeringangle_rad) + input_timesteps,
                                                output_shape:input_shape]

    


    results = src.prepare_data.scaler_reverse(path_dict=path_dict,
                                              path2scaler=path2scaler,
                                              params_dict=params_dict,
                                              dataset=results)

    
    all_activations_combined = np.vstack(all_activations)
    np.savetxt(os.path.join(path_dict['path2results_matfiles'], 'prediction_result_' + nn_mode + str(counter) + '.csv'),
               results)

    all_output_combined = np.vstack(all_output_states)

    # Extract the first 3 outputs from all_output_combined
    all_output_combined = all_output_combined[:, :3]
    
    # Apply the scaler_reverse function to these outputs
    all_output_combined = src.prepare_data.scaler_reverse_new(
                                                        path_dict=path_dict,
                                                        path2scaler=path2scaler,
                                                        params_dict=params_dict,
                                                        dataset=all_output_combined)

    output_states_filepath = os.path.join(path_dict['path2results_matfiles'], 'output_state_activations_' + str(counter) + '.csv')
    np.savetxt(output_states_filepath, all_output_combined, delimiter=',')
   

    activations_filepath = os.path.join(path_dict['path2results_matfiles'], 'layer11_activations_' + str(counter) + '.csv')
    np.savetxt(activations_filepath, all_activations_combined, delimiter=',')
