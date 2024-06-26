import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.layers import Lambda, Input, Dense, LeakyReLU, Dropout
from tensorflow.keras.models import Model
# custom modules
import helper_funcs_NN
from tensorflow.keras.callbacks import Callback

"""
Created by: Rainer Trauth
Created on: 01.04.2020
"""

def VehicleDynamics_model(inputs, slip_coefficients):
    # Unpack inputs
    vx, vy, yaw_rate,a1,a2, steering, Trl, Trr, pbarF, pbarR,wF, wR = tf.split(inputs, num_or_size_splits=12, axis=-1)
    #tf.print("Slip coefficients: ", slip_coefficients)
    
    # Unpack slip coefficients
    (long_sf, long_sr, lat_sf, lat_sr, long_pac1, long_pac2, long_pac3, long_pac4, long_pac5, long_pac6, 
     lat_pac1, lat_pac2, lat_pac3, lat_pac4, lat_pac5, lat_pac6) = tf.split(slip_coefficients, num_or_size_splits=16, axis=-1)

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

    return tf.concat([vx_new, vy_new, yaw_rate_new,long_sf, long_sr, lat_sf, lat_sr], axis=-1)

def create_nnmodel(path_dict: dict,
                   params_dict: dict,
                   nn_mode: str):
    """Creates a new neural network model or loads an existing one.

    :param path_dict:           dictionary which contains paths to all relevant folders and files of this module
    :type path_dict: dict
    :param params_dict:         dictionary which contains all parameters necessary to run this module
    :type params_dict: dict
    :param nn_mode:             Neural network mode which defines type of NN (feedforward or recurrent)
    :type nn_mode: str
    :return: [description]
    :rtype: [type]
    """

    if not nn_mode == "feedforward" or not nn_mode == "recurrent":
        ValueError('unknown "neural network mode"; must be either "feedforard" or "recurrent"')

    if nn_mode == "feedforward":
        filepath2inputs_trainedmodel = path_dict['filepath2inputs_trainedmodel_ff']

    elif nn_mode == "recurrent":
        filepath2inputs_trainedmodel = path_dict['filepath2inputs_trainedmodel_recurr']

    if params_dict['General']['bool_load_existingmodel']:
        print('LOAD ALREADY CREATED MODEL FOR FURTHER TRAINING')
        model_create = keras.models.load_model(filepath2inputs_trainedmodel)

    else:
        print('CREATE NEW MODEL')

        if nn_mode == "feedforward":
            model_create = create_model_feedforward(path_dict=path_dict,
                                                    params_dict=params_dict)

        elif nn_mode == "recurrent":
            model_create = create_model_recurrent(path_dict=path_dict,
                                                  params_dict=params_dict)

        optimizer = helper_funcs_NN.src.select_optimizer.select_optimizer(
            optimizer=params_dict['NeuralNetwork_Settings']['Optimizer']['optimizer_set'],
            learning_rate=params_dict['NeuralNetwork_Settings']['learning_rate'],
            clipnorm=params_dict['NeuralNetwork_Settings']['Optimizer']['clipnorm'])

        model_create.compile(optimizer=optimizer,
                             loss=params_dict['NeuralNetwork_Settings']['Optimizer']['loss_function'],
                             metrics=[keras.metrics.mae, keras.metrics.mse])

        model_create.summary()

    return model_create


# ----------------------------------------------------------------------------------------------------------------------

def create_model_feedforward(path_dict: dict,
                             params_dict: dict):
    """Set up a new feedforward NN model

    :param path_dict:           dictionary which contains paths to all relevant folders and files of this module
    :type path_dict: dict
    :param params_dict:         dictionary which contains all parameters necessary to run this module
    :type params_dict: dict
    :return:                    neural network model
    :rtype: [type]
    """

    print('CREATE FEEDFORWARD NEURAL NETWORK')

    model_create = keras.models.Sequential()

    if params_dict['NeuralNetwork_Settings']['Initializer'] == "he":
        kernel_init = keras.initializers.he_uniform(seed=42)
        #kernel_init = keras.initializers.RandomNormal(mean=0.0, stddev=0.05, seed=None)
    elif params_dict['NeuralNetwork_Settings']['Initializer'] == "glorot":
        kernel_init = keras.initializers.GlorotUniform(seed=True)

    
    reg_dense = keras.regularizers.l1_l2(params_dict['NeuralNetwork_Settings']['l1regularization'],
                                         params_dict['NeuralNetwork_Settings']['l2regularization'])

    input_shape = params_dict['NeuralNetwork_Settings']['input_shape'] \
        * params_dict['NeuralNetwork_Settings']['input_timesteps']
    
    num_Dense_layers_1 = 1;
    num_Dense_layers_2 = 1;
    num_Dense_layers_3 = 1;
    num_Dense_layers_4 = 1;
    inputs = Input(shape=(input_shape,))
    x = inputs
    for i in range(num_Dense_layers_1):
        x = Dense(units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_first_layer'],
                  use_bias=True,
                  bias_initializer='zeros',
                  activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_1'],
                  kernel_initializer=kernel_init,
                  kernel_regularizer=reg_dense)(x)

        if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
            x = LeakyReLU(alpha=0.2)(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_1'])(x)

    for i in range(num_Dense_layers_2):
        x = Dense(units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_second_layer'],
                  use_bias=True,
                  bias_initializer='zeros',
                  activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_1'],
                  kernel_initializer=kernel_init,
                  kernel_regularizer=reg_dense)(x)

        if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
            x = LeakyReLU(alpha=0.2)(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_1'])(x) 

    for i in range(num_Dense_layers_3):
        x = Dense(units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_third_layer'],
                  use_bias=True,
                  bias_initializer='zeros',
                  activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_1'],
                  kernel_initializer=kernel_init,
                  kernel_regularizer=reg_dense)(x)

        if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
            x = LeakyReLU(alpha=0.2)(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_1'])(x)   

    for i in range(num_Dense_layers_4):
        x = Dense(units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_fourth_layer'],
                  use_bias=True,
                  bias_initializer='zeros',
                  activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_1'],
                  kernel_initializer=kernel_init,
                  kernel_regularizer=reg_dense)(x)

        if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
            x = LeakyReLU(alpha=0.2)(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_1'])(x)     

    slip_coefficients = Dense(units=16, activation='sigmoid')(x)
    print("slip_coefficients from model: " , slip_coefficients)

# Define bounds for each of the 16 neurons within the Lambda layer
    def apply_bounds(slip_coefficients):
        min_bounds = tf.constant([-0.05, -0.05, -0.4, -0.4, 5.0, 0.5, 0.1, -2.0, 1000.0, -1.0, 5.0, 0.5, 0.1, -2.0, 1000.0, -1.0], dtype=tf.float32)
        max_bounds = tf.constant([0.05, 0.05, 0.4, 0.4, 30.0, 2.0, 2.0, 1.0, 5000.0, 0.0, 30.0, 2.0, 2.0, 1.0, 5000.0, 0.0], dtype=tf.float32)
        return min_bounds + slip_coefficients * (max_bounds - min_bounds)

    bounded_slip_coefficients = Lambda(apply_bounds)(slip_coefficients)

    vehicle_inputs = Lambda(lambda inputs: inputs[:, -12:])(inputs)
    vehicle_dynamics = Lambda(lambda inputs: VehicleDynamics_model(inputs[0], inputs[1]))([vehicle_inputs, bounded_slip_coefficients])

    model_create = Model(inputs=inputs, outputs=vehicle_dynamics)
    # model_create.add(
    #     keras.layers.Dense(input_shape=(input_shape,),
    #                        units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_first_layer'],
    #                        use_bias=True,
    #                        bias_initializer='zeros',
    #                        activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_1'],
    #                        kernel_initializer=kernel_init,
    #                        kernel_regularizer=reg_dense))

    # if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
    #     model_create.add(keras.layers.LeakyReLU(alpha=0.2))

    # if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
    #     model_create.add(keras.layers.Dropout(params_dict['NeuralNetwork_Settings']['drop_1']))

    # model_create.add(
    #     keras.layers.Dense(units=params_dict['NeuralNetwork_Settings']['Feedforward']['neurons_second_layer'],
    #                        bias_initializer='zeros',
    #                        use_bias=True,
    #                        activation=params_dict['NeuralNetwork_Settings']['Feedforward']['activation_2']
    #                        ))

    # if params_dict['NeuralNetwork_Settings']['Feedforward']['leakyrelu'] == 1:
    #     model_create.add(keras.layers.LeakyReLU(alpha=0.2))

    # if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
    #     model_create.add(keras.layers.Dropout(params_dict['NeuralNetwork_Settings']['drop_2']))

    # model_create.add(
    #     keras.layers.Dense(units=params_dict['NeuralNetwork_Settings']['output_shape'], activation='linear'))

    return model_create


# ----------------------------------------------------------------------------------------------------------------------

def create_model_recurrent(path_dict: dict,
                           params_dict: dict):
    """Set up a new recurrent NN model

    :param path_dict:           dictionary which contains paths to all relevant folders and files of this module
    :type path_dict: dict
    :param params_dict:         dictionary which contains all parameters necessary to run this module
    :type params_dict: dict
    :return:                    neural network model
    :rtype: [type]
    """

    print('CREATE RECURRENT NEURAL NETWORK')

    #model_create = keras.models.Sequential()

    if params_dict['NeuralNetwork_Settings']['Initializer'] == "he":
        kernel_init = keras.initializers.he_uniform(seed=42)

    elif params_dict['NeuralNetwork_Settings']['Initializer'] == "glorot":
        kernel_init = keras.initializers.GlorotUniform(seed=True)

    reg_layer = keras.regularizers.l1_l2(params_dict['NeuralNetwork_Settings']['l1regularization'],
                                         params_dict['NeuralNetwork_Settings']['l2regularization'])

    # load specified recurrent layer type (from parameter file)
    if params_dict['NeuralNetwork_Settings']['Recurrent']['recurrent_mode'] == 'GRU':
        recurrent_mode = keras.layers.GRU

    elif params_dict['NeuralNetwork_Settings']['Recurrent']['recurrent_mode'] == 'LSTM':
        recurrent_mode = keras.layers.LSTM

    elif params_dict['NeuralNetwork_Settings']['Recurrent']['recurrent_mode'] == 'SimpleRNN':
        recurrent_mode = keras.layers.SimpleRNN

    elif params_dict['NeuralNetwork_Settings']['Recurrent']['recurrent_mode'] == 'ConvLSTM2D':
        recurrent_mode = keras.layers.ConvLSTM2D

    elif params_dict['NeuralNetwork_Settings']['Recurrent']['recurrent_mode'] == 'RNN':
        recurrent_mode = keras.layers.RNN

    # Input layer
    inputs = Input(shape=(params_dict['NeuralNetwork_Settings']['input_timesteps'],
                                    params_dict['NeuralNetwork_Settings']['input_shape']))
    x = inputs
    num_GRU_layers = 1
    num_Dense_layers = 9
    for i in range(num_GRU_layers):
        return_sequences = (i< (num_GRU_layers - 1))
        x = recurrent_mode(
                          units=params_dict['NeuralNetwork_Settings']['Recurrent']['neurons_first_layer_recurrent'],
                          return_sequences=return_sequences,
                          use_bias=True,
                          bias_initializer='zeros',
                          kernel_initializer=kernel_init,
                          kernel_regularizer=reg_layer,
                          activation=params_dict['NeuralNetwork_Settings']['Recurrent']['activation_1_recurrent'])(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_1'])(x)

#1D layer
    for i in range(num_Dense_layers):
        x = Dense(units=params_dict['NeuralNetwork_Settings']['Recurrent']['neurons_second_layer_recurrent'],
                              use_bias=True,
                              bias_initializer='zeros',
                              kernel_initializer=kernel_init,
                              activation=params_dict['NeuralNetwork_Settings']['Recurrent']['activation_dense_recurrent'],
                              kernel_regularizer=reg_layer)(x)

        if params_dict['NeuralNetwork_Settings']['bool_use_dropout']:
            x = Dropout(params_dict['NeuralNetwork_Settings']['drop_2'])(x)
#
   

    slip_coefficients = Dense(units=16, activation='sigmoid')(x)
    print("slip_coefficients from model: " , slip_coefficients)

# Define bounds for each of the 16 neurons within the Lambda layer
    def apply_bounds(slip_coefficients):
        min_bounds = tf.constant([-0.05, -0.001, -0.4, -0.4, 5.0, 0.5, 0.1, -2.0, 1000.0, -1.0, 5.0, 0.5, 0.1, -2.0, 1000.0, -1.0], dtype=tf.float32)
        max_bounds = tf.constant([0.001, 0.05, 0.4, 0.4, 30.0, 2.0, 2.0, 1.0, 5000.0, 0.0, 30.0, 2.0, 2.0, 1.0, 5000.0, 0.0], dtype=tf.float32)
        return min_bounds + slip_coefficients * (max_bounds - min_bounds)

    bounded_slip_coefficients = Lambda(apply_bounds)(slip_coefficients)

    vehicle_inputs = Lambda(lambda inputs: inputs[:, -1,:])(inputs)
    vehicle_dynamics = Lambda(lambda inputs: VehicleDynamics_model(inputs[0], inputs[1]))([vehicle_inputs, bounded_slip_coefficients])

    model_create = Model(inputs=inputs, outputs=vehicle_dynamics)
    
    return model_create
