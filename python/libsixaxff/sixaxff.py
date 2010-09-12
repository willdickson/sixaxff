"""
-----------------------------------------------------------------------
sixaxff
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of sixaxff.

sixaxff is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
    
sixaxff is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with sixaxff.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------
"""
import ctypes 
import scipy
from libmove_motor import convert2int

lib = ctypes.cdll.LoadLibrary("libsixaxff.so.1")


# Constants 
S2NS = 1.0e9
MAX_MOTOR = lib.define_max_motor()
MAX_DT = lib.define_max_dt()
MIN_DT = lib.define_min_dt()
CLOCK_HI_NS = lib.define_clock_hi_ns()
INTEG_EULER = lib.define_integ_euler()
INTEG_RKUTTA = lib.define_integ_rkutta()
INTEG_UNKNOWN = lib.define_integ_unknown()
EMPTY_ARRAY = lib.define_empty_array()
INT_ARRAY = lib.define_int_array()
FLT_ARRAY = lib.define_flt_array()
DBL_ARRAY = lib.define_dbl_array()
UNKOWN_ARRAY = lib.define_unknown_array()
SUCCESS = lib.define_success()
FAIL = lib.define_fail()
FF_ON = lib.define_ff_on()
FF_OFF = lib.define_ff_off()
NUM_FF = lib.define_num_ff()


# Structures
class array_t(ctypes.Structure):
    _fields_ = [
        ('data', ctypes.c_void_p), 
        ('nrow', ctypes.c_int), 
        ('ncol', ctypes.c_int),
        ('s0', ctypes.c_int),
        ('s1', ctypes.c_int),
        ('type', ctypes.c_int), 
        ]

class config_t(ctypes.Structure):
    _fields_ = [
        ('dev_name', NUM_FF*ctypes.c_char_p),
        ('ain_dev', ctypes.c_uint),
        ('ain_subdev', ctypes.c_uint),
        ('ain_zero_num', ctypes.c_uint),
        ('ain_zero_dt', ctypes.c_float),
        ('ain_filt_lpcut', ctypes.c_float),
        ('cal_file_path', ctypes.c_char_p),
        ('dio_dev', ctypes.c_uint),
        ('dio_subdev', ctypes.c_uint),
        ('dio_clk', MAX_MOTOR*ctypes.c_uint),
        ('dio_dir', MAX_MOTOR*ctypes.c_uint),
        ('kine_map', MAX_MOTOR*ctypes.c_uint),
        ('kine_label', MAX_MOTOR*ctypes.c_char_p),
        ('num_motor', ctypes.c_uint),
        ('ff_motor', NUM_FF*ctypes.c_uint),
        ('ff_ft', NUM_FF*ctypes.c_uint),
        ('ff_tooltrans', 6*ctypes.c_float),
        ('ff_mass', NUM_FF*ctypes.c_float),
        ('ff_ind2unit', NUM_FF*ctypes.c_float),
        ('ff_axesunits', NUM_FF*ctypes.c_char_p),
        ('ff_damping', NUM_FF*ctypes.c_float),
        ('ff_flag', NUM_FF*ctypes.c_uint),
        ('ff_integ_type', ctypes.c_uint),
        ('dt', ctypes.c_uint),
        ('startup_t', ctypes.c_float),
        ]

class data_t(ctypes.Structure):
    _fields_ = [
        ('t', array_t),
        ('pos', array_t),
        ('vel', array_t),
        ('ft', array_t),
        ]

# Functions 
lib.sixaxff.restype = ctypes.c_int
lib.sixaxff.argstype = [
    array_t,
    config_t,
    data_t,
    ctypes.c_void_p,
]


lib.print_config.argstype = [
    config_t,
]


def get_c_array_struct(x):
    """
    Get the C array structure associated with the scipy or numpy array.
    """
    x_struct = array_t()
    x_struct.data = x.ctypes.data_as(ctypes.c_void_p)
    x_struct.nrow = x.ctypes.shape[0]
    x_struct.ncol = x.ctypes.shape[1]
    x_struct.s0 = x.ctypes.strides[0]
    x_struct.s1 = x.ctypes.strides[1]
    if x.dtype == scipy.dtype('int'):
        x_struct.type = INT_ARRAY
    elif x.dtype == scipy.dtype('float32'):
        x_struct.type = FLT_ARRAY
    elif x.dtype == scipy.dtype('float64'):
        x_struct.type = DBL_ARRAY
    else:
        raise ValueError, "array must be of type INT_ARRAY, FLT_ARRAY or DBL_ARRAY" 
    return x_struct


def create_config_struct(config):
    """
    Create c configuration structure
    """

    ######################################################################
    # DEBUG NOTE: 
    # 
    # Should probably check that all the required fields are here
    #####################################################################
    config_struct = config_t()
    for key in config:
        if key == 'dt':
            value = int(S2NS*config[key])
        else:
            value = config[key]
        setattr(config_struct,key,value)
    return config_struct

def sixaxff_c_wrapper(kine, config):
    """
    Python wrapper for sixaxwff function. Performs force-feedback
    task. Spawns real-time thread to handle kinematics outscan, data
    acquisition, and yaw dynamics. During outscaning displays
    information regarding ongoing real-time task.
    
    Inputs:
      kine    = Nx7 array of wing kinematics in indices, one column is for 
                yaw.
      config  = system configuration dictionary 
    """

    # Convert kinematics to integers
    kine = convert2int(kine)

    config_struct = create_config_struct(config)

    # Create c kinematics array structure
    kine_int = kine.astype(scipy.dtype('int'))
    kine_struct =  get_c_array_struct(kine_int)
        
    # Time, position, velocity, and torque arrays for return data
    n = kine.shape[0]
    t = scipy.zeros((n,1), dtype = scipy.dtype('float64'))
    pos = scipy.zeros((n,2), dtype = scipy.dtype('float32'))
    vel = scipy.zeros((n,2), dtype = scipy.dtype('float32'))
    ft = scipy.zeros((n,6), dtype = scipy.dtype('float32'))

    # Create c data structure
    data_struct = data_t()
    data_struct.t = get_c_array_struct(t)
    data_struct.pos = get_c_array_struct(pos)
    data_struct.vel = get_c_array_struct(vel)
    data_struct.ft = get_c_array_struct(ft)
    
    # Create array for ending positions
    end_pos = (ctypes.c_int*config['num_motor'])()

    # Call C library sixaxff function
    ret_val = lib.sixaxff(kine_struct, config_struct, data_struct, end_pos)
    if ret_val == FAIL:
        raise RuntimeError, "lib.sizaxff call failed"

    end_pos = scipy.array(end_pos)

    return t, pos, vel, ft, end_pos


def print_config(config):
    config_struct = create_config_struct(config)
    lib.print_config(config_struct)
    return


