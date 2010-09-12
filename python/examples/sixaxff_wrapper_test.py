#!/usr/bin/env python
"""
sixaxff_wrapper_test.py 

The file demonstrates the use of the basic python wrapper of the libsixaxff functions
provided by sixaxff.py.  Note, the higher level interface is provided by the class
Sixaxff in sixaxff_utils.py. 

"""
import libmove_motor
import scipy 
import pylab
import libsixaxff

RAD2DEG = 180.0/scipy.pi
N=10000

# Read in data from motor map
mapfile = 'sixaxff_motor_maps.conf'
motor_maps = libmove_motor.read_motor_maps(mapfile)
clk_pins, dir_pins = libmove_motor.get_clkdir_pins(motor_maps)
motor_num_list = libmove_motor.get_motor_num_list(motor_maps)
num_motor = len(motor_num_list)
motor_num2name = libmove_motor.get_num2name_map(motor_maps)
kine_label = tuple([motor_num2name[n] for n in motor_num_list])

config = {
    'dev_name'           : ('/dev/comedi0','/dev/comedi1'),
    'ain_dev'            : 1,
    'ain_subdev'         : 0,
    'ain_zero_num'       : 50,
    'ain_zero_dt'        : 0.01,
    'ain_filt_lpcut'     : 10.0,
    'cal_file_path'      : '/home/wbd/.borfrc/sensor_cal/FT8652.cal',
    'dio_dev'            : 0, 
    'dio_subdev'         : 2,
    'dio_clk'            : clk_pins,
    'dio_dir'            : dir_pins,
    'kine_map'           : tuple(motor_num_list),
    'kine_label'         : tuple(kine_label),
    'num_motor'          : num_motor,
    'ff_motor'           : (6,7), 
    'ff_ft'              : (0,4),
    'ff_basic_tooltrans' : (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
    'ff_dynam_tooltrans' : (0,2,1),
    'ff_mass'            : (1.0, 2.0),
    'ff_ind2unit'        : (3.0, 4.0),
    'ff_axesunits'       : ('deg', 'm'),
    'ff_damping'         : (0.0, 0.0),
    'ff_flag'            : (libsixaxff.FF_ON,libsixaxff.FF_ON),
    'ff_integ_type'      : libsixaxff.INTEG_RKUTTA,
    'dt'                 : 1.0/3000.0,
    'startup_t'          : 0.0,
    }


# Print configuration as loaded into C structure
libsixaxff.print_config(config)

# Print sensor calibration information
libsixaxff.print_calinfo(config['cal_file_path'])

# Create dummy kinematics and run sixaxff real-time thread
kine = scipy.zeros((N,config['num_motor']))
t, pos, vel, ft, end_pos = libsixaxff.sixaxff_c_wrapper(kine, config)

# Convert positions and velocities to degrees
pos = RAD2DEG*pos
vel = RAD2DEG*vel

# Print shapes for return arrays
print 't.shape = ', t.shape
print 'pos.shape = ', pos.shape
print 'vel.shape = ', vel.shape
print 'ft.shape = ', ft.shape

# Plot returned values
pylab.figure(1)
pylab.subplot(211)
pylab.plot(t,pos[:,0])
pylab.ylabel('pos[:,0] (deg)')
pylab.subplot(212)
pylab.plot(t,pos[:,1])
pylab.ylabel('pos[:,1] (deg)')

pylab.figure(2)
pylab.subplot(211)
pylab.plot(t,vel[:,0])
pylab.ylabel('vel[:,0] (deg/s)')
pylab.subplot(212)
pylab.plot(t,vel[:,1])
pylab.ylabel('vel[:,1] (deg/s)')

pylab.figure(3)
for i in range(0,6):
    pylab.subplot(2,3,i+1)
    pylab.plot(t,ft[:,i])

pylab.show()
