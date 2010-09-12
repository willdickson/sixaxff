#!/usr/bin/env python
import libmove_motor
import scipy 
import pylab
import libsixaxff

RAD2DEG = 180.0/scipy.pi
N=10000

# Read in data from motor map
mapfile = 'sixaxff_motor_maps.conf'
motor_maps = libmove_motor.read_motor_maps(mapfile)
kine_label = motor_maps.keys()
clk_pins, dir_pins = libmove_motor.get_clkdir_pins(motor_maps)
motor_num_list = libmove_motor.get_motor_num_list(motor_maps)
num_motor = len(motor_num_list)
motor_num2name = libmove_motor.get_num2name_map(motor_maps)

kine_label = tuple([motor_num2name[n] for n in motor_num_list])

############################################################
# DEBUG NOTE
# 
# kine_label - are not in the correct order as motor_maps
# is a dict ... look int libmovemotor to see if there is
# a function which return names to motor indices.
#############################################################

config = {
    'dev_name'          : ('/dev/comedi0','/dev/comedi1'),
    'ain_dev'           : 1,
    'ain_subdev'        : 0,
    'ain_zero_num'      : 50,
    'ain_zero_dt'       : 0.01,
    'ain_filt_lpcut'    : 10.0,
    'cal_file_path'     : '/home/wbd/.borfrc/sensor_cal/FT8652.cal',
    'dio_dev'           : 0, 
    'dio_subdev'        : 2,
    'dio_clk'           : clk_pins,
    'dio_dir'           : dir_pins,
    'kine_map'          : tuple(motor_num_list),
    'kine_label'        : tuple(kine_label),
    'num_motor'         : num_motor,
    'ff_motor'          : (6,7), 
    'ff_ft'             : (0,4),
    'ff_tooltrans'      : (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
    'ff_mass'           : (1.0, 2.0),
    'ff_ind2unit'       : (3.0, 4.0),
    'ff_axesunits'      : ('deg', 'm'),
    'ff_damping'        : (0.1, 0.2),
    'ff_flag'           : (libsixaxff.FF_ON,libsixaxff.FF_ON),
    'ff_integ_type'     : libsixaxff.INTEG_RKUTTA,
    'dt'                : 1.0/3000.0,
    'startup_t'         : 0.0,
    }


#for k,v in config.iteritems():
#    print k,v 

libsixaxff.print_config(config)

kine = scipy.zeros((N,config['num_motor']))
t, pos, vel, torq, end_pos = libsixaxff.sixaxff_c_wrapper(kine, config)

#pos = RAD2DEG*pos
#vel = RAD2DEG*vel
#
#print torq.shape
#
#pylab.subplot(311)
#pylab.plot(t,pos)
#pylab.ylabel('positin (deg)')
#
#pylab.subplot(312)
#pylab.plot(t,vel)
#pylab.ylabel('velocity (deg/s)')
#
#pylab.subplot(313)
#pylab.plot(t,torq[:,1], 'b')
#pylab.plot(t,torq[:,0], 'r')
#pylab.xlabel('time (s)')
#pylab.ylabel('torque (Nm)')
#
#pylab.show()
