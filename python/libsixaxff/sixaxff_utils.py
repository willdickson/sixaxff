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
import sys
import os
import os.path
import ConfigParser
import optparse
import libmove_motor
import libsixaxff
import scipy
import pylab
import scipy.interpolate
import time
import clkdirpwm
import qarray

PI = scipy.pi
DEG2RAD = scipy.pi/180.0
RAD2DEG = 180.0/scipy.pi
BORFRC_DIR = os.path.join(os.environ['HOME'],'.borfrc')
DFLT_CONFIG_FILE = os.path.join(BORFRC_DIR, 'defaults')
DFLT_SENSOR_CAL_DIR = os.path.join(BORFRC_DIR,'sensor_cal')
DFLT_COMEDI_CONF_DIR = os.path.join(BORFRC_DIR, 'comedi_conf')
DFLT_MOVE_VMAX = 10.0
DFLT_MOVE_ACCEL = 40.0
DFLT_MOVE_VMAX_IND = 100.0
DFLT_MOVE_ACCEL_IND = 400.0
DFLT_MOVE_DT = 1.0/3000.0
DFLT_PAUSE_T = 10.0
DFLT_K_STROKE = 0.01
DFLT_K_ROTATION = 1.5
DFLT_ROTATION_OFFSET = 0.0 

class Sixaxff_Base(object):

    """
    Base class for Sixaxff 
    """

    def __init__(self, 
                 run_params, 
                 motor_maps_file=None, 
                 sensor_cal_file=None, 
                 comedi_conf_file=None,
                 defaults_file=DFLT_CONFIG_FILE,
                 move_vmax=DFLT_MOVE_VMAX,
                 move_accel=DFLT_MOVE_ACCEL,
                 move_dt=DFLT_MOVE_DT,
                 pause_t = None):
        """
        Initialize Sixaxff_Base class
        """
        if motor_maps_file==None or sensor_cal_file==None or comedi_conf_file==None:
            dflts = read_defaults(defaults_file)
            dflt_motor_maps_file, dflt_sensor_cal_file, dflt_comedi_conf_file = dflts

        # Read motor maps file
        if motor_maps_file == None:
            self.motor_maps_file = dflt_motor_maps_file
        else:
            self.motor_maps_file = motor_maps_file
        self.motor_maps = libmove_motor.read_motor_maps(self.motor_maps_file)

        # Read sensor calibration file
        if sensor_cal_file == None:
            self.sensor_cal_file = dflt_sensor_cal_file
        else:
            self.sensor_cal_file = sensor_cal_file
        self.sensor_cal = read_sensor_cal(self.sensor_cal_file)

        # Read comedi configuration file
        if comedi_conf_file == None:
            self.comedi_conf_file = dflt_comedi_conf_file
        else:
            self.comedi_conf_file = comedi_conf_file
        self.comedi_conf = read_comedi_conf(self.comedi_conf_file)

        self.run_params = run_params
        self.move_vmax = move_vmax
        self.move_accel = move_accel
        self.move_dt = move_dt
        self.config_dict = self.create_config_dict()

        if pause_t == None:
            self.pause_t = DFLT_PAUSE_T
        else:
            self.pause_t = pause_t


    def create_config_dict(self):
        """
        Create configuration dictionary which is passed to sixaxff_c_wrapper
        """
        config = {}
        
        # Add comedi configuration parameters
        config.update(self.comedi_conf)

        # Add motor map parameters
        clk_pins, dir_pins = libmove_motor.get_clkdir_pins(self.motor_maps)
        yaw_motor = self.motor_maps['yaw']['number']
        yaw_ind2deg = self.motor_maps['yaw']['deg_per_ind']
        motor_num_list = libmove_motor.get_motor_num_list(self.motor_maps)
        num_motor = len(motor_num_list)
        kine_map = tuple([i for i in motor_num_list if i != yaw_motor]) 
        config['num_motor'] = num_motor
        config['yaw_motor'] = yaw_motor
        config['dio_clk'] = clk_pins
        config['dio_dir'] = dir_pins
        config['kine_map'] = kine_map
        config['yaw_ind2deg'] = yaw_ind2deg

        # Add sensor_calibration parameters
        config['yaw_volt2torq'] = self.sensor_cal['calibration']
        config['yaw_ain'] = self.sensor_cal['ain_chan']

        # Add run parameters
        config.update(self.run_params)
        return config

    def move_to_pos(self,name_list,pos_list,noreturn=False,at_zero_ind=True,
                    vmax=DFLT_MOVE_VMAX, accel=DFLT_MOVE_ACCEL):
        """
        Move the robot to the given positions in degrees.
        """
        n = self.num_motors()
        pos = scipy.zeros((n,))

        if len(pos_list) == len(name_list):
            for p, name in zip(pos_list,name_list):
                motor_num = self.get_motor_num(name)
                pos[motor_num] = p
        elif len(pos_list) == 1:
            p = pos_list[0]
            for name in name_list:
                motor_num = self.get_motor_num(name)
                pos[motor_num] = p
        else:
            raise ValueError, 'len(pos_list) must equal len(name_list) or 1'

        config = self.config_dict
        # Move to kinematics starting positon
        print 'moving to position'
        if at_zero_ind == True:
            zero_pos = libmove_motor.get_zero_indpos_deg(self.motor_maps)
        else:
            zero_pos = scipy.zeros((n,))
        ramps_deg = libmove_motor.get_ramp_moves(zero_pos,
                                                 pos,
                                                 vmax, 
                                                 accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)

        if noreturn == False:
            # Wait until done
            ans = raw_input('Press enter to return to zero index position:')
            # Return to the zero index position 
            print 'returning to zero index positon'
            ramps_deg = libmove_motor.get_ramp_moves(pos,
                                                     zero_pos,
                                                     vmax, 
                                                     accel, 
                                                     self.move_dt)
            ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
            end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)


    def move_by_ind(self, name_list, ind_list, noreturn=False,vmax=DFLT_MOVE_VMAX_IND, accel=DFLT_MOVE_ACCEL_IND): 
        """ 
        Move motor given by motor_name by the specified number of indices.  
        """
        config=self.config_dict
        n = self.num_motors()
        zero_pos = scipy.zeros((n,))
        next_pos = scipy.zeros((n,))
        if len(ind_list) == len(name_list):
            for ind, name in zip(ind_list,name_list):
                motor_num = self.get_motor_num(name)
                next_pos[motor_num] = ind 
        elif len(ind_list) == 1:
            ind = ind_list[0]
            for name in name_list:
                motor_num = self.get_motor_num(name)
                next_pos[motor_num] = ind 
        else:
            raise ValueError, 'len(pos_list) must equal len(name_list) or 1'
        
        ramp_move = libmove_motor.get_ramp_moves(zero_pos,
                                               next_pos,
                                               vmax,
                                               accel,
                                               self.move_dt)
        ramp_move = libmove_motor.convert2int(ramp_move)
        end_pos, ret_val = libmove_motor.outscan_kine(ramp_move,config,self.move_dt)

        if noreturn == False:
            # Wait until done
            ans = raw_input('Press enter to return to starting position:')
            print 'returning to starting positon'
        
            ramp_move = libmove_motor.get_ramp_moves(zero_pos,
                                                   -next_pos,
                                                   vmax,
                                                   accel,
                                                   self.move_dt)
            ramp_move = libmove_motor.convert2int(ramp_move)
            end_pos, ret_val = libmove_motor.outscan_kine(ramp_move,config,self.move_dt)

    def num_motors(self):
        """
        Returns the number of motors
        """
        motor_num_list = libmove_motor.get_motor_num_list(self.motor_maps)
        return len(motor_num_list)

    def get_motor_num(self,name):
        """
        Returns motor number given the motor name
        """
        return self.motor_maps[name]['number']

    def get_motor_names(self):
        """
        Returns list of motor names sorted by motor number
        """
        motor_nums_and_names = [(self.get_motor_num(name),name) for name in self.motor_maps.keys()]
        motor_nums_and_names.sort()
        motor_names = [name for num,name in motor_nums_and_names]
        return motor_names


class Sixaxff(Sixaxff_Base):

    """
    Sixaxff force feedback device. In this version the kinematics are prescribed.
    """

    def __init__(self, 
                 run_params, 
                 motor_maps_file=None, 
                 sensor_cal_file=None, 
                 comedi_conf_file=None,
                 defaults_file=DFLT_CONFIG_FILE,
                 move_vmax=DFLT_MOVE_VMAX,
                 move_accel=DFLT_MOVE_ACCEL,
                 move_dt=DFLT_MOVE_DT,
                 pause_t=None):

        # Call parent class initialization method
        super(Sixaxff,self).__init__(
            run_params,
            motor_maps_file = motor_maps_file,
            sensor_cal_file = sensor_cal_file,
            comedi_conf_file = comedi_conf_file,
            defaults_file = defaults_file,
            move_vmax = move_vmax,
            move_accel = move_accel,
            move_dt = move_dt,
            pause_t = pause_t
        )

        # Custom initialization
        self.kine_deg = None
        self.t = None


    def run(self, kine_deg=None):
        """
        Run yaw force-feedback function.
        """
        if kine_deg == None:
            if self.kine_deg == None:
                raise RuntimeError, 'no kinematics to run'
            else:
                kine_deg = self.kine_deg
            
        config = self.config_dict

        # Move to kinematics starting positon
        zero_indpos_deg = libmove_motor.get_zero_indpos_deg(self.motor_maps)
        start_pos_deg = kine_deg[0,:]
        ramps_deg = libmove_motor.get_ramp_moves(zero_indpos_deg,
                                                 start_pos_deg,
                                                 self.move_vmax, 
                                                 self.move_accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)

        # Sleep to wait for force transients to die down - can effect autozero
        time.sleep(self.pause_t)

        # Run force-feed back function
        kine_ind = libmove_motor.deg2ind(kine_deg, self.motor_maps)
        t, pos, vel, torq, end_pos_ind = libsixaxff.sixaxff_c_wrapper(kine_ind, config)
        
        # Convert positin and velocity from radians to degrees
        pos = RAD2DEG*pos
        vel = RAD2DEG*vel

        # Extract filtered and raw torque data
        torq_flt = torq[:,0]
        torq_raw = torq[:,1]
        
        # Return to zero position
        end_pos_deg = libmove_motor.ind2deg(end_pos_ind,self.motor_maps)
        ramps_deg = libmove_motor.get_ramp_moves(end_pos_deg,
                                                 zero_indpos_deg,
                                                 self.move_vmax, 
                                                 self.move_accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)
        
        # Reset all pwm signal to there default positions. This is a kludge to help
        # w/ the fact that we seem to sometimes lose a bit of position in the pwm 
        # signals. I am not sure what is causing this - hardware or software. 
        # I will need to to some more tests after this next set of experiments.
        clkdirpwm.set_pwm_to_default(None)

        return t, pos, vel, torq_flt, torq_raw


    def set_zero_kine(self, T):
        config = self.config_dict
        dt = config['dt']
        num_motors = self.num_motors()
        t = scipy.arange(0.0, (T+dt)/dt)*dt
        kine = scipy.zeros((t.shape[0],num_motors))
        self.kine_deg = kine
        self.t = t


    ##############################################################################################
    # REPLACE WITH PITCH KINEMATICS FUNCITONS 
    ##############################################################################################

    def set_combo_kine(self, t, f, u_diff_aoa, u_stroke_rot, u_stroke_tilt, u_asym_vel,
                       amp_stroke, amp_rotation, k_stroke=DFLT_K_STROKE,k_rotation=DFLT_K_ROTATION,
                       rotation_offset=DFLT_ROTATION_OFFSET):

        # Create control inputs for wings 0 and 1
        u_diff_aoa_0, u_diff_aoa_1 = -u_diff_aoa, u_diff_aoa
        u_stroke_rot_0, u_stroke_rot_1 = -u_stroke_rot, u_stroke_rot
        u_stroke_tilt_0, u_stroke_tilt_1 = u_stroke_tilt, -u_stroke_tilt
        u_asym_vel_0, u_asym_vel_1 = 0.5 - u_asym_vel, 0.5 + u_asym_vel 

        # Set up base kinematics
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        kine_base_0 = scipy.zeros((t.shape[0],3))
        kine_base_1 = scipy.zeros((t.shape[0],3))

        kine_base_0[:,0] = get_stroke_angle(t, f, amp_stroke, u_asym_vel_0, k_stroke)
        kine_base_1[:,0] = get_stroke_angle(t, f, amp_stroke, u_asym_vel_1, k_stroke)
        kine_base_0[:,1] = get_deviation_angle(t, f, u_stroke_tilt_0) 
        kine_base_1[:,1] = get_deviation_angle(t, f, u_stroke_tilt_1) 
        kine_base_0[:,2] = get_rotation_angle(t, f, amp_rotation, u_diff_aoa_0 + ro_0, k_rotation)
        kine_base_1[:,2] = get_rotation_angle(t, f, amp_rotation, u_diff_aoa_1 + ro_1, k_rotation)

        # Rotate kinematics
        if type(u_stroke_rot) == float or type(u_stroke_rot) == int or type(u_stroke_rot) == scipy.float_:
            ax = scipy.array([1.0,0.0,0.0])
        else:
            ax = scipy.zeros((kine.shape[0],3))
            ax[:,0] = 1.0
        kine_base_0 = RAD2DEG*qarray.rotate_euler_array(DEG2RAD*kine_base_0, ax, DEG2RAD*u_stroke_rot_0)
        kine_base_1 = RAD2DEG*qarray.rotate_euler_array(DEG2RAD*kine_base_1, ax, DEG2RAD*u_stroke_rot_1)

        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        
        kine[:,s0] = kine_base_0[:,0]
        kine[:,s1] = kine_base_1[:,0]
        kine[:,d0] = kine_base_0[:,1] 
        kine[:,d1] = kine_base_1[:,1] 
        kine[:,r0] = kine_base_0[:,2] 
        kine[:,r1] = kine_base_1[:,2] 
        self.kine_deg = kine
        self.t = t

    def set_stroke_rot_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=DFLT_K_STROKE, 
                            k_rotation=DFLT_K_ROTATION, rotation_offset=DFLT_ROTATION_OFFSET):
        """
        Sets current kinematics to those with differential rotation of the stroke
        plane. 
        """
        u_0, u_1 = -u, u
        ro_0, ro_1 = -rotation_offset, rotation_offset

        # Set up base kinematics
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        kine_base_0 = scipy.zeros((t.shape[0],3))
        kine_base_1 = scipy.zeros((t.shape[0],3))

        kine_base_0[:,0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine_base_1[:,0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine_base_0[:,1] = get_deviation_angle(t, f, 0.0) 
        kine_base_1[:,1] = get_deviation_angle(t, f, 0.0) 
        kine_base_0[:,2] = get_rotation_angle(t, f, amp_rotation, ro_0, k_rotation)
        kine_base_1[:,2] = get_rotation_angle(t, f, amp_rotation, ro_1, k_rotation)

        # Rotate kinematics
        if type(u) == float or type(u) == int or type(u) == scipy.float_:
            ax = scipy.array([1.0,0.0,0.0])
        else:
            ax = scipy.zeros((kine.shape[0],3))
            ax[:,0] = 1.0
        kine_base_0 = RAD2DEG*qarray.rotate_euler_array(DEG2RAD*kine_base_0, ax, DEG2RAD*u_0)
        kine_base_1 = RAD2DEG*qarray.rotate_euler_array(DEG2RAD*kine_base_1, ax, DEG2RAD*u_1)

        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        
        kine[:,s0] = kine_base_0[:,0]
        kine[:,s1] = kine_base_1[:,0]
        kine[:,d0] = kine_base_0[:,1] 
        kine[:,d1] = kine_base_1[:,1] 
        kine[:,r0] = kine_base_0[:,2] 
        kine[:,r1] = kine_base_1[:,2] 
        self.kine_deg = kine
        self.t = t
    
    def set_stroke_tilt_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=DFLT_K_STROKE, 
                             k_rotation=DFLT_K_ROTATION, rotation_offset=DFLT_ROTATION_OFFSET):
        """
        Sets current kinemtics to those with differential tilt in stroke-plane angle determined by 
        control signal u.
        """
        u_0, u_1 = u, -u
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        kine[:,s0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,s1] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,r0] = get_rotation_angle(t, f, amp_rotation, ro_0, k_rotation)
        kine[:,r1] = get_rotation_angle(t, f, amp_rotation, ro_1, k_rotation)
        kine[:,d0] = get_deviation_angle(t, f, u_0) 
        kine[:,d1] = get_deviation_angle(t, f, u_1) 
        self.kine_deg = kine
        self.t = t

    def set_diff_aoa_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=DFLT_K_STROKE, 
                          k_rotation=DFLT_K_ROTATION, rotation_offset=DFLT_ROTATION_OFFSET):
        """
        Sets current kinematics to those with a differential angle of attach on the up stroke
        and downstroke. The amount of asymmetry is determined by the control input u.
        """
        u_0, u_1 = -u, u
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        kine[:,s0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,s1] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,r0] = get_rotation_angle(t, f, amp_rotation, u_0 + ro_0, k_rotation)
        kine[:,r1] = get_rotation_angle(t, f, amp_rotation, u_1 + ro_1, k_rotation)
        kine[:,d0] = get_deviation_angle(t, f, 0.0) 
        kine[:,d1] = get_deviation_angle(t, f, 0.0) 
        self.kine_deg = kine
        self.t = t

    def set_asym_stroke_vel_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=DFLT_K_STROKE,
                                 k_rotation=DFLT_K_ROTATION, rotation_offset=DFLT_ROTATION_OFFSET):
        """
        Sets current kinematics to those with asymmetric stroke velocity. The amount of velocity asymmetry
        is determined by the control input u.
        """
        u_0, u_1 = 0.5 - u, 0.5 + u 
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0], num_motors))
        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        kine[:,s0] = get_stroke_angle(t, f, amp_stroke, u_0, k_stroke)
        kine[:,s1] = get_stroke_angle(t, f, amp_stroke, u_1, k_stroke)
        kine[:,r0] = get_rotation_angle(t, f, amp_rotation, ro_0, k_rotation)
        kine[:,r1] = get_rotation_angle(t, f, amp_rotation, ro_1, k_rotation)
        kine[:,d0] = get_deviation_angle(t, f, 0.0) 
        kine[:,d1] = get_deviation_angle(t, f, 0.0) 
        self.kine_deg = kine
        self.t = t

    def set_yaw_to_const_vel(self,vel,accel,center=False):
        """
        Sets the kinematics of the yaw motor to constant velocity.
        """
        if self.t == None or self.kine_deg == None:
            raise RuntimeError, 'cannot set yaw to constant vel - no t or kine_deg'
        n = self.get_motor_num('yaw')
        self.kine_deg[:,n] = ramp_to_const_vel(self.t,vel,accel)
        if center == True:
            mid_pt = 0.5*(self.kine_deg[:,n].max() + self.kine_deg[:,n].min())
            self.kine_deg[:,n] = self.kine_deg[:,n] - mid_pt

    def set_yaw_to_ramp(self,x0,x1,vmax,a,startup_pause=False,startup_t=10):
        """
        Sets kinematics of the yaw motor to a point to point ramp
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        dt = self.config_dict['dt']
        ramp = libmove_motor.get_ramp(x0,x1,vmax,a,dt,output='ramp only')
        if startup_pause == True:
            n_pause = int(startup_t/dt)
            pause = ramp[0]*scipy.ones((n_pause,))
            ramp = scipy.concatenate((pause,ramp))
        if self.kine_deg == None:
            self.kine_deg = scipy.zeros((ramp.shape[0],num_motors))
            self.t = scipy.arange(0,ramp.shape[0])*dt
            self.kine_deg[:,n] = ramp
        else:
            raise RuntimeError, 'set_yaw_to_ramp w/ existing kinematics not implemented yet'

    def set_yaw_to_const(self,val):
        """
        Set yaw kinematics to constant position
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        self.kine_deg[:,n] = val*scipy.ones((self.kine_deg.shape[0],))

    def set_cable_test_kine(self,yaw_range,yaw_step,T_meas,dt):
        """
        Set kinematics for testing for cable effects 
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        t, yaw_kine, yaw_meas_pos = get_cable_test_kine(yaw_range,yaw_step,T_meas,dt)
        self.kine_deg = scipy.zeros((yaw_kine.shape[0],num_motors))
        self.kine_deg[:,n] = yaw_kine
        self.t = t
        return yaw_meas_pos
       
    def plot_kine(self,kine_deg=None):
        """
        Plot wing kinematics
        """
        if kine_deg == None:
            if self.kine_deg == None:
                raise RuntimeError, 'no kinematics to plot'
            else:
                kine_deg = self.kine_deg

        dt = self.config_dict['dt']
        t = scipy.arange(0.0,kine_deg.shape[0])*dt

        for motor, map in self.motor_maps.iteritems():
            ind = map['number']
            try:
                side_num = int(motor[-1])
                name = motor[:-2]
            except:
                side_num = None
                name = motor

            # Select figure    
            if side_num == 0:
                pylab.figure(1)
            elif side_num == 1:
                pylab.figure(2)
            else:
                pylab.figure(3)

            # Select subplot
            if name == 'stroke':
                pylab.subplot(3,1,1)
                pylab.title('side num: %d'%(side_num,))
                pylab.ylabel('stroke')
            elif name == 'rotation':
                pylab.subplot(3,1,2)
                pylab.ylabel('rotation')
            elif name == 'deviation':
                pylab.subplot(3,1,3)
                pylab.ylabel('deviation')
                pylab.ylabel('deviation')
            elif name == 'yaw':
                pylab.ylabel('yaw')
                pylab.xlabel('t')
            pylab.plot(t,kine_deg[:,ind])
        pylab.show()

    ################################################################################


def read_defaults(defaults_file=BORFRC_DIR):
    """
    Read borfrc defaults file for default motor maps, sensor calibration
    and comedi configuration files.
    """
    print defaults_file
    config = ConfigParser.ConfigParser()
    if os.path.exists(defaults_file):
        config.read(defaults_file)
    else:
        raise IOError, 'defaults file does not exist'

    if not 'sixaxff' in config.sections():
        raise RuntimeError, 'sixaxff section not found in defaults file: %s'%(defaults_file,)

    file_dict = {}
    for f in config.options('sixaxff'):
        val = config.get('sixaxff',f)
        file_dict[f] = val

    if not file_dict.has_key('motor_maps'):
        raise RuntimeError, 'defaults file does not specify motor maps file'
    if not file_dict.has_key('sensor_cal'):
        raise RuntimeError, 'defaults file does not specify sensor cal file'
    if not file_dict.has_key('comedi_conf'):
        raise RuntimeError, 'defaults file does not specify comedi conf file'

    motor_maps_file = file_dict['motor_maps']
    sensor_cal_file = file_dict['sensor_cal']
    comedi_conf_file = file_dict['comedi_conf']
    return motor_maps_file, sensor_cal_file, comedi_conf_file

def read_sensor_cal(sensor_cal_file,sensor_cal_dir=DFLT_SENSOR_CAL_DIR): 
    """
    Read sensor calibration file.
    """
    config = ConfigParser.ConfigParser()
    if os.path.exists(sensor_cal_file):
        config.read(sensor_cal_file)
    else:
        sensor_cal_file = os.path.join(sensor_cal_dir,sensor_cal_file)
        if os.path.exists(sensor_cal_file):
            config.read(sensor_cal_file)
        else:
            raise RuntimeError, 'sensor cal file not found'

    if not 'sensor_0' in config.sections():
        raise RuntimeError, 'sensor_0 in found in calibration file: %s'%(sensor_cal_file,)

    sensor_cal = {}
    for opt in config.options('sensor_0'):
        val = config.get('sensor_0', opt)
        sensor_cal[opt] = val

    sensor_cal['ain_chan'] = int(sensor_cal['ain_chan'])
    sensor_cal['calibration'] = float(sensor_cal['calibration'])
    return sensor_cal

def read_comedi_conf(comedi_conf_file, comedi_conf_dir=DFLT_COMEDI_CONF_DIR):
    """
    Read comedi configuration file.
    """
    config = ConfigParser.ConfigParser()
    if os.path.exists(comedi_conf_file):
        config.read(comedi_conf_file)
    else:
        comedi_conf_file = os.path.join(comedi_conf_dir,comedi_conf_file)
        if os.path.exists(comedi_conf_file):
            config.read(comedi_conf_file)
        else:
            raise RuntimeError, 'comedi conf file not found'

    if not 'device_0' in config.sections():
        raise RuntimeError, 'device_0 not found in comedi configuration file: %s'%(comedi_conf_file,)

    comedi_conf = {}
    for opt in config.options('device_0'):
        val = config.get('device_0', opt)
        comedi_conf[opt] = val

    comedi_conf['ain_subdev'] = int(comedi_conf['ain_subdev'])
    comedi_conf['dio_subdev'] = int(comedi_conf['dio_subdev'])
    comedi_conf['dio_disable'] = int(comedi_conf['dio_disable'])
    return comedi_conf
    

# Wing kinematics functions ------------------------------------------------------

def get_stroke_angle(t, f, amp, u, k, phase=0.5*PI):
    """
    Stroke position function w/ J. Wang's shape parameter. The control input 
    adds a asymmetric velicty to the stroke positon fuction.

    Arguments:
        t     = array of time points (s)
        f     = flapping frequency (Hz)
        amp   = stroke amplitude 
        u     = control input, scalar or array
        k     = shape parameter near 0 => sine wave, near 1=> sawtooth
    """
    f1=f/(2*u)
    f2=1/(2/f-1/f1)
    phi_A=amp/scipy.arcsin(k)*scipy.arcsin(k*scipy.sin(2*PI*f1*scipy.fmod(t, 1/f)+phase))
    phi_B=amp/scipy.arcsin(k)*scipy.arcsin(k*scipy.sin(2*PI*f2*(1/(2*f2)-1/(2*f1)+scipy.fmod(t,1/f))+phase))
    phi=phi_A*(scipy.fmod(t, 1/f)<1/(2*f1))+phi_B*(scipy.fmod(t, 1/f)>=1/(2*f1))
    return phi

def get_rotation_angle(t, f, amp, u, k, phase=0.0):
    """
    Rotation angle function w/ J. Wang's shape parameter. The control input shifts
    the mean rotation angle.

    Arguments:
        t     = array of time points 
        f     = flapping frequency
        amp   = rotation angle amplitude
        u     = control input, scalar or array
        k     = shape parameter, k near 0 implies sinewave, k large implies
                square wave.
    """
    phase = 0.0
    alpha=amp/scipy.tanh(k)*scipy.tanh(k*scipy.sin(2*PI*f*t+phase))+u
    return alpha

def get_deviation_angle(t, f, u, phase=0.0):
    """
    Deviation angle function w/ control input u.

    Arguments:
        t     = array of time points
        f     = flapping frequency
        amp   = deviation angle amplitude
        u     = control input, scalar or array
    """
    phase = 0.0
    theta = u*scipy.cos(2*PI*f*t + phase)
    return theta

def control_step(t,u0,u1,t0,t1,t2,t3):
    """
    Control function, has value u0 for t < t0, linearly transitions from u0 
    to u1 when t0 < t < t1, has value u1 when t1 < t < t2, linearly transistions
    from u1 to u0 when t2 < t < t3, and has value u0 when t > t3.
    """
    f = scipy.zeros(t.shape)
    # Masks for selecting time sections
    mask0 = t < t0
    mask1 = scipy.logical_and(t >= t0, t < t1)
    mask2 = scipy.logical_and(t >= t1, t < t2)
    mask3 = scipy.logical_and(t >= t2, t < t3)
    mask4 = t >= t3
    # Constants for linear transition regions
    a_01 = (u0 - u1)/(t0 - t1)
    a_23 = (u1 - u0)/(t2 - t3)
    b_01 = u0 - a_01*t0
    b_23 = u1 - a_23*t2
    # Assign functin values
    f[mask0] = u0*scipy.ones(t[mask0].shape)
    f[mask1] = a_01*t[mask1] + b_01
    f[mask2] = u1*scipy.ones(t[mask2].shape)
    f[mask3] = a_23*t[mask3] + b_23
    f[mask4] = u0*scipy.ones(t[mask4].shape)
    return f

def sqr_wave(t,amp,T,epsilon):
    """
    Generates a square wave with the given frequency and amplitude
    """
    f = scipy.zeros(t.shape)
    t_curr = 0.5*T
    cnt = 0
    while (t_curr - T < t[-1]):
        if cnt%2 == 0:
            val = amp
        else:
            val = -amp

        t0 = t_curr - 0.5*T 
        t1 = t_curr - 0.5*T + 0.5*epsilon
        mask0 = scipy.logical_and(t >= t0, t < t1)
        interp_func = scipy.interpolate.interp1d([t0,t1],[0,val])
        f[mask0] = interp_func(t[mask0])

        t0 = t_curr - 0.5*T + 0.5*epsilon
        t1 = t_curr - 0.5*epsilon
        mask1 = scipy.logical_and(t >= t0, t < t1)
        f[mask1] = val

        t0 = t_curr - 0.5*epsilon
        t1 = t_curr
        mask2 = scipy.logical_and(t >= t0, t < t1)
        interp_func = scipy.interpolate.interp1d([t0,t1],[val,0])
        f[mask2] = interp_func(t[mask2])

        t_curr += 0.5*T
        cnt += 1
    return f

def step_func(t,mag,t_start,t_stop,epsilon):
    """
    Generates a step function with given magnitude starting at t_start
    and finishing at t_stop.
    """
    
    if type(t) == scipy.ndarray:
        vals = scipy.zeros(t.shape)
        for i in range(t.shape[0]):
            vals[i] = step_func(t[i],mag,t_start,t_stop,epsilon)
        return vals
    else:
        if (t > t_start) and (t < t_start + epsilon):
            interp_func = scipy.interpolate.interp1d([t_start,t_start+epsilon],[0,mag])
            return interp_func(t)
        elif (t >= t_start + epsilon) and (t <= t_stop):
            return mag
        elif (t > t_stop) and (t < t_stop + epsilon):
            interp_func = scipy.interpolate.interp1d([t_stop,t_stop+epsilon],[mag,0])
            return interp_func(t)
        else:
            return 0

def approx_step_func(t,t0,u0,delta_t):
    """
    Approximate step function. Transitions from 0 to u0 w/ transition midpoint
    t0. The transition is linear starting at t0-delta_t and ending at
    t0+delta_t.

    Inputs:
      t       = time points
      t0      = midpoint of step transition
      u0      = step height
      delta_t = transition width.

    Returns: array of function values evaluated at the points in t 
    """

    t_orig_shape = t.shape
    if len(t.shape) == 2:
        assert ((t.shape[0] == 1) or (t.shape[1] == 1)), 'array shape must be (n,), (n,1) or (1,n)'
        if t.shape[1] == 1:
            tt = scipy.reshape(t,(t.shape[0],))
        else:
            tt =scipy.reshape(t,(t.shape[1],))
    else:
        tt = t

    mask0 = tt<(t0-0.5*delta_t)
    mask1 = scipy.logical_and(tt>=(t0-0.5*delta_t), tt<=(t0+0.5*delta_t))
    mask2 = tt>(t0+0.5*delta_t)
    vals = scipy.zeros(tt.shape)

    vals[mask0] = 0.0
    vals[mask1] = (u0/delta_t)*tt[mask1] - (u0/delta_t)*(t0 - 0.5*delta_t)
    vals[mask2] = u0
    vals = scipy.reshape(vals,t_orig_shape)
    return vals

def ramp_to_const_vel(t,vel,accel):
    """
    Generates a ramp trajectory to constant velocity.
    """
    accel = abs(accel)
    if vel < 0:
        accel = -accel
    x = scipy.zeros(t.shape)
    t_accel = vel/accel
    mask0 = t < t_accel
    mask1 = t >= t_accel
    x[mask0] = accel*t**2
    x[mask1] = vel*t + accel*(t_accel**2)
    return x

def get_cable_test_kine(yaw_range,yaw_step,T_meas,dt,max_vel=10.0,accel=10.0):
    """
    Get yaw kinematics for testing for cable effects 
    
    Inputs:
        yaw_range = range for measurement sample points
        yaw_step  = step size between measurement samples
        T_meas    = sample duration
        dt        = sample rate

    Output:
        t         = array of time points 
        yaw_kine  = array of yaw kinematics
        pos_array = array of yaw measurement positions
    """
    N_meas = int(T_meas/dt)
    pos_array = scipy.arange(-0.5*yaw_range,0.5*yaw_range+yaw_step,yaw_step)
    pos_last = 0.0
    for i, pos in enumerate(pos_array):
        # Create ramp to position
        ramp = libmove_motor.get_ramp(pos_last, pos, max_vel, accel, dt) 
        if i == 0:
            yaw_kine = ramp
        else:
            yaw_kine = scipy.concatenate((yaw_kine,ramp))
        # Dwell at positoin for measurement
        dwell = pos*scipy.ones((N_meas,))
        yaw_kine = scipy.concatenate((yaw_kine,dwell))
        pos_last =  pos
    t = scipy.arange(0,yaw_kine.shape[0])*dt
    return t, yaw_kine, pos_array

def resample(x,n):
    """
    Resample data in array x with step size n.
    """
    if len(x.shape) == 1:
        return x[0:-1:n]
    else:
        return x[0:-1:n,:]

def resample_tuple(data_tuple,n):
    return tuple([resample(x,n) for x in data_tuple])

def resample_dict(data_dict,n):
    data_dict_new = {}
    for k,v in data_dict.iteritems():
        data_dict_new[k] = resample(v,n)
    return data_dict_new



# ------------------------------------------------------------------------------
# Command line utilties

class sixaxff_cmd_line:

    zero_help = """\
command: zero 

usage: %prog [options] zero 


Zeroing routine for pitch and stroke position motors. Run and follow the
instructions.
"""

    move2pos_help = """\
command: move2pos 

usage: %prog [options] move2pos motor_0, ..., motor_k, pos

       or

       %prog [options] move2pos motor_0, ..., motor_k, pos_0, ... pos_k

Move motors, specified by name, to the given positions specified in degrees. If
one position value is given then all motors specified are moved to that position. 
Otherwise the number of positions specified must equal the number of motors and 
each motor is moved to the corresponding position which is determined by the order.  
By default, after the move, the routine will prompt the user press enter and then 
return  the motors to the zero position unless the noreturn option is specified.  
"""

    move2zero_help = """\
command: move2zero 

usage: %prog [options] move2zero 

Move motors to zero (degree) position. By default, after the move, the routine
will prompt the user press enter and then return  the motors to the zero
position unless the noreturn option is specified.  
"""

    move_by_ind_help = """\
command: move-by-ind 

usage: %prog [options] move-by-ind motor_0, ..., motor_k, pos
 
       or

       %prog [options] move-by-ind motor_0, ..., motor_k, pos_0, ..., pos_k

Move motors, specified by name, by the given number of indices. If one value 
is given then all motors specified are moved by that number of indices. Otherwise 
the number of values specified must equal the number of motors and each motor
is moved by the corresponding value which is determined by the order. By 
default, after the move, the routine will prompt the user press enter and then 
return  the motors to the zero position unless the noreturn option is specified.
"""

    reset_pwm_help = """\
command: reset-pwm

usage: %prog [options] reset-pwm 

Resets all pwm signal to their default (start-up) positions.
"""

    motor_names_help = """\
command: motor-names

usage: %prog [options] motor-names

Displays a list of all motors names.
"""

    sensor_cal_help = """\
command: sensor-cal
    
This command has not been implemented yet.
"""

    help_help = """\
command: help

usage: %prog [options] help [COMMAND]

Prints help information. If the optional argument COMMAND is not given
then general usage information for the %prog is displayed. If a specific 
command, COMMAND, is given then help for that command will be displayed.

Examples:
 %prog help         # prints general usage information
 %prog help status  # prints help for the status command 
"""

    usage = """%prog [OPTION] command [arg0, ...] 

%prog is a command line utility providing simple positioning and other useful 
commands for working with the sixaxff controlled robot. This utitily is intended 
for aid the experimenter with zeroing and calibration.

Commands:

    zero          - zero yaw and stroke position motors
    move2pos      - move motors to specified positions (in degrees)
    move2zero     - move motors to zero position (in degrees)
    move-by-ind   - move motor by specified number of indices 
    reset-pwm     - reset pwm output to their default positions
    motor-names   - list motor names
    sensor-cal    - calibrate torque sensor
    help          - get help  
     
* To get help for a specific command type: %prog help COMMAND
"""

    def __init__(self):
        self.cmd_table = {
            'zero' : self.zero,
            'move2pos': self.move2pos,
            'move2zero' : self.move2zero,
            'move-by-ind': self.move_by_ind,
            'reset-pwm': self.reset_pwm, 
            'motor-names': self.motor_names,
            'sensor-cal': self.sensor_cal,
            'help': self.help,
        }
        self.help_table = {
            'zero' : sixaxff_cmd_line.zero_help,
            'move2pos': sixaxff_cmd_line.move2pos_help,
            'move2zero': sixaxff_cmd_line.move2zero_help,
            'move-by-ind': sixaxff_cmd_line.move_by_ind_help,
            'reset-pwm': sixaxff_cmd_line.reset_pwm_help,
            'motor-names': sixaxff_cmd_line.motor_names_help,
            'sensor-cal': sixaxff_cmd_line.sensor_cal_help,
            'help': sixaxff_cmd_line.help_help,
        }

        self.progname = os.path.split(sys.argv[0])[1]
        self.options_cmd, self.args, self.parser = self.parse_cmd_options()
        
        self.run_params = {
                'dt'                : 1.0/5000.0, 
                'yaw_inertia'       : 3.22,
                'yaw_damping'       : 0.0,
                'yaw_torq_lim'      : 0.5,
                'yaw_torq_deadband' : 1.5,
                'yaw_filt_lpcut'    : 3.0,
                'yaw_filt_hpcut'    : 0.0,
                'yaw_ain_zero_dt'   : 0.01,
                'yaw_ain_zero_num'  : 500, 
                'integ_type'        : libsixaxff.INTEG_RKUTTA,
                'startup_t'         : 0.0,
                'ff_flag'           : libsixaxff.FF_ON,
        }
        

    def parse_cmd_options(self):
        """
        Parse command line options 
        """

        parser = optparse.OptionParser(usage=sixaxff_cmd_line.usage)

        parser.add_option('-v', '--verbose',
                               action='store_true',
                               dest = 'verbose',
                               help = 'verbose mode - print additional information',
                               default = False)

        parser.add_option('-n', '--noreturn',
                               action='store_true',
                               dest = 'noreturn',
                               help = 'noreturn mode - do not return to starting positin after making move',
                               default = False)

        options, args = parser.parse_args()

        # Convert options to dictionary
        options = options.__dict__
        return options, args, parser

    def run(self):
        """
        Run command given on the command line
        """
        if len(self.args) == 0:
            print "ERROR: no command given"
            print 
            self.parser.print_help()
            sys.exit(0)
        else:
            cmd_str = self.args[0]
            try:
                cmd = self.cmd_table[cmd_str]
            except KeyError:
                print "ERROR: command, '%s', not found"%(cmd_str,)
                print 
                self.parser.print_help()
                sys.exit(1)
            # Run command
            cmd()
        return

    def help(self):
        """
        Print help messages
        """
        if len(self.args)==1:
            self.parser.print_help()
        elif len(self.args)==2:
            cmd_str = self.args[1].lower()
            try:
                help_str = self.help_table[cmd_str]
            except KeyError:
                print "ERROR: can't get help unkown command"
                sys.exit(1)
            print help_str.replace('%prog', self.progname)
        else:
            print "ERROR: too many arguments for command help"
            sys.exit(1)

    def move2pos(self):
        self.args.remove('move2pos')
        motor_names, values = self.get_motors_and_value_from_args()
        if self.options_cmd['verbose'] == True:
            if len(motor_names) == len(values):
                print 'moving motors %s to positions %s'%(motor_names,values)
            else:
                print 'moving motors %s to position %s'%(motor_names,values)
        sixaxff = Yawff(self.run_params)
        sixaxff.move_to_pos(motor_names, values, noreturn = self.options_cmd['noreturn'])

    def move2zero(self):
        self.args.remove('move2zero')
        if self.options_cmd['verbose'] == True:
            if len(motor_names) == len(values):
                print 'moving motors %s to positions %s'%(motor_names,values)
            else:
                print 'moving motors %s to position %s'%(motor_names,values)
        sixaxff = Yawff(self.run_params)
        motor_names = sixaxff.get_motor_names()
        pos = scipy.zeros((len(motor_names),))
        sixaxff.move_to_pos(motor_names, pos, noreturn = self.options_cmd['noreturn'])

    def zero(self):

        print 
        print 'zeroing yaw and stroke position motors'
        print '-'*60
        print 

        print '** Step 1: adjust yaw motor until system is squared with tank'
        print 
        print 'At the prompt enter the angle adjustments to yaw in degrees.'
        print "Enter 'done' when system is squared."

        sixaxff = Yawff(self.run_params)
        motor_names = sixaxff.get_motor_names()
        pos = scipy.zeros((len(motor_names),))
        sixaxff.move_to_pos(motor_names, pos,noreturn=True,at_zero_ind=True)
        self.zero_adj_loop(sixaxff,'yaw')

        print
        print 'Rotating system by 90 degrees'
        pos = scipy.zeros((len(motor_names),))
        pos[sixaxff.get_motor_num('yaw')] = 90.0
        sixaxff.move_to_pos(motor_names,pos,noreturn=True,at_zero_ind=False)
        print 

        print '** Step 2: adjust position of stroke_0 until squared with tank'
        print
        print 'at prompt enter the position adjustments in motor indices'
        print "enter 'done' when wing is square"
        self.zero_adj_loop(sixaxff, 'stroke_0')

        print 
        print 'rotating system to -180 degrees'
        pos = scipy.zeros((len(motor_names),))
        pos[sixaxff.get_motor_num('yaw')] = -180.0 
        sixaxff.move_to_pos(motor_names,pos,noreturn=True,at_zero_ind=False)
        print 

        print '** Step 3: adjust position of stroke_1 until squared with tank'
        print 
        print 'at prompt enter position adjustments in indices'
        print "enter 'done' when wing is square"
        self.zero_adj_loop(sixaxff,'stroke_1')

        print 
        print 'returning to zero position'
        pos = scipy.zeros((len(motor_names),))
        pos[sixaxff.get_motor_num('yaw')] = 90.0 
        sixaxff.move_to_pos(motor_names,pos,noreturn=True,at_zero_ind=False)

        print 'returning RC motors to zero index position'
        pos = libmove_motor.get_zero_indpos_deg(sixaxff.motor_maps)
        sixaxff.move_to_pos(motor_names,pos,noreturn=True,at_zero_ind=False)

    def zero_adj_loop(self, sixaxff, motor_name):
        """
        Yaw and stroke position zeroing routine adjustment loop
        """
        motor_names = sixaxff.get_motor_names()
        pos = scipy.zeros((len(motor_names),))

        done = False
        while not done:

            print
            ans = raw_input('adj> ')
            if ans.lower() == 'done':
                done = True
                continue

            try:
                val = float(ans)
            except ValueError:
                print 'unable to convert entry to float - please try again'
                continue

            print val
            pos[sixaxff.get_motor_num(motor_name)] = val
            sixaxff.move_to_pos(motor_names,pos,noreturn=True,at_zero_ind=False)

        return 



    def move_by_ind(self):
        self.args.remove('move-by-ind')
        motor_names, values = self.get_motors_and_value_from_args()
        if self.options_cmd['verbose'] == True:
            print 'moving motors %s by indices %s'%(motor_names,values)
        sixaxff = Yawff(self.run_params)
        sixaxff.move_by_ind(motor_names, values, noreturn = self.options_cmd['noreturn'])

    def reset_pwm(self):
        """
        Reset pwm outputs to default values.
        """
        if self.options_cmd['verbose'] == True:
            print 'resetting pwms to default values' 
        clkdirpwm.set_pwm_to_default(None)

    def motor_names(self):
        motor_names = self.get_motor_names()
        for name in motor_names:
            print ' ', name

    def sensor_cal(self):
        print  'sensor_cal - not implemented yet'

    def get_motor_names(self):
        sixaxff = Yawff(self.run_params)
        motor_names = sixaxff.motor_maps.keys()
        motor_names.sort()
        return motor_names

    def get_motors_and_value_from_args(self):
        motor_names = self.get_motor_names()
        motors_in_args = [x for x in self.args if x in motor_names]
        others_in_args = [x for x in self.args if x not in motor_names]
        if len(others_in_args) == 1:
            v_str = others_in_args[0]
            v = self.get_float_value(v_str)
            values = [v]
        elif len(others_in_args) == len(motors_in_args):
            values = []
            for v_str in others_in_args:
                v = self.get_float_value(v_str)
                values.append(v)
        else:
            print 'ERROR: incorrect number of angle values -  must equal 1 or number of motor names given' 
            sys.exit(1)
        return motors_in_args, values

    def get_float_value(self,v_str):
        if v_str[0] == 'n':
            v_str = v_str[1:]
            sign = -1
        else:
            sign = 1
        try:
            v = sign*float(v_str)
        except ValueError:
            print 'ERROR: cannot cast value to float'
            sys.exit(1)
        return v


def cmd_line_main():
    """
    Command line interface entry point.
    """
    cmd_line = sixaxff_cmd_line()
    cmd_line.run()
    pass
