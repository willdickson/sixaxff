#!/usr/bin/env python
import scipy
import pylab
import libsixaxff
import cPickle as pickle

RAD2DEG = 180.0/scipy.pi

run_param = {
    'ff_mass' : (10.0, 1.0), 
}

dev = libsixaxff.Sixaxff(run_param)

if 0:
    dev.print_config()
else:
    kine = scipy.zeros((100000,dev.num_motors()))
    t, pos, vel, ft = dev.run(kine)

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
