/*---------------------------------------------------------------------
  sixaxff
  Copyright (C) William Dickson, 2008.

  wbd@caltech.edu
  www.willdickson.com

  Released under the LGPL Licence, Version 3

  This file is part of sixaxff.

  sixaxff is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  sixaxff is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
  Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with sixaxff.  If not, see
  <http://www.gnu.org/licenses/>.

  ----------------------------------------------------------------------
  util.c

Puropse: contains vairous utility functions used by the sixaxff
library.

Functions:

integrator         = 2nd order integrator
dynamics_func      = function for yaw dynamics
lowpass_filt1      = first order lowpass filter
highpass_filt1     = first order highpas filter
init_array         = initializes array object
free_array         = frees array object
set_array_val      = sets array element value
get_array_val      = gets array  element value
print_array        = prints array values
print_config       = prints configuration structure
print_err_msg      = prints error messages
fflush_printf      = printf followed by fflush(stdout)
get_max_dt         = returns maximum allowed real-time period
get_min_dt         = returns minimum allowed real-time period
get_clock_hi_ns    = returns the clock high time in ns

Author: Will Dickson
----------------------------------------------------------------------- */
#include "sixaxff.h"
#include "util.h"

// Function prototypes
state_t dynamics_func(
        state_t state_in, 
        float force, 
        float mass, 
        float damping
        );

// ----------------------------------------------------------------
// Function: integrator
//
// Purpose: integrator for turning yaw torque into position. 
// Currently can select bewteen 1st order Euler method and 4-5th 
// order Runge-Kutta. 
//
// Arguments:
//   state_curr  = dynamic state (position + velocity) at current 
//                 time step
//   state_next  = dynamic state a the next time step (output) 
//   force       = external force or torque 
//   mass        = mass or moment of inertia
//   damping     = damping constant
//   dt          = time step
//   method      = integration method (INTEG_EULER or INTEG_RKUTTA)
//
// Return: SUCCESS or FAIL
// 
// ----------------------------------------------------------------
int integrator(
        state_t *state,
        float force, 
        float mass, 
        float damping, 
        float dt, 
        int method
        )
{
    state_t x;
    state_t k0,k1,k2,k3;
    int ret_flag = SUCCESS;

    // Set previous state to current state
    state -> pos_prev = state -> pos;
    state -> vel_prev = state -> vel;

    switch (method) {

        case INTEG_RKUTTA: // Integrate using Runge-Kutta method

            x.pos = state -> pos;
            x.vel = state -> vel;
            k0 = dynamics_func(x,force,mass,damping);

            x.pos = state -> pos + k0.pos*dt/2.0;
            x.vel = state -> vel + k0.vel*dt/2.0;
            k1 = dynamics_func(x,force,mass,damping);

            x.pos = state -> pos + k1.pos*dt/2.0;
            x.vel = state -> vel + k1.vel*dt/2.0;
            k2 = dynamics_func(x,force,mass,damping);

            x.pos = state -> pos + k2.pos*dt;
            x.vel = state -> vel + k2.vel*dt;
            k3 = dynamics_func(x,force,mass,damping);

            state -> pos = state -> pos + (dt/6.0)*(k0.pos + 2.0*k1.pos + 2.0*k2.pos + k3.pos);
            state -> vel = state -> vel + (dt/6.0)*(k0.vel + 2.0*k1.vel + 2.0*k2.vel + k3.vel);
            break;

        case INTEG_EULER: // Integrate using Euler method

            x.pos = state -> pos;
            x.vel = state -> vel;
            k0 = dynamics_func(x,force,mass,damping);
            state -> pos = state -> pos + dt*k0.pos;
            state -> vel = state -> vel + dt*k0.vel;
            break;

        default: // Error unkown integration method

            PRINT_ERR_MSG("unknown integration method");
            ret_flag = FAIL;
            break;

    }
    return ret_flag;

} 

// ----------------------------------------------------------------
// Function: dynamics_func
//
// Purpose: implements function governing the dynamics of the 
// system, i.e., the function f such that  dx/dt = f(x,t) where
// x is the system state.  
// 
// ----------------------------------------------------------------
state_t dynamics_func(state_t state_in, float force, float mass, float damping)
{
    state_t state_out;
    state_out.pos = state_in.vel;
    state_out.vel = force/mass - (damping/mass)*state_in.vel;
    state_out.pos_prev = state_in.pos; 
    state_out.vel_prev = state_in.vel;
    return state_out;
}


// ----------------------------------------------------------------
// Function: lowpass_filt1
//
// Purpose: Discrete implementation of a simple first order lowpass 
// filter.
//
// Arguments:
// 
//   x       = current input value
//   y_old   = last output value
//   f_cut   = cutoff frequency in (Hz)
//   dt      = Update time step
//
// Return: new filter output y value
// 
// ----------------------------------------------------------------
float lowpass_filt1(float x,float y_old, float f_cut, float dt)
{
    float tc = 1.0/(2.0*M_PI*f_cut);
    float y_new;
    float alpha;
    alpha = dt/(tc + dt);
    y_new = alpha*x  + (1.0 - alpha)*y_old;
    return y_new;
}

// -----------------------------------------------------------------
// Function: highpass_filt1
//
// Purpose: Discrete implementation of a simple first order highpass
// filter.
//
// Arguments:
//
// Return filter ouput value.
//
// -----------------------------------------------------------------
float highpass_filt1(float dx,  float y_old, float f_cut, float dt)
{
    float tc = 1.0/(2.0*M_PI*f_cut);
    float y_new;
    float alpha;
    alpha = tc/(tc + dt);
    y_new = alpha*y_old + alpha*dx;
    return y_new;
}

// ----------------------------------------------------------------
// Function: get_array_val
//
// Puropse: gets value of array at given row and column
//
// -----------------------------------------------------------------
int get_array_val(array_t array, int row, int col, void *val)
{
    int s0, s1;
    int *iptr;
    float *fptr;
    double *dptr;
    int rtn_val = SUCCESS;

    // Check row and col ranges
    if ((row < 0) || (row >= array.nrow)) {
        PRINT_ERR_MSG("row out of range");
        return FAIL;
    }
    if ((col < 0) || (col >= array.ncol)) {
        PRINT_ERR_MSG("col out of range");
        return FAIL;
    }

    // Get array value
    s0 = array.s0;
    s1 = array.s1;

    switch(array.type) {

        case INT_ARRAY:
            iptr = val;
            *iptr = *((int*)(array.data + row*s0 + col*s1));
            break;

        case FLT_ARRAY:
            fptr = val;
            *fptr = *((float*)(array.data + row*s0 + col*s1));
            //printf("get_array_val: FLT_ARRAY\n");
            break;

        case DBL_ARRAY:
            dptr = val;
            *dptr = *((double*)(array.data + row*s0 + col*s1));
            //printf("get_array_val: DBL_ARRAY\n");
            break;

        default:
            PRINT_ERR_MSG("unknown array type");
            rtn_val = FAIL;
            break;

    }
    return rtn_val;
}

// -------------------------------------------------------------------
// Function: set_array_val
//
// Purpose: sets the value of an array at the given row
// and column.
//
// --------------------------------------------------------------------
int set_array_val(array_t array, int row, int col, void *val)
{
    int s0, s1;
    int *iptr;
    float *fptr;
    double *dptr;
    int rtn_val = SUCCESS;

    // Check row and col ranges
    if ((row < 0) || (row >= array.nrow)) {
        PRINT_ERR_MSG("row out of range");
        return FAIL;
    }
    if ((col < 0) || (col >= array.ncol)) {
        PRINT_ERR_MSG("col out of range");
        return FAIL;
    }

    // Set array value
    s0 = array.s0;
    s1 = array.s1;

    switch(array.type) {

        case INT_ARRAY:
            iptr = val;
            *((int*)((array.data) + row*s0 + col*s1)) = *iptr;
            break;

        case FLT_ARRAY:
            fptr = val;
            *((float*)((array.data) + row*s0 + col*s1)) = *fptr;    
            break;

        case DBL_ARRAY:
            dptr = val;
            *((double*)((array.data) + row*s0 + col*s1)) = *dptr;    
            break;

        default:
            PRINT_ERR_MSG("unknown array type");
            rtn_val = FAIL;
            break;

    }
    return rtn_val;
}
// -----------------------------------------------------
// Function: print_array
//
// Purpose: prints array values
// -----------------------------------------------------
void print_array(array_t array)
{
    int i,j;
    int ival;
    float fval;
    double dval;

    printf("size: (%d,%d)\n", array.nrow,array.ncol);
    printf("type: ");
    switch(array.type) {
        case INT_ARRAY:
            printf("int\n");
            break;
        case FLT_ARRAY:
            printf("float\n");
            break;
        case DBL_ARRAY:
            printf("double\n");
            break;
        default:
            break;
    }

    for (i=0; i<array.nrow; i++) {
        for (j=0; j<array.ncol; j++) {

            printf("i: %d, ", i);

            switch(array.type) {

                case INT_ARRAY:
                    get_array_val(array,i,j,&ival);
                    printf("%d, ",ival);
                    break;

                case FLT_ARRAY:
                    get_array_val(array,i,j,&fval);
                    printf("%f, ",fval);
                    break;

                case DBL_ARRAY:
                    get_array_val(array,i,j,&dval);
                    printf("%f, ",dval);
                    break;

                default:
                    PRINT_ERR_MSG("unknown array type");
                    break;

            }
        }
        printf("\n");
    }
    return;
}

// -----------------------------------------------------
// Function: init_array
//
// Purpose: initializes an array structure w/ the given
// number of rows and columns
//
// -----------------------------------------------------
int init_array(array_t *array, int nrow, int ncol, int type)
{
    size_t size;

    // Check dimensions
    if (nrow <= 0) {
        PRINT_ERR_MSG("nrow <= 0");
        return FAIL;
    }
    if (ncol <= 0) {
        PRINT_ERR_MSG("ncol <= 0");
        return FAIL;
    }

    // Determine size based on type
    switch(type){

        case INT_ARRAY:
            size = sizeof(int);
            break;

        case FLT_ARRAY:
            size = sizeof(float);
            break;

        case DBL_ARRAY:
            size = sizeof(double);
            break;

        default:
            PRINT_ERR_MSG("unknown type");
            return FAIL;
            break;
    }

    // Allocate array
    array -> nrow = nrow;
    array -> ncol = ncol;
    array -> s0 = (array->ncol)*size;
    array -> s1 = size;
    array -> data = calloc((array->nrow)*(array->ncol), size);
    array -> type = type;

    if (array->data == NULL) {
        PRINT_ERR_MSG("unable to allocate memory");
        return FAIL;
    }
    else {
        return SUCCESS;
    }

}

// ------------------------------------------------------
// Function: free_array
//
// Purpose: free array memory
//
// ------------------------------------------------------
void free_array(array_t *array)
{
    if (array->data != NULL) {
        free(array -> data);
        array -> data = NULL;
        array -> nrow = 0;
        array -> ncol = 0;
        array -> s0 = 0;
        array -> s1 = 0;
        array -> type = EMPTY_ARRAY;
    }
}

// ----------------------------------------------------------------
// Function: print_config
//
// Purpose: prints configuration
//
// ----------------------------------------------------------------
void print_config(config_t config)
{
    int i;

    printf("\n              configuration\n");
    printf(" ------------------------------------------------\n");
    printf("  dev0_name:           %s\n", config.dev_name[0]);
    printf("  dev1_name:           %s\n", config.dev_name[1]);
    printf("  ain_dev:             %d\n", config.ain_dev);
    printf("  ain_subdev:          %d\n", config.ain_subdev);
    printf("  ain_zero_num:        %d\n", config.ain_zero_num);
    printf("  ain_zero_dt:         %f\n", config.ain_zero_dt);
    printf("  ain_filt_lpcut:      %f\n", config.ain_filt_lpcut);
    printf("  cal_file_path:       %s\n", config.cal_file_path);
    printf("  dio_dev:             %d\n", config.dio_dev);
    printf("  dio_sub_dev:         %d\n", config.dio_subdev);

    // Print dio clock
    printf("  dio_clk:             {");
    for (i=0;i<config.num_motor;i++) {
        printf("%d", config.dio_clk[i]);
        if (i < config.num_motor-1) printf(", ");
    }
    printf("}\n");

    // Print dio direction
    printf("  dio_dir:             {");
    for (i=0;i<config.num_motor;i++) {
        printf("%d", config.dio_dir[i]);
        if (i < config.num_motor-1) printf(", ");
    }
    printf("}\n");

    // Print kine_map
    printf("  kine_map:            {");
    for (i=0;i<config.num_motor;i++) {
        printf("%d", config.kine_map[i]);
        if (i < config.num_motor-1) printf(", ");
    }
    printf("}\n");

    // Print kinematics labels
    printf("  kine_label:          ");
    for (i=0; i<config.num_motor; i++) {
        if (i>0) printf("                       ");                      
        printf("%d  %s\n", i, config.kine_label[i]);
    }
    printf("  num_motor:           %d\n", config.num_motor);

    // Print force feedback axes, ff_motor
    printf("  ff_motor:            {");
    for (i=0; i<NUM_FF;i++) {
        printf("%d", config.ff_motor[i]);
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");

    // Print force/torque for force feedback, ff_ft
    printf("  ff_ft:               {");
    for (i=0; i<NUM_FF; i++) {
        printf("%d", config.ff_ft[i]);
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");

    // Print basic 6 axis sensor tool transform
    printf("  ff_basic_tooltrans:  {");
    for (i=0; i<6; i++) {
        printf("%f", config.ff_basic_tooltrans[i]);
        if (i<5) printf(", ");
    }
    printf("}\n");

    // Print dynamic 6 axis sensor tool transform 
    printf("  ff_dyam_tooltrans:   {");
    for (i=0; i<3; i++) {
        printf("%d",config.ff_dynam_tooltrans[i]);
        if (i<2) printf(", ");
    }
    printf("}\n");

    // Print masses for force feedback model
    printf("  ff_mass:             {");
    for (i=0; i<NUM_FF; i++) {
        printf("%f", config.ff_mass[i]);
        if (i<NUM_FF-1) printf(", ");
    } 
    printf("}\n");

    // Print conversion from motor indices to user axis units
    printf("  ff_ind2unit:         {");
    for (i=0; i<NUM_FF; i++) {
        printf("%f", config.ff_ind2unit[i]);
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");

    // Print axes units
    printf("  ff_axesunits:        {");
    for (i=0; i<NUM_FF; i++) {
        printf("%s",config.ff_axesunits[i]);
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");
    
    // Print damping terms
    printf("  ff_damping:          {");
    for (i=0; i<NUM_FF; i++) {
        printf("%f",config.ff_damping[i]);
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");

    // Print force feedback flags
    printf("  ff_flag:             {");
    for (i=0; i<NUM_FF; i++) {
        if (config.ff_flag[i]  == FF_ON) {
            printf("FF_ON");
        }
        else if (config.ff_flag[i] == FF_OFF) {
            printf("FF_OFF");
        }
        else {
            printf("UNKNOWN");

        }
        if (i<NUM_FF-1) printf(", ");
    }
    printf("}\n");

    // Print integrator type
    printf("  integ_type:          ");
    if (config.ff_integ_type == INTEG_RKUTTA) {
        printf("Runge-Kutta\n");
    }
    else if (config.ff_integ_type == INTEG_EULER) {
        printf("Euler\n");
    }
    else {
        printf("unkown\n");
    }

    printf("  dt:                  %f\n", NS2S*config.dt);
    printf("  startup_t:           %f\n", config.startup_t);
    printf(" ------------------------------------------------\n");
    printf("\n");

    return;
}

// -----------------------------------------------------------------
// Function: print_err_msg
//
// Purpose: prints simple error message
// -----------------------------------------------------------------
void print_err_msg(const char *file, int line, const char *func, char *err_msg)
{
    fprintf(stderr, "%s:%d, %s, Error: %s\n",file, line, func, err_msg);
    return;
}

// -----------------------------------------------------------------
// Function: fflush_printf
//
// Purpose: print and flush stream
// -----------------------------------------------------------------
void fflush_printf(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    vfprintf(stdout, format, ap);
    va_end(ap);
    fflush(stdout);
    return;
}

// Simple functions for getting constants to python ctypes interface
int define_max_motor(void) {return MAX_MOTOR;};
int define_max_dt(void) {return MAX_DT_NS;};
int define_min_dt(void) {return MIN_DT_NS;};
int define_clock_hi_ns(void) {return CLOCK_HI_NS;};
int define_integ_euler(void) {return INTEG_EULER;};
int define_integ_rkutta(void) {return INTEG_RKUTTA;};
int define_integ_unknown(void) {return INTEG_UNKNOWN;};
int define_empty_array(void) {return EMPTY_ARRAY;};
int define_int_array(void) {return INT_ARRAY;};
int define_flt_array(void) {return FLT_ARRAY;};
int define_dbl_array(void) {return DBL_ARRAY;};
int define_unknown_array(void) {return UNKNOWN_ARRAY;};
int define_success(void) {return SUCCESS;};
int define_fail(void) {return FAIL;};
int define_ff_on(void) {return FF_ON;};
int define_ff_off(void) {return FF_OFF;};
int define_num_ff(void) {return NUM_FF;};
