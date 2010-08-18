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
  sixax.c 

  Contains code for initializing the 6 axis sensor calibration and
  converting from samples to forces and torques

  Author: Will Dickson
---------------------------------------------------------------------- */
#include "sixax.h"
#include "util.h"

int sixax_init_cal(Calibration **cal, char *cal_file_path, float tool_trans[])
{
	short rtn_flag;    
    unsigned short index = 1;

	// create Calibration struct
    *cal=createCalibration(cal_file_path,index);
    if (cal==NULL) {
        PRINT_ERR_MSG("Specified calibration could not be loaded.\n");
        return FAIL;
    }

	// Set force units.
	rtn_flag=SetForceUnits(*cal,"N");
	switch (rtn_flag) {
		case 0: 
            // successful completion
            break;	
		case 1: 
                PRINT_ERR_MSG("Invalid Calibration struct\n"); 
                return FAIL;
		case 2: 
                PRINT_ERR_MSG("Invalid force units\n"); 
                return FAIL;
		default: 
                PRINT_ERR_MSG("Unknown error\n"); 
                return FAIL;
	}

	// Set torque units.
	rtn_flag=SetTorqueUnits(*cal,"N-m");
	switch (rtn_flag) {
		case 0: 
            // successful completion
            break;	
		case 1: 
            PRINT_ERR_MSG("Invalid Calibration struct\n"); 
            return FAIL;
		case 2: 
            PRINT_ERR_MSG("Invalid torque units\n"); 
            return FAIL;
		default: 
            PRINT_ERR_MSG("Unknown error\n"); 
            return FAIL;
	}

	// Set tool transform.
	// This line is only required if you want to move or rotate the sensor's coordinate system.
	// This example tool transform translates the coordinate system 20 mm along the Z-axis 
	// and rotates it 45 degrees about the X-axis.
    
	// This sample transform includes a translation along the Z-axis and a rotation about the X-axis.
	// float tool_trans[6]={0,0,20,45,0,0};
    
	rtn_flag=SetToolTransform(*cal,tool_trans,"mm","degrees");
	switch (rtn_flag) {
		case 0: 
            // successful completion
            break;	
		case 1: 
            PRINT_ERR_MSG("Invalid Calibration struct\n"); 
            return FAIL;
		case 2: 
            PRINT_ERR_MSG("Invalid distance units\n"); 
            return FAIL;
		case 3: 
            PRINT_ERR_MSG("Invalid angle units\n"); 
            return FAIL;
		default: 
            PRINT_ERR_MSG("Unknown error\n"); 
            return FAIL;
	}

	// Temperature compensation is on by default if it is available.
	// To explicitly disable temperature compensation, uncomment the following code
	rtn_flag = SetTempComp(*cal,FALSE);                   // disable temperature compensation
	switch (rtn_flag) {
		case 0: 
            // successful completion
            break;	
		case 1: 
            PRINT_ERR_MSG("Invalid Calibration struct\n"); 
            return FAIL;
		case 2: 
            PRINT_ERR_MSG("Temperature Compensation not available on this transducer\n"); 
            return FAIL;
		default: 
            PRINT_ERR_MSG("Unknown error\n"); 
            return FAIL;
	}
    return SUCCESS;
} 

void sixax_free_cal(Calibration *cal)
{
	// free memory allocated to Calibration structure
	destroyCalibration(cal);
    return;
}

int sixax_set_bias(Calibration *cal, float bias[])
{
	// store an unloaded measurement; this removes the effect of tooling weight
	Bias(cal,bias);
    return SUCCESS;
} 

int sixax_sample2ft(Calibration *cal, float sample[], float ft[])
{
	// convert a loaded measurement into forces and torques
	ConvertToFT(cal,sample,ft);
    return SUCCESS;
}

void sixax_print_calinfo(Calibration *cal)
{

    int i;
    int j;

	// display info from calibration file
    printf("\n");
    printf("Six-axis sensor calibration information \n");
    printf("\n");
	printf("                  Serial: %s\n",cal->Serial);
	printf("              Body Style: %s\n",cal->BodyStyle);
	printf("             Calibration: %s\n",cal->PartNumber);
	printf("        Calibration Date: %s\n",cal->CalDate);
	printf("                  Family: %s\n",cal->Family);
	printf("              # Channels: %i\n",cal->rt.NumChannels);
	printf("                  # Axes: %i\n",cal->rt.NumAxes);
	printf("             Force Units: %s\n",cal->ForceUnits);
	printf("            Torque Units: %s\n",cal->TorqueUnits);
	printf("Temperature Compensation: %s\n",(cal->TempCompAvailable ? "Yes" : "No"));
    printf("\n");
	
	// print maximum loads of axes
	printf("\nRated Loads\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		char *units;
		if ((cal->AxisNames[i])[0]=='F') {
			units=cal->ForceUnits;
		} else units=cal->TorqueUnits;
		printf("%s: %f %s\n",cal->AxisNames[i],cal->MaxLoads[i],units);
	}

	// print working calibration matrix
	printf("\nWorking Calibration Matrix\n");
	printf("     ");
	for (i=0;i<cal->rt.NumChannels-1;i++)
		printf("G%i            ",i);
	printf("\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		printf("%s: ",cal->AxisNames[i]);
		for (j=0;j<cal->rt.NumChannels-1;j++)
			printf("%13.5e ",cal->rt.working_matrix[i][j]);
		printf("\n");
	}

	// print temperature compensation information, if available
	if (cal->TempCompAvailable) {
		printf("\nTemperature Compensation Information\n");
		printf("BS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.bias_slopes[i]);
		}
		printf("\nGS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.gain_slopes[i]);
		}
		printf("\nTherm: %f\n",cal->rt.thermistor);
	}

    // Print bias vector
    printf("\nBias vector\n");
    for (i=0;i<cal->rt.NumChannels-1;i++) {
        printf("%13.5e ",cal->rt.bias_vector[i]);
    }
    printf("\n");

    // Print user transform
    printf("\nTool transform\n");
    for (i=0;i<6;i++){
        printf("%13.5e ",cal->cfg.UserTransform.TT[i]);
    }
    printf("\n");

    return;
}
