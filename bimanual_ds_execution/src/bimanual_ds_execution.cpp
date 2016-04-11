/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "bimanual_ds_execution.h"

bimanual_ds_execution::bimanual_ds_execution()
{
    // Here I will Initialize all parameters

    // Some default values
    dt = 0.002;
    reachingThr=0.005;

}

bimanual_ds_execution::~bimanual_ds_execution()
{
}


void bimanual_ds_execution::init(double dt, double Gamma, double DGamma, double Gain_A, double Gain_K_l, double Gain_K_r){

    vo_DS = new bimanual_ds();
    vo_DS->initialize(dt, Gamma, DGamma, Gain_A, Gain_K_l, Gain_K_r);

}
