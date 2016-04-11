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

#ifndef BIMANUAL_DS_EXECUTION_H
#define BIMANUAL_DS_EXECUTION_H

#include "MathLib/MathLib.h"
#include "bimanual_ds.h"

using namespace MathLib;

class bimanual_ds_execution
{
public:
    bimanual_ds_execution();
    ~bimanual_ds_execution();

    void init(double dt, double Gamma, double DGamma, double Gain_A, double Gain_K_l, double Gain_K_r);

private:

    bimanual_ds *vo_DS;


    double dt;
    double reachingThr;

};


#endif // BIMANUAL_DS_EXECUTION_H
