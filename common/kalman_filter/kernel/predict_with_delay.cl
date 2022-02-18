/*
 * Software License Agreement (BSD License)
 *  Copyright (c) 2022, harihitode
 *
 *  This OpenCL kernel is ported from PCL library.
 *  The original license is below.
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#define THREAD_NUM (512)
#define DIM_X (6)

/** \brief calculate kalman filter covariance by precision model with time delay. This is mainly for EKF of nonlinear
 * process model.
 * \param[in] x_next 1-D array of x_next which is the argument of predictWithDelay (6 elements)
 * \param[in] A 2-D array of A which is the argument of predictWithDelay (6*6 elements)
 * \param[in] A_transpose 2-D array of transpose of A (6*6 elements)
 * \param[in] Q 2-D array of Q which is the argument of predictWithDelay (6*6 elements)
 * \param[in] x_in 1-D array of current estimated state (dim_x_ex(300 by default) elements)
 * \param[out] x_out 1-D array of result of estimated state (dim_x_ex(300 by default) elements)
 * \param[in] P_in 1-D array of covariance of estimated state (dim_x_ex*dim_x_ex(300 * 300 by default) elements)
 * \param[out] P_out 1-D array of result of covariance of estimated state (dim_x_ex*dim_x_ex(300 * 300 by default) elements)
 * \param[in] dim_x_ex dimension of extended state with dime delay
 */
kernel void predictWithDelayCL(
                               const global TYPE x_next[DIM_X], const global TYPE A[DIM_X][DIM_X], const global TYPE A_transpose[DIM_X][DIM_X],
                               const global TYPE Q[DIM_X][DIM_X], const global TYPE * x_in, global TYPE * x_out,
                               const global TYPE * P_in, global TYPE * P_out, const int dim_x_ex)
{
  int target_row;
  int item_index = get_global_id(0);
  if (item_index >= dim_x_ex) {
    return;
  }

  // calculate each row with THREAD_NUM threads
  for (target_row = item_index; target_row < dim_x_ex; target_row += THREAD_NUM) {
    if (target_row < DIM_X) {
      // slide states in the time direction
      x_out[target_row] = x_next[target_row];

      // update P with delayed measurement A matrix structure
      // P_tmp.block(0, 0, dim_x_, dim_x_) = A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;
      // first, calculate target row of A * P_
      TYPE AxP_in[DIM_X];
      for (int i = 0; i < DIM_X; i++) {
        AxP_in[i] =
          A[target_row][0] * P_in[0 * dim_x_ex + i] + A[target_row][1] * P_in[1 * dim_x_ex + i] + A[target_row][2] * P_in[2 * dim_x_ex + i] +
          A[target_row][3] * P_in[3 * dim_x_ex + i] + A[target_row][4] * P_in[4 * dim_x_ex + i] + A[target_row][5] * P_in[5 * dim_x_ex + i];
      }
      // second, calculate (A * P_) * A.transport() + Q
      for (int i = 0; i < DIM_X; i++) {
        P_out[target_row * dim_x_ex + i] =
          AxP_in[0] * A_transpose[0][i] + AxP_in[1] * A_transpose[1][i] + AxP_in[2] * A_transpose[2][i] +
          AxP_in[3] * A_transpose[3][i] + AxP_in[4] * A_transpose[4][i] + AxP_in[5] * A_transpose[5][i] +
          Q[target_row][i];
      }

      // P_tmp.block(0, dim_x_, dim_x_, d_dim_x) = A * P_.block(0, 0, dim_x_, d_dim_x);
      for (int i = DIM_X; i < dim_x_ex; i++) {
        int ref_col = i - DIM_X;
        P_out[target_row * dim_x_ex + i] =
          A[target_row][0] * P_in[0 * dim_x_ex + ref_col] + A[target_row][1] * P_in[1 * dim_x_ex + ref_col] + A[target_row][2] * P_in[2 * dim_x_ex + ref_col] +
          A[target_row][3] * P_in[3 * dim_x_ex + ref_col] + A[target_row][4] * P_in[4 * dim_x_ex + ref_col] + A[target_row][5] * P_in[5 * dim_x_ex + ref_col];
      }
    }
    else {
      int ref_row = target_row - DIM_X;
      // slide states in the time direction
      x_out[target_row] = x_in[ref_row];

      // update P with delayed measurement A matrix structure
      // P_tmp.block(dim_x_, 0, d_dim_x, dim_x_) = P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();
      for (int i = 0; i < DIM_X; i++) {
        P_out[target_row * dim_x_ex + i] =
          P_in[ref_row * dim_x_ex + 0] * A_transpose[0][i] + P_in[ref_row * dim_x_ex + 1] * A_transpose[1][i] + P_in[ref_row * dim_x_ex + 2] * A_transpose[2][i] +
          P_in[ref_row * dim_x_ex + 3] * A_transpose[3][i] + P_in[ref_row * dim_x_ex + 4] * A_transpose[4][i] + P_in[ref_row * dim_x_ex + 5] * A_transpose[5][i];
      }

      // P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_.block(0, 0, d_dim_x, d_dim_x);
      for (int i = DIM_X; i < dim_x_ex; i++) {
        int ref_col = i - DIM_X;
        P_out[target_row * dim_x_ex + i] = P_in[ref_row * dim_x_ex + ref_col];
      }
    }
  }
}
