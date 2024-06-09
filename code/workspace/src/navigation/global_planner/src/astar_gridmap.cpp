/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar_gridmap.h>
#include<costmap_2d/cost_values.h>

#define lethal_cost_gridmap 850

namespace global_planner {

AStarExpansion_gridmap::AStarExpansion_gridmap(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion_gridmap::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential, std::int16_t* costs_grid) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index_gridmap(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    std::cout<<"ASTAR_GRIDMAP CALLED"<<std::endl;

    // int length =0;
    // for(int i = 0; i < 1000*1000; i++){
    // std::cout<<costs_grid[i]<<" ";
    // }

    while (queue_.size() > 0 && cycle < cycles) {
        Index_gridmap top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1_gridmap());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y, costs_grid);
        add(costs, potential, potential[i], i - 1, end_x, end_y, costs_grid);
        add(costs, potential, potential[i], i + nx_, end_x, end_y, costs_grid);
        add(costs, potential, potential[i], i - nx_, end_x, end_y, costs_grid);

        cycle++;
    }

    return false;
}

void AStarExpansion_gridmap::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y, std::int16_t* costs_grid) {
    if (next_i < 0 || next_i >= ns_) //check bounds
        return;

    // check if it is revisiting the cell, skip
    if (potential[next_i] < POT_HIGH) // POT_HIGH = 1e10 
        return;

    // if unknown skip
    if(costs_grid[next_i] < 0)
        return;

    // if too close to obstacle, skip   
    if(costs_grid[next_i] >= lethal_cost_gridmap){
    // if(costs_grid[next_i] >= lethal_cost_){
    // if(costs_grid[next_i] >= 956){
        return;
    }

    // potential[next_i] = p_calc_->calculatePotential(potential, costs_grid[next_i] + neutral_cost_, next_i, prev_potential);
    // int x = next_i % nx_, y = next_i / nx_;
    // float distance = abs(end_x - x) + abs(end_y - y);

    // queue_.push_back(Index_gridmap(next_i, potential[next_i]));
    // std::push_heap(queue_.begin(), queue_.end(), greater1_gridmap());

    // // Orig
    potential[next_i] = p_calc_->calculatePotential(potential, costs_grid[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index_gridmap(next_i, potential[next_i] + distance*neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1_gridmap());

//    -------------------------------------

}

} //end namespace global_planner
