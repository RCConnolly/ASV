/*! \class wFM2
    \brief Implements the Weighted Fast Marching Square (wFM2) planning algorithm.

    It uses as a main container the nDGridMap class. The nDGridMap template parameter
    has to be an FMCell or something inherited from it. It also uses a heap type in order
    to specify the underlying FMM.

    IMPORTANT NOTE: When running FM2 many times on the same grid it is recommended
    to completely restart the grid (erase and create or resize). See test_fm2.cpp.

    @par External documentation:
        FM2:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno,
          The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories,
          IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.

    Copyright (C) 2014 Javier V. Gomez and Jose Pardeiro
    www.javiervgomez.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see  < http://www.gnu.org/licenses/>.
*/

#ifndef WFM2_HPP_
#define WFM2_HPP_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/fm/fmm.hpp>
#include <fast_methods/fm2/fm2.hpp>
#include <fast_methods/gradientdescent/gradientdescent.hpp>

template < class grid_t, class heap_t = FMDaryHeap<FMCell> > class wFM2 : public FM2<grid_t, heap_t> {

    /** \brief Path type encapsulation. */
    typedef std::vector< std::array<double, grid_t::getNDims()> > path_t;
    
    /** \brief Shorthand of the base clase. */
    typedef FM2<grid_t, heap_t > FM2Base;

    public:
        /** \brief maxDistance sets the velocities map saturation distance in real units (before normalization). */
        wFM2 (double vel_scale = 1.5, HeurStrategy heurStrategy = DISTANCE, double maxDistance = -1) : scale_(vel_scale), FM2Base("wFM2", maxDistance), heurStrategy_(heurStrategy) {}

        /** \brief maxDistance sets the velocities map saturation distance in real units (before normalization). */
        wFM2 (const char * name, double vel_scale = 1.5, HeurStrategy heurStrategy = TIME, double maxDistance = -1) : scale_(vel_scale), FM2Base(name, maxDistance), heurStrategy_(heurStrategy) {}

        /** \brief Overloaded from FM2. In this case the precomputeDistances() method is called. */
        // precomputeDistances() stores precomputed heuristic distances in distances_ field of FMM
        virtual void setInitialAndGoalPoints
        (const std::vector<unsigned int> & init_points, unsigned int goal_idx) {
            FM2Base::setInitialAndGoalPoints(init_points, goal_idx);
            solver_->precomputeDistances();
        }

        /** \brief Sets up the solver to check whether is ready to run. */
        virtual void setup
        () {
            FM2Base::setup();
            if(int(goal_idx_) == -1)
            {
                console::error("A goal point has to be set for FM2-based solvers.");
                exit(1);
            }
        }
        
        /** \brief Implements the actual wFM2 method. */
        virtual void computeInternal
        () {
            if (!setup_)
                 setup();

            // sets grid_ to be the safety map
            // sets occupancy of each cell (between 0 and 1)
            computeVelocitiesMap();

            // Reset time counter so that time_ returns the time of the second wave.
            start_ = std::chrono::steady_clock::now();

            // According to the theoretical basis the wave is expanded from the goal point to the initial point.
            std::vector <unsigned int> wave_init;
            wave_init.push_back(goal_idx_);
            unsigned int wave_goal = init_points_[0];

            solver_->setInitialAndGoalPoints(wave_init, wave_goal); // also precomputes distances
            solver_->setHeuristics(heurStrategy_);


            double maxSpeed = grid_->getMaxSpeed();            
            for(unsigned int i = 0; i < grid_->size(); ++i) {
                double vel = (scale_)*(grid_->getCell(i).getVelocity()); 

                if (vel > maxSpeed) {
                    grid_->getCell(i).setVelocity(1);
                }
                else {
                    grid_->getCell(i).setVelocity(vel);
                }
           }
            


            solver_->compute();
            // Restore the actual grid status.
            grid_->setClean(false);
        }

    private:
        double scale_;

    protected:
        using FM2Base::grid_;
        using FM2Base::init_points_;
        using FM2Base::goal_idx_;
        using FM2Base::solver_;
        using FM2Base::setup;
        using FM2Base::setup_;
        using FM2Base::start_;
        using FM2Base::computeVelocitiesMap;
        using FM2Base::maxDistance_;

        /** \brief Stores the heuristic strategy to be used. */
        HeurStrategy heurStrategy_;
};

#endif /* FM2STAR_HPP_*/
