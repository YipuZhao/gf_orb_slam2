/**
 * @file MacroDefinitions.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 01-28-2023
 * @copyright Copyright (c) 2023
 */

#ifndef GF_ORB_SLAM2_MACRO_DEFINITIONS_H_
#define GF_ORB_SLAM2_MACRO_DEFINITIONS_H_

////////////////////////////////////////////////////////////////////////////////
//////  This file defines the macros that control the running modes       //////
//////    e.g. ORB2_BASELINE, GF, GF+SW, GF+CV, GF+GG                     //////
////////////////////////////////////////////////////////////////////////////////

//######################### MODE definition. #################################//

// {GF_GG_MODE, GF_CV_MODE, CF_SW_MODE, GF_MODE, ORB2_BASELINE}
#define GF_GG_MODE

//########################### Relocalization. ################################//

//-------------- Tracking.h -----------------//

/* --- options to fair comparison wrt other VO pipelines --- */
// #define DISABLE_RELOC

//############################## LoopClosure. ################################//

//------------ LoopClosing.h ------------------//

// #define DISABLE_LOOP_CLOSURE


//############################## MotionModel. ################################//

//-------------- Tracking.h -----------------//

/* --- options of enabling anticipation in motion-based tracking --- */
//
// NOTE
// For simulating closed-loop performance with open-loop benchmarks
// By enabling PRED_WITH_ODOM, the GT trajectory is loaded and purturbed
// with random error.  It is then utilized as pose prediction (replacement
// of constant motion model) in tracking thread.

// #define PRED_WITH_ODOM

/* --- options of anticipating poses with closed-loop planner  --- */
//
// NOTE
// For closed-loop navigation application ONLY
// By ENABLE_PLANNER_PREDICTION, please make sure the the trajectory state
// predictor package is included in your catkin workspace:
// https://github.gatech.edu/ivabots/trajectory_state_predictor
// Otherwise, you might write your own predictor by grabbing output from the
// controller

// #define ENABLE_PLANNER_PREDICTION

////////////////////////////////////////////////////////////////////////////////
// !!! YOU NEED NOT WORRY ABOUT THE PARAMETERS DEFINED BELOW !!!              //
////////////////////////////////////////////////////////////////////////////////
//############################## RunningMode. ################################//
#ifdef GF_GG_MODE

    //--------------- Optimizer.h --------------//
    #define ENABLE_GOOD_GRAPH

    //--------------- Frame.h ------------------//
    
    // For anticipation in good graph, it uses future camera pose to predict
    // visible map points, thus determines the local BA time budget
    #define ENABLE_ANTICIPATION_IN_GRAPH

#elif defined GF_MODE

    // Do nothing.
    // GF is default on as long as ORB_SLAM_BASELINE is off,
    // since ORB2_BASELINE and GF are mutually exclusive.

#elif defined GF_SW_MODE

    //--------------- Optimizer.h --------------//
    #define ENABLE_SLIDING_WINDOW_FILTER

#elif defined GF_CV_MODE

    //--------------- Optimizer.h --------------//
    #define ENABLE_COVIS_GRAPH

#elif defined ORB2_BASELINE

    // @TODO (yanwei) This mode seems NOT working as expected. Better to use 
    // origin ORB2 code for baseline benchmark.

    //-------------- Tracking.h -----------------//

    // options of baseline methods
    #define ORB_SLAM_BASELINE

#endif

#endif  // GF_ORB_SLAM2_MACRO_DEFINITIONS_H_