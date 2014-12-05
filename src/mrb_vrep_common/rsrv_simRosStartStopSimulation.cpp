/********************************************************************
 * 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Max-Planck-Gesellschaft
 * Copyright (c) 2012-2015, Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************/


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  rsrv_simRosStartStopSimulation
#define S_FUNCTION_LEVEL 2

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#pragma push_macro("RT")
#undef RT

#include <ros/ros.h>

// Generic Publisher
#include <matlab_ros_bridge/GenericPublisher.hpp>

// Message
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>

#pragma pop_macro("RT")

#include <matlab_ros_bridge/RosMatlabBrigdeDefines.hpp>

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify:
   *     o The numerator must be of a lower order than the denominator.
   *     o The sample time must be a real positive nonzero value.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
	//  SFUNPRINTF("Calling mdlCheckParameters");
    
    // Tsim
	if (mxIsEmpty( ssGetSFcnParam(S,0)) ||
			mxIsSparse( ssGetSFcnParam(S,0)) ||
			mxIsComplex( ssGetSFcnParam(S,0)) ||
			mxIsLogical( ssGetSFcnParam(S,0)) ||
			!mxIsNumeric( ssGetSFcnParam(S,0)) ||
			!mxIsDouble( ssGetSFcnParam(S,0)) ||
			mxGetNumberOfElements(ssGetSFcnParam(S,0)) != 1) {
		ssSetErrorStatus(S,"Simulation time must be a single double Value");
		return;
	}

	// Simulation Start Service Topic
	if (!mxIsChar( ssGetSFcnParam(S,1)) ) {
		ssSetErrorStatus(S,"Simulation start service name must be a char array (string)");
		return;
	}
    
    // Simulation Stop Service Topic
	if (!mxIsChar( ssGetSFcnParam(S,2)) ) {
		ssSetErrorStatus(S,"Simulation stop service name must be a char array (string)");
		return;
	}

    // Simulation Pause Service Topic
    if (!mxIsChar( ssGetSFcnParam(S,3)) ) {
        ssSetErrorStatus(S,"Simulation pause service name must be a char array (string)");
        return;
    }
  }
#endif /* MDL_CHECK_PARAMETERS */



/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/


//double Tsim;

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 4);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink. */
    }
#endif

//    int_T nRobots = mxGetNumberOfElements(ssGetSFcnParam(S,2));

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;

	for (int_T i = 0; i < ssGetNumInputPorts(S); ++i) {
		/*direct input signal access*/
    	ssSetInputPortRequiredContiguous(S, i, true); 
		
		/*
		 * Set direct feedthrough flag (1=yes, 0=no).
		 * A port has direct feedthrough if the input is used in either
		 * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
		 * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
		 */
		ssSetInputPortDirectFeedThrough(S, i, 1);
	}

    if (!ssSetNumOutputPorts(S, 0)) return;

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 3); // Start + Stop + Pause service clients
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T Tsim = mxGetScalar(ssGetSFcnParam(S, 0));
    ssSetSampleTime(S, 0, Tsim);                      //DISCRETE_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */

static void mdlStart(SimStruct *S)
{   
    SFUNPRINTF("Starting Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
    // init ROS if not yet done.
    initROS(S);

    void** vecPWork = ssGetPWork(S);
    // save nRobots in IWorkVector
//    int_T nRobots = mxGetNumberOfElements(ssGetSFcnParam(S,2));
//    *ssGetIWork(S) = nRobots;

    ros::NodeHandle nodeHandle(ros::this_node::getName());

    // get Topic Strings
    size_t start_buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1;
    size_t stop_buflen = mxGetN((ssGetSFcnParam(S, 2)))*sizeof(mxChar)+1;
    size_t pause_buflen = mxGetN((ssGetSFcnParam(S, 3)))*sizeof(mxChar)+1;
    char* start_service = (char*)mxMalloc(start_buflen);
    char* stop_service = (char*)mxMalloc(stop_buflen);
    char* pause_service = (char*)mxMalloc(pause_buflen);
    mxGetString((ssGetSFcnParam(S, 1)), start_service, start_buflen);
    mxGetString((ssGetSFcnParam(S, 2)), stop_service, stop_buflen);
    mxGetString((ssGetSFcnParam(S, 3)), pause_service, pause_buflen);

    //SFUNPRINTF("The string being passed as a Paramater is - %s\n ", topic);

    ros::ServiceClient* start_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosStartSimulation>(start_service));
    vecPWork[0] = start_client;

    if (!start_client->exists()){
        ssSetErrorStatus(S,"Start service does not exist!");
    }

    ros::ServiceClient* stop_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosStopSimulation>(stop_service));
    vecPWork[1] = stop_client;

    if (!stop_client->exists()){
        ssSetErrorStatus(S,"Stop service does not exist!");
    }

    ros::ServiceClient* pause_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosPauseSimulation>(pause_service));
    vecPWork[2] = pause_client;

    if (!pause_client->exists()){
        ssSetErrorStatus(S,"Pause service does not exist!");
    }

    vrep_common::simRosStartSimulation srv;
    start_client->call(srv);

    // free char array
    mxFree(start_service);
    mxFree(stop_service);
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
    // get Objects
//    void** vecPWork = ssGetPWork(S);
//    int_T nRobots = *ssGetIWork(S);

    // get Pointers
    // accessing inputs
//    const real_T *position = (const real_T*) ssGetInputPortSignal(S,0);
   // const real_T *orientation = (const real_T*) ssGetInputPortSignal(S,1);
    
//    vrep_common::simRosStartSimulation srv;
//    for (unsigned int i = 0; i < nRobots; ++i) {
        //GenericPublisher<geometry_msgs::PointStamped>* pub 
        //        = (GenericPublisher<geometry_msgs::PointStamped>*)vecPWork[i];
//        ros::ServiceClient* client = (ros::ServiceClient*)vecPWork[i];

        // define send Time.

//        client->call(srv);
//	}
//    ros::ServiceClient* stop_client = (ros::ServiceClient*)vecPWork[1];
//
//        vrep_common::simRosStopSimulation srv;
//        stop_client->call(srv);
//
//        SFUNPRINTF("Calling stopping service in %s. Result: %d\n", TOSTRING(S_FUNCTION_NAME), srv.response.result);

}


#if defined(MATLAB_MEX_FILE)
#define MDL_SIM_STATUS_CHANGE
static void mdlSimStatusChange(SimStruct *S, ssSimStatusChangeType simStatus) {

    ros::NodeHandle nodeHandle(ros::this_node::getName());
    void** vecPWork = ssGetPWork(S);

    if (simStatus == SIM_PAUSE) {
        SFUNPRINTF("Pause has been called! \n");
        ros::ServiceClient* pause_client = (ros::ServiceClient*)vecPWork[2];
        vrep_common::simRosPauseSimulation srv;
        pause_client->call(srv);
    } else if (simStatus == SIM_CONTINUE) {
        SFUNPRINTF("Continue has been called! \n");
        ros::ServiceClient* start_client = (ros::ServiceClient*)vecPWork[0];
        vrep_common::simRosStartSimulation srv;
        start_client->call(srv);
    }
}
#endif


#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // get Objects
    ros::NodeHandle nodeHandle(ros::this_node::getName());

    void** vecPWork = ssGetPWork(S);

    ros::ServiceClient* start_client = (ros::ServiceClient*)vecPWork[0];
    delete start_client;

    ros::ServiceClient* stop_client = (ros::ServiceClient*)vecPWork[1];

    vrep_common::simRosStopSimulation srv;
    stop_client->call(srv);

    SFUNPRINTF("Calling stopping service in %s. Result: %d\n", TOSTRING(S_FUNCTION_NAME), srv.response.result);

    delete stop_client;

    SFUNPRINTF("Terminating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
