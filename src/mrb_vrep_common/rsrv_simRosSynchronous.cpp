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

#define S_FUNCTION_NAME  rsrv_simRosSynchronous
#define S_FUNCTION_LEVEL 2

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


#define ROUND(x) floor( x + 0.5 )


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#pragma push_macro("RT")
#undef RT

#include <ros/ros.h>

// Generic Publisher
#include <matlab_ros_bridge/GenericSubscriber.hpp>

// Message
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/simRosGetInfo.h>
#include <vrep_common/simRosSetFloatingParameter.h>
#include <topic_tools/shape_shifter.h>

#pragma pop_macro("RT")

#include <matlab_ros_bridge/RosMatlabBrigdeDefines.hpp>

#include <v_repConst.h>

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

	// Vrep services base name
	if (!mxIsChar( ssGetSFcnParam(S,1)) ) {
		ssSetErrorStatus(S,"Vrep services base name must be a char array (string)");
		return;
	}

	// wait topic name
	if (!mxIsChar( ssGetSFcnParam(S,2)) ) {
		ssSetErrorStatus(S,"Wait topic name must be a char array (string)");
		return;
	}

	// wait timeout
	if (mxIsEmpty( ssGetSFcnParam(S,3)) ||
			mxIsSparse( ssGetSFcnParam(S,3)) ||
			mxIsComplex( ssGetSFcnParam(S,3)) ||
			mxIsLogical( ssGetSFcnParam(S,3)) ||
			!mxIsNumeric( ssGetSFcnParam(S,3)) ||
			!mxIsDouble( ssGetSFcnParam(S,3)) ||
			mxGetNumberOfElements(ssGetSFcnParam(S,3)) != 1) {
		ssSetErrorStatus(S,"Wait timeout must be a single double Value");
		return;
	}

	// wait extra_wait
	if (mxIsEmpty( ssGetSFcnParam(S,4)) ||
			mxIsSparse( ssGetSFcnParam(S,4)) ||
			mxIsComplex( ssGetSFcnParam(S,4)) ||
			mxIsLogical( ssGetSFcnParam(S,4)) ||
			!mxIsNumeric( ssGetSFcnParam(S,4)) ||
			!mxIsDouble( ssGetSFcnParam(S,4)) ||
			mxGetNumberOfElements(ssGetSFcnParam(S,4)) != 1) {
		ssSetErrorStatus(S,"Extra wait time must be a single double Value");
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

    ssSetNumSFcnParams(S, 5);  /* Number of expected parameters */
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
//    ssSetNumPWork(S, 4); // Start + Stop + Synchronous + Trigger

    const real_T timeout = mxGetScalar(ssGetSFcnParam(S, 3));
    if (timeout > 0){
    	ssSetNumPWork(S, 6); // Start + Stop + Synchronous + Trigger services + wait_topic Subscriber + AsyncSpinner
    } else {
    	ssSetNumPWork(S, 4); // Start + Stop + Synchronous + Trigger
    }

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
    const real_T Tsim = mxGetScalar(ssGetSFcnParam(S, 0));
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

    ros::NodeHandle nodeHandle(ros::this_node::getName());

    // get base service name strings
    size_t buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1;
    char* base_name = (char*)mxMalloc(buflen);
    mxGetString((ssGetSFcnParam(S, 1)), base_name, buflen);

    const std::string start_name = std::string(base_name) + "/simRosStartSimulation";
    ros::ServiceClient* start_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosStartSimulation>(start_name));
    vecPWork[0] = start_client;
    if (!start_client->exists()){
        ssSetErrorStatus(S,"Start service does not exist!");
    }

    const std::string stop_name = std::string(base_name) + "/simRosStopSimulation";
    ros::ServiceClient* stop_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosStopSimulation>(stop_name));
    vecPWork[1] = stop_client;
    if (!stop_client->exists()){
        ssSetErrorStatus(S,"Stop service does not exist!");
    }

    const std::string synchronous_name = std::string(base_name) + "/simRosSynchronous";
    ros::ServiceClient* synchronous_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosSynchronous>(synchronous_name));
	vecPWork[2] = synchronous_client;
	if (!synchronous_client->exists()){
		ssSetErrorStatus(S,"Synchronous service does not exist!");
	}

	const std::string trigger_name = std::string(base_name) + "/simRosSynchronousTrigger";
    ros::ServiceClient* trigger_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosSynchronousTrigger>(trigger_name));
	vecPWork[3] = trigger_client;
	if (!trigger_client->exists()){
		ssSetErrorStatus(S,"Trigger service does not exist!");
	}

	const std::string info_name = std::string(base_name) + "/simRosGetInfo";
    ros::ServiceClient* info_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosGetInfo>(info_name));
	if (!info_client->exists()){
		ssSetErrorStatus(S,"Info service does not exist!");
	}

    const std::string parameter_name = std::string(base_name) + "/simRosSetFloatingParameter";
	ros::ServiceClient* parameter_client = new ros::ServiceClient(nodeHandle.serviceClient<vrep_common::simRosSetFloatingParameter>(parameter_name));
	if (!parameter_client->exists()){
		ssSetErrorStatus(S,"Set parameter service does not exist!");
	}

	// Set V-Rep time step to this block time step
	const real_T tSim = mxGetScalar(ssGetSFcnParam(S, 0));
	vrep_common::simRosSetFloatingParameter paramSrv;
	paramSrv.request.parameter = sim_floatparam_simulation_time_step;
	paramSrv.request.parameterValue = tSim;
	parameter_client->call(paramSrv);

	if (paramSrv.response.result == -1){
		ssSetErrorStatus(S,"Error setting V-REP simulation time step.");
	}

	vrep_common::simRosGetInfo infoSrv;
	info_client->call(infoSrv);
	const real_T vrepTimeStep = ROUND(infoSrv.response.timeStep*1e6)*1e-6;

	if (fabs(vrepTimeStep-tSim) > 1e-5){
		char str[100];
		sprintf(str, "V-REP time step (%e) is different from block time step (%e).", vrepTimeStep, tSim);
		ssSetErrorStatus(S,str);
	}

	// Set synchronous mode
	vrep_common::simRosSynchronous syncSrv;
	syncSrv.request.enable = true;
	synchronous_client->call(syncSrv);
	if (syncSrv.response.result == -1){
		ssSetErrorStatus(S,"Error setting V-REP synchronous simulation mode.");
	}

	// Start V-Rep simulation
    vrep_common::simRosStartSimulation startSrv;
    start_client->call(startSrv);
    if (startSrv.response.result == -1){
		ssSetErrorStatus(S,"Error starting V-REP simulation.");
	}

    const real_T timeout = mxGetScalar(ssGetSFcnParam(S, 3));
    if (timeout > 0){
    	size_t wait_buflen = mxGetN((ssGetSFcnParam(S, 2)))*sizeof(mxChar)+1;
		char* wait_name = (char*)mxMalloc(wait_buflen);
		mxGetString((ssGetSFcnParam(S, 2)), wait_name, wait_buflen);

//		SFUNPRINTF("Subscribing to %s\n", wait_name);
		GenericSubscriber<topic_tools::ShapeShifter>* sub
		                = new GenericSubscriber<topic_tools::ShapeShifter>(nodeHandle, wait_name, 1);
		mxFree(wait_name);
		vecPWork[4] = sub;

		ros::AsyncSpinner* spinner = new ros::AsyncSpinner(1);
		spinner->start();
		vecPWork[5] = spinner;
    }



    // free char array
    mxFree(base_name);
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
}


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
	  void** vecPWork = ssGetPWork(S);
	  ros::ServiceClient* trigger_client = (ros::ServiceClient*)vecPWork[3];

	  const real_T timeout = mxGetScalar(ssGetSFcnParam(S, 3));
	  sem_t * sem = (sem_t*)vecPWork[4];

	  GenericSubscriber<topic_tools::ShapeShifter>* sub;

	  if (timeout > 0){
		  sub = static_cast< GenericSubscriber<topic_tools::ShapeShifter>* >(vecPWork[4]);
//		  SFUNPRINTF("Resetting semaphore...\n");
		  sub->resetSem();
	  }

	  vrep_common::simRosSynchronousTrigger trigSrv;
	  trigger_client->call(trigSrv);

	  if (trigSrv.response.result == -1){
		  ssSetErrorStatus(S, "Error triggering V-REP simulation");
	  }

	  if (timeout > 0){
		  const int result = sub->waitMsg(timeout);
		  if (result == -1){
			  ssWarning(S, "Timeout expired");
		  } else {
//			  SFUNPRINTF("Message arrived.\n");
			  const real_T extraWaitTime = mxGetScalar(ssGetSFcnParam(S, 4));
			  if (extraWaitTime>0){
//				  SFUNPRINTF("Waiting extra %f seconds\n", extraWaitTime);
				  ros::Duration extra_wait(extraWaitTime);
				  extra_wait.sleep();
			  }
		  }
	  }
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

    ros::ServiceClient* synchronous_client = (ros::ServiceClient*)vecPWork[2];
    vrep_common::simRosSynchronous syncSrv;
	syncSrv.request.enable = false;
	synchronous_client->call(syncSrv);
	if (syncSrv.response.result == -1){
		ssSetErrorStatus(S,"Error setting V-REP synchronous simulation mode.");
	}
    delete synchronous_client;

    ros::ServiceClient* trigger_client = (ros::ServiceClient*)vecPWork[3];
    delete trigger_client;

    const real_T timeout = mxGetScalar(ssGetSFcnParam(S, 3));
    if (timeout>0){
    	GenericSubscriber<topic_tools::ShapeShifter>* sub =
    			static_cast< GenericSubscriber<topic_tools::ShapeShifter>* >(vecPWork[4]);
    	delete sub;
    	ros::AsyncSpinner* spinner = static_cast<ros::AsyncSpinner*>(vecPWork[5]);
		delete spinner;
    }

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
