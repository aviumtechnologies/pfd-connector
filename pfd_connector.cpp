/*  File    : pfd_connector.cpp
 *  Abstract:
 * 
 * C++ S-function that sends data from Simulink to the Aviumtechnologies Primary 
 * Flight Display (PFD) application. The S-function uses the Asio C++ and the
 * MAVLink C libraries.
 *
 *  Copyright 2022 Aviumtechnologies Ltd.
 */

#include <sstream>
#include <string>
#include <regex>
#include <iostream>

#include <asio.hpp>
#include "mavlink/ardupilotmega/mavlink.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME pfd_connector

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*====================*
 * S-function methods *
 *====================*/

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
}

#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0); /* Number of expected parameters */

#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
    {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL)
        {
            return;
        }
    }
    else
    {
        return; /* Parameter mismatch will be reported by Simulink. */
    }
#endif

    if (!ssSetNumInputPorts(S, 12)) /* Number of input ports */
    {
        return;
    }

    /* Input ports size */
    ssSetInputPortWidth(S, 0, 3); // phithetapsi (rad)
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1); // Vb (m/s)
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1); // Vbned (m/s)
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortWidth(S, 3, 1); // psig (rad)
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortWidth(S, 4, 1); // alpha (rad)
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortWidth(S, 5, 1); // beta (rad)
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortWidth(S, 6, 1); // h_dot (m/s)
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortWidth(S, 7, 1); // h (m/s)
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortWidth(S, 8, 1); // battery_remaining (%)
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortWidth(S, 9, 1); // battery_current (A)
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortWidth(S, 10, 1); // battery_voltage (V)
    ssSetInputPortDirectFeedThrough(S, 10, 1);
    ssSetInputPortWidth(S, 11, 3); // phithetapsi_nav (rad)
    ssSetInputPortDirectFeedThrough(S, 11, 3);

    if (!ssSetNumOutputPorts(S, 0)) /* Number of output ports */
    {
        return;
    }

    ssSetNumSampleTimes(S, 1);

    ssSetNumPWork(S, 1);

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
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    try
    {
        void **PWork = ssGetPWork(S);
    
        static asio::io_service io_service;
        static asio::ip::udp::socket socket(io_service);
        if (!socket.is_open())
        {
            socket.open(asio::ip::udp::v4());
        }
        PWork[0] = &socket;       
    }
    catch (const asio::system_error &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
    }
}
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    try
    {
        void **PWork = ssGetPWork(S);

        asio::ip::udp::socket *socket = (asio::ip::udp::socket *)PWork[0];
    
        InputRealPtrsType phithetapsi = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType Vb = ssGetInputPortRealSignalPtrs(S, 1);
        InputRealPtrsType Vbned = ssGetInputPortRealSignalPtrs(S, 2);
        InputRealPtrsType psig = ssGetInputPortRealSignalPtrs(S, 3);
        InputRealPtrsType alpha = ssGetInputPortRealSignalPtrs(S, 4);
        InputRealPtrsType beta = ssGetInputPortRealSignalPtrs(S, 5);
        InputRealPtrsType h_dot = ssGetInputPortRealSignalPtrs(S, 6);
        InputRealPtrsType h = ssGetInputPortRealSignalPtrs(S, 7);
        InputRealPtrsType battery_remaining = ssGetInputPortRealSignalPtrs(S, 8);
        InputRealPtrsType battery_current = ssGetInputPortRealSignalPtrs(S, 9);
        InputRealPtrsType battery_voltage = ssGetInputPortRealSignalPtrs(S, 10);
        InputRealPtrsType phithetapsi_nav = ssGetInputPortRealSignalPtrs(S, 11);


        uint8_t buffer[65535];
        mavlink_message_t encoded_msg;
        
        //VFR_HUD
        {
            mavlink_vfr_hud_t msg;

            msg.airspeed = (float)(*Vb[0]);
            msg.groundspeed = (float)(*Vbned[0]);    
            msg.heading = (int16_t)(*psig[0]);   
            msg.throttle = (uint16_t)(0.0);   
            msg.alt = (float)(*h[0]);   
            msg.climb = (float)(*h_dot[0]);    
            
            mavlink_msg_vfr_hud_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

            asio::ip::udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 5760);     

            if (socket->send_to(asio::buffer(buffer,length), remote_endpoint) != length)
            {
                std::runtime_error("Error sending VFR_HUD message!");
            }
        }

        //ATTITUDE
        {
            mavlink_attitude_t msg;

            msg.roll = (float)(*phithetapsi[0]);
            msg.pitch = (float)(*phithetapsi[1]);    
            msg.yaw = (float)(*phithetapsi[2]);   
            
            mavlink_msg_attitude_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

            asio::ip::udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 5760);     

            if (socket->send_to(asio::buffer(buffer,length), remote_endpoint) != length)
            {
                std::runtime_error("Error sending ATTITUDE message!");
            }
        }

        //AOA_SSA
        {
            mavlink_aoa_ssa_t msg;

            msg.AOA = (float)(*alpha[0]);
            msg.SSA = (float)(*beta[0]);
            
            mavlink_msg_aoa_ssa_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

            asio::ip::udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 5760);     

            if (socket->send_to(asio::buffer(buffer,length), remote_endpoint) != length)
            {
                std::runtime_error("Error sending AOA_SSA message!");
            }
        }

        //BATTERY_STATUS

        {

            mavlink_battery_status_t msg;

            msg.id = (uint8_t)(0);
            msg.battery_function = (uint8_t)(1);
            msg.type = (uint8_t)(1);
            msg.temperature=(int16_t)(15);
            msg.voltages[0]=(uint16_t)(*battery_voltage[0]*1000);
            msg.voltages[1]=(uint16_t)(UINT16_MAX);
            msg.voltages[2]=(uint16_t)(UINT16_MAX);
            msg.voltages[3]=(uint16_t)(UINT16_MAX);
            msg.voltages[4]=(uint16_t)(UINT16_MAX);
            msg.voltages[5]=(uint16_t)(UINT16_MAX);
            msg.voltages[6]=(uint16_t)(UINT16_MAX);
            msg.voltages[8]=(uint16_t)(UINT16_MAX);
            msg.voltages[9]=(uint16_t)(UINT16_MAX);
            msg.current_battery=(int16_t)(*battery_current[0]*100);
            msg.current_consumed=(int32_t)(-1);
            msg.energy_consumed=(int32_t)(-1);
            msg.battery_remaining=(int8_t)(*battery_remaining[0]);

            mavlink_msg_battery_status_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

            asio::ip::udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 5760);     

            if (socket->send_to(asio::buffer(buffer,length), remote_endpoint) != length)
            {
                std::runtime_error("Error sending BATTERY_STATUS message!");
            }

        }

        //NAV_CONTROLLER_OUTPUT 
        {
            mavlink_nav_controller_output_t msg;

            msg.nav_roll = (float)(*phithetapsi_nav[0]);
            msg.nav_pitch = (float)(*phithetapsi_nav[1]);
            msg.nav_bearing= (int16_t)(*phithetapsi_nav[2]);
            
            mavlink_msg_nav_controller_output_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

            asio::ip::udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 5760);     

            if (socket->send_to(asio::buffer(buffer,length), remote_endpoint) != length)
            {
                std::runtime_error("Error sending NAV_CONTROLLER_OUTPUT message!");
            }
        }

    }
    catch (const asio::system_error &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    try
    {
        void **PWork = ssGetPWork(S);
        if (PWork[0] != nullptr)
        {
            ((asio::ip::udp::socket *)PWork[0])->close();
        }
    }
    catch (const asio::system_error &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
    }
}
/*======================================================*
 * See sfuntmpl.doc for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
