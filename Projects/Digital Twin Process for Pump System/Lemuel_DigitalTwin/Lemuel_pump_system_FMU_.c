/************** -*-C-*- *********************************************
 * $Id$
 *                                                                  *
 *       AMESim C code for cosimulation written by code generator.  *

                                 Lemuel_pump_system_FMU_
 *
 *******************************************************************/

#include "Lemuel_pump_system_FMU_.h"
#include "ame_interfaces.h"

#include <assert.h>
#include <stdio.h>

#if !defined(AME_DS1006)
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#endif

#ifndef AME_DS1006
#include <fcntl.h>
#endif

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <setjmp.h>

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#endif

#include "ameutils.h"
#include "codegen_ameutils_public.h"
#include "itf_amesubintf_public.h"

#if defined (WIN32) || defined(XPCMSVISUALC)
#include <io.h>
#elif !defined(AME_DS1006)
#include <unistd.h>
#endif

#if defined(_WINDOWS) || defined(_WIN32) || defined(WIN32)
#define DLLEXPORTDEF __declspec(dllexport)
#else
#define DLLEXPORTDEF
#endif

#if defined (RTLAB) || defined(AME_DS1006) ||  defined(AME_DS1007) || defined(LABCAR) || defined(AME_ADX) || defined(AME_HWA) || defined(RT) ||defined(AMEVERISTAND)
#ifdef DLLEXPORTDEF
#undef DLLEXPORTDEF
#endif
#define DLLEXPORTDEF
#endif

/* If we are in Simulink we should set things up for a AMEMULTIDLL (only static symbols) */
#if defined(AME_CS_SIMULINK) || defined(AME_ME_SIMULINK)
#ifndef AMEMULTIDLL
#define AMEMULTIDLL
#endif
#endif

/* If we are in Veristand we should set things up for a AMEMULTIDLL (only static symbols) */
#ifdef AMEVERISTAND
#ifndef AMEMULTIDLL
#define AMEMULTIDLL
#endif
#endif

/* avoid globally exported functions - required for exporting several models in one executable */
#if defined AMEMULTIDLL
#define DLLEXPORTDEF_OR_STATIC static
#else
#define DLLEXPORTDEF_OR_STATIC DLLEXPORTDEF
#endif

#if defined CREATING_STATIC_LIB
#define DLLEXPORTDEF_OR_EXTERN extern
#else
#define DLLEXPORTDEF_OR_EXTERN DLLEXPORTDEF
#endif

#define TIME_ROUNDOFF 1.0e-10
#define TLAST_MIN     -1E30

#if defined(FMICS1) || defined(FMICS2) || defined(FMICS3) || defined(FMIME1) || defined(FMIME2)
#ifdef FMIX
#undef FMIX
#endif
#define FMI
#define MODEL_IDENTIFIER         Lemuel_pump_system_FMU_
#define MODEL_IDENTIFIER_STRING "Lemuel_pump_system_FMU_"
#endif

/* *****************************************************************

   code inserted by AMESim

   *****************************************************************/

#define AMEVERSION                  202410000
#define SUB_LENGTH                  63

static const char soldToId[] = "7376667";

/**** Model structure definition ****/
/* Structural definition of the model */
#define AME_MODEL_ISEXPLICIT        0

/* Number of explicit declared states  */
#define AME_NBOF_EXPLICIT_STATE     4

/* Number of implicit declared states  */
#define AME_NBOF_IMPLICIT_STATE_DECLARED  0
#define AME_NBOF_IMPLICIT_STATE_GENERATED 1
#define AME_NBOF_IMPLICIT_STATE           1

/* Number of discrete declared states  */
#define AME_NBOF_DISCRETE_STATE     11

/* Total number of states manipulated by the Amesim solver including generated states */
#define AME_NBOF_SOLVER_STATES      5

#define AME_NBOF_PARAMS             215
#define AME_NBOF_REAL_PARAMS        94
#define AME_NBOF_INT_PARAMS         78
#define AME_NBOF_TEXT_PARAMS        16
#define AME_NBOF_CSTATE_PARAMS      4
#define AME_NBOF_DSTATE_PARAMS      11
#define AME_NBOF_FIXED_VAR_PARAMS   12

#define AME_NBOF_VARS               96
#define AME_NBOF_INPUTS             2
#define AME_NBOF_OUTPUTS            3

#define AME_NBOF_REAL_STORES        29
#define AME_NBOF_INT_STORES         23
#define AME_NBOF_POINTER_STORES     3

#define AME_SIZEOF_DBWORK_ARRAY     0

#define AME_NB_VAR_INFO             96

#define AME_NB_NETWORK              0

#define AME_NB_SUBMODEL_REF         25

static const S_AMEModelDef GmodelDef = {
   AME_MODEL_ISEXPLICIT,
   AME_NBOF_EXPLICIT_STATE,
   AME_NBOF_IMPLICIT_STATE_DECLARED,
   AME_NBOF_IMPLICIT_STATE_GENERATED,
   AME_NBOF_DISCRETE_STATE,
   AME_NBOF_VARS,
   AME_NBOF_PARAMS,
   AME_NBOF_REAL_PARAMS,
   AME_NBOF_INT_PARAMS,
   AME_NBOF_TEXT_PARAMS,
   AME_NBOF_CSTATE_PARAMS,
   AME_NBOF_DSTATE_PARAMS,
   AME_NBOF_FIXED_VAR_PARAMS,
   AME_NBOF_INPUTS,
   AME_NBOF_OUTPUTS,
   AME_NBOF_REAL_STORES,
   AME_NBOF_INT_STORES,
   AME_NBOF_POINTER_STORES,
   AME_SIZEOF_DBWORK_ARRAY,
   AME_NB_VAR_INFO,
   AME_NB_NETWORK,
   AME_NB_SUBMODEL_REF
};

static const int Gis_constraint[AME_NBOF_SOLVER_STATES] = {
     0, 0, 0, 0, 1
};

static const char *GinputVarTitles[AME_NBOF_INPUTS] = {
     "pump_speed"
   , "flowrate_target"
};
static const char *GoutputVarTitles[AME_NBOF_OUTPUTS] = {
     "flow_rate"
   , "pressure_increase"
   , "mech_power"
};

/**** Parameters structure definition ****/

#define NB_SUBMODELNAME       22
static const char* GsubmodelNameArray[NB_SUBMODELNAME] = {
     "TFFD3 instance 1"
   , "TFTK0 instance 1"
   , "TFTK0 instance 2"
   , "UD00 instance 1"
   , "TFVORF0 instance 1"
   , "TFVF000 instance 1"
   , "CTRPID0 instance 1"
   , "CONS00 instance 1"
   , "SIGRECEI0 instance 1"
   , "SIGRECEI0 instance 2"
   , "SIGTRANS0 instance 1"
   , "SIGTRANS0 instance 2"
   , "SIGTRANS0 instance 3"
   , "SIGTRANS0 instance 4"
   , "SIGTRANS0 instance 5"
   , "SIGRECEI0 instance 3"
   , "SIGRECEI0 instance 4"
   , "SIGRECEI0 instance 5"
   , "TFPUC0 instance 1"
   , "INTVARSENSOR0 instance 1"
   , "INTVARSENSOR0 instance 2"
   , "INTVARSENSOR0 instance 3"
};

static const S_AMEParamInfo GParamInfo[215] = {
     {E_Real_Param           , E_Expression_Param, 0, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 1, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 2, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 3, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 4, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 5, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 6, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 7, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 8, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 9, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 10, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 11, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 12, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 13, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 14, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 15, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 16, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 17, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 18, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 19, -1, 0, 0}
   , {E_Real_Param           , E_Expression_Param, 20, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 0, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 1, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 2, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 3, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 4, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 5, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 6, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 7, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 8, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 9, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 0, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 1, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 2, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 3, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 4, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 5, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 6, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 7, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 8, -1, 0, 0}
   , {E_Text_Param           , E_Expression_Param, 9, -1, 0, 0}
   , {E_Int_Param            , E_Expression_Param, 10, -1, 1, 0}
   , {E_Int_Param            , E_Expression_Param, 11, -1, 1, 0}
   , {E_Int_Param            , E_Expression_Param, 12, -1, 1, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 0, 7, 1, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 1, 8, 1, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 2, 9, 1, 0}
   , {E_Int_Param            , E_Expression_Param, 13, -1, 2, 0}
   , {E_Int_Param            , E_Expression_Param, 14, -1, 2, 0}
   , {E_Int_Param            , E_Expression_Param, 15, -1, 2, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 3, 16, 2, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 4, 17, 2, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 5, 18, 2, 0}
   , {E_Real_Param           , E_Expression_Param, 21, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 22, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 23, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 24, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 25, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 26, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 27, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 28, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 29, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 30, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 31, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 32, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 33, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 34, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 35, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 36, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 37, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 38, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 39, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 40, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 41, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 42, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 43, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 44, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 45, -1, 3, 0}
   , {E_Int_Param            , E_Expression_Param, 16, -1, 3, 0}
   , {E_Int_Param            , E_Expression_Param, 17, -1, 3, 0}
   , {E_Real_Param           , E_Expression_Param, 46, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 47, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 48, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 49, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 50, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 51, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 52, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 53, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 54, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 55, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 56, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 57, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 58, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 59, -1, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 60, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 18, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 19, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 20, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 21, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 22, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 23, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 24, -1, 4, 0}
   , {E_Int_Param            , E_Expression_Param, 25, -1, 4, 0}
   , {E_Text_Param           , E_Expression_Param, 10, -1, 4, 0}
   , {E_Text_Param           , E_Expression_Param, 11, -1, 4, 0}
   , {E_Text_Param           , E_Expression_Param, 12, -1, 4, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 6, 27, 4, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 7, 28, 4, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 8, 35, 4, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 9, 36, 4, 0}
   , {E_Real_Param           , E_Expression_Param, 61, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 62, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 63, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 64, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 65, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 66, -1, 5, 0}
   , {E_Int_Param            , E_Expression_Param, 26, -1, 5, 0}
   , {E_Int_Param            , E_Expression_Param, 27, -1, 5, 0}
   , {E_Int_Param            , E_Expression_Param, 28, -1, 5, 0}
   , {E_Int_Param            , E_Expression_Param, 29, -1, 5, 0}
   , {E_Real_Param           , E_Expression_Param, 67, -1, 6, 0}
   , {E_Real_Param           , E_Expression_Param, 68, -1, 6, 0}
   , {E_Real_Param           , E_Expression_Param, 69, -1, 6, 0}
   , {E_Real_Param           , E_Expression_Param, 70, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 71, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 72, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 73, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 74, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 75, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 76, -1, 6, 1}
   , {E_Real_Param           , E_Expression_Param, 77, -1, 6, 1}
   , {E_Int_Param            , E_Expression_Param, 30, -1, 6, 0}
   , {E_Int_Param            , E_Expression_Param, 31, -1, 6, 1}
   , {E_Int_Param            , E_Expression_Param, 32, -1, 6, 1}
   , {E_ContinuousState_Param, E_Expression_Param, 2, 58, 6, 0}
   , {E_ContinuousState_Param, E_Expression_Param, 3, 60, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 0, 61, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 1, 62, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 2, 63, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 3, 64, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 4, 65, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 5, 66, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 6, 67, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 7, 68, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 8, 69, 6, 0}
   , {E_DiscreteState_Param  , E_Expression_Param, 9, 70, 6, 0}
   , {E_Real_Param           , E_Expression_Param, 78, -1, 7, 1}
   , {E_DiscreteState_Param  , E_Expression_Param, 10, 72, 7, 0}
   , {E_Int_Param            , E_Expression_Param, 33, -1, 8, 0}
   , {E_Int_Param            , E_Expression_Param, 34, -1, 8, 0}
   , {E_Int_Param            , E_Expression_Param, 35, -1, 9, 0}
   , {E_Int_Param            , E_Expression_Param, 36, -1, 9, 0}
   , {E_Int_Param            , E_Expression_Param, 37, -1, 10, 0}
   , {E_Int_Param            , E_Expression_Param, 38, -1, 10, 0}
   , {E_Int_Param            , E_Expression_Param, 39, -1, 11, 0}
   , {E_Int_Param            , E_Expression_Param, 40, -1, 11, 0}
   , {E_Int_Param            , E_Expression_Param, 41, -1, 12, 0}
   , {E_Int_Param            , E_Expression_Param, 42, -1, 12, 0}
   , {E_Int_Param            , E_Expression_Param, 43, -1, 13, 0}
   , {E_Int_Param            , E_Expression_Param, 44, -1, 13, 0}
   , {E_Int_Param            , E_Expression_Param, 45, -1, 14, 0}
   , {E_Int_Param            , E_Expression_Param, 46, -1, 14, 0}
   , {E_Int_Param            , E_Expression_Param, 47, -1, 15, 0}
   , {E_Int_Param            , E_Expression_Param, 48, -1, 15, 0}
   , {E_Int_Param            , E_Expression_Param, 49, -1, 16, 0}
   , {E_Int_Param            , E_Expression_Param, 50, -1, 16, 0}
   , {E_Int_Param            , E_Expression_Param, 51, -1, 17, 0}
   , {E_Int_Param            , E_Expression_Param, 52, -1, 17, 0}
   , {E_Real_Param           , E_Expression_Param, 79, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 80, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 81, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 82, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 83, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 84, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 85, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 86, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 87, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 88, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 89, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 90, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 91, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 92, -1, 18, 0}
   , {E_Real_Param           , E_Expression_Param, 93, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 53, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 54, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 55, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 56, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 57, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 58, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 59, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 60, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 61, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 62, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 63, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 64, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 65, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 66, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 67, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 68, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 69, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 70, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 71, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 72, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 73, -1, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 74, -1, 18, 0}
   , {E_Text_Param           , E_Expression_Param, 13, -1, 18, 0}
   , {E_Text_Param           , E_Expression_Param, 14, -1, 18, 0}
   , {E_Text_Param           , E_Expression_Param, 15, -1, 18, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 10, 13, 18, 0}
   , {E_FixedVar_Param       , E_Expression_Param, 11, 14, 18, 0}
   , {E_ContinuousState_Param, E_Expression_Param, 0, 30, 18, 0}
   , {E_ContinuousState_Param, E_Expression_Param, 1, 31, 18, 0}
   , {E_Int_Param            , E_Expression_Param, 75, -1, 19, 0}
   , {E_Int_Param            , E_Expression_Param, 76, -1, 20, 0}
   , {E_Int_Param            , E_Expression_Param, 77, -1, 21, 0}
};
#define NB_REAL_TUNABLE_PARAM 9
static const int GRealTunableParam[NB_REAL_TUNABLE_PARAM][2] = {
     {70, 62}
   , {71, 63}
   , {72, 64}
   , {73, 65}
   , {74, 66}
   , {75, 67}
   , {76, 68}
   , {77, 69}
   , {78, 72}
};
#define NB_INT_TUNABLE_PARAM 2
static const int GIntTunableParam[NB_INT_TUNABLE_PARAM][2] = {
     {31, 70}
   , {32, 61}
};

static const int GcontStateVarNum[AME_NBOF_SOLVER_STATES] = {
     30, 31, 58, 60, 21
};

static const int GdiscStateVarNum[AME_NBOF_DSTATE_PARAMS] = {
     61, 62, 63, 64, 65, 66, 67, 68, 69, 70
   , 72
};

static const int GInputVarNum[2] = {
     85, 84
};

#define HAVE_GInputStartValues

static const double GInputStartValues[2] = {
     0.00000000000000e+00, 0.00000000000000e+00
};

static const int GOutputVarNum[3] = {
     86, 87, 88
};

static const int GFixedVarNum[AME_NBOF_FIXED_VAR_PARAMS] = {
     7, 8, 9, 16, 17, 18, 27, 28, 35, 36
   , 13, 14
};

#define RS0 (&amesys->pModel->realStoreArray[0])
#define IS0 (&amesys->pModel->intStoreArray[0])
#define RP0 (&amesys->pModel->realParamArray[0])
#define IP0 (&amesys->pModel->integerParamArray[0])
#define TP0 (&amesys->pModel->textParamArray[0])
#define RS1 (&amesys->pModel->realStoreArray[4])
#define IS1 (&amesys->pModel->intStoreArray[4])
#define IP1 (&amesys->pModel->integerParamArray[10])
#define RS2 (&amesys->pModel->realStoreArray[6])
#define IS2 (&amesys->pModel->intStoreArray[5])
#define IP2 (&amesys->pModel->integerParamArray[13])
#define RS3 (&amesys->pModel->realStoreArray[8])
#define IS3 (&amesys->pModel->intStoreArray[6])
#define PS3 (&amesys->pModel->pointerStoreArray[0])
#define RP3 (&amesys->pModel->realParamArray[21])
#define IP3 (&amesys->pModel->integerParamArray[16])
#define RS4 (&amesys->pModel->realStoreArray[19])
#define IS4 (&amesys->pModel->intStoreArray[9])
#define RP4 (&amesys->pModel->realParamArray[46])
#define IP4 (&amesys->pModel->integerParamArray[18])
#define TP4 (&amesys->pModel->textParamArray[10])
#define RS5 (&amesys->pModel->realStoreArray[27])
#define RP5 (&amesys->pModel->realParamArray[61])
#define IP5 (&amesys->pModel->integerParamArray[26])
#define IS6 (&amesys->pModel->intStoreArray[19])
#define RP6 (&amesys->pModel->realParamArray[67])
#define IP6 (&amesys->pModel->integerParamArray[30])
#define RP7 (&amesys->pModel->realParamArray[78])
#define IP9 (&amesys->pModel->integerParamArray[33])
#define IP10 (&amesys->pModel->integerParamArray[35])
#define IP12 (&amesys->pModel->integerParamArray[37])
#define IP13 (&amesys->pModel->integerParamArray[39])
#define IP14 (&amesys->pModel->integerParamArray[41])
#define IP15 (&amesys->pModel->integerParamArray[43])
#define IP16 (&amesys->pModel->integerParamArray[45])
#define IP17 (&amesys->pModel->integerParamArray[47])
#define IP18 (&amesys->pModel->integerParamArray[49])
#define IP19 (&amesys->pModel->integerParamArray[51])
#define IS22 (&amesys->pModel->intStoreArray[21])
#define PS22 (&amesys->pModel->pointerStoreArray[1])
#define RP22 (&amesys->pModel->realParamArray[79])
#define IP22 (&amesys->pModel->integerParamArray[53])
#define TP22 (&amesys->pModel->textParamArray[13])
#define IP23 (&amesys->pModel->integerParamArray[75])
#define IP24 (&amesys->pModel->integerParamArray[76])
#define IP25 (&amesys->pModel->integerParamArray[77])

static int SP9[2] = {1, 1};
static int SP10[2] = {1, 1};
static int SP12[2] = {1, 1};
static int SP13[2] = {1, 1};
static int SP14[2] = {1, 1};
static int SP15[2] = {1, 1};
static int SP16[2] = {1, 1};
static int SP17[2] = {1, 1};
static int SP18[2] = {1, 1};
static int SP19[2] = {1, 1};

static const S_AMEVariableInfo GVarInfo[AME_NB_VAR_INFO] = {
   { 0, 1, 1, 1, "TFFD3", "volume" }
  ,{ 1, 1, 1, 1, "TFFD3", "mass" }
  ,{ 2, 1, 1, 1, "TFVF000", "dm3" }
  ,{ 3, 1, 1, 1, "TFVF000", "dmh3" }
  ,{ 4, 1, 1, 1, "TFVF000", "vol3" }
  ,{ 5, 1, 1, 1, "TFVF000", "dvol3" }
  ,{ 6, 1, 1, 1, "TFVF000", "flowid3" }
  ,{ 7, 1, 1, 1, "TFTK0", "p1" }
  ,{ 8, 1, 1, 1, "TFTK0", "t1" }
  ,{ 9, 0, 1, 1, "TFTK0", "nodeid1" }
  ,{ 10, 0, 1, 1, "TFTK0", "powerStank" }
  ,{ 11, 1, 1, 1, "TFPUC0_SENSED", "dm1" }
  ,{ 12, 1, 1, 1, "TFPUC0_SENSED", "dmh1" }
  ,{ 13, 0, 1, 1, "TFPUC0_SENSED", "vol1" }
  ,{ 14, 0, 1, 1, "TFPUC0_SENSED", "dvol1" }
  ,{ 15, 0, 1, 1, "TFPUC0_SENSED", "flowid1" }
  ,{ 16, 1, 1, 2, "TFTK0", "p1" }
  ,{ 17, 1, 1, 2, "TFTK0", "t1" }
  ,{ 18, 0, 1, 2, "TFTK0", "nodeid1" }
  ,{ 19, 0, 1, 2, "TFTK0", "powerStank" }
  ,{ 20, 1, 1, 1, "UD00", "output" }
  ,{ 21, 1, 1, 1, "CTRPID0", "v" }
  ,{ 22, 1, 1, 1, "TFVF000", "p1" }
  ,{ 23, 1, 1, 1, "TFVF000", "t1" }
  ,{ 24, 1, 1, 1, "TFVF000", "nodeid1" }
  ,{ 25, 1, 1, 1, "TFVORF0", "dm2" }
  ,{ 26, 1, 1, 1, "TFVORF0", "dmh2" }
  ,{ 27, 0, 1, 1, "TFVORF0", "vol2" }
  ,{ 28, 0, 1, 1, "TFVORF0", "dvol2" }
  ,{ 29, 0, 1, 1, "TFVORF0", "flowid2" }
  ,{ 30, 1, 1, 1, "TFPUC0_SENSED", "p2" }
  ,{ 31, 1, 1, 1, "TFPUC0_SENSED", "t2" }
  ,{ 32, 0, 1, 1, "TFPUC0_SENSED", "nodeid2" }
  ,{ 33, 1, 1, 1, "TFVORF0", "dm3" }
  ,{ 34, 1, 1, 1, "TFVORF0", "dmh3" }
  ,{ 35, 0, 1, 1, "TFVORF0", "vol3" }
  ,{ 36, 0, 1, 1, "TFVORF0", "dvol3" }
  ,{ 37, 0, 1, 1, "TFVORF0", "flowid3" }
  ,{ 38, 1, 1, 1, "TFVORF0", "cq" }
  ,{ 39, 1, 1, 1, "TFVORF0", "zeta" }
  ,{ 40, 1, 1, 1, "TFVORF0", "cv" }
  ,{ 41, 1, 1, 1, "TFVORF0", "kv" }
  ,{ 42, 1, 1, 1, "TFVORF0", "lambda" }
  ,{ 43, 1, 1, 1, "TFVORF0", "re" }
  ,{ 44, 1, 1, 1, "TFVORF0", "flv" }
  ,{ 45, 1, 1, 1, "TFVORF0", "marea" }
  ,{ 46, 1, 1, 1, "TFVORF0", "mhdiam" }
  ,{ 47, 1, 1, 1, "TFVORF0", "deltap" }
  ,{ 48, 1, 1, 1, "TFVORF0", "tjet" }
  ,{ 49, 1, 1, 1, "TFVORF0", "q2" }
  ,{ 50, 0, 1, 1, "TFVORF0", "powerRSorif" }
  ,{ 51, 1, 1, 1, "TFVF000", "outv" }
  ,{ 52, 1, 1, 1, "TFVF000", "qv" }
  ,{ 53, 1, 1, 1, "TFVF000", "rho" }
  ,{ 54, 1, 1, 1, "SIGRECEI0", "output" }
  ,{ 55, 1, 1, 1, "CTRPID0", "u" }
  ,{ 56, 1, 1, 1, "CTRPID0", "du" }
  ,{ 57, 1, 1, 1, "CTRPID0", "ppart" }
  ,{ 58, 1, 1, 1, "CTRPID0", "ipart" }
  ,{ 59, 1, 1, 1, "CTRPID0", "dpart" }
  ,{ 60, 1, 1, 1, "CTRPID0", "w" }
  ,{ 61, 1, 1, 1, "CTRPID0", "antiwind0GEN" }
  ,{ 62, 1, 1, 1, "CTRPID0", "Kp0GEN" }
  ,{ 63, 1, 1, 1, "CTRPID0", "Ki0GEN" }
  ,{ 64, 1, 1, 1, "CTRPID0", "Kd0GEN" }
  ,{ 65, 1, 1, 1, "CTRPID0", "tau0GEN" }
  ,{ 66, 1, 1, 1, "CTRPID0", "outmin0GEN" }
  ,{ 67, 1, 1, 1, "CTRPID0", "outmax0GEN" }
  ,{ 68, 1, 1, 1, "CTRPID0", "Ks0GEN" }
  ,{ 69, 1, 1, 1, "CTRPID0", "hyst0GEN" }
  ,{ 70, 1, 1, 1, "CTRPID0", "outlim0GEN" }
  ,{ 71, 1, 1, 1, "CONS00", "out" }
  ,{ 72, 1, 1, 1, "CONS00", "k0GEN" }
  ,{ 73, 1, 1, 4, "SIGTRANS0", "transout" }
  ,{ 74, 1, 1, 2, "SIGRECEI0", "output" }
  ,{ 75, 1, 1, 5, "SIGTRANS0", "transout" }
  ,{ 76, 1, 1, 1, "TFPUC0_SENSED", "TShaft" }
  ,{ 77, 1, 1, 1, "PMV00", "omega" }
  ,{ 78, 1, 1, 1, "SIGTRANS0", "transout" }
  ,{ 79, 1, 1, 1, "INTVARSENSOR0", "output" }
  ,{ 80, 1, 1, 2, "SIGTRANS0", "transout" }
  ,{ 81, 1, 1, 2, "INTVARSENSOR0", "output" }
  ,{ 82, 1, 1, 3, "SIGTRANS0", "transout" }
  ,{ 83, 1, 1, 3, "INTVARSENSOR0", "output" }
  ,{ 84, 1, 1, 1, "AMESIM_INTERFACE", "flowrate_target" }
  ,{ 85, 1, 1, 1, "AMESIM_INTERFACE", "pump_speed" }
  ,{ 86, 1, 1, 3, "SIGRECEI0", "output" }
  ,{ 87, 1, 1, 4, "SIGRECEI0", "output" }
  ,{ 88, 1, 1, 5, "SIGRECEI0", "output" }
  ,{ 89, 1, 1, 1, "TFPUC0_SENSED", "q2" }
  ,{ 90, 1, 1, 1, "TFPUC0_SENSED", "rhoref" }
  ,{ 91, 1, 1, 1, "TFPUC0_SENSED", "dP" }
  ,{ 92, 1, 1, 1, "TFPUC0_SENSED", "pmech" }
  ,{ 93, 1, 1, 1, "TFPUC0_SENSED", "feff" }
  ,{ 94, 0, 1, 1, "TFPUC0_SENSED", "powerRSpump" }
  ,{ 95, 0, 1, 1, "TFPUC0_SENSED", "powerCpump" }
};

#if !defined(AME_IMPLICIT_MODEL_ACCEPTED) && (AME_MODEL_ISEXPLICIT == 0)
#error "Implicit model not supported for the current interface."
#endif


/* ============================================================== */
/* If the interface needs linearisation (cosim and Amesim) */

#ifndef AME_NO_LA
#ifndef AME_NEED_LINEAR_ANALYSIS
#define AME_NEED_LINEAR_ANALYSIS
#endif
#endif

#if( AME_MODEL_ISEXPLICIT == 1)
#define AMEfuncPerturb LPerturbIfNecessary
#else
#define AMEfuncPerturb DPerturbIfNecessary
#endif

#ifdef AME_ADVANCEDDEBUG
static void AME_POST_SUBMODCALL_WITH_DISCON(AMESIMSYSTEM *amesys, int *flag, int *sflag, int *oflag, int *panic, char *submodelname, int instancenum)
{
   if(*sflag < 3)*sflag = getnfg_();
#ifdef AME_NEED_LINEAR_ANALYSIS
   if(*flag == 5)
   {
      AMEfuncPerturb(amesys, flag);
   }
   else if(*oflag != 5)
   {
      resdis(amesys, flag, sflag, oflag, submodelname, instancenum, panic);
   }
#else
   resdis(amesys, flag, sflag, oflag, submodelname, instancenum, panic);
#endif
}

static void AME_POST_SUBMODCALL_NO_DISCON(AMESIMSYSTEM *amesys, int *flag)
{
#ifdef AME_NEED_LINEAR_ANALYSIS
   if(*flag == 5)
   {
      AMEfuncPerturb(amesys, flag);
   }
#endif
}
#endif


#ifndef AME_ADVANCEDDEBUG
#ifdef AME_NEED_LINEAR_ANALYSIS
/* Typically for normal runs and cosim */
#define AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,sflag,oflag,panic,submodelname,instancenum) if(*sflag < 3)*sflag = getnfg_(); if(*flag == 5) AMEfuncPerturb(amesys, flag); else if(*oflag != 5) resdis(amesys, flag, sflag, oflag, submodelname, instancenum, panic)
#define AME_POST_SUBMODCALL_NO_DISCON(amesys,flag) if(*flag == 5) AMEfuncPerturb(amesys, flag)
#else
/* Typically for code exchange (simulink for instance) */
#define AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,sflag,oflag,panic,submodelname,instancenum) if(*sflag < 3)*sflag = getnfg_(); resdis(amesys, flag, sflag, oflag,submodelname,instancenum,panic)
#define AME_POST_SUBMODCALL_NO_DISCON(amesys,flag)
#endif
#endif

#ifdef AMERT
/* We dont need LA nor resdis for real-time - so set them to (almost) empty macros. (reset sflag to orig value "oflag") */
#undef AME_POST_SUBMODCALL_WITH_DISCON
#undef AME_POST_SUBMODCALL_NO_DISCON
#define AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,sflag,oflag,panic,submodelname,instancenum) *sflag = *oflag
#define AME_POST_SUBMODCALL_NO_DISCON(amesys,flag)
#endif

/* ============================================================== */

#ifdef AMERT
double IL_Lemuel_pump_system_FMU_step_ratio=0;
#endif

#ifdef AME_MEMORY_ACCESS_RT_EXPORT
/* For memory access in case of RT target such as dSpace targets */
#include "Lemuel_pump_system_FMU_.export.h"

static void RT_Get_Watch_Var(AMESIMSYSTEM *amesys)
{
#if (NB_WATCH_VAR>0)
   int i;

   for(i=0; i<NB_WATCH_VAR; i++) {
      RT_Export_Vars[i] = amesys->v[RT_Watch_Vars_Idx[i]];
   }
#endif
}

static void RT_Get_Watch_Param(AMESIMSYSTEM *amesys)
{
#if (NB_WATCH_REAL_PARAM>0) || (NB_WATCH_INT_PARAM>0)
   int i;

   for(i=0; i<NB_WATCH_REAL_PARAM; i++) {
      RT_Export_RealParam[i] = amesys->pModel->realParamArray[RT_Watch_RP_Idx[i]];
   }
   for(i=0; i<NB_WATCH_INT_PARAM; i++) {
      RT_Export_IntParam[i] = amesys->pModel->integerParamArray[RT_Watch_IP_Idx[i]];
   }
#endif
}

static void RT_Set_Watch_Param(AMESIMSYSTEM *amesys)
{
#if (NB_WATCH_REAL_PARAM>0) || (NB_WATCH_INT_PARAM>0)
   int i;

   for(i=0; i<NB_WATCH_REAL_PARAM; i++) {
      amesys->pModel->realParamArray[RT_Watch_RP_Idx[i]] = RT_Export_RealParam[i];
   }
   for(i=0; i<NB_WATCH_INT_PARAM; i++) {
      amesys->pModel->integerParamArray[RT_Watch_IP_Idx[i]] = RT_Export_IntParam[i];
   }
#endif
}

#endif

#ifdef AME_INPUT_IN_MEMORY
#include "Lemuel_pump_system_FMU_.sim.h"
#endif

#if( AME_MODEL_ISEXPLICIT == 0 )
#define LRW (40+9*AME_NBOF_SOLVER_STATES+AME_NBOF_SOLVER_STATES*AME_NBOF_SOLVER_STATES)
#define LIW (21+AME_NBOF_SOLVER_STATES)
#endif

// Submodel TFFD3 . 1
// Submodel UD00 . 1
// variable output (Basic var)
static const int EVIA_V0[7] = {20, -1, 0, -1, 1, 1, 1};
// Submodel TFVF000 . 1
// variable p1 (Duplicate var)
static const int EVIA_V1[7] = {22, -1, 1, -1, 1, 1, 1};
// variable t1 (Duplicate var)
static const int EVIA_V2[7] = {23, -1, 2, -1, 1, 1, 1};
// variable nodeid1 (Duplicate var)
static const int EVIA_V3[7] = {24, -1, 3, -1, 1, 1, 1};
// variable dm1 (Basic var)
static const int EVIA_V4[7] = {25, -1, 4, -1, 1, 1, 1};
// variable dmh1 (Basic var)
static const int EVIA_V5[7] = {26, -1, 5, -1, 1, 1, 1};
// variable vol (Basic var)
static const int EVIA_V6[7] = {27, -1, 6, -1, 1, 1, 1};
// variable dvol (Basic var)
static const int EVIA_V7[7] = {28, -1, 7, -1, 1, 1, 1};
// variable flowid1 (Basic var)
static const int EVIA_V8[7] = {29, -1, 8, -1, 1, 1, 1};
// variable outv (Basic var)
static const int EVIA_V9[7] = {51, -1, 9, -1, 1, 1, 1};
// variable dm3 (Duplicate var)
static const int EVIA_V10[7] = {2, -1, 10, -1, 1, 1, 1};
// variable dmh3 (Duplicate var)
static const int EVIA_V11[7] = {3, -1, 11, -1, 1, 1, 1};
// variable vol3 (Duplicate var)
static const int EVIA_V12[7] = {4, -1, 12, -1, 1, 1, 1};
// variable dvol3 (Duplicate var)
static const int EVIA_V13[7] = {5, -1, 13, -1, 1, 1, 1};
// variable flowid3 (Duplicate var)
static const int EVIA_V14[7] = {6, -1, 14, -1, 1, 1, 1};
// variable p3 (Basic var)
static const int EVIA_V15[7] = {7, -1, 15, -1, 1, 1, 1};
// variable t3 (Basic var)
static const int EVIA_V16[7] = {8, -1, 16, -1, 1, 1, 1};
// variable nodeid3 (Basic var)
static const int EVIA_V17[7] = {9, -1, 17, -1, 1, 1, 1};
// Submodel CONS00 . 1
// variable out (Basic var)
static const int EVIA_V18[7] = {71, -1, 18, -1, 1, 1, 1};
// Submodel SSINK . 1
// variable sink (Basic var)
static const int EVIA_V19[7] = {20, -1, 19, -1, 1, 1, 1};
// Submodel SIGTRANS0 . 4
// variable transout (Basic var)
static const int EVIA_V20[7] = {73, -1, 20, -1, 1, 1, 1};
// variable transin (Basic var)
static const int EVIA_V21[7] = {84, -1, 21, -1, 1, 1, 1};
// Submodel SIGTRANS0 . 5
// variable transout (Basic var)
static const int EVIA_V22[7] = {75, -1, 22, -1, 1, 1, 1};
// variable transin (Basic var)
static const int EVIA_V23[7] = {85, -1, 23, -1, 1, 1, 1};
// Submodel SSINK . 2
// variable sink (Basic var)
static const int EVIA_V24[7] = {71, -1, 24, -1, 1, 1, 1};
// Submodel SIGRECEI0 . 1
// variable output (Basic var)
static const int EVIA_V25[7] = {54, -1, 25, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V26[7] = {73, -1, 26, -1, 1, 1, 1};
// Submodel SIGRECEI0 . 2
// variable output (Basic var)
static const int EVIA_V27[7] = {74, -1, 27, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V28[7] = {75, -1, 28, -1, 1, 1, 1};
// Submodel PMV00 . 1
// variable omega (Basic var)
static const int EVIA_V29[7] = {77, -1, 29, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V30[7] = {74, -1, 30, -1, 1, 1, 1};
// Submodel TFPUC0 . 1
// variable dm1 (Basic var)
static const int EVIA_V31[7] = {11, -1, 31, -1, 1, 1, 1};
// variable dmh1 (Basic var)
static const int EVIA_V32[7] = {12, -1, 32, -1, 1, 1, 1};
// variable vol1 (Fixed var)
static const int EVIA_V33[7] = {13, -1, 33, -1, 1, 1, 1};
// variable dvol1 (Fixed var)
static const int EVIA_V34[7] = {14, -1, 34, -1, 1, 1, 1};
// variable flowid1 (Basic var)
static const int EVIA_V35[7] = {15, -1, 35, -1, 1, 1, 1};
// variable p1 (Basic var)
static const int EVIA_V36[7] = {16, -1, 36, -1, 1, 1, 1};
// variable t1 (Basic var)
static const int EVIA_V37[7] = {17, -1, 37, -1, 1, 1, 1};
// variable nodeid1 (Basic var)
static const int EVIA_V38[7] = {18, -1, 38, -1, 1, 1, 1};
// variable p2 (Explicit state)
static const int EVIA_V39[7] = {30, 0, 39, 40, 1, 1, 1};
// variable t2 (Explicit state)
static const int EVIA_V40[7] = {31, 1, 41, 42, 1, 1, 1};
// variable nodeid2 (Multiline var)
static const int EVIA_V41[7] = {32, -1, 43, -1, 1, 1, 1};
// variable dm2 (Basic var)
static const int EVIA_V42[7] = {33, -1, 44, -1, 1, 1, 1};
// variable dmh2 (Basic var)
static const int EVIA_V43[7] = {34, -1, 45, -1, 1, 1, 1};
// variable vol2 (Basic var)
static const int EVIA_V44[7] = {35, -1, 46, -1, 1, 1, 1};
// variable dvol2 (Basic var)
static const int EVIA_V45[7] = {36, -1, 47, -1, 1, 1, 1};
// variable flowid2 (Basic var)
static const int EVIA_V46[7] = {37, -1, 48, -1, 1, 1, 1};
// variable TShaft (Basic var)
static const int EVIA_V47[7] = {76, -1, 49, -1, 1, 1, 1};
// variable nmot (Basic var)
static const int EVIA_V48[7] = {77, -1, 50, -1, 1, 1, 1};
// Submodel TFVORF0 . 1
// variable input (Basic var)
static const int EVIA_V49[7] = {21, -1, 51, -1, 1, 1, 1};
// variable dm2 (Basic var)
static const int EVIA_V50[7] = {25, -1, 52, -1, 1, 1, 1};
// variable dmh2 (Basic var)
static const int EVIA_V51[7] = {26, -1, 53, -1, 1, 1, 1};
// variable vol2 (Fixed var)
static const int EVIA_V52[7] = {27, -1, 54, -1, 1, 1, 1};
// variable dvol2 (Fixed var)
static const int EVIA_V53[7] = {28, -1, 55, -1, 1, 1, 1};
// variable flowid2 (Basic var)
static const int EVIA_V54[7] = {29, -1, 56, -1, 1, 1, 1};
// variable p2 (Basic var)
static const int EVIA_V55[7] = {22, -1, 57, -1, 1, 1, 1};
// variable t2 (Basic var)
static const int EVIA_V56[7] = {23, -1, 58, -1, 1, 1, 1};
// variable nodeid2 (Basic var)
static const int EVIA_V57[7] = {24, -1, 59, -1, 1, 1, 1};
// variable dm3 (Duplicate var)
static const int EVIA_V58[7] = {33, -1, 60, -1, 1, 1, 1};
// variable dmh3 (Duplicate var)
static const int EVIA_V59[7] = {34, -1, 61, -1, 1, 1, 1};
// variable vol3 (Fixed var)
static const int EVIA_V60[7] = {35, -1, 62, -1, 1, 1, 1};
// variable dvol3 (Fixed var)
static const int EVIA_V61[7] = {36, -1, 63, -1, 1, 1, 1};
// variable flowid3 (Basic var)
static const int EVIA_V62[7] = {37, -1, 64, -1, 1, 1, 1};
// variable p3 (Basic var)
static const int EVIA_V63[7] = {30, -1, 65, -1, 1, 1, 1};
// variable t3 (Basic var)
static const int EVIA_V64[7] = {31, -1, 66, -1, 1, 1, 1};
// variable nodeid3 (Basic var)
static const int EVIA_V65[7] = {32, -1, 67, -1, 1, 1, 1};
// Submodel CTRPID0 . 1
// variable y (Basic var)
static const int EVIA_V66[7] = {51, -1, 68, -1, 1, 1, 1};
// variable v (Basic var)
static const int EVIA_V67[7] = {21, -1, 69, -1, 1, 1, 1};
// variable cmd (Basic var)
static const int EVIA_V68[7] = {54, -1, 70, -1, 1, 1, 1};
// Submodel INTVARSENSOR0 . 1
// variable output (Basic var)
static const int EVIA_V69[7] = {79, -1, 71, -1, 1, 1, 1};
// variable sensedvar (Sensor var)
static const int EVIA_V70[7] = {89, -1, 72, -1, 1, 1, 1};
// Submodel INTVARSENSOR0 . 2
// variable output (Basic var)
static const int EVIA_V71[7] = {81, -1, 73, -1, 1, 1, 1};
// variable sensedvar (Sensor var)
static const int EVIA_V72[7] = {91, -1, 74, -1, 1, 1, 1};
// Submodel INTVARSENSOR0 . 3
// variable output (Basic var)
static const int EVIA_V73[7] = {83, -1, 75, -1, 1, 1, 1};
// variable sensedvar (Sensor var)
static const int EVIA_V74[7] = {92, -1, 76, -1, 1, 1, 1};
// Submodel TFTK0 . 1
// variable p1 (Fixed var)
static const int EVIA_V75[7] = {7, -1, 77, -1, 1, 1, 1};
// variable t1 (Fixed var)
static const int EVIA_V76[7] = {8, -1, 78, -1, 1, 1, 1};
// variable nodeid1 (Fixed var)
static const int EVIA_V77[7] = {9, -1, 79, -1, 1, 1, 1};
// variable dm1 (Basic var)
static const int EVIA_V78[7] = {2, -1, 80, -1, 1, 1, 1};
// variable dmh1 (Basic var)
static const int EVIA_V79[7] = {3, -1, 81, -1, 1, 1, 1};
// variable vol1 (Basic var)
static const int EVIA_V80[7] = {4, -1, 82, -1, 1, 1, 1};
// variable dvol1 (Basic var)
static const int EVIA_V81[7] = {5, -1, 83, -1, 1, 1, 1};
// variable flowid1 (Basic var)
static const int EVIA_V82[7] = {6, -1, 84, -1, 1, 1, 1};
// Submodel TFTK0 . 2
// variable p1 (Fixed var)
static const int EVIA_V83[7] = {16, -1, 85, -1, 1, 1, 1};
// variable t1 (Fixed var)
static const int EVIA_V84[7] = {17, -1, 86, -1, 1, 1, 1};
// variable nodeid1 (Fixed var)
static const int EVIA_V85[7] = {18, -1, 87, -1, 1, 1, 1};
// variable dm1 (Basic var)
static const int EVIA_V86[7] = {11, -1, 88, -1, 1, 1, 1};
// variable dmh1 (Basic var)
static const int EVIA_V87[7] = {12, -1, 89, -1, 1, 1, 1};
// variable vol1 (Basic var)
static const int EVIA_V88[7] = {13, -1, 90, -1, 1, 1, 1};
// variable dvol1 (Basic var)
static const int EVIA_V89[7] = {14, -1, 91, -1, 1, 1, 1};
// variable flowid1 (Basic var)
static const int EVIA_V90[7] = {15, -1, 92, -1, 1, 1, 1};
// Submodel SIGTRANS0 . 1
// variable transout (Basic var)
static const int EVIA_V91[7] = {78, -1, 93, -1, 1, 1, 1};
// variable transin (Basic var)
static const int EVIA_V92[7] = {79, -1, 94, -1, 1, 1, 1};
// Submodel SIGTRANS0 . 2
// variable transout (Basic var)
static const int EVIA_V93[7] = {80, -1, 95, -1, 1, 1, 1};
// variable transin (Basic var)
static const int EVIA_V94[7] = {81, -1, 96, -1, 1, 1, 1};
// Submodel SIGTRANS0 . 3
// variable transout (Basic var)
static const int EVIA_V95[7] = {82, -1, 97, -1, 1, 1, 1};
// variable transin (Basic var)
static const int EVIA_V96[7] = {83, -1, 98, -1, 1, 1, 1};
// Submodel SIGRECEI0 . 3
// variable output (Basic var)
static const int EVIA_V97[7] = {86, -1, 99, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V98[7] = {78, -1, 100, -1, 1, 1, 1};
// Submodel SIGRECEI0 . 4
// variable output (Basic var)
static const int EVIA_V99[7] = {87, -1, 101, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V100[7] = {80, -1, 102, -1, 1, 1, 1};
// Submodel SIGRECEI0 . 5
// variable output (Basic var)
static const int EVIA_V101[7] = {88, -1, 103, -1, 1, 1, 1};
// variable input (Basic var)
static const int EVIA_V102[7] = {82, -1, 104, -1, 1, 1, 1};
static const int* EVIA[103] = {
     EVIA_V0, EVIA_V1, EVIA_V2, EVIA_V3, EVIA_V4, EVIA_V5, EVIA_V6, EVIA_V7, EVIA_V8, EVIA_V9
   , EVIA_V10, EVIA_V11, EVIA_V12, EVIA_V13, EVIA_V14, EVIA_V15, EVIA_V16, EVIA_V17, EVIA_V18, EVIA_V19
   , EVIA_V20, EVIA_V21, EVIA_V22, EVIA_V23, EVIA_V24, EVIA_V25, EVIA_V26, EVIA_V27, EVIA_V28, EVIA_V29
   , EVIA_V30, EVIA_V31, EVIA_V32, EVIA_V33, EVIA_V34, EVIA_V35, EVIA_V36, EVIA_V37, EVIA_V38, EVIA_V39
   , EVIA_V40, EVIA_V41, EVIA_V42, EVIA_V43, EVIA_V44, EVIA_V45, EVIA_V46, EVIA_V47, EVIA_V48, EVIA_V49
   , EVIA_V50, EVIA_V51, EVIA_V52, EVIA_V53, EVIA_V54, EVIA_V55, EVIA_V56, EVIA_V57, EVIA_V58, EVIA_V59
   , EVIA_V60, EVIA_V61, EVIA_V62, EVIA_V63, EVIA_V64, EVIA_V65, EVIA_V66, EVIA_V67, EVIA_V68, EVIA_V69
   , EVIA_V70, EVIA_V71, EVIA_V72, EVIA_V73, EVIA_V74, EVIA_V75, EVIA_V76, EVIA_V77, EVIA_V78, EVIA_V79
   , EVIA_V80, EVIA_V81, EVIA_V82, EVIA_V83, EVIA_V84, EVIA_V85, EVIA_V86, EVIA_V87, EVIA_V88, EVIA_V89
   , EVIA_V90, EVIA_V91, EVIA_V92, EVIA_V93, EVIA_V94, EVIA_V95, EVIA_V96, EVIA_V97, EVIA_V98, EVIA_V99
   , EVIA_V100, EVIA_V101, EVIA_V102
};

// Submodel TFFD3 . 1
// variable volume (Basic var)
static const int IVIA_V0[4] = {0, -1, 1, 1};
// variable mass (Basic var)
static const int IVIA_V1[4] = {1, -1, 1, 1};
// Submodel UD00 . 1
// Submodel TFVF000 . 1
// variable qv (Basic var)
static const int IVIA_V2[4] = {52, -1, 1, 1};
// variable rho (Basic var)
static const int IVIA_V3[4] = {53, -1, 1, 1};
// Submodel CONS00 . 1
// variable k0GEN (Discrete var)
static const int IVIA_V4[4] = {72, -1, 1, 1};
// Submodel SSINK . 1
// Submodel SIGTRANS0 . 4
// Submodel SIGTRANS0 . 5
// Submodel SSINK . 2
// Submodel SIGRECEI0 . 1
// Submodel SIGRECEI0 . 2
// Submodel PMV00 . 1
// Submodel TFPUC0 . 1
// variable q2 (Basic var)
static const int IVIA_V5[4] = {89, -1, 1, 1};
// variable rhoref (Basic var)
static const int IVIA_V6[4] = {90, -1, 1, 1};
// variable dP (Basic var)
static const int IVIA_V7[4] = {91, -1, 1, 1};
// variable pmech (Basic var)
static const int IVIA_V8[4] = {92, -1, 1, 1};
// variable feff (Basic var)
static const int IVIA_V9[4] = {93, -1, 1, 1};
// variable fvoid2 (Basic var)
static const int IVIA_V10[4] = {-1, -1, 1, 0};
// variable xg2 (Explicit state)
static const int IVIA_V11[4] = {-1, -1, 1, 0};
// variable xu2 (Explicit state)
static const int IVIA_V12[4] = {-1, -1, 1, 0};
// variable xuv2 (Basic var)
static const int IVIA_V13[4] = {-1, -1, 1, 0};
// variable dmxu1 (Basic var)
static const int IVIA_V14[4] = {-1, -1, 1, 0};
// variable dmxu2 (Basic var)
static const int IVIA_V15[4] = {-1, -1, 1, 0};
// variable powerRSpump (Activity var)
static const int IVIA_V16[4] = {94, -1, 1, 1};
// variable energyRSpump (Explicit state)
static const int IVIA_V17[4] = {-1, -1, 1, 0};
// variable actRSpump (Explicit state)
static const int IVIA_V18[4] = {-1, -1, 1, 0};
// variable powerCpump (Activity var)
static const int IVIA_V19[4] = {95, -1, 1, 1};
// variable energyCpump (Explicit state)
static const int IVIA_V20[4] = {-1, -1, 1, 0};
// variable actCpump (Explicit state)
static const int IVIA_V21[4] = {-1, -1, 1, 0};
// Submodel TFVORF0 . 1
// variable cq (Basic var)
static const int IVIA_V22[4] = {38, -1, 1, 1};
// variable zeta (Basic var)
static const int IVIA_V23[4] = {39, -1, 1, 1};
// variable cv (Basic var)
static const int IVIA_V24[4] = {40, -1, 1, 1};
// variable kv (Basic var)
static const int IVIA_V25[4] = {41, -1, 1, 1};
// variable lambda (Basic var)
static const int IVIA_V26[4] = {42, -1, 1, 1};
// variable re (Basic var)
static const int IVIA_V27[4] = {43, -1, 1, 1};
// variable flv (Basic var)
static const int IVIA_V28[4] = {44, -1, 1, 1};
// variable marea (Basic var)
static const int IVIA_V29[4] = {45, -1, 1, 1};
// variable mhdiam (Basic var)
static const int IVIA_V30[4] = {46, -1, 1, 1};
// variable deltap (Basic var)
static const int IVIA_V31[4] = {47, -1, 1, 1};
// variable tjet (Basic var)
static const int IVIA_V32[4] = {48, -1, 1, 1};
// variable q2 (Basic var)
static const int IVIA_V33[4] = {49, -1, 1, 1};
// variable dmxu2 (Basic var)
static const int IVIA_V34[4] = {-1, -1, 1, 0};
// variable powerRSorif (Activity var)
static const int IVIA_V35[4] = {50, -1, 1, 1};
// variable energyRSorif (Explicit state)
static const int IVIA_V36[4] = {-1, -1, 1, 0};
// variable actRSorif (Explicit state)
static const int IVIA_V37[4] = {-1, -1, 1, 0};
// Submodel CTRPID0 . 1
// variable u (Basic var)
static const int IVIA_V38[4] = {55, -1, 1, 1};
// variable du (Basic var)
static const int IVIA_V39[4] = {56, -1, 1, 1};
// variable ppart (Basic var)
static const int IVIA_V40[4] = {57, -1, 1, 1};
// variable ipart (Explicit state)
static const int IVIA_V41[4] = {58, 2, 1, 1};
// variable dpart (Basic var)
static const int IVIA_V42[4] = {59, -1, 1, 1};
// variable w (Explicit state)
static const int IVIA_V43[4] = {60, 3, 1, 1};
// variable antiwind0GEN (Discrete var)
static const int IVIA_V44[4] = {61, -1, 1, 1};
// variable Kp0GEN (Discrete var)
static const int IVIA_V45[4] = {62, -1, 1, 1};
// variable Ki0GEN (Discrete var)
static const int IVIA_V46[4] = {63, -1, 1, 1};
// variable Kd0GEN (Discrete var)
static const int IVIA_V47[4] = {64, -1, 1, 1};
// variable tau0GEN (Discrete var)
static const int IVIA_V48[4] = {65, -1, 1, 1};
// variable outmin0GEN (Discrete var)
static const int IVIA_V49[4] = {66, -1, 1, 1};
// variable outmax0GEN (Discrete var)
static const int IVIA_V50[4] = {67, -1, 1, 1};
// variable Ks0GEN (Discrete var)
static const int IVIA_V51[4] = {68, -1, 1, 1};
// variable hyst0GEN (Discrete var)
static const int IVIA_V52[4] = {69, -1, 1, 1};
// variable outlim0GEN (Discrete var)
static const int IVIA_V53[4] = {70, -1, 1, 1};
// Submodel INTVARSENSOR0 . 1
// Submodel INTVARSENSOR0 . 2
// Submodel INTVARSENSOR0 . 3
// Submodel TFTK0 . 1
// variable xg1 (Fixed var)
static const int IVIA_V54[4] = {-1, -1, 1, 0};
// variable xu1 (Fixed var)
static const int IVIA_V55[4] = {-1, -1, 1, 0};
// variable powerStank (Activity var)
static const int IVIA_V56[4] = {10, -1, 1, 1};
// variable energyStank (Explicit state)
static const int IVIA_V57[4] = {-1, -1, 1, 0};
// variable actStank (Explicit state)
static const int IVIA_V58[4] = {-1, -1, 1, 0};
// Submodel TFTK0 . 2
// variable xg1 (Fixed var)
static const int IVIA_V59[4] = {-1, -1, 1, 0};
// variable xu1 (Fixed var)
static const int IVIA_V60[4] = {-1, -1, 1, 0};
// variable powerStank (Activity var)
static const int IVIA_V61[4] = {19, -1, 1, 1};
// variable energyStank (Explicit state)
static const int IVIA_V62[4] = {-1, -1, 1, 0};
// variable actStank (Explicit state)
static const int IVIA_V63[4] = {-1, -1, 1, 0};
// Submodel SIGTRANS0 . 1
// Submodel SIGTRANS0 . 2
// Submodel SIGTRANS0 . 3
// Submodel SIGRECEI0 . 3
// Submodel SIGRECEI0 . 4
// Submodel SIGRECEI0 . 5
static const int* IVIA[64] = {
     IVIA_V0, IVIA_V1, IVIA_V2, IVIA_V3, IVIA_V4, IVIA_V5, IVIA_V6, IVIA_V7, IVIA_V8, IVIA_V9
   , IVIA_V10, IVIA_V11, IVIA_V12, IVIA_V13, IVIA_V14, IVIA_V15, IVIA_V16, IVIA_V17, IVIA_V18, IVIA_V19
   , IVIA_V20, IVIA_V21, IVIA_V22, IVIA_V23, IVIA_V24, IVIA_V25, IVIA_V26, IVIA_V27, IVIA_V28, IVIA_V29
   , IVIA_V30, IVIA_V31, IVIA_V32, IVIA_V33, IVIA_V34, IVIA_V35, IVIA_V36, IVIA_V37, IVIA_V38, IVIA_V39
   , IVIA_V40, IVIA_V41, IVIA_V42, IVIA_V43, IVIA_V44, IVIA_V45, IVIA_V46, IVIA_V47, IVIA_V48, IVIA_V49
   , IVIA_V50, IVIA_V51, IVIA_V52, IVIA_V53, IVIA_V54, IVIA_V55, IVIA_V56, IVIA_V57, IVIA_V58, IVIA_V59
   , IVIA_V60, IVIA_V61, IVIA_V62, IVIA_V63
};

static const int PVIA[105] = {
     20, 22, 23, 24, 25, 26, 27, 28, 29, 51
   , 2, 3, 4, 5, 6, 7, 8, 9, 71, 20
   , 73, 84, 75, 85, 71, 54, 73, 74, 75, 77
   , 74, 11, 12, 13, 14, 15, 16, 17, 18, 30
   , 0, 31, 1, 32, 33, 34, 35, 36, 37, 76
   , 77, 21, 25, 26, 27, 28, 29, 22, 23, 24
   , 33, 34, 35, 36, 37, 30, 31, 32, 51, 21
   , 54, 79, 89, 81, 91, 83, 92, 7, 8, 9
   , 2, 3, 4, 5, 6, 16, 17, 18, 11, 12
   , 13, 14, 15, 78, 79, 80, 81, 82, 83, 86
   , 78, 87, 80, 88, 82
};

// Submodel TFFD3 . 1
static const int RPARAM_0[2] = {0, 1};
static const int RPARAM_1[2] = {1, 1};
static const int RPARAM_2[2] = {2, 1};
static const int RPARAM_3[2] = {3, 1};
static const int RPARAM_4[2] = {4, 1};
static const int RPARAM_5[2] = {5, 1};
static const int RPARAM_6[2] = {6, 1};
static const int RPARAM_7[2] = {7, 1};
static const int RPARAM_8[2] = {8, 1};
static const int RPARAM_9[2] = {9, 1};
static const int RPARAM_10[2] = {10, 1};
static const int RPARAM_11[2] = {11, 1};
static const int RPARAM_12[2] = {12, 1};
static const int RPARAM_13[2] = {13, 1};
static const int RPARAM_14[2] = {14, 1};
static const int RPARAM_15[2] = {15, 1};
static const int RPARAM_16[2] = {16, 1};
static const int RPARAM_17[2] = {17, 1};
static const int RPARAM_18[2] = {18, 1};
static const int RPARAM_19[2] = {19, 1};
static const int RPARAM_20[2] = {20, 1};
static const int IPARAM_21[2] = {0, 1};
static const int IPARAM_22[2] = {1, 1};
static const int IPARAM_23[2] = {2, 1};
static const int IPARAM_24[2] = {3, 1};
static const int IPARAM_25[2] = {4, 1};
static const int IPARAM_26[2] = {5, 1};
static const int IPARAM_27[2] = {6, 1};
static const int IPARAM_28[2] = {7, 1};
static const int IPARAM_29[2] = {8, 1};
static const int IPARAM_30[2] = {9, 1};
static const int TPARAM_31[2] = {0, 1};
static const int TPARAM_32[2] = {1, 1};
static const int TPARAM_33[2] = {2, 1};
static const int TPARAM_34[2] = {3, 1};
static const int TPARAM_35[2] = {4, 1};
static const int TPARAM_36[2] = {5, 1};
static const int TPARAM_37[2] = {6, 1};
static const int TPARAM_38[2] = {7, 1};
static const int TPARAM_39[2] = {8, 1};
static const int TPARAM_40[2] = {9, 1};
// Submodel UD00 . 1
static const int RPARAM_41[2] = {0, 1};
static const int RPARAM_42[2] = {1, 1};
static const int RPARAM_43[2] = {2, 1};
static const int RPARAM_44[2] = {3, 1};
static const int RPARAM_45[2] = {4, 1};
static const int RPARAM_46[2] = {5, 1};
static const int RPARAM_47[2] = {6, 1};
static const int RPARAM_48[2] = {7, 1};
static const int RPARAM_49[2] = {8, 1};
static const int RPARAM_50[2] = {9, 1};
static const int RPARAM_51[2] = {10, 1};
static const int RPARAM_52[2] = {11, 1};
static const int RPARAM_53[2] = {12, 1};
static const int RPARAM_54[2] = {13, 1};
static const int RPARAM_55[2] = {14, 1};
static const int RPARAM_56[2] = {15, 1};
static const int RPARAM_57[2] = {16, 1};
static const int RPARAM_58[2] = {17, 1};
static const int RPARAM_59[2] = {18, 1};
static const int RPARAM_60[2] = {19, 1};
static const int RPARAM_61[2] = {20, 1};
static const int RPARAM_62[2] = {21, 1};
static const int RPARAM_63[2] = {22, 1};
static const int RPARAM_64[2] = {23, 1};
static const int RPARAM_65[2] = {24, 1};
static const int IPARAM_66[2] = {0, 1};
static const int IPARAM_67[2] = {1, 1};
// Submodel TFVF000 . 1
static const int RPARAM_68[2] = {0, 1};
static const int RPARAM_69[2] = {1, 1};
static const int RPARAM_70[2] = {2, 1};
static const int RPARAM_71[2] = {3, 1};
static const int RPARAM_72[2] = {4, 1};
static const int RPARAM_73[2] = {5, 1};
static const int IPARAM_74[2] = {0, 1};
static const int IPARAM_75[2] = {1, 1};
static const int IPARAM_76[2] = {2, 1};
static const int IPARAM_77[2] = {3, 1};
// Submodel CONS00 . 1
static const int RPARAM_78[2] = {0, 1};
// Submodel SSINK . 1
// Submodel SIGTRANS0 . 4
static const int IPARAM_79[2] = {0, 1};
static const int IPARAM_80[2] = {1, 1};
// Submodel SIGTRANS0 . 5
static const int IPARAM_81[2] = {0, 1};
static const int IPARAM_82[2] = {1, 1};
// Submodel SSINK . 2
// Submodel SIGRECEI0 . 1
static const int IPARAM_83[2] = {0, 1};
static const int IPARAM_84[2] = {1, 1};
// Submodel SIGRECEI0 . 2
static const int IPARAM_85[2] = {0, 1};
static const int IPARAM_86[2] = {1, 1};
// Submodel PMV00 . 1
// Submodel TFPUC0 . 1
static const int RPARAM_87[2] = {0, 1};
static const int RPARAM_88[2] = {1, 1};
static const int RPARAM_89[2] = {2, 1};
static const int RPARAM_90[2] = {3, 1};
static const int RPARAM_91[2] = {4, 1};
static const int RPARAM_92[2] = {5, 1};
static const int RPARAM_93[2] = {6, 1};
static const int RPARAM_94[2] = {7, 1};
static const int RPARAM_95[2] = {8, 1};
static const int RPARAM_96[2] = {9, 1};
static const int RPARAM_97[2] = {10, 1};
static const int RPARAM_98[2] = {11, 1};
static const int RPARAM_99[2] = {12, 1};
static const int RPARAM_100[2] = {13, 1};
static const int RPARAM_101[2] = {14, 1};
static const int IPARAM_102[2] = {0, 1};
static const int IPARAM_103[2] = {1, 1};
static const int IPARAM_104[2] = {2, 1};
static const int IPARAM_105[2] = {3, 1};
static const int IPARAM_106[2] = {4, 1};
static const int IPARAM_107[2] = {5, 1};
static const int IPARAM_108[2] = {6, 1};
static const int IPARAM_109[2] = {7, 1};
static const int IPARAM_110[2] = {8, 1};
static const int IPARAM_111[2] = {9, 1};
static const int IPARAM_112[2] = {10, 1};
static const int IPARAM_113[2] = {11, 1};
static const int IPARAM_114[2] = {12, 1};
static const int IPARAM_115[2] = {13, 1};
static const int IPARAM_116[2] = {14, 1};
static const int IPARAM_117[2] = {15, 1};
static const int IPARAM_118[2] = {16, 1};
static const int IPARAM_119[2] = {17, 1};
static const int IPARAM_120[2] = {18, 1};
static const int IPARAM_121[2] = {19, 1};
static const int IPARAM_122[2] = {20, 1};
static const int IPARAM_123[2] = {21, 1};
static const int TPARAM_124[2] = {0, 1};
static const int TPARAM_125[2] = {1, 1};
static const int TPARAM_126[2] = {2, 1};
// Submodel TFVORF0 . 1
static const int RPARAM_127[2] = {0, 1};
static const int RPARAM_128[2] = {1, 1};
static const int RPARAM_129[2] = {2, 1};
static const int RPARAM_130[2] = {3, 1};
static const int RPARAM_131[2] = {4, 1};
static const int RPARAM_132[2] = {5, 1};
static const int RPARAM_133[2] = {6, 1};
static const int RPARAM_134[2] = {7, 1};
static const int RPARAM_135[2] = {8, 1};
static const int RPARAM_136[2] = {9, 1};
static const int RPARAM_137[2] = {10, 1};
static const int RPARAM_138[2] = {11, 1};
static const int RPARAM_139[2] = {12, 1};
static const int RPARAM_140[2] = {13, 1};
static const int RPARAM_141[2] = {14, 1};
static const int IPARAM_142[2] = {0, 1};
static const int IPARAM_143[2] = {1, 1};
static const int IPARAM_144[2] = {2, 1};
static const int IPARAM_145[2] = {3, 1};
static const int IPARAM_146[2] = {4, 1};
static const int IPARAM_147[2] = {5, 1};
static const int IPARAM_148[2] = {6, 1};
static const int IPARAM_149[2] = {7, 1};
static const int TPARAM_150[2] = {0, 1};
static const int TPARAM_151[2] = {1, 1};
static const int TPARAM_152[2] = {2, 1};
// Submodel CTRPID0 . 1
static const int RPARAM_153[2] = {0, 1};
static const int RPARAM_154[2] = {1, 1};
static const int RPARAM_155[2] = {2, 1};
static const int RPARAM_156[2] = {3, 1};
static const int RPARAM_157[2] = {4, 1};
static const int RPARAM_158[2] = {5, 1};
static const int RPARAM_159[2] = {6, 1};
static const int RPARAM_160[2] = {7, 1};
static const int RPARAM_161[2] = {8, 1};
static const int RPARAM_162[2] = {9, 1};
static const int RPARAM_163[2] = {10, 1};
static const int IPARAM_164[2] = {0, 1};
static const int IPARAM_165[2] = {1, 1};
static const int IPARAM_166[2] = {2, 1};
// Submodel INTVARSENSOR0 . 1
static const int IPARAM_167[2] = {0, 1};
// Submodel INTVARSENSOR0 . 2
static const int IPARAM_168[2] = {0, 1};
// Submodel INTVARSENSOR0 . 3
static const int IPARAM_169[2] = {0, 1};
// Submodel TFTK0 . 1
static const int IPARAM_170[2] = {0, 1};
static const int IPARAM_171[2] = {1, 1};
static const int IPARAM_172[2] = {2, 1};
// Submodel TFTK0 . 2
static const int IPARAM_173[2] = {0, 1};
static const int IPARAM_174[2] = {1, 1};
static const int IPARAM_175[2] = {2, 1};
// Submodel SIGTRANS0 . 1
static const int IPARAM_176[2] = {0, 1};
static const int IPARAM_177[2] = {1, 1};
// Submodel SIGTRANS0 . 2
static const int IPARAM_178[2] = {0, 1};
static const int IPARAM_179[2] = {1, 1};
// Submodel SIGTRANS0 . 3
static const int IPARAM_180[2] = {0, 1};
static const int IPARAM_181[2] = {1, 1};
// Submodel SIGRECEI0 . 3
static const int IPARAM_182[2] = {0, 1};
static const int IPARAM_183[2] = {1, 1};
// Submodel SIGRECEI0 . 4
static const int IPARAM_184[2] = {0, 1};
static const int IPARAM_185[2] = {1, 1};
// Submodel SIGRECEI0 . 5
static const int IPARAM_186[2] = {0, 1};
static const int IPARAM_187[2] = {1, 1};
static const int* PIA[188] = {
     RPARAM_0, RPARAM_1, RPARAM_2, RPARAM_3, RPARAM_4, RPARAM_5, RPARAM_6, RPARAM_7, RPARAM_8, RPARAM_9
   , RPARAM_10, RPARAM_11, RPARAM_12, RPARAM_13, RPARAM_14, RPARAM_15, RPARAM_16, RPARAM_17, RPARAM_18, RPARAM_19
   , RPARAM_20, IPARAM_21, IPARAM_22, IPARAM_23, IPARAM_24, IPARAM_25, IPARAM_26, IPARAM_27, IPARAM_28, IPARAM_29
   , IPARAM_30, TPARAM_31, TPARAM_32, TPARAM_33, TPARAM_34, TPARAM_35, TPARAM_36, TPARAM_37, TPARAM_38, TPARAM_39
   , TPARAM_40, RPARAM_41, RPARAM_42, RPARAM_43, RPARAM_44, RPARAM_45, RPARAM_46, RPARAM_47, RPARAM_48, RPARAM_49
   , RPARAM_50, RPARAM_51, RPARAM_52, RPARAM_53, RPARAM_54, RPARAM_55, RPARAM_56, RPARAM_57, RPARAM_58, RPARAM_59
   , RPARAM_60, RPARAM_61, RPARAM_62, RPARAM_63, RPARAM_64, RPARAM_65, IPARAM_66, IPARAM_67, RPARAM_68, RPARAM_69
   , RPARAM_70, RPARAM_71, RPARAM_72, RPARAM_73, IPARAM_74, IPARAM_75, IPARAM_76, IPARAM_77, RPARAM_78, IPARAM_79
   , IPARAM_80, IPARAM_81, IPARAM_82, IPARAM_83, IPARAM_84, IPARAM_85, IPARAM_86, RPARAM_87, RPARAM_88, RPARAM_89
   , RPARAM_90, RPARAM_91, RPARAM_92, RPARAM_93, RPARAM_94, RPARAM_95, RPARAM_96, RPARAM_97, RPARAM_98, RPARAM_99
   , RPARAM_100, RPARAM_101, IPARAM_102, IPARAM_103, IPARAM_104, IPARAM_105, IPARAM_106, IPARAM_107, IPARAM_108, IPARAM_109
   , IPARAM_110, IPARAM_111, IPARAM_112, IPARAM_113, IPARAM_114, IPARAM_115, IPARAM_116, IPARAM_117, IPARAM_118, IPARAM_119
   , IPARAM_120, IPARAM_121, IPARAM_122, IPARAM_123, TPARAM_124, TPARAM_125, TPARAM_126, RPARAM_127, RPARAM_128, RPARAM_129
   , RPARAM_130, RPARAM_131, RPARAM_132, RPARAM_133, RPARAM_134, RPARAM_135, RPARAM_136, RPARAM_137, RPARAM_138, RPARAM_139
   , RPARAM_140, RPARAM_141, IPARAM_142, IPARAM_143, IPARAM_144, IPARAM_145, IPARAM_146, IPARAM_147, IPARAM_148, IPARAM_149
   , TPARAM_150, TPARAM_151, TPARAM_152, RPARAM_153, RPARAM_154, RPARAM_155, RPARAM_156, RPARAM_157, RPARAM_158, RPARAM_159
   , RPARAM_160, RPARAM_161, RPARAM_162, RPARAM_163, IPARAM_164, IPARAM_165, IPARAM_166, IPARAM_167, IPARAM_168, IPARAM_169
   , IPARAM_170, IPARAM_171, IPARAM_172, IPARAM_173, IPARAM_174, IPARAM_175, IPARAM_176, IPARAM_177, IPARAM_178, IPARAM_179
   , IPARAM_180, IPARAM_181, IPARAM_182, IPARAM_183, IPARAM_184, IPARAM_185, IPARAM_186, IPARAM_187
};


static const int** NGIA = NULL;

static const int* NIA = NULL;

static const int SUBSIMU[25][6][2] = {
     {{21, 0}, {10, 0}, {10, 0}, {4, 0}, {4, 0}, {0, 0}}
   , {{25, 21}, {2, 16}, {0, 10}, {11, 8}, {3, 6}, {1, 0}}
   , {{6, 61}, {4, 26}, {0, 13}, {2, 27}, {0, 19}, {0, 1}}
   , {{1, 78}, {0, 33}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {0, 33}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 43}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 45}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 94}, {0, 78}, {0, 16}, {0, 29}, {0, 23}, {0, 3}}
   , {{0, 79}, {2, 33}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 35}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {0, 37}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{15, 79}, {22, 53}, {3, 13}, {0, 29}, {2, 21}, {2, 1}}
   , {{15, 46}, {8, 18}, {3, 10}, {8, 19}, {10, 9}, {0, 1}}
   , {{11, 67}, {3, 30}, {0, 13}, {0, 29}, {2, 19}, {0, 1}}
   , {{0, 94}, {1, 75}, {0, 16}, {0, 29}, {0, 23}, {0, 3}}
   , {{0, 94}, {1, 76}, {0, 16}, {0, 29}, {0, 23}, {0, 3}}
   , {{0, 94}, {1, 77}, {0, 16}, {0, 29}, {0, 23}, {0, 3}}
   , {{0, 21}, {3, 10}, {0, 10}, {2, 4}, {1, 4}, {0, 0}}
   , {{0, 21}, {3, 13}, {0, 10}, {2, 6}, {1, 5}, {0, 0}}
   , {{0, 79}, {2, 37}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 39}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 41}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 47}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 49}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
   , {{0, 79}, {2, 51}, {0, 13}, {0, 29}, {0, 21}, {0, 1}}
};

static const S_AMESubStructuralInfo STLI_0 = {1, NULL, &IVIA[0], NULL, &PIA[0], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_1 = {1, &EVIA[0], NULL, PVIA, &PIA[41], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_2 = {1, &EVIA[1], &IVIA[2], PVIA, &PIA[68], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_3 = {1, &EVIA[18], &IVIA[4], PVIA, &PIA[78], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_4 = {1, &EVIA[19], NULL, PVIA, NULL, NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_5 = {4, &EVIA[20], NULL, PVIA, &PIA[79], SP15, NULL, NULL};
static const S_AMESubStructuralInfo STLI_6 = {5, &EVIA[22], NULL, PVIA, &PIA[81], SP16, NULL, NULL};
static const S_AMESubStructuralInfo STLI_7 = {2, &EVIA[24], NULL, PVIA, NULL, NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_8 = {1, &EVIA[25], NULL, PVIA, &PIA[83], SP9, NULL, NULL};
static const S_AMESubStructuralInfo STLI_9 = {2, &EVIA[27], NULL, PVIA, &PIA[85], SP10, NULL, NULL};
static const S_AMESubStructuralInfo STLI_10 = {1, &EVIA[29], NULL, PVIA, NULL, NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_11 = {1, &EVIA[31], &IVIA[5], PVIA, &PIA[87], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_12 = {1, &EVIA[49], &IVIA[22], PVIA, &PIA[127], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_13 = {1, &EVIA[66], &IVIA[38], PVIA, &PIA[153], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_14 = {1, &EVIA[69], NULL, PVIA, &PIA[167], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_15 = {2, &EVIA[71], NULL, PVIA, &PIA[168], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_16 = {3, &EVIA[73], NULL, PVIA, &PIA[169], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_17 = {1, &EVIA[75], &IVIA[54], PVIA, &PIA[170], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_18 = {2, &EVIA[83], &IVIA[59], PVIA, &PIA[173], NULL, NULL, NULL};
static const S_AMESubStructuralInfo STLI_19 = {1, &EVIA[91], NULL, PVIA, &PIA[176], SP12, NULL, NULL};
static const S_AMESubStructuralInfo STLI_20 = {2, &EVIA[93], NULL, PVIA, &PIA[178], SP13, NULL, NULL};
static const S_AMESubStructuralInfo STLI_21 = {3, &EVIA[95], NULL, PVIA, &PIA[180], SP14, NULL, NULL};
static const S_AMESubStructuralInfo STLI_22 = {3, &EVIA[97], NULL, PVIA, &PIA[182], SP17, NULL, NULL};
static const S_AMESubStructuralInfo STLI_23 = {4, &EVIA[99], NULL, PVIA, &PIA[184], SP18, NULL, NULL};
static const S_AMESubStructuralInfo STLI_24 = {5, &EVIA[101], NULL, PVIA, &PIA[186], SP19, NULL, NULL};

/* *******************  Function prototypes ************ */

extern void tffd3prein_(int *n, double *RP, int *IP, char **TP
                 , double *RS, int *IS);
extern void tffd3in_(int *n, double *RP, int *IP, char **TP, double *RS
                 , int *IS);
extern void tffd3_(int *n, double *vi0, double *vi1, double *RP
                 , int *IP, char **TP, double *RS, int *IS);
extern AMEModelStatus ud00_in(const S_AMESubIntf *sitf);
extern AMEModelStatus ud00_calc(const S_AMESubIntf *sitf);
extern AMEModelStatus ud00_end(const S_AMESubIntf *sitf);
extern AMEModelStatus cons00_in(const S_AMESubIntf *sitf);
extern AMEModelStatus cons00_calc(const S_AMESubIntf *sitf);
extern AMEModelStatus cons00_end(const S_AMESubIntf *sitf);
extern AMEModelStatus ssink_in(const S_AMESubIntf *sitf);
extern AMEModelStatus ssink_end(const S_AMESubIntf *sitf);
extern AMEModelStatus sigrecei0_in(const S_AMESubIntf *sitf);
extern AMEModelStatus sigrecei0_calc(const S_AMESubIntf *sitf);
extern AMEModelStatus sigrecei0_end(const S_AMESubIntf *sitf);
extern void pmv00in_(int *n);
extern void pmv00_(int *n, double *ve0, double *ve1);
extern void tfvorf0in_(int *n, double *RP, int *IP, char **TP
                 , double *RS, int *IS, double *y0, double *y1
                   , double *y2, double *y3, double *y4, double *y5
                   , double *y6, double *y7);
extern void tfvorf0_(int *n, double *port_1_v1, double *port_2_v1
                   , double *port_2_v2, double *port_2_v5, double *port_2_v6
                   , double *port_2_v7, double *port_2_v8, double *port_3_v5
                   , double *port_3_v6, double *port_3_v7, double *port_3_v8
                   , double *int_v1, double *int_v2, double *int_v3
                   , double *int_v4, double *int_v5, double *int_v6
                   , double *int_v7, double *int_v8, double *int_v9
                   , double *int_v10, double *int_v11, double *int_v12
                   , double *int_v13, double *int_v14, double *int_v15
                   , double *int_dv15, double *int_v16, double *int_dv16
                   , double *RP, int *IP, char **TP, double *RS
                 , int *IS);
extern void tfvf000in_(int *n, double *RP, int *IP, double *RS
                 , double *y0, double *y1, double *y2, double *y3
                   );
extern void tfvf000_(int *n, double *ve0, double *ve1, double *ve2
                   , double *ve3, double *ve4, double *ve5, double *ve6
                   , double *ve7, double *ve8, double *vi9, double *vi10
                   , double *RP, int *IP, double *RS);
extern AMEModelStatus ctrpid0_in(const S_AMESubIntf *sitf);
extern AMEModelStatus ctrpid0_calc(const S_AMESubIntf *sitf);
extern AMEModelStatus ctrpid0_end(const S_AMESubIntf *sitf);
extern void tfpuc0in_(int *n, double *RP, int *IP, char **TP, int *IS
                 , void **PS, double *y0, double *y1, double *y2
                   , double *y3, double *y4, double *y5, double *y6
                   , double *y7, double *y8, double *y9, double *y10
                   , double *y11, double *y12, double *y13);
extern void tfpuc0_(int *n, double *port_1_v1, double *port_1_v2
                   , double *port_1_v5, double *port_1_v6, double *port_1_v7
                   , double *port_1_v8, double *port_2_v1, double *port_2_dv1
                   , double *port_2_v2, double *port_2_dv2, double *port_2_v3
                   , double *port_2_v4, double *port_2_v5, double *port_2_v6
                   , double *port_2_v7, double *port_2_v8, double *port_3_v1
                   , double *port_3_v2, double *int_v1, double *int_v2
                   , double *int_v3, double *int_v4, double *int_v5
                   , double *int_v6, double *int_v7, double *int_dv7
                   , double *int_v8, double *int_dv8, double *int_v9
                   , double *int_v10, double *int_v11, double *int_v12
                   , double *int_v13, double *int_dv13, double *int_v14
                   , double *int_dv14, double *int_v15, double *int_v16
                   , double *int_dv16, double *int_v17, double *int_dv17
                   , double *RP, int *IP, char **TP, int *IS, void **PS
                 );
extern void tfpuc0end_(int *n, double *RP, int *IP, char **TP
                 , int *IS, void **PS, double *y0, double *y1
                   , double *y2, double *y3, double *y4, double *y5
                   , double *y6, double *y7, double *y8, double *y9
                   , double *y10, double *y11, double *y12, double *y13
                   );
extern double tfpuc0_nodeid2_(int *n, double *m0, double *m1, double *m2
              , double *m3, double *RP, int *IP, char **TP, int *IS
                 , void **PS, int *macindex);
extern void intvarsensor0in_(int *n, int *IP);
extern void intvarsensor0_(int *n, double *port_1_v1, double *port_2_v1
                   , int *IP);
extern void tftk0in_(int *n, int *IP, double *RS, int *IS, double *y0
                   , double *y1, double *y2, double *y3, double *y4
                   , double *y5, double *y6, double *y7, double *y8
                   , double *y9);
extern void tftk0_(int *n, double *port_1_v4, double *port_1_v5
                   , double *port_1_v6, double *port_1_v7, double *port_1_v8
                   , double *int_v3, double *int_v4, double *int_dv4
                   , double *int_v5, double *int_dv5, int *IP
                 , double *RS, int *IS);
extern AMEModelStatus sigtrans0_in(const S_AMESubIntf *sitf);
extern AMEModelStatus sigtrans0_calc(const S_AMESubIntf *sitf);
extern AMEModelStatus sigtrans0_end(const S_AMESubIntf *sitf);

/* ******************    Here comes the functions ************ */
static void PreInitialize(AMESIMSYSTEM *amesys, double *y)
{
   int n = 0;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
   double *dbk_wk = amesys->pModel->dbk_wk;
   const double t = getstarttime_();
   S_AMESubIntf sitf = {amesys, NULL, NULL, t, NULL, v, NULL};
   AMEModelStatus status = AME_MODEL_OK;

   double dummy[4];

   (void)n;

   /* Assign continuous state variables calculated by the integrator */
#if( (AME_MODEL_ISEXPLICIT == 0) && (AME_NBOF_SOLVER_STATES > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_SOLVER_STATES; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#elif( (AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_EXPLICIT_STATE; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#endif

   /* Assign discrete state variables */
#if( AME_NBOF_DISCRETE_STATE > 0 )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_DISCRETE_STATE; idxState++) {
         v[GdiscStateVarNum[idxState]] = Z[idxState];
      }
   }
#endif

   InitializeUnitsManagerAsDefault();


   n = 1;
   tffd3prein_(&n, RP0, IP0, TP0, RS0, IS0);

   /* Assign continuous state variables initialized by submodels */
#if( (AME_MODEL_ISEXPLICIT == 0) && (AME_NBOF_SOLVER_STATES > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_SOLVER_STATES; idxState++) {
         y[idxState] = v[GcontStateVarNum[idxState]];
      }
   }
#elif( (AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_EXPLICIT_STATE; idxState++) {
         y[idxState] = v[GcontStateVarNum[idxState]];
      }
   }
#endif

   /* Assign discrete state initialized by submodels */
#if( AME_NBOF_DISCRETE_STATE > 0 )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_DISCRETE_STATE; idxState++) {
         Z[idxState] = v[GdiscStateVarNum[idxState]];
      }
   }
#endif

   if(status & AME_MODEL_FATAL) AmeExit(1);
}

static void Initialize(AMESIMSYSTEM *amesys, double *y)
{
   int n;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
   double *dbk_wk = amesys->pModel->dbk_wk;
   const double t = getstarttime_();
   S_AMESubIntf sitf = {amesys, NULL, NULL, t, NULL, v, NULL};
   AMEModelStatus status = AME_MODEL_OK;

   double dummy[4];

   (void)n;

   /* Assign continuous state variables */
#if( (AME_MODEL_ISEXPLICIT == 0) && (AME_NBOF_SOLVER_STATES > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_SOLVER_STATES; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#elif( (AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_EXPLICIT_STATE; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#endif

   /* Assign discrete state variables */
#if( AME_NBOF_DISCRETE_STATE > 0 )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_DISCRETE_STATE; idxState++) {
         v[GdiscStateVarNum[idxState]] = Z[idxState];
      }
   }
#endif


   n = 1;
   tffd3in_(&n, RP0, IP0, TP0, RS0, IS0);
   sitf.structuralInfo = &STLI_1;
   sitf.simulationInfo = &amesys->pModel->subArray[1];
   status |= ud00_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_3;
   sitf.simulationInfo = &amesys->pModel->subArray[3];
   status |= cons00_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_4;
   sitf.simulationInfo = &amesys->pModel->subArray[4];
   status |= ssink_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_5;
   sitf.simulationInfo = &amesys->pModel->subArray[5];
   status |= sigtrans0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_6;
   sitf.simulationInfo = &amesys->pModel->subArray[6];
   status |= sigtrans0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_7;
   sitf.simulationInfo = &amesys->pModel->subArray[7];
   status |= ssink_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_8;
   sitf.simulationInfo = &amesys->pModel->subArray[8];
   status |= sigrecei0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_9;
   sitf.simulationInfo = &amesys->pModel->subArray[9];
   status |= sigrecei0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;

   n = 1;
   pmv00in_(&n);

   n = 1;
   {


      tfvorf0in_(&n, RP4, IP4, TP4, RS4, IS4, &v[27], &v[28], &dummy[0]
         , &v[35], &v[36], &dummy[1], NULL, NULL);
   }

   n = 1;
   tfvf000in_(&n, RP5, IP5, RS5, &dummy[0], &dummy[1], &dummy[2]
      , &dummy[3]);
   sitf.structuralInfo = &STLI_13;
   sitf.simulationInfo = &amesys->pModel->subArray[13];
   status |= ctrpid0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;

   n = 1;
   {


      tfpuc0in_(&n, RP22, IP22, TP22, IS22, PS22, &v[13], &v[14]
         , &dummy[0], &v[30], &v[31], &dummy[1], &dummy[2], &dummy[3]
         , NULL, NULL, NULL, NULL, NULL, NULL);
   }

   n = 1;
   {


      intvarsensor0in_(&n, IP23);
   }

   n = 2;
   {


      intvarsensor0in_(&n, IP24);
   }

   n = 3;
   {


      intvarsensor0in_(&n, IP25);
   }

   n = 1;
   {


      tftk0in_(&n, IP1, RS1, IS1, &v[7], &v[8], &v[9], &dummy[0]
         , &dummy[1], &dummy[2], NULL, NULL, NULL, NULL);
   }

   n = 2;
   {


      tftk0in_(&n, IP2, RS2, IS2, &v[16], &v[17], &v[18], &dummy[0]
         , &dummy[1], &dummy[2], NULL, NULL, NULL, NULL);
   }
   sitf.structuralInfo = &STLI_19;
   sitf.simulationInfo = &amesys->pModel->subArray[19];
   status |= sigtrans0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_20;
   sitf.simulationInfo = &amesys->pModel->subArray[20];
   status |= sigtrans0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_21;
   sitf.simulationInfo = &amesys->pModel->subArray[21];
   status |= sigtrans0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_22;
   sitf.simulationInfo = &amesys->pModel->subArray[22];
   status |= sigrecei0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_23;
   sitf.simulationInfo = &amesys->pModel->subArray[23];
   status |= sigrecei0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;
   sitf.structuralInfo = &STLI_24;
   sitf.simulationInfo = &amesys->pModel->subArray[24];
   status |= sigrecei0_in(&sitf);
   if(status & AME_MODEL_FATAL) goto _amefunc_init_end;

   /* Assign continuous state variables initialized by submodels */
#if( (AME_MODEL_ISEXPLICIT == 0) && (AME_NBOF_SOLVER_STATES > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_SOLVER_STATES; idxState++) {
         y[idxState] = v[GcontStateVarNum[idxState]];
      }
   }
#elif( (AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_EXPLICIT_STATE; idxState++) {
         y[idxState] = v[GcontStateVarNum[idxState]];
      }
   }
#endif

   /* Assign discrete state initialized by submodels */
#if( AME_NBOF_DISCRETE_STATE > 0 )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_DISCRETE_STATE; idxState++) {
         Z[idxState] = v[GdiscStateVarNum[idxState]];
      }
   }
#endif

_amefunc_init_end:
   if(status & AME_MODEL_FATAL) AmeExit(1);
}

#define AME_HAS_ENABLED_SUBMODEL    0

static void localFuncEval(AMESIMSYSTEM *amesys, double t, double *y, double *yprime, double *delta, int *flag)
{
   int sflag, oflag, n, panic, i=0;
   int *oldflag, *newflag;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
   double *input = amesys->inputs;
   double *output = amesys->outputs;
   double *dbk_wk = amesys->pModel->dbk_wk;
   AMEModelStatus status = AME_MODEL_OK;

#if(AME_MODEL_ISEXPLICIT == 1)
   double *dot = yprime;
   S_AMESubIntf sitf = {amesys, NULL, NULL, t, &sflag, v, dot};

#if(AME_HAS_ENABLED_SUBMODEL == 1)
   memset(dot,0,AME_NBOF_SOLVER_STATES*sizeof(double));
#elif (AME_NBOF_EXPLICIT_STATE == 0)
   dot[0] = 0.0;
#endif

#else
   double dot[AME_NBOF_SOLVER_STATES];
   S_AMESubIntf sitf = {amesys, NULL, NULL, t, &sflag, v, dot};

   /* Initialize the dot vector to the yprime vector. */
   memcpy((void *)dot, (void *)yprime, AME_NBOF_SOLVER_STATES*sizeof(double));
#endif

   SetGlobalSystem(amesys);

   (void)n;

#if(defined(AME_COSIM) && (AME_NBOF_INPUTS > 0))
   if(amesys->doInterpol) {
      if( getInputsCosimSlave(amesys->csSlave, t, input) != AME_NO_ERROR ) {
         AmeExit(1);
      }
   }
#endif

   /* Record old value of flag (oflag) and set
      flag value for use in submodels (sflag).
      Also get addresses of main discontinuity flags. */

   oflag = *flag;
   sflag = *flag;

   if(amesys->first_call)
   {
      GetFlagAddresses(&amesys->oldflag, &amesys->newflag);
   }
   oldflag = amesys->oldflag;
   newflag = amesys->newflag;

   /* Initialize everything ready for potential calls to stepdn
      in submodels. */

   panic = 0;
   getredstep();

   if(isstabrun_())
   {
      t = amesys->simOptions->fixedTime;
      sitf.t = t;
   }
   else if(*flag == 2)
   {
      /* Record current simulation time for message passing. */

      SetSimTime(t);
   }
   /* Record current simulation time for ametim_(). */

   SetTimeAtThisStep(t);

   if (holdinputs_())
   {
      /* We reset artificially the time to the initial value
         to give the illusion of held inputs. */
      t = getstarttime_();
      sitf.t = t;
   }
   /* Assign the state variables y[] calculated by the integrator
      to the appropriate variables v[]. */

   /* Assign continuous state variables calculated by the integrator */
#if( (AME_MODEL_ISEXPLICIT == 0) && (AME_NBOF_SOLVER_STATES > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_SOLVER_STATES; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#elif( (AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE > 0) )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_EXPLICIT_STATE; idxState++) {
         v[GcontStateVarNum[idxState]] = y[idxState];
      }
   }
#endif

   /* Assign discrete state variables */
#if( AME_NBOF_DISCRETE_STATE > 0 )
   {
      int idxState;
      for(idxState = 0; idxState < AME_NBOF_DISCRETE_STATE; idxState++) {
         v[GdiscStateVarNum[idxState]] = Z[idxState];
      }
   }
#endif

   /* Assign the interface input variables to the appropriate variable v(). */
#if(AME_NBOF_INPUTS > 0)
   {
      int idxInput;
      for(idxInput = 0; idxInput < AME_NBOF_INPUTS; idxInput++) {
         v[GInputVarNum[idxInput]] = input[idxInput];
      }
   }
#endif

#if(AME_MODEL_ISEXPLICIT == 1)
  /* The following call ensures that lsoda does not integrate past
      time amesys->t_end_of_time_slice. This does not matter in a standard AMESim run but is
      very important with cosimulation. */

#ifdef AME_COSIM
   *oldflag = *newflag = sflag;
   discInput(&amesys->t_end_of_time_slice);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag,&sflag,&oflag,&panic,"_TunableParam",1);
#endif

#else
   if(*flag == 5)
   {
      DPerturbIfNecessary(amesys, flag);
   }

#ifdef AME_COSIM
   *oldflag = *newflag = sflag;
   discInput(&amesys->t_end_of_time_slice);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag,&sflag,&oflag,&panic,"_TunableParam",1);
#endif

#endif

   /* Call submodel calculation subroutine in the order
      that ensures the input requirements of each submodel
      are satisfied. */

#if (NB_REAL_TUNABLE_PARAM > 0) || (NB_INT_TUNABLE_PARAM > 0)
   if(*flag == 0)
   {
      int idxParam;
#if (NB_REAL_TUNABLE_PARAM > 0)
      for(idxParam = 0; idxParam < NB_REAL_TUNABLE_PARAM; idxParam++) {
         v[GRealTunableParam[idxParam][1]] = amesys->pModel->realParamArray[GRealTunableParam[idxParam][0]];
      }
#endif
#if(NB_INT_TUNABLE_PARAM > 0)
      for(idxParam = 0; idxParam < NB_INT_TUNABLE_PARAM; idxParam++) {
         v[GIntTunableParam[idxParam][1]] = amesys->pModel->integerParamArray[GIntTunableParam[idxParam][0]];
      }
#endif
   }
#endif


   n = 1;
   *oldflag = *newflag = sflag;
   tffd3_(&n, &v[0], &v[1], RP0, IP0, TP0, RS0, IS0);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFFD3",1);

   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_1;
      sitf.simulationInfo = &amesys->pModel->subArray[1];
      status |= ud00_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "UD00", 1);
   }
   v[22] = v[7] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[23] = v[8] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[24] = v[9] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[4] = v[27] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[5] = v[28] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_3;
      sitf.simulationInfo = &amesys->pModel->subArray[3];
      status |= cons00_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "CONS00", 1);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_5;
      sitf.simulationInfo = &amesys->pModel->subArray[5];
      status |= sigtrans0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGTRANS0", 4);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_6;
      sitf.simulationInfo = &amesys->pModel->subArray[6];
      status |= sigtrans0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGTRANS0", 5);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_8;
      sitf.simulationInfo = &amesys->pModel->subArray[8];
      status |= sigrecei0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGRECEI0", 1);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_9;
      sitf.simulationInfo = &amesys->pModel->subArray[9];
      status |= sigrecei0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGRECEI0", 2);
   }
   n = 1;
   *oldflag = *newflag = sflag;
   pmv00_(&n, &v[77], &v[74]);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"PMV00",1);

   n = 1;
   *oldflag = *newflag = sflag;
   {  /* DBK macro start. */
      int izero = 0;
      v[32] = tfpuc0_nodeid2_(&n, &v[30], &v[31], NULL, NULL, RP22
         , IP22, TP22, IS22, PS22, &izero);
   }  /* DBK macro end. */
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFPUC0",1);

   n = 1;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      tfvorf0_(&n, &v[21], &v[25], &v[26], &v[29], &v[22], &v[23]
      , &v[24], &v[37], &v[30], &v[31], &v[32], &v[38], &v[39]
      , &v[40], &v[41], &v[42], &v[43], &v[44], &v[45], &v[46]
      , &v[47], &v[48], &v[49], NULL, &v[50], NULL, NULL, NULL
      , NULL, RP4, IP4, TP4, RS4, IS4);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFVORF0",1);

   v[33] = -v[25] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[34] = -v[26] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   n = 1;
   *oldflag = *newflag = sflag;
   tfvf000_(&n, &v[25], &v[26], &v[27], &v[28], &v[29], &v[51]
      , &v[7], &v[8], &v[9], &v[52], &v[53], RP5, IP5, RS5);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFVF000",1);

   v[2] = v[25] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[3] = v[26] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   v[6] = v[29] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(amesys,flag);

   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "CTRPID0", 1);
   }
   n = 1;
   SetVisibility(amesys);
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      tfpuc0_(&n, &v[11], &v[12], &v[15], &v[16], &v[17], &v[18]
      , &v[30], &dot[0], &v[31], &dot[1], &v[32], &v[33], &v[34]
      , &v[35], &v[36], &v[37], &v[76], &v[77], &v[89], &v[90]
      , &v[91], &v[92], &v[93], NULL, NULL, NULL, NULL, NULL, NULL
      , NULL, NULL, &v[94], NULL, NULL, NULL, NULL, &v[95], NULL
      , NULL, NULL, NULL, RP22, IP22, TP22, IS22, PS22);
   }
   ClearVisibility(amesys);
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFPUC0",1);

   n = 1;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      intvarsensor0_(&n, &v[79], &v[89], IP23);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"INTVARSENSOR0",1);

   n = 2;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      intvarsensor0_(&n, &v[81], &v[91], IP24);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"INTVARSENSOR0",2);

   n = 3;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      intvarsensor0_(&n, &v[83], &v[92], IP25);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"INTVARSENSOR0",3);

   n = 1;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      tftk0_(&n, &v[2], &v[3], &v[4], &v[5], &v[6], &v[10], NULL
      , NULL, NULL, NULL, IP1, RS1, IS1);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFTK0",1);

   n = 2;
   *oldflag = *newflag = sflag;
   {  /* DBK specific start. */


      tftk0_(&n, &v[11], &v[12], &v[13], &v[14], &v[15], &v[19]
      , NULL, NULL, NULL, NULL, IP2, RS2, IS2);
   }
   AME_POST_SUBMODCALL_WITH_DISCON(amesys,flag,&sflag,&oflag,&panic,"TFTK0",2);

   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_19;
      sitf.simulationInfo = &amesys->pModel->subArray[19];
      status |= sigtrans0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGTRANS0", 1);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_20;
      sitf.simulationInfo = &amesys->pModel->subArray[20];
      status |= sigtrans0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGTRANS0", 2);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_21;
      sitf.simulationInfo = &amesys->pModel->subArray[21];
      status |= sigtrans0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGTRANS0", 3);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_22;
      sitf.simulationInfo = &amesys->pModel->subArray[22];
      status |= sigrecei0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGRECEI0", 3);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_23;
      sitf.simulationInfo = &amesys->pModel->subArray[23];
      status |= sigrecei0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGRECEI0", 4);
   }
   {
      *oldflag = *newflag = sflag;
      sitf.structuralInfo = &STLI_24;
      sitf.simulationInfo = &amesys->pModel->subArray[24];
      status |= sigrecei0_calc(&sitf);
      AME_POST_SUBMODCALL_WITH_DISCON(amesys, flag, &sflag, &oflag, &panic, "SIGRECEI0", 5);
   }


   /* Set interface outputs here. */
#if(AME_NBOF_OUTPUTS > 0)
   {
      int idxOutput;
      for(idxOutput = 0; idxOutput < AME_NBOF_OUTPUTS; idxOutput++) {
         output[idxOutput] = v[GOutputVarNum[idxOutput]];
      }
   }
#endif

#if(AME_MODEL_ISEXPLICIT == 1)
   applyStateStatus(dot,AME_NBOF_SOLVER_STATES);
#else
#if( AME_NBOF_EXPLICIT_STATE > 0)
   applyStateStatus(dot,AME_NBOF_EXPLICIT_STATE);

   for(i=0;i<AME_NBOF_EXPLICIT_STATE;i++)
   {
      delta[i] = dot[i] - yprime[i];
   }
#endif
   for(i=AME_NBOF_EXPLICIT_STATE;i<(AME_NBOF_EXPLICIT_STATE + AME_NBOF_IMPLICIT_STATE_DECLARED);i++)
   {
      delta[i] = dot[i];
   }
#endif

   if(*flag == 0)
   {
      /* It is an initialization call and the user
         is permitted to change the state variables
         and discrete variables. */
      updateStatesFromModel(amesys, y, AME_CONTINUOUS_STATE|AME_DISCRETE_STATE);
   }

#if( AME_NBOF_DISCRETE_STATE > 0)
   if(is_sample_time()) {
      /* Change discrete variables */
      updateStatesFromModel(amesys, y, AME_DISCRETE_STATE);
   }
#endif

   UpFECount(amesys);

   amesys->first_call = 0;
   
   if(status & AME_MODEL_FATAL) {
      AmeExit(1);
   }
}

static AMEModelStatus localJFuncEval_0(AMESIMSYSTEM * amesys, double t, double * y, double * dot, double * v, double * yprime, double * delta, int col)
{
   int sflag = 1;
   int n;
   double *dbk_wk = amesys->pModel->dbk_wk;
   S_AMESubIntf sitf = { amesys, NULL, NULL, t, &sflag, v, dot };
   AMEModelStatus status = AME_MODEL_OK;

   (void)n;

   v[30] = y[col];
   n = 1;
   {  /* DBK macro start. */
      int izero = 0;
      v[32] = tfpuc0_nodeid2_(&n, &v[30], &v[31], NULL, NULL, RP22
         , IP22, TP22, IS22, PS22, &izero);
   }  /* DBK macro end. */
   n = 1;
   {  /* DBK specific start. */


      tfvorf0_(&n, &v[21], &v[25], &v[26], &v[29], &v[22], &v[23]
      , &v[24], &v[37], &v[30], &v[31], &v[32], &v[38], &v[39]
      , &v[40], &v[41], &v[42], &v[43], &v[44], &v[45], &v[46]
      , &v[47], &v[48], &v[49], NULL, &v[50], NULL, NULL, NULL
      , NULL, RP4, IP4, TP4, RS4, IS4);
   }
   v[33] = -v[25] /* Duplicate variable. */;
   v[34] = -v[26] /* Duplicate variable. */;
   n = 1;
   tfvf000_(&n, &v[25], &v[26], &v[27], &v[28], &v[29], &v[51]
      , &v[7], &v[8], &v[9], &v[52], &v[53], RP5, IP5, RS5);
   v[2] = v[25] /* Duplicate variable. */;
   v[3] = v[26] /* Duplicate variable. */;
   v[6] = v[29] /* Duplicate variable. */;
   {
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      delta[2] = dot[2] - yprime[2];
      delta[3] = dot[3] - yprime[3];
   }
   n = 1;
   SetVisibility(amesys);
   {  /* DBK specific start. */


      tfpuc0_(&n, &v[11], &v[12], &v[15], &v[16], &v[17], &v[18]
      , &v[30], &dot[0], &v[31], &dot[1], &v[32], &v[33], &v[34]
      , &v[35], &v[36], &v[37], &v[76], &v[77], &v[89], &v[90]
      , &v[91], &v[92], &v[93], NULL, NULL, NULL, NULL, NULL, NULL
      , NULL, NULL, &v[94], NULL, NULL, NULL, NULL, &v[95], NULL
      , NULL, NULL, NULL, RP22, IP22, TP22, IS22, PS22);
   }
   ClearVisibility(amesys);
   delta[0] = dot[0] - yprime[0];
   delta[1] = dot[1] - yprime[1];
   return status;
}


static AMEModelStatus localJFuncEval_1(AMESIMSYSTEM * amesys, double t, double * y, double * dot, double * v, double * yprime, double * delta, int col)
{
   int sflag = 1;
   int n;
   double *dbk_wk = amesys->pModel->dbk_wk;
   S_AMESubIntf sitf = { amesys, NULL, NULL, t, &sflag, v, dot };
   AMEModelStatus status = AME_MODEL_OK;

   (void)n;

   v[31] = y[col];
   n = 1;
   {  /* DBK macro start. */
      int izero = 0;
      v[32] = tfpuc0_nodeid2_(&n, &v[30], &v[31], NULL, NULL, RP22
         , IP22, TP22, IS22, PS22, &izero);
   }  /* DBK macro end. */
   n = 1;
   {  /* DBK specific start. */


      tfvorf0_(&n, &v[21], &v[25], &v[26], &v[29], &v[22], &v[23]
      , &v[24], &v[37], &v[30], &v[31], &v[32], &v[38], &v[39]
      , &v[40], &v[41], &v[42], &v[43], &v[44], &v[45], &v[46]
      , &v[47], &v[48], &v[49], NULL, &v[50], NULL, NULL, NULL
      , NULL, RP4, IP4, TP4, RS4, IS4);
   }
   v[33] = -v[25] /* Duplicate variable. */;
   v[34] = -v[26] /* Duplicate variable. */;
   n = 1;
   tfvf000_(&n, &v[25], &v[26], &v[27], &v[28], &v[29], &v[51]
      , &v[7], &v[8], &v[9], &v[52], &v[53], RP5, IP5, RS5);
   v[2] = v[25] /* Duplicate variable. */;
   v[3] = v[26] /* Duplicate variable. */;
   v[6] = v[29] /* Duplicate variable. */;
   {
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      delta[2] = dot[2] - yprime[2];
      delta[3] = dot[3] - yprime[3];
   }
   n = 1;
   SetVisibility(amesys);
   {  /* DBK specific start. */


      tfpuc0_(&n, &v[11], &v[12], &v[15], &v[16], &v[17], &v[18]
      , &v[30], &dot[0], &v[31], &dot[1], &v[32], &v[33], &v[34]
      , &v[35], &v[36], &v[37], &v[76], &v[77], &v[89], &v[90]
      , &v[91], &v[92], &v[93], NULL, NULL, NULL, NULL, NULL, NULL
      , NULL, NULL, &v[94], NULL, NULL, NULL, NULL, &v[95], NULL
      , NULL, NULL, NULL, RP22, IP22, TP22, IS22, PS22);
   }
   ClearVisibility(amesys);
   delta[0] = dot[0] - yprime[0];
   delta[1] = dot[1] - yprime[1];
   return status;
}


static AMEModelStatus localJFuncEval_2(AMESIMSYSTEM * amesys, double t, double * y, double * dot, double * v, double * yprime, double * delta, int col)
{
   int sflag = 1;
   int n;
   double *dbk_wk = amesys->pModel->dbk_wk;
   S_AMESubIntf sitf = { amesys, NULL, NULL, t, &sflag, v, dot };
   AMEModelStatus status = AME_MODEL_OK;

   (void)n;

   v[58] = y[col];
   {
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      delta[2] = dot[2] - yprime[2];
      delta[3] = dot[3] - yprime[3];
   }
   return status;
}


static AMEModelStatus localJFuncEval_3(AMESIMSYSTEM * amesys, double t, double * y, double * dot, double * v, double * yprime, double * delta, int col)
{
   int sflag = 1;
   int n;
   double *dbk_wk = amesys->pModel->dbk_wk;
   S_AMESubIntf sitf = { amesys, NULL, NULL, t, &sflag, v, dot };
   AMEModelStatus status = AME_MODEL_OK;

   (void)n;

   v[60] = y[col];
   {
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      delta[2] = dot[2] - yprime[2];
      delta[3] = dot[3] - yprime[3];
   }
   return status;
}


static AMEModelStatus localJFuncEval_4(AMESIMSYSTEM * amesys, double t, double * y, double * dot, double * v, double * yprime, double * delta, int col)
{
   int sflag = 1;
   int n;
   double *dbk_wk = amesys->pModel->dbk_wk;
   S_AMESubIntf sitf = { amesys, NULL, NULL, t, &sflag, v, dot };
   AMEModelStatus status = AME_MODEL_OK;

   (void)n;

   v[21] = y[col];
   n = 1;
   {  /* DBK specific start. */


      tfvorf0_(&n, &v[21], &v[25], &v[26], &v[29], &v[22], &v[23]
      , &v[24], &v[37], &v[30], &v[31], &v[32], &v[38], &v[39]
      , &v[40], &v[41], &v[42], &v[43], &v[44], &v[45], &v[46]
      , &v[47], &v[48], &v[49], NULL, &v[50], NULL, NULL, NULL
      , NULL, RP4, IP4, TP4, RS4, IS4);
   }
   v[33] = -v[25] /* Duplicate variable. */;
   v[34] = -v[26] /* Duplicate variable. */;
   n = 1;
   tfvf000_(&n, &v[25], &v[26], &v[27], &v[28], &v[29], &v[51]
      , &v[7], &v[8], &v[9], &v[52], &v[53], RP5, IP5, RS5);
   v[2] = v[25] /* Duplicate variable. */;
   v[3] = v[26] /* Duplicate variable. */;
   v[6] = v[29] /* Duplicate variable. */;
   {
      sitf.structuralInfo = &STLI_13;
      sitf.simulationInfo = &amesys->pModel->subArray[13];
      status |= ctrpid0_calc(&sitf);
      delta[4] = v[21] - y[4]; v[21] = y[4];
      delta[2] = dot[2] - yprime[2];
      delta[3] = dot[3] - yprime[3];
   }
   n = 1;
   SetVisibility(amesys);
   {  /* DBK specific start. */


      tfpuc0_(&n, &v[11], &v[12], &v[15], &v[16], &v[17], &v[18]
      , &v[30], &dot[0], &v[31], &dot[1], &v[32], &v[33], &v[34]
      , &v[35], &v[36], &v[37], &v[76], &v[77], &v[89], &v[90]
      , &v[91], &v[92], &v[93], NULL, NULL, NULL, NULL, NULL, NULL
      , NULL, NULL, &v[94], NULL, NULL, NULL, NULL, &v[95], NULL
      , NULL, NULL, NULL, RP22, IP22, TP22, IS22, PS22);
   }
   ClearVisibility(amesys);
   delta[0] = dot[0] - yprime[0];
   delta[1] = dot[1] - yprime[1];
   return status;
}


static AMEModelStatus (*jfunc_table[])(AMESIMSYSTEM *, double, double *, double *, double *, double *, double *, int) = {
      localJFuncEval_0, localJFuncEval_1, localJFuncEval_2, localJFuncEval_3, localJFuncEval_4
};



static void localJFuncEval(AMESIMSYSTEM *amesys, double t, double *y, double *yprime, double *delta, int col)
{
   int sflag=1; /* Only one flag value is required. */
   int n=1, i=0;
   double *v = amesys->v;
   double *vcopy = amesys->vcopy;
   double *input = amesys->inputs;
   double *output = amesys->outputs;
   double *Z = amesys->discrete_states;
   double *dbk_wk = amesys->pModel->dbk_wk;
   AMEModelStatus status = AME_MODEL_OK;
   
#if(AME_MODEL_ISEXPLICIT == 1)
   double *dot = yprime;

#else
   double dot[AME_NBOF_SOLVER_STATES];

   /* Initialize the dot vector to the yprime vector. */
   memcpy((void *)dot, (void *)yprime, AME_NBOF_SOLVER_STATES*sizeof(double));
#endif

#if(AME_MODEL_ISEXPLICIT == 0)
   /* Initialize everything ready for potential calls to stepdn
      in submodels. */

#if(defined(AME_COSIM) && (AME_NBOF_INPUTS > 0))
   if(amesys->doInterpol) {
      if( getInputsCosimSlave(amesys->csSlave, t, input) != AME_NO_ERROR ) {
         AmeExit(1);
      }
   }
#endif

   if(isstabrun_())
   {
      t = amesys->simOptions->fixedTime;
   }
#endif

   /* Record current simulation time for ametim_(). */

   SetTimeAtThisStep(t);

   if (holdinputs_())
   {
      /* We reset artificially the time to the initial value
         to give the illusion of held inputs. */

      t = getstarttime_();
   }

   /* Assign the state variables y[] calculated by the integrator
      to the appropriate variables v[] and right calls necessary
      for that state in a case of a switch. */


   status |= (*jfunc_table[col])(amesys, t, y, dot, v, yprime, delta, col);


   UpFECount(amesys);
   
   if(status & AME_MODEL_FATAL) {
      AmeExit(1);
   }
}

static void EndOfSimulation(AMESIMSYSTEM *amesys)
{
   int n=1;
   double *y = amesys->states;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
   double *dbk_wk = amesys->pModel->dbk_wk;
   const double t = getfinaltime_();
   S_AMESubIntf sitf = {amesys, NULL, NULL, t, NULL, v, NULL};
   AMEModelStatus status = AME_MODEL_OK;
   
   double dummy[4];

   (void)n;
   (void)status;

   sitf.structuralInfo = &STLI_24;
   sitf.simulationInfo = &amesys->pModel->subArray[24];
   status |= sigrecei0_end(&sitf);
   sitf.structuralInfo = &STLI_23;
   sitf.simulationInfo = &amesys->pModel->subArray[23];
   status |= sigrecei0_end(&sitf);
   sitf.structuralInfo = &STLI_22;
   sitf.simulationInfo = &amesys->pModel->subArray[22];
   status |= sigrecei0_end(&sitf);
   sitf.structuralInfo = &STLI_21;
   sitf.simulationInfo = &amesys->pModel->subArray[21];
   status |= sigtrans0_end(&sitf);
   sitf.structuralInfo = &STLI_20;
   sitf.simulationInfo = &amesys->pModel->subArray[20];
   status |= sigtrans0_end(&sitf);
   sitf.structuralInfo = &STLI_19;
   sitf.simulationInfo = &amesys->pModel->subArray[19];
   status |= sigtrans0_end(&sitf);

   n = 1;
   {


      tfpuc0end_(&n, RP22, IP22, TP22, IS22, PS22, &v[13], &v[14]
         , &dummy[0], &v[30], &v[31], &dummy[1], &dummy[2], &dummy[3]
         , NULL, NULL, NULL, NULL, NULL, NULL);
   }
   sitf.structuralInfo = &STLI_13;
   sitf.simulationInfo = &amesys->pModel->subArray[13];
   status |= ctrpid0_end(&sitf);
   sitf.structuralInfo = &STLI_9;
   sitf.simulationInfo = &amesys->pModel->subArray[9];
   status |= sigrecei0_end(&sitf);
   sitf.structuralInfo = &STLI_8;
   sitf.simulationInfo = &amesys->pModel->subArray[8];
   status |= sigrecei0_end(&sitf);
   sitf.structuralInfo = &STLI_7;
   sitf.simulationInfo = &amesys->pModel->subArray[7];
   status |= ssink_end(&sitf);
   sitf.structuralInfo = &STLI_6;
   sitf.simulationInfo = &amesys->pModel->subArray[6];
   status |= sigtrans0_end(&sitf);
   sitf.structuralInfo = &STLI_5;
   sitf.simulationInfo = &amesys->pModel->subArray[5];
   status |= sigtrans0_end(&sitf);
   sitf.structuralInfo = &STLI_4;
   sitf.simulationInfo = &amesys->pModel->subArray[4];
   status |= ssink_end(&sitf);
   sitf.structuralInfo = &STLI_3;
   sitf.simulationInfo = &amesys->pModel->subArray[3];
   status |= cons00_end(&sitf);
   sitf.structuralInfo = &STLI_1;
   sitf.simulationInfo = &amesys->pModel->subArray[1];
   status |= ud00_end(&sitf);
}

static void ameTerminate(AMESIMSYSTEM *amesys)
{
#ifdef AME_WRITE_RUNSTAT
   if(amesys->simOptions ) {
      if(amesys->simOptions->statistics)
      {
         WriteRunStats(amesys);
      }
   }
#endif
#ifdef AME_PROCESS_TIME
   ProcessTime(2);
#endif

   /* Save state count, discontinuities and finalize the Performance Analyzer */
#ifdef AME_PERFORMANCE_ANALYZER
   if(!isfixedstepsolver_()){
      PerfAnalyzer_SaveStateCount (amesys);
      PerfAnalyzer_SaveDiscontinuities(amesys);
      PerfAnalyzer_Close(amesys);
   }
#endif

#ifdef AME_RESULT_FILE
   amesys->CloseResultFile(amesys);
#endif

   EndOfSimulation(amesys);
   AmeCallAtEnd(amesys->ameExitStatus);
   modelCleanStore(amesys->pModel);
}

#ifdef AME_EXPOSE_JACOBIAN
static int getPartialDerivatives(double *A, double *B, double *C, double *D)
{
   int res;
   AMESIMSYSTEM  *amesys = GetGlobalSystem();

   SPARSE_MATRIX *Amat, *Bmat, *Cmat, *Dmat;

#if(AME_MODEL_ISEXPLICIT == 1)
   if (amesys->tlast > TLAST_MIN)
      res = LDoLinearAnalysisOnDemand(amesys, amesys->numstates,
                                       amesys->tlast, amesys->states,
                                       &Amat, &Bmat, &Cmat, &Dmat);
#else
   if (amesys->tlast > TLAST_MIN)
      res = DDoLinearAnalysisOnDemand(amesys, amesys->neq,
                                       amesys->tlast,
                                       amesys->states, amesys->dotstates,
                                       &Amat, &Bmat, &Cmat, &Dmat);
#endif
   else
      res = 0;  /* failed: system probably not initialized */

   if(res != 0) {
      if(Amat) {
         GetMatAsDense(Amat,A);
      }
      if(Bmat) {
         GetMatAsDense(Bmat,B);
      }
      if(Cmat) {
         GetMatAsDense(Cmat,C);
      }
      if(Dmat) {
         GetMatAsDense(Dmat,D);
      }
      FreeSparseMatrix(Amat);
      FreeSparseMatrix(Bmat);
      FreeSparseMatrix(Cmat);
      FreeSparseMatrix(Dmat);
   }

   return res;
} /* getPartialDerivatives */

static int setLASettingsFromIO(int num_input_index, int num_output_index,
                              int *input_index, int *output_index, int *nbState)
{
   *nbState = AME_NBOF_EXPLICIT_STATE + AME_NBOF_IMPLICIT_STATE;

   return SetLADetailsFromIO(AME_NBOF_EXPLICIT_STATE, AME_NBOF_IMPLICIT_STATE,
                              AME_NBOF_INPUTS, AME_NBOF_OUTPUTS,
                              num_input_index, num_output_index, input_index, output_index,
                              GInputVarNum, GOutputVarNum, 0.0, 1.0, 1.0, 0.1);
}
#endif

/* ============================================================== */

static void ModelAmeExit(AMESIMSYSTEM *amesys, int status)
{
   /* Will be catch by the state machine */
   amesys->ameExitStatus = status;
   longjmp(amesys->jump_env, status);
}

#ifdef AME_INPUT_IN_MEMORY
#include "Lemuel_pump_system_FMU_.ssf.h"
static const char **getssflist(int *num_items)
{
   *num_items = savestatusflags_length;
   return savestatusflags;
}
#endif

/**************************************************************
*                                                             *
* Function load_subdata_tables reads data for lookuptables    *
* mostly/only used for realtime                               *
*                                                             *
* 0106429                                                     *
* Move the include outside of function. The include now       *
* contains one function per table and a function that calls   *
* them all. This reduces the risk that a compiler crashes due *
* to a huge function.                                         *
**************************************************************/
#ifdef AME_TABLES_IN_MEMORY
#include "Lemuel_pump_system_FMU_.subdata.h"
#endif

static void load_subdata_tables(void)
{
#ifdef AME_TABLES_IN_MEMORY
   add_all_tables_to_memory();
#endif
}

/***********************************************************
   Purpose  :  Return Simcenter Amesim version used to generate the model C code
               It allows the client to update interface management for
               backward compatibility.
   Author	:  J.Andre
   Creation :  2016 - 09 - 05
   Inputs   :  None
   Outputs  :  Simcenter Amesim version
   Revision :
************************************************************/
static unsigned int AmesysGetVersion()
{
	/* Returned number indicates 10000* main verion + 100* SL version + minor */
   /* Eg. Rev15     (15.0.0)  => 150000 */
   /* Eg. Rev15SL1  (15.1.0)  => 150100 */
   /* Eg. Rev15.0.1 (15.0.1)  => 150001 */

   return AMEVERSION;
}

/***********************************************************
   Purpose  :  Return SoldTo ID which Simcenter Amesim model has been generated.
   Author	:  J.Andre
   Creation :  2017 - 02 - 13
   Inputs   :  None
   Outputs  :  SoldToID
   Revision :
************************************************************/
static const char* getSoldToID()
{
   return "7376667";
}

#ifdef AME_INPUT_IN_MEMORY
#include "Lemuel_pump_system_FMU_.globalparams.h"
#endif

/***********************************************************
   Purpose  : Load and evaluate model parameters from files
   Author   : J.Andre
   Created  : 2016 - 09 - 08
   Inputs   :
      amesys  : system
      errmsg  : error message
   Outputs  : Error code
   Revision :
************************************************************/
static AMESystemError loadModelParamFromDisk(AMESIMSYSTEM *amesys, AMESystemError *gpError,  AMESystemError *lpError, char *errmsg)
{
   AMESystemError ret;
   *gpError = AME_NO_ERROR;
   *lpError = AME_NO_ERROR;

   ret = AmeReadGPFile(amesys);

   if(ret == AME_NO_ERROR) {
      *gpError = AmeEvalGlobalParamList(amesys, 1, errmsg);
      *lpError = loadParameterFromFile(amesys->pModel, GetDataFileName(), errmsg);
   }
   return ret;
}

#ifdef AME_INPUT_IN_MEMORY

/***********************************************************
   Purpose    : Load and evaluate model parameters from memory
   Author	  : J.Andre
   Created on : 2016 - 09 - 08
   Inputs	  :
      amesys  : system
      errmsg  : error message
   Outputs	  : Parameter error code
   Revision   :
************************************************************/
static AMESystemError loadModelParamFromMemory(AMESIMSYSTEM *amesys, AMESystemError *gpError,  AMESystemError *lpError, char * errmsg)
{
#include "Lemuel_pump_system_FMU_.data.h"

   ameAddGlobalParamsFromMemory(amesys,errmsg);

   if(amecheckversion(AMESIMRUNTIMEVERSION, __TIME__, __DATE__, NULL)) {
      amefprintf(stderr, "A problem with incompatible versions of the AMESim runtime library has been found.");
   }

   *gpError = AmeEvalGlobalParamList(amesys, 1, errmsg);
   *lpError = loadParameterFromDataTable(amesys->pModel, allparams, errmsg);

   return AME_NO_ERROR;
}

#endif

/***********************************************************
   Purpose    : Load and evaluate model parameters
   Author	  : J.Andre
   Created on : 2016 - 09 - 08
   Inputs	  :
      amesys  : system
      errmsg  : error message
   Outputs	  : Parameter error code
   Revision   :
************************************************************/
static AMESystemError loadModelParameters(AMESIMSYSTEM *amesys)
{
   AMESystemError res = AME_PARAMETER_ERROR;
   char errmsg[PATH_MAX+128];
   AMESystemError ret_gp = AME_NO_ERROR;
   AMESystemError ret_lp = AME_NO_ERROR;

   errmsg[0] = '\0';

#ifdef AME_INPUT_IN_MEMORY
#ifdef AME_RT_CAN_READ_FILE
   res = loadModelParamFromDisk(amesys, &ret_gp, &ret_lp, errmsg);

   if (res != AME_NO_ERROR) {
      /* If the file is not there - we say nothing */
      if (res != AME_PARAM_FILE_OPEN_ERROR) {
         amefprintf(stderr,"%s",errmsg);
         amefprintf(stderr,"loadParameterFromFile> %s\n",errmsg);
         ClearGPs();
      }
   }
   else {
      amefprintf(stderr,"Using data from disk (%s)\n",GetDataFileName());
   }
#endif
   if(res != AME_NO_ERROR) {
      /* Read all from memory */
      res = loadModelParamFromMemory(amesys, &ret_gp, &ret_lp, errmsg);
      SetGlobalParamReadFile(0);
   }
#else
   /* Read all from disk */
   res = loadModelParamFromDisk(amesys, &ret_gp, &ret_lp, errmsg);
#endif

   if( (res != AME_NO_ERROR) || (ret_gp != AME_NO_ERROR) || (ret_lp != AME_NO_ERROR) ) {
      amefprintf(stderr,"%s",errmsg);

      if( (res != AME_NO_ERROR) || (ret_lp != AME_NO_ERROR) ) {
         res = AME_GLOBAL_PARAMETER_ERROR;
      }
   }
   return res;
}

static AMESystemError Input(AMESIMSYSTEM *amesys)
{
   int retval;

   /* Load data files for submodels */
   load_subdata_tables();

   /* Load parameters for submodels */
   retval = loadModelParameters(amesys);

   /* Ensure parameters seen by RT is initialized correctly */
#ifdef AME_MEMORY_ACCESS_RT_EXPORT
   RT_Get_Watch_Param(amesys);
#endif

   return retval;
}

static int ameSetOptions(AMESIMSYSTEM *amesys,
                         double tsaveinc,
                         double maxstepsize,
                         double tolerance,
                         int errorcontrol,
                         int writelevel,
                         int extradisconprint,
                         int runstats,
                         int theruntype,
                         int thesolvertype)
{
   amesys->simOptions->errorType = errorcontrol;
   amesys->simOptions->tol = tolerance;
   amesys->simOptions->rStrtp = extradisconprint;
   amesys->simOptions->statistics = runstats;
   amesys->simOptions->solverType = thesolvertype;
   amesys->simOptions->runType = theruntype;
   amesys->simOptions->iWrite = writelevel;
   amesys->simOptions->tInc = tsaveinc;
   amesys->simOptions->hMax = maxstepsize;

   /* Copy sim option to share with libraries before to modify it */
   memcpy(amesys->sharedSimOptions, amesys->simOptions, sizeof(SIMOptions));

   if(amesys->simOptions->solverType) {
      /* It is the cautious option. The maximum time step
         should not exceed the print interval. */
      setmaxstep_(&amesys->simOptions->tInc);
   }

#if( AME_MODEL_ISEXPLICIT == 1) && (AME_NBOF_EXPLICIT_STATE == 0 )
   if(maxstepsize > tsaveinc) {
      amefprintf(stderr, "Since the model has no state variable,\n");
      amefprintf(stderr, "the maximum time step has been reduced to %gs.\n", tsaveinc);
      setmaxstep_(&amesys->simOptions->tInc);
   }
#endif

   SetInitPrintIntervalFromSimOpts(amesys);

   ValidateRuntype(theruntype);

   ameSetupTolerance(amesys->simOptions);

   return 1;
}

static int ameSetOptionsFixed(AMESIMSYSTEM *amesys,
                              double printinterval,
                              int fixed_type,
                              int fixedOrder,
                              double fixed_h,
                              int run_type)
{
   amesys->simOptions->iWrite = 2;

   ValidateRuntype(run_type);

	/* Ensure that runflag StabilizingRun=0. It might have been set to true */
	/* due to a previous selection for the variable step integrator. */
   ClearStabilizingRun();

   amesys->simOptions->fixedOrder = fixedOrder;
   amesys->simOptions->fixedStep  = 1; /* Yes - fixed step */
   amesys->simOptions->fixedH     = fixed_h;

   amesys->simOptions->tInc =  printinterval;
   amesys->simOptions->rStrtp = 0;  /* No discontinuity printout using fixed step solver */

   amesys->simOptions->fixedType  = fixed_type;
   SetIsUsingFixedSolver(( fixed_type == 1)*100 +  (fixed_type != 1)*200 + fixedOrder);

   SetFixedTimeStep(fixed_h);

   return 1;
}

static int ameInputs(AMESIMSYSTEM *amesys, int numInputs, const double *inputARG)
{
   if(numInputs != amesys->numinputs)
   {
      char error_message[256];
      sprintf(error_message, "AMEInputs> Expected %d inputs but got %d\n", amesys->numinputs, numInputs);
      DisplayMessage(error_message);
      return 0;
   }
#if (AME_NBOF_INPUTS > 0)
   memcpy(amesys->inputs, inputARG, amesys->numinputs*sizeof(double) );
#endif
   return 1;
}

static void doAssembly(AMESIMSYSTEM *amesys)
{
   if (IsAssemblyNecessary_())
   {
      double time = getstarttime_();
      double tmp[AME_NBOF_SOLVER_STATES];
      int local_flag;

      /* Perform the assembly. */
      amesys->consflag = 1;
      local_flag = 0;
#if(AME_MODEL_ISEXPLICIT == 1)
      amesys->FuncEval(amesys, time, amesys->states, tmp, NULL, &local_flag);
#else
      amesys->res(amesys, time, amesys->states, amesys->dotstates, tmp, &local_flag);
#endif
      amesys->first_call = 1;

      amesys->consflag = 2;
      local_flag = 0;
#if(AME_MODEL_ISEXPLICIT == 1)
      amesys->FuncEval(amesys, time, amesys->states, tmp, NULL, &local_flag);
#else
      amesys->res(amesys, time, amesys->states, amesys->dotstates, tmp, &local_flag);
#endif
      amesys->first_call = 1;
      amesys->consflag = 0;
   }
}

static int ameEvalTstart(AMESIMSYSTEM *amesys, const double *modelInputs, double *modelOutputs)
{
#ifndef AMERT
   double time = getstarttime_();

   ameInputs(amesys, AME_NBOF_INPUTS, modelInputs);

   doAssembly(amesys);

   /* Initialize, maybe perform an initialising run */

   if(isstabrun_())
   {
      amesys->simOptions->fixedTime = time;
   }
   amesys->simOptions->stabilOption += 4*amesys->simOptions->solverType;

#if(AME_MODEL_ISEXPLICIT == 1)

   if(!IntegrateInit(amesys, time, time))
   {
      return 0;
   }
#else
   amesys->internal_discont = 1;

   DIntegrateInit(amesys, AME_NBOF_EXPLICIT_STATE, time, getfinaltime_(), amesys->simOptions->tInc, amesys->states,
                  amesys->dotstates, amesys->simOptions->hMax, AME_NBOF_SOLVER_STATES, amesys->iwork, amesys->simOptions->reltol, amesys->simOptions->abstol, amesys->simOptions->rStrtp, LIW, LRW, amesys->simOptions->statistics,amesys->simOptions->stabilOption, amesys->simOptions->iWrite, amesys->simOptions->minimalDiscont, Gis_constraint);

   if(isstabrun_())
   {
      /* HELP !!!! */
   }
#endif

#if (AME_NBOF_OUTPUTS > 0)
   memcpy(modelOutputs, amesys->outputs, amesys->numoutputs*sizeof(double) );
#endif
   amesys->tlast = time;

   return 1;
#else
   char error_message[256];
   sprintf(error_message, "ameEvalTstart> Should never be called for real-time simulation\n");
   DisplayMessage(error_message);
   return 0;
#endif
}

static int ameSetUpLockedStatus(AMESIMSYSTEM *amesys)
{
#ifdef AME_INPUT_IN_MEMORY
#include "Lemuel_pump_system_FMU_.lock.h"
   if(0 != SetUpLockedStatesFromMemory(amesys, lockedstates_length, lockedstates_array))
   {
      amefprintf(stderr,"Failed to set the locked states status.\n");
   }
#else
   SetUpLockedStates(GetCircuitName());
#endif
   return 0;
}

static int ameInitializeConditions(AMESIMSYSTEM *amesys)
{
#ifdef AME_VSOLVER_ACCEPTED
   double time = amesys->simOptions->tStart;

   /* Initialise some static variables */
   setfinaltime_(amesys->simOptions->tFinal);

   amesys->first_call=1;  /* should this be done or not ?*/
   amesys->internal_discont = 1;

   amesys->tlast = TLAST_MIN;

   memset(amesys->ecount,0,amesys->numstates*sizeof(int));
   memset(amesys->dotstates,0,amesys->numstates*sizeof(double));

#if(AME_MODEL_ISEXPLICIT == 0)
   memset(amesys->iwork,0,LIW*sizeof(int));
#endif
   /*  The following statement covers the case where there are
       no state variables y. */
#if( AME_NBOF_EXPLICIT_STATE == 0 )
   amesys->states[0] = 0.0;
#endif

   /* Call Input to read submodel and simulation parameters. */
   if(Input(amesys) != AME_NO_ERROR) return 0;
   modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);

   /* Register print interval that maybe used bys some submodels */
   SetInitPrintIntervalFromSimOpts(amesys);

   setstarttime_(time);

   /* Call pre-initialize function */

   PreInitialize(amesys,amesys->states);

#ifndef AME_INPUT_IN_MEMORY
   if( NeedReloadInputFiles() != 0 )
   {
      ClearGPs();
      if(Input(amesys) != AME_NO_ERROR) return 0;
      modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);
      ClearReloadedFiles();
   }
#endif

   /* Call initialize subroutine to set con and icon array members */

   Initialize(amesys,amesys->states);

   /* Overwriting initial state values with requests emitted by */
   /* submodels that have a more global view (cf. register.c mechanism) */
   /* Can also fire some callbacks to 'fix' float and integer store */
   OverloadStatesWithRegister(amesys, amesys->states, SVREGISTER_DEFAULT);

   CheckSimParams(amesys, &amesys->simOptions->abstol,
                  &amesys->simOptions->reltol,
                  &amesys->simOptions->hMax);

#ifdef AME_RESULT_FILE
   /*  Open file for results. */
   amesys->AmeReadFile(amesys, &time, amesys->states);
#endif

   if(isconrun_() || isusefinval_())
   {
      updateStatesFromModel(amesys, amesys->states, AME_CONTINUOUS_STATE|AME_DISCRETE_STATE);
   }

   /* Read linear analysis specification. */
#ifndef FMI
   SetLADetails(GetLAFileName(), amesys->numstates, amesys->numvars, time,  amesys->simOptions->reltol, amesys->simOptions->abstol, getfinaltime_()-time);

   /* Remove old err file */

   remove(GetErrorFileName());

   /* Initialize the Performance analyzer */
   if(!isfixedstepsolver_()) {
      PerfAnalyzer_Init(amesys, time, getfinaltime_() );
   }
#endif
   if(isconrun_())
      setstarttime_(time);

   /* Set the locked states info */
   ameSetUpLockedStatus(amesys);

   return 1;
#else
   char error_message[256];
   sprintf(error_message, "AMEInitializeConditions> Should never be called for real-time simulation\n");
   DisplayMessage(error_message);
   return 0;
#endif
}

#if(AME_MODEL_ISEXPLICIT == 1)
static int ameEvalTstartFixed(AMESIMSYSTEM *amesys, const double *modelInputs, double *modelOutputs)
{
   double time = getstarttime_();

   ameInputs(amesys, AME_NBOF_INPUTS, modelInputs);

   doAssembly(amesys);

   amesys->tlast = time;

   /* Evaluation of the model at tStart */
   {
      int local_flag = 0;
      SetIsPrinting(amesys);
      amesys->FuncEval(amesys, time, amesys->states, amesys->yh, NULL, &local_flag);
      ClearIsPrinting(amesys);

#if defined(AME_RESULT_FILE) && !defined(STANDALONESIMULATOR)
      amesys->OutputResults(amesys, amesys->tlast);
      amesys->resultFileStructPtr->lastprinttime = amesys->tlast;
#endif

      /* Now, amesys->first_call == 0 (set at the end of FunctionEval) */
   }

#if (AME_NBOF_OUTPUTS > 0)
   memcpy(modelOutputs, amesys->outputs,amesys->numoutputs*sizeof(double) );
#endif
   amesys->tlast = time;

#ifdef AME_PROCESS_TIME
   /* Initialization of time timers */
   ProcessTime(0);
#endif

   return 1;
}

static int ameInitializeConditionsFixed(AMESIMSYSTEM *amesys)
{
   double start_time = amesys->simOptions->tStart;

   assert(amesys);

   amesys->first_call=1;  /* should this be done or not ?*/

   amesys->tlast=TLAST_MIN;

   /* Init solver */
   memset(amesys->yh,0,(AME_NBOF_SOLVER_STATES*13)*sizeof(double));

   InitFixedStepIntegrate(amesys);

   setfinaltime_(amesys->simOptions->tFinal);

   /* Call Input to read submodel and simulation parameters. */
   if(Input(amesys) != AME_NO_ERROR) return 0;
   modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);

   SetInitPrintIntervalFromSimOpts(amesys);

   setstarttime_(start_time);

   /* Call pre-initialize function */

   PreInitialize(amesys,amesys->states);

#ifndef AME_INPUT_IN_MEMORY
   if( NeedReloadInputFiles() != 0 )
   {
      ClearGPs();
      if(Input(amesys) != AME_NO_ERROR) return 0;
      modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);
      ClearReloadedFiles();
   }
#endif

   /* Call initialize subroutine to set con and icon array members */

   Initialize(amesys,amesys->states);

   /* Overwriting initial state values with requests emitted by */
   /* submodels that have a more global view (cf. register.c mechanism) */
   /* Can also fire some callbacks to 'fix' float and integer store */
   OverloadStatesWithRegister(amesys, amesys->states, SVREGISTER_DEFAULT);

#ifdef AME_RESULT_FILE
   /*  Open file for results. */
   amesys->AmeReadFile(amesys, &start_time, amesys->states);
#endif

   if(isconrun_() || isusefinval_())
   {
      updateStatesFromModel(amesys, amesys->states, AME_CONTINUOUS_STATE|AME_DISCRETE_STATE);
   }

   /* Set the locked states info */
   ameSetUpLockedStatus(amesys);

   if(isconrun_()) {
      setstarttime_(start_time);
   }

   return 1;
}
#endif

/*=============================================================================*/

/*=============================================================================*/

/***********************************************************
   Purpose    : Test request acceptance
   Author	  : J.Andre
   Created on : 2016 - 09 - 05
   Inputs	  :
      event   : entry event
   Outputs	  :
      AME_NO_ERROR : event accepted
      AME_SEQUENCE_ERROR : event refused
   Revision   :
************************************************************/
static AMESystemError AmesysControlRequest(AMESIMSYSTEM *amesys, AMESystemCmd event)
{
   AMESystemError res = AME_NO_ERROR;

   if(!amesys) {
      if(event == AME_CMD_GET_MODEL_INFO) {
         return AME_NO_ERROR;
      }
      return AME_SEQUENCE_ERROR;
   }

   switch(event) {
      case AME_CMD_RELEASE: {
         if( !(amesys->systemState & (AMESTATE_TERMINATED | AMESTATE_FATAL | AMESTATE_INSTANTIATED)) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_SETUP: {
         if( !(amesys->systemState & AMESTATE_INSTANTIATED) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_INITIALIZE: {
         if(amesys->systemState != AMESTATE_READY) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_TERMINATE: {
         if( !(amesys->systemState & (AMESTATE_RUN | AMESTATE_READY | AMESTATE_ERROR)) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_TSTART: {
         if( !(amesys->systemState & AMESTATE_INITIALIZED) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_STEP: {
         if( !(amesys->systemState & AMESTATE_RUNNING) ) {
            res = AME_SEQUENCE_ERROR;
         }
         else {
            amesys->systemState |= AMESTATE_STEP_IN_PROGRESS;
         }
      }
      break;
      case AME_CMD_RESTART: {
         /* At this time, it is not still implemented */
         /* Depends of clean-up in terminated state and static variables */
         res = AME_SEQUENCE_ERROR;
      }
      break;
      case AME_CMD_GET_MODEL_INFO: {
         res = AME_NO_ERROR;
      }
      break;
      case AME_CMD_SET_MODEL_PARAM_TUNABLE: {
         if( !(amesys->systemState & (AMESTATE_READY | AMESTATE_RUN)) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_SET_MODEL_PARAM:
      case AME_CMD_SET_RUN_PARAM:
      case AME_CMD_SET_SOLVER_PARAM: {
         if(amesys->systemState != AMESTATE_READY) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_GET_MODEL_PARAM:
      case AME_CMD_GET_SOLVER_PARAM:
      case AME_CMD_GET_RUN_PARAM: {
         if( !(amesys->systemState & (AMESTATE_READY | AMESTATE_RUN | AMESTATE_ERROR)) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_REQ_RUN_INTERRUPT: {
         if( !(amesys->systemState & (AMESTATE_INSTANTIATED | AMESTATE_READY | AMESTATE_INITIALIZED | AMESTATE_RUNNING)) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_SET_ENV: {
         if( !(amesys->systemState & AMESTATE_INSTANTIATED) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      case AME_CMD_FUNCEVAL: {
         if( !(amesys->systemState & AMESTATE_RUNNING) ) {
            res = AME_SEQUENCE_ERROR;
         }
      }
      break;
      default:
         res = AME_SEQUENCE_ERROR;
      break;
   }

   return res;
}

/***********************************************************
   Purpose    : Update state and manage result of request
   Author	  : J.Andre
   Created on : 2016 - 09 - 05
   Inputs	  :
      event   : entry event
      reqResult : result of request achievement
   Outputs	  :
      AME_NO_ERROR : event accepted
      AME_SEQUENCE_ERROR : event refused
   Revision   :
************************************************************/
static AMESystemError AmesysUpdateState(AMESIMSYSTEM *amesys, AMESystemCmd event, AMESystemError reqResult)
{
   unsigned int newState;
   AMESystemError res;

   if(!amesys) {
      return AME_SEQUENCE_ERROR;
   }

   newState = amesys->systemState;   /* Default: no change in state */
   res = reqResult;                  /* Default: no change in result */

   switch(event) {
      case AME_CMD_INSTANTIATE: {
         if(reqResult != AME_NO_ERROR) {
            newState = AMESTATE_FATAL;
         }
         else {
            newState = AMESTATE_INSTANTIATED;
         }
      }
      break;
      case AME_CMD_SETUP: {
         if(reqResult != AME_NO_ERROR) {
            newState = AMESTATE_FATAL;
         }
         else {
            newState = AMESTATE_READY;
         }
      }
      break;
      case AME_CMD_INITIALIZE: {
         if(reqResult != AME_NO_ERROR) {
            newState = AMESTATE_ERROR;
         }
         else {
            newState = AMESTATE_INITIALIZED;
         }
      }
      break;
      case AME_CMD_TERMINATE: {
         if(reqResult != AME_NO_ERROR) {
            amesys->systemState = AMESTATE_TERMINATED; /* Avoid automatic call to ameterminate */
            newState = AMESTATE_FATAL;
         }
         else {
            newState = AMESTATE_TERMINATED;
         }
      }
      break;
      case AME_CMD_TSTART: {
         if(reqResult != AME_NO_ERROR) {
            newState = AMESTATE_ERROR;
         }
         else {
            newState = AMESTATE_RUNNING;
         }
      }
      break;
      case AME_CMD_STEP: {
         if(reqResult != AME_NO_ERROR) {
            /* test ameExitStatus */
            if(reqResult == AME_EXIT_ERROR) {
               if(amesys->ameExitStatus == 0) {
                  /* Simulation stopped early but normally */
                  amesys->requestinterrupt = 0;
                  newState = AMESTATE_STOPPED_EARLY;
                  res = AME_NO_ERROR;
               }
               else
               {
                  newState = AMESTATE_ERROR;
               }
            }
            else {
               newState = AMESTATE_ERROR;
            }
         }
         else if(amesys->requestinterrupt) {
            amesys->requestinterrupt = 0;
            newState = AMESTATE_STOPPED_END;
         }
         newState &= ~AMESTATE_STEP_IN_PROGRESS;
      }
      break;
      case AME_CMD_REQ_RUN_INTERRUPT:
      break;
      case AME_CMD_FUNCEVAL: {
         if(reqResult != AME_NO_ERROR) {
            /* test ameExitStatus */
            if(reqResult == AME_EXIT_ERROR) {
               if(amesys->ameExitStatus == 0) {
                  /* Simulation stopped early but normally */
                  newState = AMESTATE_STOPPED_EARLY;
                  res = AME_NO_ERROR;
               }
               else
               {
                  newState = AMESTATE_ERROR;
               }
            }
            else {
               newState = AMESTATE_ERROR;
            }
         }
      }
      break;
      default:
      /* No state change, no exception to catch, just pass result */
      break;
   }

   if(amesys->systemState != newState) {
      if(newState == AMESTATE_FATAL) {
         if(amesys->systemState & (AMESTATE_RUN | AMESTATE_READY | AMESTATE_ERROR)) {
            /* Terminating the simulation */
            ameTerminate(amesys);
         }
      }

#if !defined(STANDALONESIMULATOR)
      if(newState & (AMESTATE_FATAL | AMESTATE_ERROR)) {
         if(amesys->systemState & AMESTATE_RUN) {
            amefprintf(stderr, "Simcenter Amesim model: simulation failed.\n");
         }
         else if(amesys->systemState & AMESTATE_IDLE) {
            amefprintf(stderr, "Simcenter Amesim model: instantiation failed.\n");
         }
         else if(amesys->systemState & AMESTATE_READY) {
            amefprintf(stderr, "Simcenter Amesim model: initialization failed.\n");
         }
      }
      else if(newState & AMESTATE_INITIALIZED) {
         amefprintf(stdout, "Simcenter Amesim model: initialization done.\n");
      }
      else if(newState & AMESTATE_TERMINATED) {
         amefprintf(stdout, "Simcenter Amesim model: simulation terminated.\n");
      }
      else if((newState & AMESTATE_RUNNING) && (amesys->systemState & AMESTATE_INITIALIZED)) {
         amefprintf(stdout, "Simcenter Amesim model: simulation started.\n");
      }
#endif

      /* Update state */
      amesys->systemState = newState;
   }

   if(amesys->systemState == AMESTATE_FATAL) {
      /* Greedy error */
      res = AME_FATAL_ERROR;
   }

   return res;
}

/***********************************************************
   Purpose    : Instantiate the system
   Author	  : J.Andre
   Created on : 2016 - 09 - 08
   Inputs	  : None
   Outputs	  : Error code
   Revision   :
************************************************************/
static AMESystemError AmesysInstantiate(AMESIMSYSTEM **amesysPtr)
{
   int jump_ret;
   AMESIMSYSTEM *amesys;
   AMESystemError result = AME_FATAL_ERROR;

   S_AME_Model *pModel;

   if(*amesysPtr != NULL) {
      return AME_SEQUENCE_ERROR;
   }

   result = createModel(&pModel, &GmodelDef, GParamInfo, GVarInfo, GsubmodelNameArray,
                        GcontStateVarNum, GdiscStateVarNum, GFixedVarNum);

   if(result == AME_NO_ERROR) {
#if(AME_MODEL_ISEXPLICIT == 1)
      amefprintf(stdout, "Instantiating a system with %d unknowns.\n", AME_NBOF_EXPLICIT_STATE);
#else
      amefprintf(stdout, "Instantiating a system with %d unknowns.\n", AME_NBOF_SOLVER_STATES);
#endif
      if (strcmp(soldToId,"not available") != 0)
         amefprintf(stdout, "Simcenter Amesim version: %s (%s).\n", "2404", soldToId);
      else
         amefprintf(stdout, "Simcenter Amesim version: %s.\n", "2404");
      result = createAMESystem(&amesys, pModel, AME_NBOF_SOLVER_STATES, AMEVERSION);
   }
#if (AME_NB_SUBMODEL_REF>0)
   if(result == AME_NO_ERROR) {
      int idx_sub;
      for(idx_sub = 0; idx_sub < AME_NB_SUBMODEL_REF; idx_sub++) {
         S_AMESubSimuInfo info;
         info.rp = SUBSIMU[idx_sub][0][0] > 0 ? &pModel->realParamArray[SUBSIMU[idx_sub][0][1]] : NULL;
         info.ip = SUBSIMU[idx_sub][1][0] > 0 ? &pModel->integerParamArray[SUBSIMU[idx_sub][1][1]] : NULL;
         info.tp = (const char **)(SUBSIMU[idx_sub][2][0] > 0 ? &pModel->textParamArray[SUBSIMU[idx_sub][2][1]] : NULL);
         info.c = SUBSIMU[idx_sub][3][0] > 0 ? &pModel->realStoreArray[SUBSIMU[idx_sub][3][1]] : NULL;
         info.ic = SUBSIMU[idx_sub][4][0] > 0 ? &pModel->intStoreArray[SUBSIMU[idx_sub][4][1]] : NULL;
         info.ps = SUBSIMU[idx_sub][5][0] > 0 ? &pModel->pointerStoreArray[SUBSIMU[idx_sub][5][1]] : NULL;
         info.na = pModel->ptrNetWorkArray;
         result |= setSubSimuParam(pModel, idx_sub, &info);
      }
   }
#endif

   if(result == AME_NO_ERROR) {
      if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* ~try */
         SetGlobalSystem(amesys);

#ifdef AME_INPUT_IN_MEMORY
         amesys->getssflist = getssflist;
#endif
         amesys->consflag = 0;

#if(AME_MODEL_ISEXPLICIT == 1)
         amesys->FuncEval = localFuncEval;
         amesys->JFuncEval = localJFuncEval;
#else
         amesys->res = localFuncEval;
         amesys->Jres = localJFuncEval;
#endif
         amesys->AmeExit = ModelAmeExit;

         amesys->ameExitStatus = 0;

         /* Set input directory to current directory */
         AmeSetInputDir(amesys,NULL);

         /* Set output directory to current directory */
         AmeSetOutputDir(amesys,NULL);

         /* Set base name of input files, no extension */
         AmeSetModelBaseName(amesys,"Lemuel_pump_system_FMU_", NULL);

         result = AME_NO_ERROR;
      }
      else { /* Catch AmeExit */
         result = AME_EXIT_ERROR;
      }
   }

   if(result == AME_NO_ERROR) {
      /* Update state and result */
      result =  AmesysUpdateState(amesys, AME_CMD_INSTANTIATE, result);
   }

   if(result != AME_NO_ERROR) {
      deleteAMESystem(&amesys);
      SetGlobalSystem(NULL);
   }
   else {
      *amesysPtr = amesys;
   }

   return result;
}

static AMESystemError AmesysTerminate(AMESIMSYSTEM *amesys)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_TERMINATE);
   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if(res == AME_NO_ERROR) { /* Request accepted */
      if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* ~try */
         ameTerminate(amesys);
      }
      else { /* Catch AmeExit */
         res = AME_EXIT_ERROR;
      }
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_TERMINATE, res);

   return res;
}

static AMESystemError AmesysRelease(AMESIMSYSTEM **amesysPtr)
{
   AMESystemError res = AME_NO_ERROR;
   if(*amesysPtr) {
      SetGlobalSystem(*amesysPtr);
      res = AmesysControlRequest(*amesysPtr, AME_CMD_RELEASE);

      if(res == AME_NO_ERROR) { /* Request accepted */
         AmeSignalModelUnload();
         res = deleteAMESystem(amesysPtr);
         SetGlobalSystem(NULL);
      }
   }
   return res;
}

/***********************************************************
   Purpose    : Go in Ready state to be able receive external
                initialization of model and simulation
   Author	  : J.Andre
   Created on : 2016 - 09 - 08
   Inputs	  : loadParam: if true, model parameters are read
   Outputs	  : Error code
   Revision   :
************************************************************/
static AMESystemError AmesysSetUp(AMESIMSYSTEM *amesys, const int loadParam)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SETUP);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* ~try */
      amesys->ameExitStatus = 0;

      /* Construct file names */
      AmeConstructFileName(amesys);

      /* Read sim file */
      {
         SIMOptions opts;

#ifndef AME_INPUT_IN_MEMORY
         char errMsg[PATH_MAX+128];
         int result = readsimfile(&opts, GetSimFileName(), errMsg);
#else
         int result = readsimfromchararrays(&opts, simparams, simparams_length);
#endif

         if(result != 0) {

            /* Initialyze Amesystem SIMOptions */
            memcpy(amesys->simOptions,&opts,sizeof(SIMOptions));

#ifdef AME_RESULT_FILE
            amesys->simOptions->outoff = 0;
#else
            amesys->simOptions->outoff = 1;
#endif

#ifdef AME_PERFORMANCE_ANALYZER
            ALA_Setparam(opts.autoLA, 1, opts.autoLAstep);
            DISCLOG_SetParam(1);
#endif
         }
         else {
            res = AME_SETUP_ERROR;
         }
      }

      if(loadParam) {
         /* Load parameters */
         loadModelParameters(amesys);
      }

   }
   else { /* Catch AmeExit */
      res = AME_EXIT_ERROR;
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_SETUP, res);

   return res;
}

/***********************************************************
   Purpose    : Initialize the model
   Author	  : J.Andre
   Created on : 2016 - 09 - 08
   Inputs	  : None
   Outputs	  : Error code
   Revision   :
************************************************************/
static AMESystemError AmesysInitialize(AMESIMSYSTEM *amesys)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_INITIALIZE);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* try */
      if(amesys->simOptions->runType & 0x20) {
         /* fixed step initialization */
#if(AME_MODEL_ISEXPLICIT == 1)
         ameSetOptionsFixed(amesys, amesys->simOptions->tInc, amesys->simOptions->fixedType,
            amesys->simOptions->fixedOrder, amesys->simOptions->fixedH, amesys->simOptions->runType);

         if(!ameInitializeConditionsFixed(amesys)) {
            res = AME_INITIALIZE_ERROR;
         }
#else
         amefprintf(stderr,"It is not possible to use a fixed step solver\nfor implicit systems.\n");
         res = AME_INITIALIZE_ERROR;
#endif
      }
      else {
#ifdef AME_VSOLVER_ACCEPTED
         /* variable step initialization */
         ameSetOptions(amesys, amesys->simOptions->tInc, amesys->simOptions->hMax, amesys->simOptions->tol,
            amesys->simOptions->errorType, amesys->simOptions->iWrite, amesys->simOptions->rStrtp,
            amesys->simOptions->statistics, amesys->simOptions->runType, amesys->simOptions->solverType);

         if(!ameInitializeConditions(amesys)) {
            res = AME_INITIALIZE_ERROR;
         }
#else
         amefprintf(stderr,"It is not possible to use a variable step solver\nfor this interface.\n");
         res = AME_INITIALIZE_ERROR;
#endif
      }

#if defined(AME_MEMORY_ACCESS_RT_EXPORT)
#if (NB_WATCH_VAR>0)
         RT_Get_Watch_Var(amesys);
#endif
#endif

   }
   else { /* Catch AmeExit */
      res = AME_EXIT_ERROR;
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_INITIALIZE, res);

   return res;
}

#if(AME_MODEL_ISEXPLICIT == 1)
static void doAFixedStep(AMESIMSYSTEM *amesys, double time)
{
   double timerange;
   double actual_timestep;
   int stepratio;
   int stepratio_ceil;
   int stepratio_floor;
   int i=0;
   int zero=0;
   double tinc;
   int isTimeForPrint=0;
   double next_print_time;

   if (amesys->first_call)
   {
      SetIsPrinting(amesys);
      amesys->FuncEval(amesys, amesys->tlast, amesys->states, amesys->yh, NULL, &zero);
      ClearIsPrinting(amesys);

#if defined(AME_RESULT_FILE) && !defined(STANDALONESIMULATOR)
      amesys->OutputResults(amesys, amesys->tlast);
      amesys->resultFileStructPtr->lastprinttime = amesys->tlast;
#endif
   }

   timerange = time - amesys->tlast;

   if(timerange <= 0.0)
   {
      return;
   }
   if(amesys->simOptions->fixedH > 0.0)
   {
      stepratio = stepratio_ceil = (int)ceil(timerange/amesys->simOptions->fixedH);
      stepratio_floor = (int)floor(timerange/amesys->simOptions->fixedH);

      actual_timestep = timerange/(double)stepratio_ceil;

      if(fabs(actual_timestep-amesys->simOptions->fixedH) > 0.001*amesys->simOptions->fixedH)
      {
         if(stepratio_floor == 0)
         {
#ifdef AMEDEBUG
            amefprintf(stdout,"skipping %14.8e  range= %14.8e  actual_timestep=%14.8e stepratio_ceil=%d   stepratio_floor=%d\n",time, timerange, actual_timestep, stepratio_ceil,stepratio_floor);
#endif
            return;
         }

#ifdef AMEDEBUGw
         amefprintf(stdout,"using floor\n");
         amefprintf(stdout,"timerange=%14.8e\n", timerange);
         amefprintf(stdout,"actual_timestep=%14.8e\n", actual_timestep);
         amefprintf(stdout,"amesys->simOptions->fixedH=%14.8e\n", amesys->simOptions->fixedH);
         amefprintf(stdout,"fabs(actual_timestep-amesys->simOptions->fixedH)=%14.8e\n", fabs(actual_timestep-amesys->simOptions->fixedH));
         amefprintf(stdout,"stepratio_floor=%d\n", stepratio_floor);
         amefprintf(stdout,"stepratio_ceil=%d\n", stepratio_ceil);
#endif
         actual_timestep = timerange/(double)stepratio_floor;
         stepratio = stepratio_floor;
      }

      if(fabs(actual_timestep-amesys->simOptions->fixedH) > 0.001*amesys->simOptions->fixedH)
      {
#ifdef AMEDEBUGw
         amefprintf(stdout,"Adjusting time step %14.8e => %14.8e\n", amesys->simOptions->fixedH, actual_timestep);
         amefprintf(stdout,"stepratio_floor=%d\n", stepratio_floor);
         amefprintf(stdout,"stepratio_ceil=%d\n", stepratio_ceil);
         amefprintf(stdout,"stepratio=%d\n", stepratio);
         amefprintf(stdout,"timerange=%14.8e\n", timerange);
#endif
      }
   }
   else
   {
      /* just single step from the last point in time when we were called */
      stepratio=1;
      actual_timestep = timerange;
   }

#if defined(AME_RESULT_FILE) && !defined(STANDALONESIMULATOR)
   if(amesys->resultFileStructPtr && !amesys->resultFileStructPtr->outoff)
   {
      next_print_time = GetNextPrintTime(&tinc, amesys->resultFileStructPtr->lastprinttime, amesys->simOptions->tFinal, amesys->simOptions->tInc, actual_timestep);
   }
#endif

#ifdef AMERT
   /* Allow changes of the stepratio - typically on RT platforms. */
   {
      double localstepratio=floor(IL_Lemuel_pump_system_FMU_step_ratio);
      if (localstepratio >= 1)
      {
         actual_timestep=timerange/localstepratio;
         stepratio = (int)localstepratio;
      }
   }
#endif
#ifdef STANDALONESIMULATOR
   FixedStepIntegrate(amesys,AME_NBOF_SOLVER_STATES,amesys->tlast,time,amesys->simOptions->tInc,amesys->states,
         amesys->yh,amesys->simOptions->fixedType,amesys->simOptions->fixedOrder,actual_timestep);
#else
	for (i=0; (i<stepratio) && (amesys->requestinterrupt == 0);i++)
   {
      /*Integrate one step */
	  if ( amesys->simOptions->fixedType == 1)
	  {
		DoAnABStep(amesys, amesys->numstates, amesys->simOptions->fixedOrder, &amesys->tlast, actual_timestep, amesys->states, amesys->yh);
	  }
	  else
	  {
		DoAnRKStep(amesys, amesys->numstates, amesys->simOptions->fixedOrder, &amesys->tlast, actual_timestep, amesys->states, amesys->yh);
	  }

#ifndef AMERT
      isTimeForPrint = amesys->resultFileStructPtr && (!amesys->resultFileStructPtr->outoff && ((amesys->tlast >= next_print_time) || ((next_print_time-amesys->tlast)/tinc < TIME_ROUNDOFF)));
      if(isTimeForPrint)
      {
#ifdef AME_PROCESS_TIME
         ProcessTime(1);
#endif
         SetIsPrinting(amesys);
         amesys->FuncEval(amesys, amesys->tlast, amesys->states, amesys->yh, NULL, &zero);
         ClearIsPrinting(amesys);
         amesys->OutputResults(amesys,amesys->tlast);
         next_print_time = GetNextPrintTime(&tinc, amesys->tlast, amesys->simOptions->tFinal, amesys->simOptions->tInc, actual_timestep);
      }
      else
#endif
      {
         amesys->FuncEval(amesys, amesys->tlast, amesys->states, amesys->yh, NULL, &zero);
      }
   }
#endif
   amesys->tlast = time;
}
#endif

static int ameOutputs(AMESIMSYSTEM *amesys, double timeARG, int numOutputs, double *outputARG)
{
   int theprintflag=1;
   double *dot;
   double *v;

   assert(amesys);

   v = amesys->v;
   dot = amesys->dotstates;

   if(numOutputs != amesys->numoutputs)
   {
      char error_message[256];
      sprintf(error_message, "AMEOutputs> Expected %d outputs but got %d\n", amesys->numoutputs, numOutputs);
      DisplayMessage(error_message);
      AmeExit(1);
   }

   if (!isdynrun_())
   {
      /* it is not a dynamic run (neither dynamic nor stabilizing + dynamic) */
      /* stabilizing has already been processed during Init.*/
      /*Exit */
      return 1;
   }

   if(timeARG < amesys->tlast)
   {
      DisplayMessage("trying to integrate backwards\n");
      return 0;
   }
#if(AME_MODEL_ISEXPLICIT == 1)
   amesys->t_end_of_time_slice = timeARG;
#ifndef AMERT
   if(!isfixedstepsolver_())
   {
      if(!IntegrateStep(amesys, amesys->tlast, timeARG))
      {
         DisplayMessage("IntegrateStep failed");
         return 0;
      }
      amesys->tlast = timeARG;
   }
   else
   {
      doAFixedStep(amesys, timeARG);
   }
#else
   doAFixedStep(amesys, timeARG);
#endif

#else
   if(!DIntegrateStep(amesys, AME_NBOF_EXPLICIT_STATE, amesys->tlast, timeARG, amesys->simOptions->tInc, amesys->states,
      amesys->dotstates, amesys->simOptions->hMax, AME_NBOF_SOLVER_STATES, amesys->iwork, amesys->simOptions->reltol,
      amesys->simOptions->abstol, amesys->simOptions->rStrtp, LIW, LRW, amesys->simOptions->statistics,
      amesys->simOptions->stabilOption, amesys->simOptions->iWrite, amesys->simOptions->minimalDiscont,
      amesys->internal_discont, Gis_constraint, &amesys->requestinterrupt))
   {
      DisplayMessage("DIntegrateStep failed");
      return 0;
   }

   amesys->tlast = timeARG;
#endif

#if (AME_NBOF_OUTPUTS > 0)
   memcpy(outputARG, amesys->outputs, amesys->numoutputs*sizeof(double) );
#endif

   return 1;
}

static AMESystemError AmesysComputeAtTstart(AMESIMSYSTEM *amesys, const double *theInputs, double *theOutputs)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_TSTART);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* try */
      if(amesys->simOptions->runType & 0x20) {
         /* fixed step solver */
#if(AME_MODEL_ISEXPLICIT == 1)
         if(!ameEvalTstartFixed(amesys, theInputs, theOutputs)) {
            res = AME_INITIALIZE_ERROR;
         }
#else
         amefprintf(stderr,"It is not possible to use a fixed step solver\nfor implicit systems.\n");
         res = AME_INITIALIZE_ERROR;
#endif
      }
      else {
         /* variable step solver */
         if(!ameEvalTstart(amesys, theInputs, theOutputs)) {
            res = AME_INITIALIZE_ERROR;
         }
      }

   }
   else { /* Catch AmeExit */
      res = AME_EXIT_ERROR;
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_TSTART, res);

   return res;
}

/***********************************************************
   Purpose    : Do a co-simulation step
   Author	  : J.Andre
   Created on : 2016 - 09 - 12
   Inputs	  : None
   Outputs	  : Error code
   Revision   :
************************************************************/
static AMESystemError AmesysStep(AMESIMSYSTEM *amesys, int stepType, const double t, const double *theInputs, double *theOutputs,
   const int doInterpol, const unsigned int *orderTab, const double **polyTab, const E_CS_Reset cs_reset)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_STEP);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* try */

#if defined(AME_MEMORY_ACCESS_RT_EXPORT)
#if (NB_WATCH_REAL_PARAM>0) || (NB_WATCH_INT_PARAM>0)
      RT_Set_Watch_Param(amesys);
#endif
#endif

#if defined(AME_COSIM) && (AME_NBOF_INPUTS > 0) && (!defined(AMERT))
      amesys->doInterpol = doInterpol;

      if(amesys->doInterpol) {
         stepType = 1;
         res = setPolyCosimSlave(amesys->csSlave, orderTab, polyTab);
      }
      else {
         res = setPolyCosimSlaveByInputs(amesys->csSlave, theInputs);
      }
#else
      amesys->doInterpol = 0;
#endif

      if(res == AME_NO_ERROR) {
         if(stepType == 0) {  /* Do a step */
            if(!ameOutputs(amesys, t, AME_NBOF_OUTPUTS, theOutputs)) {
               res = AME_STEP_ERROR;
            }
#ifndef AMERT
            amesys->input_discont = CheckIfColdStartNeed(amesys, theInputs, cs_reset);
            /* Need to do something here if no internal disc and disc printout */
            AddExtraPrintOut(amesys, amesys->tlast);
#endif
            if(amesys->doInterpol == 0) {
               if(!ameInputs(amesys, AME_NBOF_INPUTS, theInputs)) {
                  res = AME_STEP_ERROR;
               }
            }
         }
         else {  /* Do a step 2 */
#ifndef AMERT
            amesys->input_discont = CheckIfColdStartNeed(amesys, theInputs, cs_reset);

            /* Need to do something here if no internal disc and disc printout */
            res |= changePolynomialSlot(amesys->csSlave);
            AddExtraPrintOut(amesys, amesys->tlast);
            res |= changePolynomialSlot(amesys->csSlave);
#endif
            if(amesys->doInterpol == 0) {
               if(!ameInputs(amesys, AME_NBOF_INPUTS, theInputs)) {
                  res = AME_STEP_ERROR;
               }
            }

            if(!ameOutputs(amesys, t, AME_NBOF_OUTPUTS, theOutputs)) {
               res = AME_STEP_ERROR;
            }
#ifndef AMERT
            if(amesys->doInterpol != 0) {
               if( getInputsCosimSlave(amesys->csSlave, t, amesys->inputs) != AME_NO_ERROR ) {
                  res = AME_STEP_ERROR;
               }
            }
#endif
         }
         amesys->num_steps_taken++;
#ifndef AMERT
         res |= changePolynomialSlot(amesys->csSlave);
         res |= setTciCosimSlave(amesys->csSlave, t);
#endif
      }

#if defined(AME_MEMORY_ACCESS_RT_EXPORT)
#if (NB_WATCH_VAR>0)
      RT_Get_Watch_Var(amesys);
#endif
#endif
   }
   else { /* Catch AmeExit */
      res = AME_EXIT_ERROR;
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_STEP, res);

   return res;
}

/***********************************************************
   Purpose    : Set solver parameters
   Author	  : J.Andre
   Created on : 2016 - 09 - 09
   Inputs	  : None
   Outputs	  :
   Revision   :
************************************************************/
static AMESystemError AmesysSetSolverParam(AMESIMSYSTEM *amesys, const solverSettings *solver)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_SOLVER_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialyze Amesystem solver parameters */
      simOptSetSolverParam(amesys->simOptions,solver);
   }

   return res;
}

/***********************************************************
   Purpose    : Get solver parameters
   Author	  : J.Andre
   Created on : 2016 - 09 - 09
   Inputs	  : None
   Outputs	  :
   Revision   :
************************************************************/
static AMESystemError AmesysGetSolverParam(AMESIMSYSTEM *amesys, solverSettings *solver)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_SOLVER_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialyze Amesystem solver parameters */
      simOptGetSolverParam(amesys->simOptions, solver);
   }

   return res;
}

/***********************************************************
   Purpose    : Set run parameters
   Author	  : J.Andre
   Created on : 2016 - 09 - 09
   Inputs	  : None
   Outputs	  :
   Revision   :
************************************************************/
static AMESystemError AmesysSetSimParam(AMESIMSYSTEM *amesys, const simSettings *simOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialyze Simulation parameters */
      simOptSetSimParam(amesys->simOptions,simOpt);
   }

   return res;
}

static AMESystemError AmesysGetSimParam(AMESIMSYSTEM *amesys, simSettings *simOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Get Simulation parameters */
      simOptGetSimParam(amesys->simOptions, simOpt);
   }

   return res;
}

static AMESystemError AmesysSetSimItem(AMESIMSYSTEM *amesys, const int Id, const int enabled)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialyze Amesystem solver parameters */
      if(simOptSetOneOfSimParam(amesys->simOptions, Id, enabled) != 0) {
         res = AME_SETTINGS_ERROR;
      }
   }

   return res;
}

static AMESystemError AmesysGetSimItem(AMESIMSYSTEM *amesys, const int Id, int *enabled)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialyze Amesystem solver parameters */
      if(simOptGetOneOfSimParam(amesys->simOptions, Id, enabled) != 0) {
         res = AME_SETTINGS_ERROR;
      }
   }

   return res;
}

/***********************************************************
   Purpose    : Set run parameters
   Author	  : J.Andre
   Created on : 2016 - 09 - 09
   Inputs	  : None
   Outputs	  :
   Revision   :
************************************************************/
static AMESystemError AmesysSetStdOptions(AMESIMSYSTEM *amesys, const stdOptions *stdOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialize standard parameters */
      simOptSetStdParam(amesys->simOptions,stdOpt);
   }

   return res;
}

static AMESystemError AmesysGetStdOptions(AMESIMSYSTEM *amesys, stdOptions *stdOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Get standard parameter */
      simOptGetStdParam(amesys->simOptions, stdOpt);
   }

   return res;
}

static AMESystemError AmesysSetStdItem(AMESIMSYSTEM *amesys, const int Id, const int enabled)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialize standard parameter */
      if(simOptSetOneOfStdParam(amesys->simOptions, Id, enabled) != 0) {
         res = AME_SETTINGS_ERROR;
      }
   }

   return res;
}

static AMESystemError AmesysGetStdItem(AMESIMSYSTEM *amesys, const int Id, int *enabled)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialize standard parameter */
      if(simOptGetOneOfStdParam(amesys->simOptions, Id, enabled) != 0) {
         res = AME_SETTINGS_ERROR;
      }
   }

   return res;
}

static AMESystemError AmesysSetALAParam(AMESIMSYSTEM *amesys, const autoLAOptions *alaOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Initialize standard parameters */
      simOptSetAlaParam(amesys->simOptions, alaOpt);
   }

   return res;
}

static AMESystemError AmesysGetALAParam(AMESIMSYSTEM *amesys, autoLAOptions *alaOpt)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      /* Get standard parameter */
      simOptGetAlaParam(amesys->simOptions, alaOpt);
   }

   return res;
}

/***********************************************************
   Purpose     : Turn off/on results
   Author	   : J.Andre
   Created on  : 2016 - 09 - 09
   Inputs	   :
      outoff   :  0 : result file off
                  1 : result file on
   Outputs	   :
   Revision    :
************************************************************/
static AMESystemError AmesysEnableResult(AMESIMSYSTEM *amesys, const int out)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      if(out) {
         amesys->simOptions->outoff = 0;
      }
      else {
         amesys->simOptions->outoff = 1;
      }
   }

   return res;
}

/***********************************************************
   Purpose     : Return the state of Amesystem
   Author	   : J.Andre
   Created     : 2016 - 09 - 05
   Inputs	   : None
   Outputs	   :
      state    : machine state
   Revision    :
************************************************************/
static unsigned int AmesysGetState(AMESIMSYSTEM *amesys)
{
   if(amesys) {
      return amesys->systemState;
   }
   else {
      return AMESTATE_IDLE;
   }
}

static AMESystemError AmesysGetModelInfo(unsigned int *numinputs, unsigned int *numoutputs, unsigned int *numstates, unsigned int *numimplicits)
{
   *numinputs = AME_NBOF_INPUTS;
   *numoutputs = AME_NBOF_OUTPUTS;
   *numstates = AME_NBOF_EXPLICIT_STATE;
   *numimplicits = AME_NBOF_IMPLICIT_STATE;

   return AME_NO_ERROR;
}

static AMESystemError AmesysGetModelPortName(const char **inputName[], const char **outputName[])
{
   *inputName = GinputVarTitles;
   *outputName = GoutputVarTitles;

   return AME_NO_ERROR;
}

static AMESystemError AmesysGetModelNumParam(unsigned int *numParam)
{
   *numParam = AME_NBOF_PARAMS;

   return AME_NO_ERROR;
}

static AMESystemError AmesysGetParamType(const int nbParam, const int idx[], E_ParamCType paramType[])
{
   AMESystemError res = AME_NO_ERROR;
   int i;

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      if(i < AME_NBOF_PARAMS) {
         switch(GParamInfo[idx[i]].category) {
            case E_Int_Param:
               paramType[i] = E_CType_IntParam;
            break;
            case E_Text_Param:
               paramType[i] = E_CType_StringParam;
            break;
            default:
               paramType[i] = E_CType_DoubleParam;
            break;
         }
         return AME_NO_ERROR;
      }
      else {
         res = AME_PARAM_IDX_ERROR;
      }
   }

   return res;
}

static AMESystemError AmesysFindParamFromVar(const int nbParam, const int varIdx[], int paramIdx[], E_ParamCategory category[])
{
   AMESystemError res = AME_NO_ERROR;
   int idx, i;

   for(idx = 0; (idx < nbParam) && (res == AME_NO_ERROR); idx++) {
      res = AME_PARAM_IDX_ERROR;
      for(i = 0; i < AME_NBOF_PARAMS; i++) {
         if(GParamInfo[i].varIdx == varIdx[idx]) {
            paramIdx[idx] = i;
            category[idx] = GParamInfo[i].category;
            res = AME_NO_ERROR;
         }
      }
   }

   return res;
}

static AMESystemError AmesysGetIntParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], int value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_MODEL_PARAM);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      res = getIntParameter(amesys->pModel, idx[i], &value[i]);
   }

   return res;
}

static AMESystemError AmesysGetDoubleParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], double value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_MODEL_PARAM);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      res = getDoubleParameter(amesys->pModel, idx[i], &value[i]);
   }

   return res;
}

static AMESystemError AmesysGetStringParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], const char* value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_MODEL_PARAM);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      res = getStringParameter(amesys->pModel, idx[i], &value[i]);
   }

   return res;
}

static AMESystemError AmesysSetIntParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], const int value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      int isTunable;
      res = isParamTunable(amesys->pModel, idx[i], &isTunable);

      if(res ==  AME_NO_ERROR) {
         if(isTunable) {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);
         }
         else {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);
         }
      }
      if(res ==  AME_NO_ERROR) {
         res = setIntParameter(amesys->pModel, idx[i], value[i]);
      }
      if(res == AME_NO_ERROR) {
         SignalInputChange();
         res = setParamAsUserDefined(amesys->pModel, idx[i]);
      }
   }

   return res;
}

static AMESystemError AmesysSetDoubleParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], const double value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      int isTunable;
      res = isParamTunable(amesys->pModel, idx[i], &isTunable);

      if(res ==  AME_NO_ERROR) {
         if(isTunable) {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);
         }
         else {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);
         }
      }
      if(res ==  AME_NO_ERROR) {
         res = setDoubleParameter(amesys->pModel, idx[i], value[i]);
      }
      if(res == AME_NO_ERROR) {
         SignalInputChange();
         res = setParamAsUserDefined(amesys->pModel, idx[i]);
      }
   }

   return res;
}

static AMESystemError AmesysSetStringParamValue(AMESIMSYSTEM *amesys, const int nbParam, const int idx[], const char* value[])
{
   AMESystemError res;
   int i;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);

   for(i = 0; (i < nbParam) && (res == AME_NO_ERROR); i++) {
      int isTunable;
      res = isParamTunable(amesys->pModel, idx[i], &isTunable);

      if(res ==  AME_NO_ERROR) {
         if(isTunable) {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM_TUNABLE);
         }
         else {
            res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);
         }
      }
      if(res ==  AME_NO_ERROR) {
         res = setStringParameter(amesys->pModel,idx[i], value[i]);
      }
      if(res == AME_NO_ERROR) {
         res = setParamAsUserDefined(amesys->pModel, idx[i]);
      }
   }

   return res;
}

static AMESystemError AmesysGetDoubleGlobalParamValue(AMESIMSYSTEM *amesys, const int nbParam, const char* name[], double value[])
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_MODEL_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      int i;
      for(i = 0; i<nbParam; i++) {
         if(getGlobalParamValueByName(name[i], &value[i]) != no_error) {
            res = AME_GLOBAL_PARAMETER_ERROR;
            break;
         }
      }
   }

   return res;
}

static AMESystemError AmesysGetStringGlobalParamValue(AMESIMSYSTEM *amesys, const int nbParam, const char* name[], const char* value[])
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_GET_MODEL_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      int i;
      for(i = 0; i<nbParam; i++) {
         if(getTextGlobalParamValueByName(name[i], &value[i]) != no_error) {
            res = AME_GLOBAL_PARAMETER_ERROR;
            break;
         }
      }
   }

   return res;
}

static AMESystemError AmesysSetIntGlobalParamValue(AMESIMSYSTEM *amesys, const int nbParam, const char* name[], const int value[])
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      int i;
      for(i = 0; i<nbParam; i++) {
         if(ChangeOrAddIntGlobalParamValue(name[i], value[i], 1) != AME_NO_ERROR) {
            res = AME_GLOBAL_PARAMETER_ERROR;
            break;
         }
      }
   }

   return res;
}

static AMESystemError AmesysSetDoubleGlobalParamValue(AMESIMSYSTEM *amesys, const int nbParam, const char* name[], const double value[])
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      int i;
      for(i = 0; i<nbParam; i++) {
         if(ChangeOrAddRealGlobalParamValue(name[i], value[i], 1) != AME_NO_ERROR) {
            res = AME_GLOBAL_PARAMETER_ERROR;
            break;
         }
      }
   }

   return res;
}

static AMESystemError AmesysSetStringGlobalParamValue(AMESIMSYSTEM *amesys, const int nbParam, const char* name[], const char* value[])
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_MODEL_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      int i;
      for(i = 0; i<nbParam; i++) {
         if(ChangeOrAddTextGlobalParamValue(name[i], value[i], 1) != AME_NO_ERROR) {
            res = AME_GLOBAL_PARAMETER_ERROR;
            break;
         }
      }
   }

   return res;
}

static AMESystemError AmesysRequestRunInterrupt(AMESIMSYSTEM *amesys)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_REQ_RUN_INTERRUPT);

   if(res == AME_NO_ERROR) { /* Request accepted */
      amesys->requestinterrupt = 1;

	   /* Update state and result */
	   res =  AmesysUpdateState(amesys, AME_CMD_REQ_RUN_INTERRUPT, res);
   }

   return res;
}

static AMESystemError AmesysSetFinalTime(AMESIMSYSTEM *amesys, const double finaltime)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      amesys->simOptions->tFinal = finaltime;
   }

   return res;
}

static AMESystemError AmesysSetInitTime(AMESIMSYSTEM *amesys, const double inittime)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      amesys->simOptions->tStart = inittime;
   }

   return res;
}

static AMESystemError AmesysSetPrintInterval(AMESIMSYSTEM *amesys, const double tInc)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      amesys->simOptions->tInc = tInc;
   }

   return res;
}

static AMESystemError AmesysSetLogger( int (*newameInternalfprintf)(FILE *fp, const char *fmt, va_list ap) )
{
   ameInstallFprintf(newameInternalfprintf);
   return AME_NO_ERROR;
}

static AMESystemError AmesysSetInputDir(AMESIMSYSTEM *amesys, const char *inputDir)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_ENV);

   if(res == AME_NO_ERROR) { /* Request accepted */
      AmeSetInputDir(amesys, inputDir);
   }

   return res;
}

static AMESystemError AmesysSetOutputDir(AMESIMSYSTEM *amesys, const char *outputDir)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_ENV);

   if(res == AME_NO_ERROR) { /* Request accepted */
      AmeSetOutputDir(amesys, outputDir);
   }

   return res;
}

static AMESystemError AmesysSetBaseFilesName(AMESIMSYSTEM *amesys, const char *baseName, const char* extension)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_ENV);

   if(res == AME_NO_ERROR) { /* Request accepted */
      AmeSetModelBaseName(amesys, baseName, extension);
   }

   return res;
}

static AMESystemError AmesysSetResultFilesName(AMESIMSYSTEM *amesys, const char *outName)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_ENV);

   if(res == AME_NO_ERROR) { /* Request accepted */
      AmeSetResultFileName(amesys, outName);
   }

   return res;
}

/* To ensure compliancy with old template and entry points */
static AMESystemError AmesysSetParamAsInitModel(AMESIMSYSTEM *amesys,
                  double time,
                  double PrintInterval,
                  double MaxTimeStep,
                  double tolerance,
                  int errCtrl,
                  int writeLevel,
                  int extraDisconPrints,
                  int runStats,
                  int runType,
                  int thesolvertype)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);
   res |= AmesysControlRequest(amesys, AME_CMD_SET_SOLVER_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      SIMOptions *sim_options = amesys->simOptions;

      solverSettings solver;
      simSettings sim;
      stdOptions std;

      solver.integType =  0; /* Variable solver */
      solver.variable.hMax = MaxTimeStep;
      solver.variable.tol = tolerance;
      solver.variable.errorType = errCtrl;

      simOptSetSolverParam(amesys->simOptions, &solver);

      sim.tStart = time;
      sim.tFinal = amesys->simOptions->tFinal;
      sim.simOpt = 0;

      if(runType & 0x01) {
         sim.simOpt |= SIMOPT_SIM_CONTINUATION_RUN;
      }
      if(runType & 0x02) {
         sim.simOpt |= SIMOPT_SIM_USE_FINAL_VALUES;
      }
      if(writeLevel != 2) {
         sim.simOpt |= SIMOPT_SIM_MONITOR_TIME;
      }
      if(runStats) {
         sim.simOpt |= SIMOPT_SIM_STATISTICS;
      }

      if(PrintInterval <= 0) {
         sim.tInc = -PrintInterval;
         amesys->simOptions->outoff = 1;
      }
      else {
         sim.tInc = PrintInterval;
         amesys->simOptions->outoff = 0;
      }

      simOptSetSimParam(amesys->simOptions, &sim);

      std.simMode = ((runType & 0x0c) >> 2);
      std.stdOpt = 0;

      if(extraDisconPrints) {
         std.stdOpt |= SIMOPT_STD_DISC_PRINTOUT;
      }
      if(runType & 0x10) {
         std.stdOpt |= SIMOPT_STD_HOLD_INPUTS;
      }
      if(amesys->simOptions->stabilOption & 0x01) {
         std.stdOpt |= SIMOPT_STD_LOCK_STATES;
      }
      if(amesys->simOptions->stabilOption & 0x02) {
         std.stdOpt |= SIMOPT_STD_DIAGNOSTICS;
      }
      if(thesolvertype) {
         std.stdOpt |= SIMOPT_STD_CAUTIOUS;
      }
      if(amesys->simOptions->minimalDiscont) {
         std.stdOpt |= SIMOPT_STD_MIN_DISC_HANDLING;
      }
      if(runType & 0x100) {
         std.stdOpt |= SIMOPT_STD_DISABLE_OPTIMAZED;
      }
      if(sim_options->activityIndex & 0x01) {
         std.stdOpt |= SIMOPT_STD_ACTIVITY;
      }
      if(sim_options->activityIndex & 0x02) {
         std.stdOpt |= SIMOPT_STD_POWER;
      }
      if(sim_options->activityIndex & 0x04) {
         std.stdOpt |= SIMOPT_STD_ENERGY;
      }
      simOptSetStdParam(amesys->simOptions, &std);
   }
   return res;
}

/* To ensure compliancy with old template and entry points */
static AMESystemError AmesysSetParamAsFixedStep(AMESIMSYSTEM *amesys,
                                       double start_time, int run_type, int solver_type,
                                       int runge_kutta_order, double fixed_h, double printinterval)
{
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_SET_RUN_PARAM);
   res |= AmesysControlRequest(amesys, AME_CMD_SET_SOLVER_PARAM);

   if(res == AME_NO_ERROR) { /* Request accepted */
      solverSettings solver;

      res = AME_NO_ERROR;

      solver.integType = 1; /* Fixed solver */
      solver.fixed.fixedH = fixed_h;
      solver.fixed.fixedOrder = runge_kutta_order;
      solver.fixed.fixedType = solver_type;

      simOptSetSolverParam(amesys->simOptions, &solver);
      amesys->simOptions->tStart = start_time;

      if(printinterval <= 0) {
         amesys->simOptions->outoff = 1;
         amesys->simOptions->tInc = -printinterval;
      }
      else {
         amesys->simOptions->outoff = 0;
         amesys->simOptions->tInc = printinterval;
      }
      simOptSetOneOfSimParam(amesys->simOptions, SIMOPT_SIM_CONTINUATION_RUN, run_type & 0x01);
      simOptSetOneOfSimParam(amesys->simOptions, SIMOPT_SIM_USE_FINAL_VALUES, (run_type & 0x02)>>1);
   }
   return res;
}

#ifndef AME_COSIM

static AMESystemError AmesysInitME(AMESIMSYSTEM *amesys, int withAssembly)
{
   AMESystemError res = AME_NO_ERROR;
   int jump_ret;

   res = AmesysControlRequest(amesys, AME_CMD_INITIALIZE);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   /* Load parameters */
   res = Input(amesys);

   /* Set initial values */
   modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);

   if(res == AME_NO_ERROR) {
      if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* try */

         setstarttime_(amesys->simOptions->tStart);
         setfinaltime_(amesys->simOptions->tFinal);

         amesys->first_call = 1;  /* should this be done or not ?*/
         amesys->internal_discont = 1;

         amesys->tlast = TLAST_MIN;

         memset(amesys->ecount,0,amesys->numstates*sizeof(int));
         memset(amesys->dotstates,0,amesys->numstates*sizeof(double));

         PreInitialize(amesys,amesys->states);

#ifndef AME_INPUT_IN_MEMORY
         if( NeedReloadInputFiles() != 0 )
         {
            ClearGPs();
            Input(amesys);
            /* Set initial values */
            modelSetInitValues(amesys->pModel, amesys->states, amesys->discrete_states);
            ClearReloadedFiles();
         }
#endif

         Initialize(amesys,amesys->states);

         OverloadStatesWithRegister(amesys, amesys->states, SVREGISTER_DEFAULT);

         if(withAssembly) {
            doAssembly(amesys);
         }
         amesys->tlast = getstarttime_();
#ifdef AME_PROCESS_TIME
         /* Initialization of time timers */
         ProcessTime(0);
#endif
      }
      else { /* Catch AmeExit */
         res = AME_EXIT_ERROR;
      }
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_INITIALIZE, res);

   if(res == AME_NO_ERROR) {
      /* Go in Run state: simulate acknowledgment of evaluation at t start command */
      res =  AmesysUpdateState(amesys, AME_CMD_TSTART, res);

#if defined(AME_MEMORY_ACCESS_RT_EXPORT)
#if (NB_WATCH_VAR>0)
         RT_Get_Watch_Var(amesys);
#endif
#endif
   }

   return res;
}

static AMESystemError AmesysFuncEvalME(AMESIMSYSTEM *amesys, double t, double *y, double *yprime, double *delta, int *flag)
{
   int jump_ret;
   AMESystemError res;

   SetGlobalSystem(amesys);
   res = AmesysControlRequest(amesys, AME_CMD_FUNCEVAL);

   if(res != AME_NO_ERROR) {
      /* Request not accepted */
      return res;
   }

   if( (jump_ret = setjmp(amesys->jump_env)) == 0) { /* try */
      amesys->doInterpol = 0;
      localFuncEval(amesys, t, y, yprime, delta, flag);
   }
   else { /* Catch AmeExit */
      res = AME_EXIT_ERROR;
   }

   /* Update state and result */
   res =  AmesysUpdateState(amesys, AME_CMD_FUNCEVAL, res);
   return res;
}

#endif

/****************************/

#define SYSNME Lemuel_pump_system_FMU_

#if defined(STANDALONESIMULATOR) || defined(OLS)
#include "ame_standalone_simulator.h"

#elif defined(FMICS1)
#include "ame_fmics1.h"
#elif defined(FMICS2)
#include "ame_fmics2.h"
#elif defined(FMICS3)
#include "ame_fmics3.h"
#elif defined(FMIME1) || defined(FMIME2)
#if(AME_MODEL_ISEXPLICIT == 0)
#error "FMI for model exchange is not allowed for implicit model."
#elif defined(AMERT)
#error "FMU for real-time is not allowed for model exchange."
#elif defined(FMIME1)
#include "ame_fmime1.h"
#else
#include "ame_fmime2.h"
#endif
#elif defined(FMIX)
#include "ame_user_cosim.h"

#elif defined(AMEUSERCOSIM)
#include "ame_user_cosim.h"

#elif defined(AME_CS_SIMULINK)
#include "ame_simulink_cosim.h"

#elif defined(AME_ME_SIMULINK)
#include "ame_simulink_me.h"

#elif defined(AME_CS_ADAMS)
#include "ame_adams_cosim.h"

#elif defined(AME_ME_ADAMS)
#include "adams_cont.h"

#elif defined(AME_CS_MOTION)
#include "ame_motion_cosim.h"

#elif defined(AME_ME_MOTION)
#include "ame_motion_me.h"

#elif defined(DISCRETEPART)
#include "ame_discrete_part.h"

#elif defined(GEN_COSIM)
#include "gen_cosim.h"

#elif defined(AME_CS_SIMULINK_SCANER)
#include "ame_simulink_scaner.h"

#else
#error "Unknown interface defined. Cannot generate Amesim model code."
#endif
