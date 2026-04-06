/* File containing any direct memory access for variables in the watch list */

#define NB_WATCH_VAR 0
static double *RT_Export_Vars = NULL;
static const int *RT_Watch_Vars_Idx = NULL;
#define NB_WATCH_REAL_PARAM 0
static double *RT_Export_RealParam = NULL;
static const int *RT_Watch_RP_Idx = NULL;
#define NB_WATCH_INT_PARAM 0
static int *RT_Export_IntParam = NULL;
static const int *RT_Watch_IP_Idx = NULL;
