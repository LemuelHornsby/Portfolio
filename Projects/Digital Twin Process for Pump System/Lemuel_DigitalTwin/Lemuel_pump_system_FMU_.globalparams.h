
/* The following functions initializes the global parameters for the Amesim model Lemuel_pump_system_FMU */


/* The following function is called to initialize the global parameters for the Amesim model Lemuel_pump_system_FMU */
static void ameAddGlobalParamsFromMemory(AMESIMSYSTEM *amesys, char *errmsg)
{
   ameReadMoreGlobalParams(NULL, 0, AMEUSEGLOBALS);
   SetGlobalParamReadFile(1);
}
