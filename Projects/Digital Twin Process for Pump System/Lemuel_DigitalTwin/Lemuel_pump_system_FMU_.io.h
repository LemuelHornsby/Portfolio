static void ameSetInputs(int numinputs, double *inputs)
{
   inputs[0]=pump_speed;
   inputs[1]=flowrate_target;
}
static void ameSetOutputs(int numoutputs, double *outputs)
{
   flow_rate = outputs[0];
   pressure_increase = outputs[1];
   mech_power = outputs[2];
}
