/*
 Function hall_sensor_get_field converts the measured voltage value to a magnetic field (in milli-tesla)
 Inputs:
 int voltage - voltage value in V
 double voltage_0 - quiescent voltage V_0 in V
 Output:

 */
float hall_sensor_get_field(float voltage, float voltage_0);
