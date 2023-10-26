//Group B4: David Goegl, Ved Varshney, Emre Eryilmaz
#include "hall_sensor.h"
// Function hall_sensor_get_field converts the measured voltage value to a magnetic field (in milli-tesla)
float hall_sensor_get_field(float voltage, float voltage_0)
{
   //sensitivity is 0.03 V/mT
   return (voltage - voltage_0) * (1 / 0.03);
}
