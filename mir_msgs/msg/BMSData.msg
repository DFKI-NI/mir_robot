float64 pack_voltage
float64 charge_current
float64 discharge_current
int32 state_of_charge
float64 remaining_time_to_full_charge
int32 remaining_capacity
int32 state_of_health
int32 DISCHARGING=1
int32 CHARGING=2
int32 status_flags
int32 temperature
uint32[] cell_voltage # In Mk2 robots and above the BMS provides data for 8 battery cells. MiR500 robots have BMS for 13 battery cells

float64 last_battery_msg_time