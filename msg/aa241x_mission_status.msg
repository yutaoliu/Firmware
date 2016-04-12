# AA241x mission status

bool in_mission				# if true, currently running a mission
bool can_start				# if false, mission starting conditions violated
float32 mission_time		# time since mission start (in minutes)
float32 battery_used		# battery capacity used since mission start (in mAh)
float32 score				# current mission score
int32 wind_direction		# the wind direction for this mission
int32 mission_index			# the index of the currently running mission