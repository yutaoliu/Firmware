# AA241x mission status

bool in_mission				# if true, currently running a mission
uint64 start_time  		    # time since mission start (in minutes)
float32 current_time		# battery capacity used since mission start (in mAh)
float32 final_time			# current mission score
bool mission_failed         # if you have failed the mission
bool in_turn                # if you are currently in a turn
int32 turn_num              # which turn are you on?
float32 turn_degrees          # amount (in degrees) of turn you have completed
int32 num_violations        # number of violations you've had
bool in_violation           # whether or not you are in violation
bool out_of_bounds          # whether you have violated the field boundaries