# AA241x mission status

bool in_mission				# if true, currently running a mission
uint64 start_time  		    # time since mission start (in minutes)
float32 mission_time		# battery capacity used since mission start (in mAh)
float32 final_time			# current mission score
bool mission_failed         # if you have failed the mission
int32 phase_num             # mission phase (1,2,3 or 4=complete)
int32 num_plumes_found      # number of plumes found
bool in_plume               # true is in a plume radius
bool out_of_bounds          # whether you have violated the field boundaries
float32[5] plume_N          # North position of plumes
float32[5] plume_E          # East position of plumes
float32[5] plume_radius     # radii of plumes
