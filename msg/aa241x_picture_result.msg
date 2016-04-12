# AA241x picture result

bool pic_taken			# if true, a picture was successfully taken
uint64 time_us		# unix timestamp in microseconds at which the picture was taken
float32 center_n			# North coordinate (North distance from center in [m]) of the center of the picture
float32 center_e			# East coordinate (East distance from center in [m]) of the center of the picture
float32 center_d			# Down coordinate (negative vertical distance from bottom) of the center of the picture
float32 pic_d			# diameter in [m] of the picture taken
int32 num_cells			# number of grid cells in view (length of vectors i, j, and state)

int32[9] i
int32[9] j
int32[9] state