#
# AA241x mission logic application
#

MODULE_COMMAND	= aa241x_mission

SRCS		= aa241x_mission_main.cpp \
		  aa241x_mission_params.c

MODULE_STACKSIZE = 1200

MAXOPTIMIZATION = -Os