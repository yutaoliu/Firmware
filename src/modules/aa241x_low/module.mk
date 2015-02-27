#
# AA241x low priority application
#

MODULE_COMMAND	= aa241x_low

SRCS		= aa241x_low_main.cpp \
		  aa241x_low.cpp \
		  aa241x_low_aux.cpp

MODULE_STACKSIZE = 1200

MAXOPTIMIZATION = -Os