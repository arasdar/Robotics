TEMPLATE	= app
CONFIG		= qt warn_on release
HEADERS		= utils.h                
SOURCES   = utils.cc \                  
            sp_eval.cc
TARGET		= SpEval
LIBS      = -lm -L/usr/X11R6/lib 
#-lXpm -lXaw -lXmu -lSM -lICE -lXext -lX11




