TEMPLATE	= app
CONFIG		= qt warn_on release
HEADERS		= utils.h                
SOURCES   = utils.cc \                  
            eval.cc
TARGET		= Eval
LIBS      = -lm -L/usr/X11R6/lib 
#-lXpm -lXaw -lXmu -lSM -lICE -lXext -lX11




