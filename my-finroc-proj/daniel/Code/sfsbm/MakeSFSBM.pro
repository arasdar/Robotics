TEMPLATE	= app
CONFIG		= qt warn_on release
HEADERS		= FunctionSFSBM.h \           
                  SFSBM.h                
SOURCES	        = FunctionSFSBM.cc \               
                  f_SFSBM.cc \ 
                  SFSBM.cc  
TARGET		= SFSBM 
LIBS            = -lm -L/usr/X11R6/lib -lXpm -lXaw -lXmu -lSM -lICE -lXext -lX11




