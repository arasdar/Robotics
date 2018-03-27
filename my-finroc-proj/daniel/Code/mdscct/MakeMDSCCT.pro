TEMPLATE	= app
CONFIG		= qt warn_on debug
HEADERS		= FunctionMDSCCT.h \             
                  MDSCCT.h                
SOURCES	        = FunctionMDSCCT.cc \                  
                  f_MDSCCT.cc \ 
                  f_MDSCCT_2.cc \
                  MDSCCT.cc
TARGET		= MDSCCT
LIBS            = -lm -L/usr/X11R6/lib 
# -lXpm -lXaw -lXmu -lSM -lICE -lXext -lX11






