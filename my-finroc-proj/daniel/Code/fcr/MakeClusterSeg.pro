TEMPLATE	= app
CONFIG		= qt warn_on release
HEADERS		= FunctionClusterSeg.h \                    
                  ClusterSeg.h                
SOURCES	        = FunctionClusterSeg.cc \                  
                  f_ClusterSeg.cc \ 
                  ClusterSeg.cc  
TARGET		= ClusterSeg 
LIBS            = -lm -L/usr/X11R6/lib -lXpm -lXaw -lXmu -lSM -lICE -lXext -lX11




