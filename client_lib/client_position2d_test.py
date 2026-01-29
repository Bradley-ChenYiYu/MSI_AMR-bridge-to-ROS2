import math
import signal

from ctypes import *
from playercpp import *
from playerc import *
import time
import ctypes


IsExit=False;

###############################################################################
def Handler(signum, frame):
    global IsExit
    IsExit = True
    
########################################################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, Handler)
    signal.signal(signal.SIGTERM, Handler)
    
    try:
        HOST='localhost';
        #HOST='172.16.114.151';
        # Create a client object and connect it
        dev0 = PlayerClient(HOST, 6665)
    
    
        # Create a proxy for dispatch:0
        p2d=Position2dProxy(dev0,0);
    
        p2d.ResetOdometry();
        time.sleep(3)
        p2d.SetSpeed(0.1,0.0)
        startTime=time.time()
        while (IsExit==False):
            if ( dev0.Peek(1000) ):
                dev0.Read();
                if ( p2d.IsFresh()==True ):
                    p2d.NotFresh();
                    x=p2d.GetXPos();
                    y=p2d.GetYPos();
                    a=p2d.GetYaw();
                    print("x:%f y:%f a:%f" % (x,y,a))
            if ( time.time()-startTime>10 ):
                break;
        
        p2d.SetSpeed(0.0,0.0)
        time.sleep(5)
        
    except RuntimeError as e:
        print(e)
    except Exception as e:
        print(e)        
    except KeyboardInterrupt:
        print("program exit\n");

    
    print("end program")
    del p2d;
    del dev0;
