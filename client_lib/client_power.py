#! /usr/bin/env python

import math
import signal
import time
import numpy as np
from ctypes import *
from playercpp import *


IsRunProgram=True;
########################################################################

########################################################################

if __name__ == "__main__":
  # Create a client object and connect it
  robot = PlayerClient('localhost')
  #robot = PlayerClient('172.16.114.217')


  power = PowerProxy(robot, 0);




  try:
    print("program start\n");

    # Retrieve the pose of the laser with respect to its parent

    startTime=time.time();
    while (True):
      if ( robot.Peek(300)==True ): 
        robot.Read();
      else:
        print("peek timeout\n");
      
      print("time=%.3f Percent=%3.3f<%%> Voltage=%3.2f<V> watts=%3.3f<w> RunTime=%3.3f<hour> ResidualLife=%3.3f<%%>\n" % (
        power.GetDataTime(),
        power.GetPercent(),
        power.GetCharge(),
        power.GetWatts(),
        power.GetRunTime()/3600.0,
        power.GetResidualLife()))
        
  except RuntimeError as e:
    print(e)  
  except Exception as e:
    print("error:",e)             
  except KeyboardInterrupt:
    print("program exit\n"); 

  # TODO: we can/should proxies and robot explicity?
  del power
  del robot
