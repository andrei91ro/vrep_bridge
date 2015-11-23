import vrep # for vrep functions
import logging
import colorlog # colors log output
from enum import Enum # for enumerations (enum from C)
import ctypes

class Signal(Enum):

    """Enumeration of signal types"""
    
    # signal names used for requesting values from simulated kilobot
    msg_distance = 'request_msg_distance'
    msg_light_sensor = 'request_light_sensor'

    # signal name used to set values of simulated kilobots
    motion = 'set_motion'
    led = 'set_led'
#end of class Signal

def waitForCmdReply():
    while True:
        result,string=vrep.simxReadStringStream(clientID, 'reply_signal', vrep.simx_opmode_streaming)
        string = str(string, 'utf-8')
        if (result==vrep.simx_return_ok and string != ''):
            return string

##########################################################################
#   MAIN
formatter = colorlog.ColoredFormatter(
        "%(log_color)s%(levelname)-8s %(message)s %(reset)s",
        datefmt=None,
        reset=True,
        log_colors={
                'DEBUG':    'cyan',
                'INFO':     'green',
                'WARNING':  'yellow',
                'ERROR':    'red',
                'CRITICAL': 'red,bg_white',
        },
        secondary_log_colors={},
        style='%'
)
colorlog.basicConfig(level = logging.DEBUG)
stream = colorlog.root.handlers[0]
stream.setFormatter(formatter);

logging.info('Attempting to connect')

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to V-REP

if (clientID != -1):
    logging.info('Connected to remote API server, clientID = %d' % clientID)
    
    vrep.simxWriteStringStream(clientID, 'signal', 'abc', vrep.simx_opmode_oneshot)
    
    logging.debug("Reply = %s", waitForCmdReply())
    
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

else:
    logging.error('Failed connecting to remote API server')
