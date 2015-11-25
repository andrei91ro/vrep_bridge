from vrep_bridge import vrep # for vrep functions
import logging
import colorlog # colors log output
from enum import IntEnum # for enumerations (with int value) (enum from C)
#import ctypes
import time # for time.sleep()

class SignalType(IntEnum):

    """Enumeration of signal types"""
    getState = 1
    setState = 2
#end class SignalType

class Motion():

    """Enumeration of motion types accepted by Kilobot"""
    
    stop = 0
    forward = 1
    left = 2
    right = 3
#end class Motion

class VrepBridge():

    """Creates a connection between a Python application and V-REP that allows bidirectional communication"""
    # signal = type_request param1 param2 param3
    # reply_signal = type val1 val2 val3

    def __init__(self):
        """"""
        logging.info('Attempting to connect')

        vrep.simxFinish(-1) # just in case, close all opened connections
        self.__clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to V-REP

        if (self.__clientID == -1):
            logging.error('Failed connecting to remote API server')
            raise()

        logging.info('Connected to remote API server, clientID = %d' % self.__clientID)
    #end __init__()

    def __waitForCmdReply(self):
        while True:
            result,string=vrep.simxReadStringStream(self.__clientID, 'reply_signal', vrep.simx_opmode_streaming)
            #result,string=vrep.simxGetStringSignal(self.__clientID, 'reply_signal', vrep.simx_opmode_streaming)
            if (result == vrep.simx_return_ok and len(string) > 0):
                logging.debug("received %s" % string)
                return string
            #string = str(string, 'utf-8')
            #if (result==vrep.simx_return_ok):

            time.sleep(0.1)
    #end __waitForCmdReply()

    def sendSignal(self, params):
        """Function that sends a signal to V-REP

        :params: [] list of values to send
        :returns: non-empty string value

        """

        #returnCode = vrep.simxWriteStringStream(self.__clientID, 'signal', 'abc', vrep.simx_opmode_oneshot)
        
        #packedData=vrep.simxPackInts([1, 2, 3])
        packedData=vrep.simxPackInts(params)
        
        returnCode = vrep.simxWriteStringStream(self.__clientID, "signal", packedData, vrep.simx_opmode_oneshot) 
        
        reply = self.__waitForCmdReply()
        logging.debug("Reply = %s", reply)

        return reply
    #end sendSignal

    def getState(self):
        """Return the current state of the kilobot, under the form of a structured dictionary
        :returns: structured dictonary that represents the state of the robot
        {
            distance : [(id1, val1), ...]
            light : (val_now, val_previous)
        }

        """
        recv = vrep.simxUnpackInts(self.sendSignal([SignalType.getState]))
        logging.debug("Received %s" % recv)

        return {'distance' : recv[0], 'light' : recv[1]}
    #end getState()

    def setState(self, motion, light):
        """Set a current state for the end-effectors of the Kilobot (Motors and RGB-led)

        :motion: one of Motion(enum) values  
        :light: [r, g, b] list with values from [0-3] interval
        :returns: TODO

        """
        recv = self.sendSignal([SignalType.setState, motion] + light)
        logging.debug("Received %s" % recv)

    def close(self):
        """Closes the connection with V-REP """

        # Now close the connection with V-REP:
        vrep.simxFinish(self.__clientID)
        logging.info("Connection closed")
#end class VrepBridge 


##########################################################################
#   MAIN
if __name__ == "__main__":
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

    bridge = VrepBridge()

    bridge.getState()
    bridge.setState(Motion.forward, [0, 2, 0])

    bridge.close()
