from vrep_bridge import vrep # for vrep functions
import logging
import colorlog # colors log output
from enum import IntEnum # for enumerations (with int value) (enum from C)
#import ctypes
import time # for time.sleep()
import numpy # for matrix multiplication (rotation matrix)
from math import sin, cos # for rotation matrix

class SignalType(IntEnum):

    """Enumeration of signal types"""

    getState         = 1
    setState         = 2
    getKnownRobotIds = 3
#end class SignalType

class IndexSignalSend(IntEnum):

    """Enumeration of outgoing signal array indexes"""

    type   = 0
    uid    = 1
    motion = 2
    led_r  = 3
    led_g  = 4
    led_b  = 5
# end class IndexSignalSend

class IndexSignalReceive(IntEnum):

    """Enumeration of incoming signal array indexes"""

    uid             = 0
    ambient_light   = 1
# end class IndexSignalSend

class Motion(IntEnum):

    """Enumeration of motion types accepted by Kilobot"""
    
    stop    = 0
    forward = 1
    left    = 2
    right   = 3
#end class Motion

class Led_rgb():

    """Enumeration of colors (returned as [r, g, b]) accepted by Kilobot"""

    red       = [3, 0, 0]
    green     = [0, 3, 0]
    blue      = [0, 0, 3]
    white     = [3, 3, 3]
    turquoise = [0, 3, 1]
    orange    = [3, 3, 0]
    magenta   = [3, 0, 3]
    cyan      = [0, 3, 3]
    yellow    = [3, 3, 0]
# end class Led_rgb

class SpawnType(IntEnum):

    """Enumeration of spawn dispersion types"""

    ox_plus      = 1
    oy_plus      = 2
    circular     = 3
# end class SpawnType

# dictionary of SpawnTypes (used for lulu_kilobot config parsing)
SpawnTypeNames = {
    "ox_plus": SpawnType.ox_plus,
    "oy_plus": SpawnType.oy_plus,
    "circular": SpawnType.circular
}

def getClonePosRot(stepNr, nr, spawnType = SpawnType.ox_plus):
    """Return a position[3], rotation[3] pair for the selected spawn type and current step 

    :stepNr: the current (in the interval [0; nr-1], where nr = nr robots) iteration step in the cloning process of the source robot
    :nr: total number of robots (including the original robot)
    :returns: position[3] (is applied relative to the source robot's position)
    :returns: rotation[3] (in Euler angles, applied relative to the clone's axis)

    """
    position = [0] * 3
    rotation = [0] * 3

    if (spawnType == SpawnType.ox_plus):
        position = [(stepNr + 1) * 0.05, 0, 0]
        rotation = [0, 0, 0]
    elif (spawnType == SpawnType.oy_plus):
        position = [0, (stepNr + 1) * 0.05, 0]
        rotation = [0, 0, 0]
    elif (spawnType == SpawnType.circular):
        angle = numpy.radians((stepNr / (nr)) * 360)
        rot_matrix = numpy.array([  [cos(angle), -sin(angle)], [sin(angle), cos(angle)] ])
        pos = numpy.dot(numpy.array([0, 0.061 * (nr / 10)]), rot_matrix)
        position = list(pos) + [0]
        rotation = [0, 0, 0]

    return position, rotation
# end getClonePosRot()

class VrepBridge():

    """Creates a connection between a Python application and V-REP that allows bidirectional communication"""
    # signal = type_request param1 param2 param3
    # reply_signal = type val1 val2 val3

    def __init__(self):
        """"""
        self.clonedRobotHandles = [] # used to store object handles of copy-pasted robots
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
                #logging.debug("received %s" % string)
                return string
            #string = str(string, 'utf-8')
            #if (result==vrep.simx_return_ok):

            #time.sleep(0.1)
    #end __waitForCmdReply()

    def sendSignal(self, params):
        """Function that sends a signal to V-REP

        :params: [] list of values to send
        :returns: non-empty string value

        """

        #returnCode = vrep.simxWriteStringStream(self.__clientID, 'signal', 'abc', vrep.simx_opmode_oneshot)

        #packedData=vrep.simxPackInts([1, 2, 3])
        packedData=vrep.simxPackInts(params)

        vrep.simxWriteStringStream(self.__clientID, "signal", packedData, vrep.simx_opmode_oneshot) 
        logging.debug("Sent %s" % params)
        reply = self.__waitForCmdReply()
        #logging.debug("Reply = %s", reply)

        return reply
    #end sendSignal

    def getState(self, uid):
        """Return the current state of the kilobot, under the form of a structured dictionary
        :uid: the target kilobot's unique id
        :returns: structured dictionary that represents the state of the robot
        {
            uid : the target kilobot's unique id
            light : (val_now, val_previous)
            distances : {robot_uid: current_distance}
        }

        """
        send = [-1] * 2 # initialize an array with -1
        send[IndexSignalSend.type] = SignalType.getState
        send[IndexSignalSend.uid] = uid

        logging.debug("getState() robot_uid = %d" % uid)

        # get a reply of the form [uid, ambient_light] | distance_keys | distance_values
        recv = self.sendSignal(send)
        #logging.debug("Received %s" % recv)
        recv = recv.split(b'|')
        for i in range(len(recv)):
            # unpack ints in place
            recv[i] = vrep.simxUnpackInts(recv[i])
            logging.debug("recv[%d] = %s" % (i, recv[i]))

        if (recv[0][IndexSignalReceive.uid] != uid):
            logging.critical("received the state from the wrong robot (req.uid = %d, response.uid = %d)" % (uid, recv[0][IndexSignalReceive.uid]))
            exit(1)

        # construct the distances dictionary (robot_uid: current_distance)
        distances = {recv[1][i]: recv[2][i] for i in range(len(recv[1]))}
        # remove distance from myself, as it is always 0 and is not needed
        del distances[uid]

        return {
                'uid' : recv[0][IndexSignalReceive.uid],
                'light' : recv[0][IndexSignalReceive.ambient_light],
                'distances' : distances}
        #end getState()

    def getKnownRobotIds(self, uid):
        """Return a list of IDs known (safe) to the target robot
        :uid: the target kilobot's unique id
        :returns: list of known robot IDs"""

        send = [-1] * 2 # initialize an array with -1
        send[IndexSignalSend.type] = SignalType.getKnownRobotIds
        send[IndexSignalSend.uid] = uid

        logging.debug("getKnownRobotIds() robot_uid = %d" % uid)

        # get a reply of the form "uid | distance_keys | distance_values"
        recv = self.sendSignal(send)
        #logging.debug("Received %s" % recv)
        recv = recv.split(b'|')
        recv[IndexSignalReceive.uid] = int(recv[IndexSignalReceive.uid]) # convert the sender's id from string to int
        # unpack id ints in place
        recv[1] = vrep.simxUnpackInts(recv[1])
        logging.debug("recv[%d] = %s" % (1, recv[1]))

        if (recv[IndexSignalReceive.uid] != uid):
            logging.critical("received the state from the wrong robot (req.uid = %d, response.uid = %d)" % (uid, recv[IndexSignalReceive.uid]))
            exit(1)

        # remove the requested robot's uid from the list
        recv[1].remove(uid)
        # return the known ids list
        return recv[1]

    #end getKnownRobotIds()

    def setState(self, uid, motion, light):
        """Set a current state for the end-effectors of the Kilobot (Motors and RGB-led)

        :uid: the target kilobot's unique id
        :motion: one of Motion(enum) values  
        :light: [r, g, b] list with values from [0-3] interval
        :returns: TODO

        """
        send = [-1] * 6
        send[IndexSignalSend.type] = SignalType.setState
        send[IndexSignalSend.uid] = uid
        send[IndexSignalSend.motion] = motion
        send[IndexSignalSend.led_r] = light[0]
        send[IndexSignalSend.led_g] = light[1]
        send[IndexSignalSend.led_b] = light[2]

        #recv = self.sendSignal(send)
        recv = vrep.simxUnpackInts(self.sendSignal(send))
        logging.debug("Received %s" % recv)

    def spawnRobots(self, sourceRobotName = "Kilobot#", nr = 2, spawnType = SpawnType.ox_plus):
        """Spawns nr robots in the current scene by copy-pasting the source robot the required number of times
        The cloned robots are placed acording to the clonePosRotFunction (see getClonePosRot_ox_plus() for an example)

        :nr: the number of copies requested
        :sourceRobotName: the complete (with # at the end) robot name
        :clonePosRotFunction: position[3], rotation[3] function(i) meaning a function that returns a position and rotation vector pair for an i - integer nr
                                see getClonePosRot_ox_plus() for an example
        """
        returnCode, sourceHandle = vrep.simxGetObjectHandle(self.__clientID, sourceRobotName, vrep.simx_opmode_oneshot_wait)
        logging.debug("Source obj handle = %d" % sourceHandle)

        logging.info("Spawning %d clones of %s source robot" % (nr, sourceRobotName))
        for i in range(nr):
            logging.debug("Spawning robot nr %d" % i)

            returnCode, auxhandles = vrep.simxCopyPasteObjects(self.__clientID, [sourceHandle], vrep.simx_opmode_oneshot_wait)
            self.clonedRobotHandles.append(auxhandles[0])
            logging.debug("copy obj handle = %s" % self.clonedRobotHandles[-1])

            position, rotation = getClonePosRot(i, nr, spawnType)
            # move the cloned robot by 'position' units away from the source robot
            vrep.simxSetObjectPosition(self.__clientID, self.clonedRobotHandles[-1], sourceHandle, position, vrep.simx_opmode_oneshot_wait)
            # rotate the cloned robot around it's center by 'rotation' euler angles
            vrep.simxSetObjectOrientation(self.__clientID, self.clonedRobotHandles[-1], self.clonedRobotHandles[-1], rotation, vrep.simx_opmode_oneshot_wait)
    # end spawnRobots()

    def removeRobots(self):
        """Removes the robots previously created with spawnRobots() from the current V-REP scene"""

        if (len(self.clonedRobotHandles) <= 0):
            return;

        logging.info("Removing %d robots from the scene" % len(self.clonedRobotHandles))
        for handle in self.clonedRobotHandles: 
            vrep.simxRemoveModel(self.__clientID, handle, vrep.simx_opmode_oneshot_wait)
    # end removeRobots()

    def close(self):
        """Closes the connection with V-REP """

        # Now close the connection with V-REP:
        vrep.simxFinish(self.__clientID)
        logging.info("Connection closed")
    #end close()
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

    bridge.spawnRobots(nr = 6, spawnType = SpawnType.circular)

    input("Press Enter to continue at least 2 seconds after starting the simulation in V-REP and pressing Run in KilobotController")
    print(bridge.getKnownRobotIds(0))

    bridge.getState(0)
    bridge.setState(0, Motion.forward, [0, 2, 0])

    bridge.getState(1)
    bridge.setState(1, Motion.left, [2, 0, 0])

    bridge.getState(2)
    bridge.setState(2, Motion.right, [0, 0, 2])

    time.sleep(5)
    bridge.removeRobots()

    bridge.close()
