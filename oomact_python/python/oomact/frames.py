from __future__ import print_function

import numpy as np
import os

from tools import *
import statistics as stat

import re;

def getFromMatrixArchive(ma, name, index, default = None):
    if ma.has_key(name):
        m = ma[name]
        if m.shape[1] == 1:
            warn(name + " has only one column");
        return m[:, index]
    else:
         if default is None:
             #warn("No default available for " + name)
             return np.zeros((1, ))
         return default

def calcEstKey(estName, index):
    return estName + str(index) if index != -1 else estName

class EstKey:
    def __init__(self, name, index = -1):
        self.index = index
        self.name = name
        self.estKey = calcEstKey(name, index)

    def __str__(self):
        return os.path.dirname(self.name) + "[%u]"%self.index

    def __eq__(self, other):
        return isinstance(other, EstKey) and self.index == other.index and self.name == other.name

    def __hash__(self):
        return hash(self.index) + hash(self.name)

class Estimate:
    def __init__(self, estKey, calibObject = None, ma = None):
        assert(isinstance(estKey, EstKey))
        self.calibObject = calibObject
        self.estKey = estKey
        if hasattr(calibObject, 'variableNames'):
            self.variables = dict()
            for v in calibObject.variableNames:
                self.variables[v] = getFromMatrixArchive(ma, v, estKey.index)

class FrameEstimate(Estimate):
    def __init__(self, estKey, frame = None, ma = None):
        Estimate.__init__(self, estKey, frame)
        if frame:
            name = frame.name
            self.translation = getFromMatrixArchive(ma, name + "_T", estKey.index, np.zeros((3, )))
            self.rotation = getFromMatrixArchive(ma, name + "_R", estKey.index, np.zeros((3, )))
            verbose(("Frame %s , %s: " % (name, estKey.index) ) + str(self.rotation))
            self.quat = rollPitchYaw2Quat(self.rotation)
            self.delay = getFromMatrixArchive(ma, name + "_d", estKey.index, np.zeros((1, )))

    def displacementTo(self, otherEstimate):
        assert(self.estKey == otherEstimate.estKey)
        f = FrameEstimate(self.estKey)
        f.translation = otherEstimate.translation - self.translation
        f.quat = quatDivR(otherEstimate.quat, self.quat)
        f.rotation =  quat2RollPitchYaw(f.quat)
        f.delay = otherEstimate.delay - self.delay
        return f

class CalibObject:
    def __init__(self, name, group):
        self.name = name
        self.estimates= dict()
        self.group = group
        self.genericVariables = []

    def getGroup(self):
        return self.group

    def _getNonGenericVariables(self):
        return []

    def register(self, calibVarToObjects, variableNames):
        self.variableNames = variableNames
        for v in variableNames:
            if calibVarToObjects.has_key(v):
                raise Exception("Variables %s already claimed" %v)
            calibVarToObjects[v] = self
            verbose("Registering %s for %s " %(v, self.getName()))

            if not v in self._getNonGenericVariables():
                self.genericVariables.append(v)

    def getName(self):
        return self.name

    def getFullName(self):
        return self.getGroup().getName() + '.' + self.getName()

    def checkNewKey(self, estKey):
        assert(isinstance(estKey, EstKey))
        assert(not self.estimates.has_key(estKey))

    def addMA(self, estKey, ma):
        self.estimates[estKey] = Estimate(estKey, self, ma)

    def getMean(self, v):
        return np.mean([ e.variables[v] for e in self.estimates.values() ])

    def __str__(self):
        t = '';
        for v in self.variableNames:
            t += "%s=%s " %( v, str(self.getMean(v)))
        return "%s(%s(%d))" % (self.name, t, len(self.estimates))

class Frame(CalibObject):
    def __init__(self, name, group, parent=None):
        CalibObject.__init__(self, name, group)
        self.parent=parent

    def addMA(self, estKey, ma):
        self.checkNewKey(estKey)
        self.estimates[estKey] = FrameEstimate(estKey, self, ma)

    def setToDisplacement (self, frameFrom, frameTo):
        assert(not self.estimates)
        for k,v in frameFrom.estimates.iteritems():
            if frameTo.estimates.has_key(k):
                 self.estimates[k] = v.displacementTo(frameTo.estimates[k])
            else :
                warn("Could not find estimate %s of %s in %s" % (k, frameFrom.name, frameTo.name))

    def hasDelay(self):
        return True

    def getDelay(self, estimate):
        return estimate.delay

    def getMeanTranslation(self):
        return stat.TransStat().compute(self)[0]

    def getMeanRotation(self):
        return stat.RotStat().compute(self)[0]

    def getMeanDelay(self):
        return stat.DelayStat().compute(self)[0]


    def __str__(self):
        return "%s(T=%s, R=%s, d=%s (%d))" % (self.name, str(self.getMeanTranslation()), str(self.getMeanRotation()), str(self.getMeanDelay()), len(self.estimates))


class Odom(CalibObject):
    def __init__(self, calibObjects, group):
        CalibObject.__init__(self, "Odometry", group)
        self.register(calibObjects, ('R_l', 'R_r', 'L', 'dTW'))

    def hasDelay(self):
        return True

    def getDelay(self, estimate):
        return estimate.variables['dTW']

    def _getNonGenericVariables(self):
        return ['dTW']


class VelodyneInt(CalibObject):
    def __init__(self, calibObjects, name, group, nameInfix):
        CalibObject.__init__(self, name, group)
        names = []
        for i in range(0, 32):
             names.append("Velodyne_B%s%d" %(nameInfix, i))
        self.register(calibObjects, names)

    def hasDelay(self):
        return False

class VelodyneBeamsPitch(VelodyneInt):
    def __init__(self, calibObjects, group):
        VelodyneInt.__init__(self, calibObjects, "VelodyneBeamsPitch", group, 'P')

class VelodyneBeamsDistance(VelodyneInt):
    def __init__(self, calibObjects, group):
        VelodyneInt.__init__(self, calibObjects, "VelodyneBeamsDistance", group, 'D')

class Rest(CalibObject):
    def __init__(self, calibObjects, group):
        CalibObject.__init__(self, "Rest", group)
        self.register(calibObjects, ('BaseJoint_x', 'BaseJoint_y', 'BaseJoint_l', 'BaseJoint_z', 'BaseJoint_w2', 'gyroBias', 'accBias', 'Gravity_m', 'g_m', 'ControlInput_tK', 'ControlInput_rK'))

    def hasDelay(self):
        return False


fullVariableNameRe=re.compile('^(\w+)_(T|R|d)$')

def splitToFrameAndVariable(fullVariableName):
    matching = fullVariableNameRe.match(fullVariableName)
    if(matching):
        return (matching.group(1), matching.group(2))
    else :
        raise UserWarning("No fullVariableName: " + fullVariableName)


RelativeMarker = ' to '

class CalibrationSet:
    def __init__(self, name, mas = None, index = -1):
        verbose("New calibration set: %s" % name)
        self.calibObjects = dict()
        self.calibVarToObjects = dict()
        self.name = name
        Odom(self.calibVarToObjects, self)
        VelodyneBeamsPitch(self.calibVarToObjects, self)
        VelodyneBeamsDistance(self.calibVarToObjects, self)
        Rest(self.calibVarToObjects, self)

        self.color = 'blue'

        if mas is not None:
            self.loadCalibMas(mas, index)


    def getName(self):
        return self.name

    def getColor(self):
        return self.color

    def getCalibObject(self, calibObjectName, create = False):
        if hasattr(calibObjectName, 'getName'):
            calibObjectName = calibObjectName.getName()

        parts = calibObjectName.split(RelativeMarker, 2)
        if len(parts) > 1 :
            return createRelativeFrame(self.getCalibObject(parts[0]), self.getCalibObject(parts[1]))

        if not self.calibObjects.has_key(calibObjectName):
            if not create:
                raise UserWarning("No calibObject called: " + calibObjectName)
            f = Frame(calibObjectName, self)
            self.calibObjects[calibObjectName] = f
            return f
        else:
            return self.calibObjects[calibObjectName]

    def loadCalibMas(self, mas, index):
        for ama in mas:
            ma = sm.MatrixArchive();
            ma.load(ama if os.path.isfile(ama) else os.path.join(ama, 'calib.ama'))
            ma = dict(ma.asDict())
            for fullVariableName in ma.keys():
                try:
                    if self.calibVarToObjects.has_key(fullVariableName):
                        calibObject = self.calibVarToObjects[fullVariableName]
                        self.calibObjects[calibObject.getName()] = calibObject
                    else:
                        frameName, variableName = splitToFrameAndVariable(fullVariableName)
                        self.getCalibObject(frameName, True)
                except UserWarning as e:
                    warn (e)

            for f in self.calibObjects.values():
                f.addMA(EstKey(ama, index), ma)

def createRelativeFrame(fromFrame, toFrame):
    assert(fromFrame.group == toFrame.group)
    f = Frame(fromFrame.name + RelativeMarker + toFrame.name, fromFrame.group);
    f.setToDisplacement(fromFrame, toFrame)
    return f
