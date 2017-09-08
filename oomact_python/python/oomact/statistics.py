from __future__ import print_function

import os
import math

import sm
import copy

from sm import quatIdentity

from tools import *

import numpy as np
import numpy.linalg
import scipy.stats as stat
import scipy

import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as mpatches

import frames
from docutils.nodes import legend


rad2degFact = 180 / math.pi; 

VelodyneBeamPrefix = "Velodyne_B";
def translateComponentName(s):
    if s.startswith(VelodyneBeamPrefix):
        return s[len(VelodyneBeamPrefix) + 1:]
    else:
        return s;

comps = ['x', 'y', 'z', 'R', 'P', 'Y', 'd']
velBeams = [str(i) for i in (0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31)];
comps.extend(velBeams)

compOrder = { v:k for k, v in enumerate(comps)};

def statComponentCmp(a, b):
    if compOrder.has_key(a) and compOrder.has_key(b):
        return cmp(compOrder[a], compOrder[b])
    else :
        return cmp(a, b)

class Quality:
    def __init__(self, unifyFactorFromSI, unifyUnitName, defaultFactorFromSI, defualtUnitName, componentNames):
        self.unifyFactorFromSI = unifyFactorFromSI
        self.unifyUnitName = unifyUnitName
        self.defaultFactorFromSI = defaultFactorFromSI
        self.defualtUnitName = defualtUnitName
        self.componentNames = componentNames

    def __eq__(self, other): 
        return self.__dict__ == other.__dict__
        
    def getUnifyFactor(self):
        return self.unifyFactorFromSI
    def getDefaultFactor(self):
        return self.defaultFactorFromSI
    
    def getUnifiedUnitName(self):
        return self.unifyUnitName

    def getDefaultUnitName(self):
        return self.defualtUnitName
    
    def getComponentNames(self):
        return self.componentNames

def angularQuatDist(q1, q2):
    return 2 * np.arccos(min(1, np.abs(np.dot(q1, q2))));

def angularQuatDists(q, quats):
    return np.array([ angularQuatDist(q, q2) for q2 in quats ]);

def totalSquaredDist(q, quats):
    return sum(angularQuatDists(q, quats)**2)

def quatMean(q0, quats):
    res = scipy.optimize.minimize(lambda a : totalSquaredDist(expR(q0, a), quats), np.zeros(3))
    return expR(q0, res.x)

def quatVar(qMean, quats):
    return totalSquaredDist(qMean, quats) / len(quats)

def quatDisplacements(qMean, quats):
    return [ logL(qMean, q) for q in quats ];

class Statistic:
    def __init__(self, name = "Generic"):
        self.name = name
        self.shortName = name[:3]
        
    def compute(self, frame):
        if len(frame.estimates):
            return self.computeImpl(frame)
        else:
            return (self.getZero(), self.getZero())

    def isApplicable(self, object):
        return not isinstance(object, frames.Frame)
    
    def getMean(self, frame):
        return np.mean(np.array(self.getData(frame)), 0)

    def computeImpl(self, frame):
        return (self.getMean(frame), np.sqrt(np.var(np.array(self.getData(frame)), 0)))

    def getZero(self):
        return np.zeros((self.getDim(),));
    
    def getDim(self):
        return 3;
    
    def getShortName(self):
        return self.shortName
    
    def getDeviations(self, frame, mean = None):
        if mean is None:
            mean = self.getMean(frame)
        return self.getData(frame) - mean
    
    def getData(self, calibObject):
        return [ np.hstack([ e.variables[v] for v in calibObject.genericVariables]) for e in calibObject.estimates.values() ]
    
    def getQuality(self, calibObject):
        compNames = [ translateComponentName(v) for v in calibObject.genericVariables ]
        print("CalibObject", calibObject)
        if isinstance(calibObject, frames.VelodyneBeamsPitch):
            return Quality(rad2degFact, 'deg', rad2degFact, 'deg', compNames);
        return Quality(100, 'cm', 1000, 'mm', compNames );

    def __str__(self):
        return self.name
    
class RotStat(Statistic):
    def __init__(self):
        Statistic.__init__(self, "Rotation")

    def isApplicable(self, object):
        return isinstance(object, frames.Frame)
    
    def computeImpl(self, frame):
        quats = self.getQuats(frame)
        mean = self.getMean(frame, quats)
        return (mean, np.sqrt(np.var(self.getDeviations(frame, mean, quats), 0)))


    def getDeviations(self, frame, mean = None, quats = None):
        if quats is None:
            quats = self.getQuats(frame)
        if mean is None:
            mean = self.getMean(frame, quats)
        return [ v for v in quatDisplacements(mean, quats)]

    def getQuats(self, frame):
        return [ e.quat for e in frame.estimates.values()]
    def getMean(self, frame, quats = None):
        if quats is None:
            quats = self.getQuats(frame)
        return quatMean(frame.estimates.values()[0].quat, quats)
    
    def getQuality(self, frame):
        return Quality(rad2degFact, 'deg', rad2degFact, 'deg', ['R', 'P', 'Y']);

class TransStat(Statistic):
    def __init__(self):
        Statistic.__init__(self, "Translation")

    def isApplicable(self, object):
        return isinstance(object, frames.Frame)

    def getData(self, frame):
        return [ e.translation for e in frame.estimates.values()]
    
    def getQuality(self, frame):
        return Quality(100, 'cm', 1000, 'mm', ['x', 'y', 'z']);

    
class DelayStat(Statistic):
    def __init__(self):
        Statistic.__init__(self, "Delay")

    def isApplicable(self, object):
        return object.hasDelay()

    def getData(self, frame):
        return [ frame.getDelay(e) for e in frame.estimates.values()]

    def getDim(self):
        return 1;

    def getQuality(self, frame):
        return Quality(100, 'cs', 1000, 'ms', ['d']);

allStats  = (Statistic(), TransStat(), RotStat(), DelayStat())

class StatPlotter:
    def __init__(self):
        self.fig = plt.figure(figsize=(8, 2))
        self.title = ""
        self.legend = None
        self.showfliers = True

    def _finish(self):
        pass

    def __finishFigure(self):
        self._finish()
        self.fig.tight_layout(pad = 0, w_pad = 0, h_pad = 0)
#         self.fig.suptitle(self.title)

    def show(self):
        self.__finishFigure()
        self.fig.show()

    def save(self, fileName, fileFormat = 'pdf'):
        fullFileName = fileName + '.' + fileFormat
        if os.path.exists(fullFileName):
            warn("Output file %s exists already! Not overwriting!" % fullFileName)
            return

        ok("Writing plot "+self.title + " to " + fullFileName)
        self.__finishFigure()
        extra = ()
        if self.legend:
            extra = (self.legend,)
            
        self.fig.savefig(fullFileName, bbox_inches='tight', format=fileFormat)

        with open(fileName + '.tex', 'w') as file:
            print (self.title.strip().replace('_', '\_').replace('->', ' to '), file=file)

def shuffled(list, indices):
    try:
        return [ list[i] for i in indices ];
    except IndexError as e:
        print (e)

class BoxPlots:
    def __init__(self, data, labels, quality, groups, name):
        self.data = list(data)
        self.labels = list(labels)
        self.groups = list(groups)
        self.quality = quality
        self.name = name

    def canBeMergedWith(self, bp):
        return self.quality == bp.quality

    def merge(self, other):
        assert(self.quality == other.quality)
        self.data.extend(other.data)
        self.labels.extend(other.labels)
        self.groups.extend(other.groups)


    def getScaledData(self, useDefaultUnits):
        d =  np.array(self.data);
        if useDefaultUnits: 
            d *= self.quality.getDefaultFactor()
        else:
            d *= self.quality.getUnifyFactor();
        return list(d);

class BoxPlotGroup:
    def __init__(self):
        self.boxPlots = []

    def append(self, stat, calibObject, calibObjectBaseLine, name):
        if stat.isApplicable(calibObject):
            d = stat.getDeviations(calibObject, stat.getMean(calibObjectBaseLine))
            qual = stat.getQuality(calibObject);
            numEntries = len(qual.getComponentNames())
            groups = [ calibObject.getGroup() for i in range(numEntries) ]

            d =  list(np.array(d).transpose());
            if numEntries != len(d):
                print (d)
            assert(numEntries == len(d))
            
            if len(d):
                bp = BoxPlots(d, qual.getComponentNames(), qual, groups, name + '_' + stat.getShortName())
                if np.max(np.var(bp.getScaledData(False), 1)) < 1e-9:
                    warn("Skipping low variance box plot : %s" % bp.name)
                else:
                    for i in self.boxPlots:
                        if i.canBeMergedWith(bp):
                            verbose("Merging" + str(i.labels) + ' with ' + str(bp.labels) )
                            i.merge(bp)
                            return
    
                    self.boxPlots.append(bp)

class MultiStatPlotter(StatPlotter):
    def __init__(self, useDefaultUnits = True, setTitle = True):
        StatPlotter.__init__(self)
        self.startBox = 1
        self.useDefaultUnits = useDefaultUnits
        self.bpg = BoxPlotGroup()
        self.lastTitle = ""
        self.setTitle = setTitle
        self.groupSet = []

    def addGroup(self, g):
        if not g in self.groupSet:
            self.groupSet.append(g)

    def plot(self, calibObject, calibObjectBaseLine, stats, options):
        self.showfliers = not options.disableFliers
        if calibObjectBaseLine is None:
            calibObjectBaseLine = calibObject
        
        name = calibObject.getFullName()
        self.title += " " + name

        for s in stats:
            self.bpg.append(s, calibObject, calibObjectBaseLine, name)

    def _finish(self):
        data = []
        labels = []
        self.plotN = 0
        self.data = []
        
        self.bpg.boxPlots.sort(cmp = statComponentCmp, key = lambda bp : bp.labels[0])
        
        widthRatios = []
        for bp in self.bpg.boxPlots:
            widthRatios.append(len(bp.data))

        self.gs = matplotlib.gridspec.GridSpec(1, len(self.bpg.boxPlots),
                       width_ratios=widthRatios)

        for bp in self.bpg.boxPlots:
            self.__nextAx()
            self.data = bp.getScaledData(self.useDefaultUnits)
            self.__finishAx(bp, bp.quality.getDefaultUnitName() if self.useDefaultUnits else bp.quality.getUnifyUnitName())


        if len(self.groupSet) > 1:
            patches = []
            labels = []
            for g in self.groupSet:
                patches.append(mpatches.Patch(color=g.getColor(), label=g.getName()))
                labels.append(g.getName())
            self.legend = self.fig.legend(patches, labels, 'upper center', ncol = len(labels), bbox_to_anchor=(0.5, 1.27))

    def __finishAx(self, bp, ylabel):
        groups = bp.groups
        data = self.data
        labels = bp.labels

        if True:
            I = sorted(range(len(labels)), cmp = statComponentCmp, key=lambda k: labels[k])
            groups = shuffled(groups, I)
            labels = shuffled(labels, I)
            data = shuffled(data, I)

        bps = self.ax.boxplot(data, patch_artist = False, labels = labels, whis = True, showfliers = self.showfliers)
        for i in range(len(groups)):
            self.addGroup(groups[i])
            plt.setp(bps['boxes'][i], color = groups[i].color)
        
        for i, l in enumerate(self.ax.get_xticklabels()):
            l.set_color(groups[i].getColor())
        for i, l in enumerate(self.ax.get_xticklines()):
            l.set_visible(False)

        self.ax.set_ylabel(ylabel)
        if self.lastTitle != self.title:
            self.lastTitle = self.title
            if self.setTitle:
                self.ax.set_title(self.title)

    def __nextAx(self):
        assert(self.plotN < len(self.bpg.boxPlots))
        self.ax = plt.subplot(self.gs[self.plotN])
        self.plotN += 1;
        self.data = []
        self.labels = []

class DebugStatPlotter(StatPlotter):
    def __init__(self):
        StatPlotter.__init__(self)
        self.ax = plt.subplot(111)
        self.startBox = 1
        self.data = []
        self.labels = []
        self.title = ""

        def onpick(event):
            thisline = event.artist
            if thisline.get_alpha() < 1:
                thisline.set_alpha(1)
            else :
                thisline.set_alpha(0.1)
            self.fig.canvas.draw()
            xdata = thisline.get_xdata()
            ydata = thisline.get_ydata()
            ind = event.ind
            print ('picked:', thisline.get_label(), zip(xdata[ind], ydata[ind]))

        self.fig.canvas.mpl_connect('pick_event', onpick)


    def plot(self, calibObject, calibObjectBaseLine, stats, options):
        if calibObjectBaseLine is None:
            calibObjectBaseLine = calibObject
            
        self.showfliers = not options.disableFliers
        data = []
        labels = []
        for s in stats:
            if s.isApplicable(calibObject):
                d = s.getDeviations(calibObject, s.getMean(calibObjectBaseLine))
                qual = s.getQuality(calibObject);
                d = np.array(d).transpose() * qual.getUnifyFactor();

                assert(len(qual.getComponentNames()) == d.shape[0])

                showUnits = len(qual.getComponentNames()) <= 10;
                for c in qual.getComponentNames():
                    labels.append('%s (%s)' % (c, qual.getUnifiedUnitName()) if showUnits else c)
                data.extend(d)

        self.data.extend(data)
        self.labels.extend(labels)
        self.title += " " + calibObject.getGroup().getName() + '.' + calibObject.getName()

        boxPositions = np.array(range(len(data))) + self.startBox
        self.startBox += len(data)
        
        dataLabels = calibObject.estimates.keys()
        for i in range(len(dataLabels)):
            d =  (np.array(data)[:, i])
            lines = self.ax.plot(boxPositions, d, picker = 5, label = dataLabels[i])
            for l in lines:
                l.set_alpha(0.1)

    def _finish(self):
        self.ax.boxplot(self.data, patch_artist = False, labels = self.labels, whis = True, showmeans = False, showfliers = self.showfliers)
        self.ax.set_title(self.title)

def createStatistics(calibObject, calibObjectBaseLine, options, stats = allStats, plotter = None):
    print('%s (%d):' % (calibObject.name, len(calibObject.estimates)))
    if calibObjectBaseLine is None:
        calibObjectBaseLine = calibObject

    for s in stats:
        if s.isApplicable(calibObject):
            mean, sd = s.compute(calibObject)

            if(isinstance(s, RotStat)):
                meanDeviation = log(quatDivL(mean, s.getMean(calibObjectBaseLine)))
                mean = log(mean)
            else:
                meanDeviation = mean - s.getMean(calibObjectBaseLine)
            
            q = s.getQuality(calibObject)
            print("\t" + s.name + '(' + q.getUnifiedUnitName() + '):')
            print ('\tmean\t='+ str(mean * q.getUnifyFactor()) + "\n\tdm\t=" + str(meanDeviation * q.getUnifyFactor()) + "\n\tsd\t=" + str(sd * q.getUnifyFactor()))

    print()

    if options.plot or options.outputDir:
        if plotter is None:
            plotter = MultiStatPlotter(setTitle = options.setTitle) if not options.debug else DebugStatPlotter()
        plotter.plot(calibObject, calibObjectBaseLine, stats, options)
        return plotter;

    return None
