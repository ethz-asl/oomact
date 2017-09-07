#!/usr/bin/env python

from __future__ import print_function

import matplotlib

#sudo pip install -U matplotlib
#sudo apt-get install tk-dev libgtk2.0-dev libqt4-dev

matplotlib.use('Qt4Agg')

import matplotlib.pyplot as plt

import sys;

import sm;
import oomact.config_writer as config_writer
import oomact.tools as tools
from oomact.tools import ok, warn, verbose


from oomact.frames import *
from oomact.statistics import createStatistics, allStats, StatPlotter, DelayStat


statShortName2Stat = { s.getShortName() : s for s in allStats }

from argparse import ArgumentParser, RawDescriptionHelpFormatter
usage=""""""

parser = ArgumentParser(description=usage,formatter_class=RawDescriptionHelpFormatter)
parser.add_argument('-v', dest='verbose', action='count', help='Turn on verbosity')
parser.add_argument('-c', dest='calibObjects', action='append', help='calibration objects')
parser.add_argument('-f', dest='fromFrames', action='append', help='from frames')
parser.add_argument('-t', dest='toFrames', action='append', help='to frames')
parser.add_argument('-p', dest='plot', action='store_true', help='show plots')
parser.add_argument('-T,--setTitle', dest='setTitle', action='store_false', help='don\'t set axis titles')
parser.add_argument('-d', dest='debug', action='store_true', help='debug experiments')
parser.add_argument('-I', dest='compareWithInitial', action='store_false', help='compare with initial calibration')
parser.add_argument('-F', dest='disableFliers', action='store_true', help='disable boxplot fliers')
parser.add_argument('-s', dest='statistics', action='append', help='statistics to evaluate ' + str(statShortName2Stat.keys()))
parser.add_argument('-o', dest='outputFile', help='output file')
parser.add_argument('-C', dest='compareCalibs', nargs='+', help='compare to amas')
parser.add_argument('--compareTimeCorrection', dest='compareTimeCorrection', type=float, help='compare time offset to compensate for different clock bases')
parser.add_argument('-D', dest='delayCalibs', nargs='+', help='delay amas')
parser.add_argument('-i', dest='compareIndices', nargs='+', help='compare indices')
parser.add_argument('-b', dest='blockImmediately', action='store_true', help='block after each figure')
parser.add_argument('-W', dest='outputDir', help='plots output folder')
parser.add_argument('--prefix', default="", dest='outputPrefix', help='plot output prefix')
parser.add_argument("calibs", nargs='+', help="The matrix archives containing the calibration or a standard calibration output folder.")

options = parser.parse_args()

tools.verbose = options.verbose

id2Label = {'OnlineWG': 'Online'}

def getLabel(id):
    if id2Label.has_key(id):
        return id2Label[id]
    return id



def toString(list):
    return str([str(l) for l in list])

if options.statistics :
    stats = []
    for s in options.statistics:
        stats.append(statShortName2Stat[s])
else:
    stats = allStats
verbose("Using these statistics:" + toString(stats))


calibs = options.calibs
calibName = None
if calibs and not (calibs[0].endswith('.ama') or calibs[0].endswith('.ma')):
    calibName = calibs.pop(0)

verbose("Reading from these matrix archives:" + toString(calibs))

initialCalib = CalibrationSet((calibName + '_' if calibName else '' ) + 'initial' , calibs, 0)
finalCalib = CalibrationSet(calibName if calibName else 'final', calibs, -1)

colors = ['magenta', 'cyan', 'brown']

colorMap = {'Static' : 'blue', 'Dynamic' : 'green', 'Odom' : 'green'}

def computeColor(name):
    if colorMap.has_key(name):
        return colorMap[name]

    return colors.pop(0)


delayCalib = finalCalib
if options.delayCalibs :
    name = options.delayCalibs.pop(0)
    calibs = options.delayCalibs
    verbose(("Reading as delay calibrations (%s) these matrix archives: " % name) + toString(calibs))
    delayCalib = CalibrationSet(name, calibs)
    delayCalib.color = computeColor(name)
    print ('Colors', colors)

compareToCalib = []
if options.compareCalibs :
    calibs = options.compareCalibs
    verbose("Reading compare to calibration these matrix archives:" + toString(calibs))
else:
    calibs = options.calibs

indices = options.compareIndices if options.compareIndices else (-1, )
if options.compareIndices or options.compareCalibs:
    for k, i in enumerate( indices ):
        cs = CalibrationSet('compare' + (' ' + str(i) if i != -1 else ''), calibs, int(i))
        cs.color = colors[len(indices) - k - 1]
        compareToCalib.append(cs)

if options.outputPrefix:
    op=options.outputPrefix.replace('Zoom', '')
    if options.compareCalibs:
        left, right = op.split('C2')
        verbose('Found %s as left compare group name and %s as right compare group name' % ( left, right) )

        if not calibName :
            finalCalib.name = getLabel(right)
        for i, cs in enumerate(compareToCalib):
            cs.name = cs.name.replace('compare', getLabel(left))
    else:
        finalCalib.name = op


for cs in compareToCalib:
    cs.color = computeColor(cs.getName());

finalCalib.color = computeColor(finalCalib.getName());


verbose("Found these initial calibration objects:")
for f in initialCalib.calibObjects.values():
    verbose(f)
verbose("Found these final calibration objects:")
for f in finalCalib.calibObjects.values():
    verbose(f)


def save(plotter, co):
    if not plotter: return
    if hasattr(co, 'getName'):
            co = co.getName()
    if options.outputDir:
        plotter.save(os.path.join(options.outputDir, options.outputPrefix + co))
    else :
        plotter.show()
        if options.blockImmediately:
            plt.show(block = True)

class CorrectingDelStat (DelayStat):
    def __init__(self, correction):
        DelayStat.__init__(self)
        self.correction = correction

    def getData(self, co):
        org = DelayStat.getData(self, co)
        if co.getGroup() in compareToCalib:
            ret = list(np.array(org) + self.correction)
            verbose("Correcting delays from %s to %s for %s" % (str(org), str(ret), co.getFullName()))
        else:
            ret = org
        return ret

compareStats = stats

if options.compareTimeCorrection :
    compareStats = []
    for s in stats:
        if isinstance(s, DelayStat):
            s = CorrectingDelStat(options.compareTimeCorrection)
            warn("Going to correct delay statistics for the comparison objects by %g seconds!" % options.compareTimeCorrection)
        compareStats.append(s)

def doStat(co):
    baseLineCo = initialCalib.getCalibObject(co) if options.compareWithInitial else co
    myStats = stats
    if options.delayCalibs:
        myStats = [ s for s in stats if not isinstance(s, DelayStat) ]

    p = createStatistics(co, baseLineCo, options, myStats)
    if p :
        if options.delayCalibs:
            p.plot(delayCalib.getCalibObject(co), baseLineCo, [DelayStat()], options)

        for cs in compareToCalib:
            p.plot(cs.getCalibObject(co), baseLineCo, compareStats, options)

        save(p, co.getName().replace(RelativeMarker, '2'))


def getFramesByNamesOrAllFrames(calibObjectNames):
    if calibObjectNames is None : return []
    return [ finalCalib.getCalibObject(frameName) for frameName in calibObjectNames ] if calibObjectNames != 'all' else finalCalib.calibObjects.values()

selectedCalibObjects = getFramesByNamesOrAllFrames(options.calibObjects) if options.calibObjects else []

for co in selectedCalibObjects:
    doStat(co)

fromFrames = getFramesByNamesOrAllFrames(options.fromFrames)
toFrames = getFramesByNamesOrAllFrames(options.toFrames)
for fromFrame in fromFrames:
    if isinstance(fromFrame, Frame):
        for toFrame in toFrames:
            if isinstance(toFrame, Frame) and toFrame.name != fromFrame.name:
                doStat(createRelativeFrame(fromFrame, toFrame))

if options.outputFile:
    sensorString=""

    for co in finalCalib.calibObjects.values() :
        if hasattr(co, 'getMeanTranslation'):
            sensorString+= config_writer.getSensorConfigString(co.getName(), co)

    meanConfig = config_writer.getMeanConfig(sensorString)
    print (meanConfig)

    with open(options.outputFile, 'w') as file:
        print (meanConfig, file=file)

if options.plot:
    plt.show(block = True)
