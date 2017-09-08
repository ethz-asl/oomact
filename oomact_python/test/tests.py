#!/usr/bin/env python
import unittest
import sys
import os
import subprocess
import numpy as np

from numpy.testing import *
from oomact.statistics import *
from oomact.tools import *


i = np.array([1, 0, 0, 0])
j = np.array([0, 1, 0, 0])
k = np.array([0, 0, 1, 0])
one = np.array([0, 0, 0, 1])

class Tests(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self, *args, **kwargs)

    def testUnits(self):
        for v in (i, j, k):
            assert_array_equal(quatMult(v, v), -one)
            assert_array_equal(quatMult(v, one), v)
            assert_array_equal(quatMult(one, v), v)

    def testMultHamilton(self):
        assert_array_equal(quatMult(i, j), k)
        assert_array_equal(quatMult(j, k), i)
        assert_array_equal(quatMult(k, i), j)

    def testInv(self):
        assert_array_equal(quatInv(i), -i)
        assert_array_equal(quatMult(quatInv(i), i), one)

    def testDiv(self):
        for v in (i, j, k, one):
            for v2 in (i, j, k, one):
                assert_array_equal(quatMult(v2, quatDivL(v, v2)), v)
                assert_array_equal(quatMult(quatDivR(v, v2), v2), v)

    def testExpLog(self):
        for v in (i, j, k, one):
            assert_array_almost_equal(exp(log(v)), v, 15)
            assert_array_almost_equal(log(exp(v[0:3])), v[0:3], 15)
            for v2 in (i, j, k, one):
                assert_array_almost_equal(expR(v, logR(v, v2)), v2, 15)
                assert_array_almost_equal(logR(v, expR(v, v2[0:3])), v2[0:3], 15)
                assert_array_almost_equal(expL(v, logL(v, v2)), v2, 15)
                assert_array_almost_equal(logL(v, expL(v, v2[0:3])), v2[0:3], 15)

    def testQuatStat(self):
        n = 500;
        sigma = 0.1
        randVector = np.random.multivariate_normal(np.zeros(3), np.identity(3) * sigma * sigma, n)
        
        qStart = sm.quatRandom()
        quats = [ expR(qStart, randVector[i, :]) for i in range(n) ]
        
        self.assertAlmostEqual(0, np.linalg.norm(angularQuatDists(qStart, quats) - np.linalg.norm(randVector, 2, 1), 2), 5)
        
        q0 = qStart
        
        qMean = quatMean(q0, quats)
        assert_array_almost_equal(qMean, qStart, 2)
        
        qVar = quatVar(qMean, quats)
        assert_array_almost_equal(qVar, np.mean(np.linalg.norm(randVector, 2, 1)**2), 2)
        assert_array_almost_equal(qVar, 3 * sigma * sigma, 2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('oomact_python', 'tests', Tests)
