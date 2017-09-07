import sm
from sm import quatInv, quatIdentity
import tf.transformations

try:
    from termcolor import colored
except ImportError:
    print("Unable to import termcolor.")
    print("Try:")
    print("sudo pip install termcolor")
    def colored(X,Y):
        return X

def quat2YawPitchRoll(quat):
    return sm.EulerAnglesYawPitchRoll().rotationMatrixToParameters(sm.quat2r(quat))
def yawPitchRoll2Quat(ypr):
    return sm.r2quat( sm.EulerAnglesYawPitchRoll().parametersToRotationMatrix(ypr))

def quat2RollPitchYaw(quat):
    return tf.transformations.euler_from_quaternion(sm.quatInv(quat), 'rxyz')

def rollPitchYaw2Quat(rpy):
    return sm.quatInv(tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rxyz')) 


def quatMult(q1, q2):
    return sm.quatOPlus(q1).dot(q2);

def quatDivL(q1, q2):
    return quatMult(quatInv(q2), q1)

def quatDivR(q1, q2):
    return quatMult(q1, quatInv(q2))

def exp(a):
    return sm.axisAngle2quat(a);

def log(q):
    if(quatIdentity().dot(q) < 0) : q *=-1
    return sm.quat2AxisAngle(q);

def expL(q, a):
    return quatMult(q, exp(a));

def logL(q, q2):
    return log(quatDivL(q2, q));

def expR(q, a):
    return quatMult(exp(a), q);

def logR(q, q2):
    return log(quatDivR(q2, q));

verbose = False

def verbose(text):
    if verbose:
        print(colored(text, 'yellow'))

def warn(text):
    print(colored(str(text), 'red'))


def error(text):
    print(colored("Error :" + str(text), 'red'))

def ok(text):
    print(colored(str(text), 'green'))
