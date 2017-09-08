import os
import tools


delayContent = "<delay>%g</delay>"

def getSensorConfigString(name, frame):
    pos = frame.getMeanTranslation()
    quat = frame.getMeanRotation()
    rpy = tools.quat2RollPitchYaw(quat)
    return """
    <{name}>
      <translation>
        <x>{x}</x>
        <y>{y}</y>
        <z>{z}</z>
      </translation>
      <rotation>
        <yaw>{yaw}</yaw>
        <pitch>{pitch}</pitch>
        <roll>{roll}</roll>
      </rotation>
      {delay}
    </{name}>""".format(name=name, x=pos[0], y=pos[1], z=pos[2], yaw = rpy[2], pitch=rpy[1], roll=rpy[0], delay=(delayContent % frame.getMeanDelay()) if frame.hasDelay() else "")


meanConfigXmlContent = """<model>
  <sensors>%s
  </sensors>
</model>
"""

def getMeanConfig(content):
    return meanConfigXmlContent % content
