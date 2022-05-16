'''-------------------------------------------
            Inverse Kinematics Test
----------------------------------------------
Simple inverse kinematics test.

Created by ultimatech
First update: 09/05/2022 11:39
Last update: 15/05/2022 16:18
Version: Beta 1.0

                    2022
-------------------------------------------'''

# ------------ Import modules --------------

# Standard imports
from math import *
from pickle import TRUE
from time import time, sleep

from win32api import EnumDisplayDevices, EnumDisplaySettings, GetKeyState

# Non-standard imports (pip install might be needed)
from graphics import *

from pyautogui import position as mousePos
import keyboard




# ----------- Initialize variables -----------

class window:
    width = 1080
    height = 1080
    vertical_resolution = 256
    horizontal_resolution = 256
    ratio = height / width


class FPS:
    value = int()
    maxValue = 'auto'     # Leave 'none' for benchmarking
    timer = 0
    counter = object

    if maxValue == 0 or maxValue == 'none':
        maxValue = 999
    elif maxValue == 'auto':
        maxValue = getattr(EnumDisplaySettings(
            EnumDisplayDevices().DeviceName, -1), 'DisplayFrequency')

    lastValue = maxValue


# Clock for FPS display
start_clock_time = time.time()
last_clock_time = 0


# Used to converts rgb values into hex
def rgb(red, green, blue):
    hexValue = '#%02x%02x%02x' % (red, green, blue)
    return hexValue


# Joint chain and render settings
class setting:

    regularChain = True 
    chainLength = 4 # Number of joints	
    jointLength = 300 # Length of each joint

    showVectors = False
    showLastVector = True
    colorFade = True # Fade color between each joint
    color = rgb(63,93,178) # Accepts color name, hex value or rgb value using: rgb(red, green, blue); Used as default if fade is off
    sizeFade = True
    maxWidth = 10 # Max width of fade; used as default if fade is off

    instantUpdate = False # Update joints instantly 

    showTrail = False # Only recommended for small amount of joints
    trailLength = chainLength*15 # Number of points in trail
    trail = []


    origin = (-300,0) # Origin of chain
    showOrigin = True 
    showTarget = True
    showFPS = True

# Sets the origin point of the chain
originPX, originPY = setting.origin[0], setting.origin[1]

# Sets the joint chain; format: [[length1,x1,y1], [length2,x2,y2], ...]
if setting.regularChain:
    joints = [[setting.jointLength, [originPX, originPY+setting.jointLength*chainJoint]]for chainJoint in range(setting.chainLength+1)]
else:
    joints = [[200, [0, 200]], [200, [0, 400]], [200, [0, 600]], [200, [0, 800]], [200, [0, 1000]], [200, [0, 1200]], [200, [0, 1400]], [200, [0, 1600]], [200, [0, 1800]], [200, [0, 2000]], [200, [0, 2200]], [200, [0, 2400]]]


# Setups render window
render = GraphWin('render', window.width, window.height, autoflush=False)
render.setCoords(-window.width, -window.height, window.width, window.height)


# Sets default target
targetX, targetY = 1000, 0
gotoX, gotoY = 1000, 0

# Rotates a point counterclockwise by a given angle around a given origin.
def rotateJoint(origin, point, shiftAngle):

    originX, originY = origin[1][0], origin[1][1]
    pointX, pointY = point[1][0], point[1][1]

    shiftX = originX + cos(shiftAngle) * (pointX - originX) - sin(shiftAngle) * (pointY - originY)
    shiftY = originY + sin(shiftAngle) * (pointX - originX) + cos(shiftAngle) * (pointY - originY)

    return [origin[0], [shiftX, shiftY]]


# ---------------- Unrenderer ----------------

def clear():
    # Undraws all faces for next frame
    for item in render.items[:]:
        item.undraw()


# ---------------- Renderer ------------------

def refresh():

    # ------------- Pre-rendering ------------

    # Clears the screen
    clear()

    global FPS, last_clock_time

    clock_time = time.time() - start_clock_time
    FPS.timer = clock_time - last_clock_time
    FPS.value += 1

    # --------------- Rendering --------------

    global joints, targetX, targetY, originPX, originPY
    

    # Draws the trail
    if setting.showTrail:

        for _trail in joints:

            setting.trail.append(Point(_trail[1][0],_trail[1][1]))

            if len(setting.trail) == setting.trailLength:
                del setting.trail[0]

        for _trail in range(len(setting.trail)-1,0,-1):

            trailFade = rgb(255,255-int((_trail+1)/len(setting.trail)*255),255-int((_trail+1)/len(setting.trail)*255)) 
            #setting.trail[_trail].draw(render).setFill(trailFade)

            if (_trail != 0) and not(setting.trail[_trail].x == originPX and int(setting.trail[_trail].y) == originPY):
                Line(setting.trail[_trail], setting.trail[_trail-1]).draw(render).setFill(trailFade)

    # Draws the joints
    for _joint in range(len(joints)):
        if _joint == 0:
            jointOrigin = Point(originPX, originPY)
        else:
            jointOrigin = Point(joints[_joint-1][1][0],joints[_joint-1][1][1])

        jointTarget = Point(joints[_joint][1][0],joints[_joint][1][1])
        jointLine = Line(jointOrigin, jointTarget)

        jointWidth = 3*abs(1-(len(joints)/(_joint+1)))+1

        if setting.sizeFade:
            if jointWidth > setting.maxWidth:
                jointWidth = setting.maxWidth
        else:
            jointWidth = setting.maxWidth

        jointLine.setWidth(jointWidth)

        if setting.showVectors:
            jointLine.setArrow('last')

        if setting.showLastVector and _joint == len(joints)-1:
            jointLine.setArrow('last')

        if setting.colorFade:
            jointLine.setFill(rgb(64,128,round(abs((_joint+1)/len(joints))*255)))
        else:
            jointLine.setFill(setting.color)

        jointLine.draw(render)

    # Refresh the joints positions
    for _joint in range(len(joints)-1,-1,-1):

        if _joint == len(joints)-1:
            anchor = (targetX, targetY)
        else:
            anchor = (joints[len(joints)-1][1][0],joints[len(joints)-1][1][1])

        if _joint == 0:
            origin = (originPX,originPY)
        else:
            origin = (joints[_joint-1][1][0],joints[_joint-1][1][1])

        target = (targetX, targetY)
        

        # Calculates the angle towards the target
        angle = -atan2((anchor[0]-origin[0]) , (anchor[1]-origin[1]))+pi/2
        targetAngle = -atan2((target[0]-origin[0]) , (target[1]-origin[1]))+pi/2
        shiftAngle = targetAngle - angle


        # Updates the joint position
        if _joint == len(joints)-1:
            joints[_joint][1][0] = origin[0] + joints[_joint][0] * cos(angle)
            joints[_joint][1][1] = origin[1] + joints[_joint][0] * sin(angle)
        else:
            for __joint in range(_joint,len(joints)):
                if _joint == 0:
                    joints[__joint] = rotateJoint([joints[0][0],[originPX,originPY]], joints[__joint], shiftAngle)
                else:
                    joints[__joint] = rotateJoint(joints[_joint-1], joints[__joint], shiftAngle)

    # ------------------ UI ------------------

    # Draws mouse position
    mouse = Circle(Point(mouseX, mouseY), 10)
    if mousePressed:
        mouse.setFill('blue')

    mouse.draw(render)

    # Draws target position
    if setting.showTarget:
        Line(Point(gotoX-10, gotoY), Point(gotoX+10, gotoY)).draw(render), Line(Point(gotoX, gotoY-10), Point(gotoX, gotoY+10)).draw(render)

    # Draws origin
    if setting.showOrigin:
        origin = Circle(Point(originPX, originPY), 5)
        origin.setFill('red')
        origin.draw(render)
    

    # Sets and displays current FPS
    if FPS.timer > 1:
        FPS.lastValue = FPS.value
        FPS.value = 0
        last_clock_time = clock_time

    FPS.counter = Text(Point(-window.width+50+len(str(FPS.lastValue))* 9, window.height-25), "FPS:"+str(FPS.lastValue))

    if FPS.lastValue <= FPS.maxValue/3:
        FPS.counter.setTextColor("red")
    elif FPS.lastValue <= FPS.maxValue/1.5:
        FPS.counter.setTextColor("orange")
    else:
        FPS.counter.setTextColor("green")

    if setting.showFPS:
        FPS.counter.draw(render)


while True:

    # Gets mouse position
    renderPos = [int(content) for content in render.master.winfo_geometry().split("+",1)[1].split("+",1)]

    mouseX = (mousePos()[0] - renderPos[0])*2 - render.width - 16
    mouseY = (-mousePos()[1] + renderPos[1])*2 + render.height + 64

    if GetKeyState(0x01) < 0:
        mousePressed = True
        gotoX, gotoY = mouseX, mouseY
    else:
        mousePressed = False

    if setting.instantUpdate:
        targetX, targetY = gotoX, gotoY
    else:
        targetX, targetY = targetX + (gotoX-targetX)/20, targetY + (gotoY-targetY)/20

    # Render frames
    clock_time = time.time() - start_clock_time
    refresh()
    update(FPS.maxValue*1.05)

    # Prevents FPS from affecting inputs
    inputSpeed = 1.0 / ((FPS.lastValue+0.01) / 120)


    # ------------------ Inputs ------------------

    # Render options
    if keyboard.is_pressed('&'):
        if not(setting.showLastVector):
            setting.showLastVector = True
        else:
            setting.showLastVector = False
        while keyboard.is_pressed('&'):
            sleep(0.1)

    if keyboard.is_pressed('é'):
        if not(setting.showTarget):
            setting.showTarget = True
        else:
            setting.showTarget = False
        while keyboard.is_pressed('é'):
            sleep(0.1)

    if keyboard.is_pressed('"'):
        if not(setting.instantUpdate):
            setting.instantUpdate= True
        else:
            setting.instantUpdate = False
        while keyboard.is_pressed('"'):
            sleep(0.1)

    if keyboard.is_pressed('('):
        if not(setting.showTrail):
            setting.showTrail = True
        else:
            setting.showTrail = False
        while keyboard.is_pressed('('):
            sleep(0.1)

    # Closes window
    if keyboard.is_pressed('esc'):
        render.close()