# -*- coding: utf-8 -*-
import os
import imp
import platform
from sys import argv

#   STLIB IMPORT
try:
	from stlib.scene.wrapper import Wrapper
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation as u

slash = '/'
if "Windows" in platform.platform():
    slash = "\\"

# Our Original Scene IMPORT
originalScene = r'D:\Users\family\SOFA-v19\SOFA_v19.06.99_custom_Win64_v8.1\plugins\ModelOrderReduction\ARM\Step-Arm_cambiado.pyscn'
originalScene = os.path.normpath(originalScene)
originalScene = imp.load_source(originalScene.split(slash)[-1], originalScene)

paramWrapper = ('/eobject', {'paramForcefield': {'performECSW': True, 'RIDPath': '\\data\\', 'modesPath': '\\data\\modes.txt', 'weightsPath': '\\data\\'}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': '\\data\\modes.txt'}, 'paramMappedMatrixMapping': {'precomputedMassPath': '\\data\\mass_reduced.txt', 'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'listActiveNodesPath': '\\data\\listActiveNodes.txt', 'template': 'Vec1d,Vec1d', 'usePrecomputedMass': True, 'timeInvariantMapping2': True, 'performECSW': True, 'timeInvariantMapping1': True, 'nodeToParse': '@./eobject'}})

def createScene(rootNode):

    if (len(argv) > 1):
        stateFileName = str(argv[1])
    else:	
        stateFileName="stateFile.state"
    originalScene.createScene(rootNode)

    path , param = paramWrapper
    pathToNode = path[1:]

    u.createDebug(rootNode,pathToNode,stateFileName)