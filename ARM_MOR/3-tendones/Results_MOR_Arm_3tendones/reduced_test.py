# -*- coding: utf-8 -*-
import os
import sys
import Sofa

#  STLIB IMPORT
from stlib.scene import MainHeader, ContactHeader
from stlib.solver import DefaultSolver
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox
from stlib.physics.collision import CollisionMesh
from stlib.scene import Node
from stlib.visuals import ShowGrid

# SOFTROBOTS IMPORT
from softrobots.actuators import PullingCable

# SPLIB IMPORT
#from splib.loaders import loadPointListFromFileimport Sofa
from numpy import add,subtract,multiply
try:
    from splib.numerics import *
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

path = os.path.dirname(os.path.abspath(__file__))

class ArmController1(Sofa.PythonScriptController):
    def __init__(self, node, tendon1):
        self.cableconstraintvalue = tendon1.getObject("CableConstraint").findData('value')
        self.name = "ArmController1"

    def onKeyPressed(self,c):
        if (c == "1"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 2.
            print ('Shortening')
            print self.cableconstraintvalue.value
        if (c == "2"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] - 2.
            print 'Lengthening'
            print (self.cableconstraintvalue.value)

class ArmController2(Sofa.PythonScriptController):
    def __init__(self, node, tendon2):
        self.cableconstraintvalue = tendon2.getObject("CableConstraint").findData('value')
        self.name = "ArmController2"

    def onKeyPressed(self,c):
        if (c == "3"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 2.
            print ('Shortening')
            print self.cableconstraintvalue.value
        if (c == "4"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] - 2.
            print 'Lengthening'
            print (self.cableconstraintvalue.value)

class ArmController3(Sofa.PythonScriptController):
    def __init__(self, node, tendon3):
        self.cableconstraintvalue = tendon3.getObject("CableConstraint").findData('value')
        self.name = "ArmController3"

    def onKeyPressed(self,c):
        if (c == "5"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 2.
            print ('Shortening')
            print self.cableconstraintvalue.value
        if (c == "6"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] - 2.
            print 'Lengthening'
            print (self.cableconstraintvalue.value)

def TRSinOrigin(positions,modelPosition,translation,rotation,scale=[1.0,1.0,1.0]):
    posOrigin = subtract(positions , modelPosition)
    if any(isinstance(el, list) for el in positions):
        posOriginTRS = transformPositions(posOrigin,translation,eulerRotation=rotation,scale=scale)
    else:
        posOriginTRS = transformPosition(posOrigin,TRS_to_matrix(translation,eulerRotation=rotation,scale=scale))
    return add(posOriginTRS,modelPosition).tolist()

def newBox(positions,modelPosition,translation,rotation,offset,scale=[1.0,1.0,1.0]):
    pos = TRSinOrigin(positions,modelPosition,translation,rotation,scale)
    offset =transformPositions([offset],eulerRotation=rotation,scale=scale)[0]
    return add(pos,offset).tolist()

def Reduced_test(
                  parentNode=None,
                  attachedTo=None,
                  name="Reduced_test",
                  rotation=[0.0, 0.0, 0.0],
                  translation=[0.0, 0.0, 0.0],
                  scale=[1.0, 1.0, 1.0],
                  surfaceMeshFileName=False,
                  surfaceColor=[1.0, 1.0, 1.0],
                  nbrOfModes=17,
                  hyperReduction=True):

    reduced_test = Node(parentNode, "Reduced_test")
    """
    Object with an elastic deformation law.

        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | argument            | type      | definition                                                                                      |
        +=====================+===========+=================================================================================================+
        | attachedTo          | Sofa.Node | Where the node is created;                                                                      |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | name                | str       | name of the Sofa.Node it will                                                                   |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | rotation            | vec3f     | Apply a 3D rotation to the object in Euler angles.                                              |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | translation         | vec3f     | Apply a 3D translation to the object.                                                           |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | scale               | vec3f     | Apply a 3D scale to the object.                                                                 |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceMeshFileName | str       | Filepath to a surface mesh (STL, OBJ). If missing there is no visual properties to this object. |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceColor        | vec3f     | The default color used for the rendering of the object.                                         |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | nbrOfModes          | int       | Number of modes we want our reduced model to work with                                          |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | hyperReduction      | Bool      | Controlled if we have the simple reduction or the hyper-reduction                               |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+

    """

    modelRoot = reduced_test.createChild('modelRoot')

    eobject_MOR = modelRoot.createChild('eobject_MOR')
    eobject_MOR.createObject('EulerImplicitSolver' , name = 'integration')
    eobject_MOR.createObject('SparseLDLSolver' , name = 'solver')
    eobject_MOR.createObject('GenericConstraintCorrection' , solverName = 'solver')
    eobject_MOR.createObject('MechanicalObject' , position = [0]*nbrOfModes, template = 'Vec1d')
    eobject_MOR.createObject('MechanicalMatrixMapperMOR' , object1 = '@./MechanicalObject', object2 = '@./MechanicalObject', listActiveNodesPath = path + r'\data\listActiveNodes.txt', template = 'Vec1d,Vec1d', usePrecomputedMass = True, timeInvariantMapping2 = True, performECSW = hyperReduction, timeInvariantMapping1 = True, precomputedMassPath = path + r'\data\mass_reduced.txt', nodeToParse = '@./eobject')


    eobject = eobject_MOR.createChild('eobject')
    eobject.createObject('MeshVTKLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), translation = add(translation,[0.0, 0.0, 0.0]), name = 'loader', filename = path + r'\mesh\reduce1.vtk')
    eobject.createObject('TetrahedronSetTopologyContainer' , src = '@loader', name = 'container')
    eobject.createObject('MechanicalObject' , name = 'dofs', template = 'Vec3d')
    eobject.createObject('UniformMass' , name = 'mass', totalMass = 0.5)
    eobject.createObject('HyperReducedTetrahedronFEMForceField' , RIDPath = path + r'\data\reducedFF_eobject_0_RID.txt', name = 'reducedFF_eobject_0', weightsPath = path + r'\data\reducedFF_eobject_0_weight.txt', youngModulus = 18000, modesPath = path + r'\data\modes.txt', template = 'Vec3d', performECSW = hyperReduction, method = 'large', poissonRatio = 0.3, nbModes = nbrOfModes)
    eobject.createObject('ModelOrderReductionMapping' , input = '@../MechanicalObject', modesPath = path + r'\data\modes.txt', output = '@./dofs')


    VisualModel = eobject.createChild('VisualModel')
    VisualModel.createObject('MeshSTLLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0.0, 0.0, 0.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), name = 'loader', filename = path + r'\mesh\Brazo_20cm.stl')
    VisualModel.createObject('OglModel' , scale3d = [1.0, 1.0, 1.0], src = '@loader', name = 'model', color = [0.0, 0.3, 0.9], updateNormals = False, rotation = [0.0, 0.0, 0.0], translation = [0.0, 0.0, 0.0])
    VisualModel.createObject('BarycentricMapping' , name = 'mapping')


    FixedBox = eobject.createChild('FixedBox')
    FixedBox.createObject('BoxROI' , name= 'BoxROI' , orientedBox= newBox([[73, 0, 65], [73, 10, 65], [0, 10, 65]] , [0.0, 0.0, 0.0],translation,rotation,[0, 0, -32.5],scale) + multiply(scale[2],[65]).tolist(),drawBoxes=True)
    FixedBox.createObject('HyperReducedRestShapeSpringsForceField' , RIDPath = path + r'\data\reducedFF_FixedBox_1_RID.txt', name = 'reducedFF_FixedBox_1', weightsPath = path + r'\data\reducedFF_FixedBox_1_weight.txt', points = '@BoxROI.indices', modesPath = path + r'\data\modes.txt', stiffness = '1e12', performECSW = hyperReduction, nbModes = nbrOfModes)


    tendon1 = eobject.createChild('tendon1')
    tendon1.createObject('MechanicalObject' , position = TRSinOrigin([[35.0, 2.0, 10.0], [35.0, 40.0, 10.0], [35.0, 80.0, 10.0], [35.0, 120.0, 10.0], [35.0, 160.0, 10.0], [35.0, 198.0, 10.0]] , [0.0, 0.0, 0.0],translation,rotation,scale), rotation = [0.0, 0.0, 0.0], scale = 1.0, translation = [0.0, 0.0, 0.0])
    tendon1.createObject('CableConstraint' , indices = [0, 1, 2, 3, 4, 5], hasPullPoint = False, valueType = 'displacement', value = 0.0)
    tendon1.createObject('BarycentricMapping' , mapMasses = False, name = 'Mapping', mapForces = False)


    tendon2 = eobject.createChild('tendon2')
    tendon2.createObject('MechanicalObject' , position = TRSinOrigin([[10.0, 2.0, 55.0], [10.0, 40.0, 55.0], [10.0, 80.0, 55.0], [10.0, 120.0, 55.0], [10.0, 160.0, 55.0], [10.0, 198.0, 55.0]] , [0.0, 0.0, 0.0],translation,rotation,scale), rotation = [0.0, 0.0, 0.0], scale = 1.0, translation = [0.0, 0.0, 0.0])
    tendon2.createObject('CableConstraint' , indices = [0, 1, 2, 3, 4, 5], hasPullPoint = False, valueType = 'displacement', value = 0.0)
    tendon2.createObject('BarycentricMapping' , mapMasses = False, name = 'Mapping', mapForces = False)


    tendon3 = eobject.createChild('tendon3')
    tendon3.createObject('MechanicalObject' , position = TRSinOrigin([[63.0, 2.0, 55.0], [63.0, 40.0, 55.0], [63.0, 80.0, 55.0], [63.0, 120.0, 55.0], [63.0, 160.0, 55.0], [63.0, 198.0, 55.0]] , [0.0, 0.0, 0.0],translation,rotation,scale), rotation = [0.0, 0.0, 0.0], scale = 1.0, translation = [0.0, 0.0, 0.0])
    tendon3.createObject('CableConstraint' , indices = [0, 1, 2, 3, 4, 5], hasPullPoint = False, valueType = 'displacement', value = 0.0)
    tendon3.createObject('BarycentricMapping' , mapMasses = False, name = 'Mapping', mapForces = False)

    ArmController1(eobject, tendon1)
    ArmController2(eobject, tendon2)
    ArmController3(eobject, tendon3)

    return eobject


#   STLIB IMPORT
# from stlib.scene import MainHeader
# def createScene(rootNode):
#
#     surfaceMeshFileName = False
#
#     MainHeader(rootNode,plugins=["SofaPython","SoftRobots","ModelOrderReduction"],
#                         dt=1.0,
#                         gravity=[0.0, 0.0, -9810.0])
#     rootNode.VisualStyle.displayFlags="showForceFields"
#
#     Reduced_test(rootNode,
#                         name="Reduced_test",
#                         surfaceMeshFileName=surfaceMeshFileName)


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    surfaceMeshFileName = False

    plugins=["SofaPython","SoftRobots","ModelOrderReduction"]
    for name in plugins:
          rootNode.createObject('RequiredPlugin', name=name, printLog=False)

    rootNode.createObject('VisualStyle', displayFlags='showVisualModels showForceFields showBehaviorModels showInteractionForceFields')

    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)

    ShowGrid(rootNode)

    rootNode.findData('gravity').value=[0.0,0.0,-9810];
    rootNode.findData('dt').value=1

    rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")

    Reduced_test(rootNode, name="Reduced_test", surfaceMeshFileName=surfaceMeshFileName)


    # translate = 300
    # rotationBlue = 60.0
    # rotationWhite = 80
    # rotationRed = 70

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_blue_"+str(i),
    #                    rotation=[rotationBlue*i, 0.0, 0.0],
    #                    translation=[i*translate, 0.0, 0.0],
    #                    surfaceColor=[0.0, 0.0, 1, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_white_"+str(i),
    #                    rotation=[0.0, rotationWhite*i, 0.0],
    #                    translation=[i*translate, translate, -translate],
    #                    surfaceColor=[0.5, 0.5, 0.5, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_red_"+str(i),
    #                    rotation=[0.0, 0.0, i*rotationRed],
    #                    translation=[i*translate, 2*translate, -2*translate],
    #                    surfaceColor=[1, 0.0, 0.0, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
