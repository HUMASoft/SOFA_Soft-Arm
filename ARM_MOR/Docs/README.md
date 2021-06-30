#  Model Order Reduction Process for a Soft-Arm with SOFA Framework

The aim of this process is to reduce the number of nodes of the Soft-Arm in order to be able to controll it in a real-time simulation.
MOR is a plugin developed by Sofa framework and the files downloaded during the installation of Sofa (in this case version 19.06.99) will be used in the reduction.

## Preparing the reduction

- Copy the code of this repository in /PathToYourSofaFolder/plugins/ModelOrderReduction  
  - Now you can choose between the code of:
      - 1 tendon
      - 3 tendon
- Open the file MOR_ARM in Sofa
- Select the scene you want to reduce: Original-Arm
- Select the folder where you want the results to be generated

## Results obtained

- Debug and data folders (I have not uploaded them in this repository because they are too heavy)
- Reduced-test.py containing the reduced model. (The reduced model of 1 and 3 tendon are uploaded but they will not run in Sofa because the data and debug folders are missing. The reduction should be performed again in order to visualize this model in Sofa. However, the code can be useful to see the changes done after the reduction (this is explained below)).

## Modifications of the reduced model

- Modify the location of the FixedBox. When the reduction is performed, this box is translated in the z axis. In order to have the same location that was specified in the original code, it is necessary to change the value 32.5 by -32.5 when the nexBox is created.
- Define the class ArmController to be able to control the length of the cables with the keys of the keyboard.
- Call ArmController giving the right parameters. Be careful to include in the scene the GenericConstraintSolver.
