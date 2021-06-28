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
- Reduced-test.py containing the reduced model

## Modifications of the reduced model

- Modify the location
