# Important Information for Firmware Developers
Untested firmware changes should be pushed to their own branches. For instance, a change made to the MAX11131 IC should be pushed to the branch `release-max11131-dev`. In general the naming convention for branches is:

`feature-IC_name_here-dev`

To create a new branch locally, first fetch all the changes from the branch that you would like to branch from, this can be done on the command line using `git pull origin branch_that_you_want_to_branch_from`. Before running this command, you will need to switch to the branch that you would like to branch from first by doing `git checkout branch_that_you_want_to_branch_from`. Once the code on that branch is thoroughly tested on its feature branch, maintainer will merge into master.
The master branch should remain stable for use at all times.

In addition, the `dev` branch will be contain the latest stable versions of each of the firmware libraries. These libraries may potientially still contain edge case bugs. However, they have all been verified to be bug free in the normal use case. When you are done working on a new firmware library for a specific chip, you will need to create a merge request on Gitlab for merge you branch into dev (not master). When a version of the firmware libraries on dev has been thoroughly vetted through field testing for any unforeseen issues, the entire dev branch will be merged in master by a maintainer to ensure that master remains bug free for flight.

## Structuring your new firmware library

It is extremely important to structure your firmware project using the steps below. Failure to do so will result in unique complications for others who want to use your firmware library, and no one likes that. 

The filestructure for your project should look like this:

### Example file structure for firmware library

```
IC_NAME/     
+--inc/      
    +--IC_NAME.h       
    +--*additional header files for project here*      
+--src/     
    +--IC_NAME.c       
    +--*additional c files for project here*       
+--README.md       
```
`IC_NAME` should be the exact name of your IC's part number (P/N) specified on the data sheet. Note that P/Ns generally start with 2-3 letters, and aren't just numbers. This ensures that firmware libraries are easy to find and use in projects. In addition, all header files for your library should be placed in the nested inc directory, and all c files should be placed in the nested src directory. Failure to do this will result in complications using your library in projects. Additionally, a README.md files is highly recommended for an firmware library, as it will inform the user of any information required to use your library in projects.

Remember to create your firmware library folder at the top level of the firmware-libraries directories. Happy firm-ing!

# How to properly create merge request

After creating your own branch using the steps above, you will need to make all your code changes on that branch. During the development process, it is crucial that you constantly pull in changes from your source branch (the branch that you originally branched from) into the branch that you are working on. You can do this by running `git pull origin source_branch_here` while you're on your development branch. By doing this frequently, you reduce the possibility of merge conflicts and save yourself a ton of headache down the line. 

Once you feel that you've made substantial code changes on the development branch, you can create a merge request for it so that other developers can help review and provide feedback on your code. To do this, first make sure you add and commit all your modified files on the development branch. After doing this, simply push your development branch to the origin remote. Then, log on to the gitlab repository that you cloned and you should see a blue button on the top of Gitlab that asks if you would like to create a MR (merge request). Upon clicking on the MR button, please document the modifications for your code in detail in the MR description. Make sure that the checkbox `delete source branch on merge` is deselected. In addition, assign the MR to the person who assigned you the task. From now on, you should discuss all code specific questions on Gitlab.

Note: you should never create a merge request to merge you branch into master!! For most cases, you should create a merge request to merge into the `dev` branch instead! We should only be merging `dev` into `master` when we know the system is stable.

# How to setup STM32CubeIDE

## Setting up STM32CubeIDE project to use firmware libraries

1. From the project root directory on the cli, enter `cd Core`. This will take you to the correct directory to clone the firmware libraries.

2. Enter `git submodule add https://gitlab.eecs.umich.edu/masa/avionics/firmware-libraries.git`, and you should see the firmware-libraries directories appear in PROJECTROOT/Core.

3. On STM32Cube IDE, go to `Project->Properties->C/C++ Build->Settings->Tool Settings->MCU GCC Compiler->Include paths`. Here you should be able to click a small paper icon with a green plus sign underneath a box called `Include paths (-I)`. Click on this box and enter the filepath to the includes files for your desired IC in the firmware-libraries directory. Remember to hit apply changes when done. Note: the filepath may look something like `/Users/arthur/Documents/MASA/firmware-nucleo-tests/Core/firmware-libraries/MAX11131/inc`

4. Once you're finished adding the inc directory for your IC to the project, the compiler will be able to properly link your firmware library's *.c files to the required *.h files. 

5. In the main.h file for your project, simply add the main header for your IC inside the user include guards. You should be adding something that looks like `#include FOO.h`, where FOO is the name of header file for your IC.

6. Now you're all set to use your firmware library in your project's main.c file!
