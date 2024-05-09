# Quanser_QCar_Competition - UIW 

This repository contains the UIW QCARdinals code for the Quanser Self Driving Car Competition hosted at ACC 2024

*Installation Procedure* 

First run the command 'pip install requirements.txt' in terminal to install all required packages for software contained in this repository.
Ensure that the directory you choose to download and run the code in includes the following folders or libraries: 'hal', 'pal', and 'qvl' which contain vital quanser definitions for other functions referenced in the code. Make sure you also place 'best.pt', 'qlabs_setup', and 'Setup_Competition' in the same directory as the code. These files are vital to running object detection and setting up the virtual environment in Quanser Interactive Labs.   

*Run Order*

Run the files in the following order:
- 'QCAR_Competition_Controls.py' will run basic vehicle control, has a 5 second delay in order to allow time for the user to run each file
- 'Traffic_Lights_Competition.py' to generate the lights into the competition world
- 'Traffic_Sign_Detection' to perform detection and react to traffic signs

  
Video Results:
https://www.youtube.com/watch?v=WCiZdiXEYGc
