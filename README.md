# An Implementation of Verilog to create Space Invaders on a De1-SoC. 

By: Jacob Rice and Noah Budde

Date: December 13, 2024

Class: ECE 287 at Miami University

Professor: Dr. Peter Jamieson


# High-Level Description
  The project focuses on implementing a hardware-based version of the classic arcade game Space Invaders using Verilog. Space Invaders is a 2D shooting game where the player controls a cannon at the bottom of the screen, tasked with defending against waves of alien invaders descending from above. The player must destroy the aliens while avoiding enemy fire from the aliens. The use of an VGA module in combination with a FPGA board is required to display and control the game. The plan of this project was to use the memory-based VGA to write and store each individual pixel as the game transitioned from state to state. 

# Background Information
  In order to implement this, a firm understanding of Verilog and its functions is required. The full solution to this problem requires an understanding of Finite State Machines (FSM), an understanding of Linear Feedback Shift Register (LFSR), and logic flow. To understand the current state of the game, an implementation of Signal Tap is required to understand why the Frame Buffer module is entering the error state.  
# Our Design
  Currently, our implementation remains highly incomplete. After ~25 hours, we have made little progress in working with the given VGA module. The initial plan that we were trying to implement involved creating a separate square drawing module that would intake the location, width, and height of a square to be drawn. From there, it would draw a black square on the white background we have completed. We were planning to use this module to draw the player ship as well as all of the aliens towards the top. After this, KEY 3 and KEY 1 were going to be implemented to allow the player to move left and right, respectively. Furthermore, a press of KEY 2 would fire the ship's cannon towards the aliens based on the current position of the player. The aliens would be "firing" back "randomly" through the use of an LFSR. Here, we were going develop collision detection in order to decide when to eliminate an alien or the player. Our collision detection will check the location of the "lasers" fired by the ships and compare those coordinates with those of the alien or player ships. 
  
# Conclusion
  In the current state, our design is not functional and has yet to be completed. We conservatively estimate that if someone were to pick this up in its current state, it would take ~25-30 hours to implement a rudimentary version of the Space Invaders game. The outline of what our design would've incorporated gives a rough outline of what is to be completed in order to get this in working order. The first step would be getting the square drawing module to work. This would require setting up Signal Tap to analyze each state and to figure out where the vga_frame_driver.v module is defaulting to a constant state of white instead of the MIF file. 
# Current State
![image1 (6)](https://github.com/user-attachments/assets/ad49c1ee-c407-447b-8ad9-a6fa63488414 "The current white background of our image")
<p align="center">An image of the current display of our project</p> <br/>So far, we have written a white background over the provided MIF file included in the VGA interface from Dr. Jamieson. The version that contains the clearest visualization for the logic we were working to implement was the commit made Thursday night, which has the commit message "10PM Tursday", due to a typo in the commit. In this version, the vga_driver_to_frame.v holds two main modules where we have done our work. The main top module is where the VGA assignments are done. From there, a start signal is sent high to the square drawing module, which is supposed to intake the width, height, and x-y coordinates to draw a square and reports back to the top module with a done signal when it is finished. The x-y coordinates would used in the calculation of the specific memory address that needs to be modified to adjust the pixel at those coordinates. This would be done by adding the equivalent of the height (120) for every pixel we want to move to the right and adding one for every pixel we want to move downward. 

# Works Referenced
VGA Module Credit - Dr. Peter Jamieson (Miami University), created October 31st, 2024 
http://www.drpeterjamieson.com/
