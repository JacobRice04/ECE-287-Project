This directory contains a bunch of projects that use the VGA.

There are 4 projects and are listed in incrementing order as I used them to design more complex systems.  You should take a look at them to understand the progression.
Each project directory has a brief README.txt file to describe the internals of the design.  Also, I've provided some comments:
- 1) vga_driver_memory - this is a starting point that combinationally changes the signals to the VGA
- 2) vga_driver_memory_2 - this one implements a memory to store some stuff and then displays a grid
- 3) vga_driver_memory_double_buf - this is my attempt to understand how to build a double buffer.  SKIP this one as it was mainly used for my experiments and understanding
- 4) vga_driver_to_frame_buf - in this code, I show how you can write pixels to the frame buf.  The frame buf is now encapsulated in a design and use 3 memories.  You just write pixels into the memory as accessed on lines 205-207

All are tested and working in the lab on Oct 31st 2024 by Peter