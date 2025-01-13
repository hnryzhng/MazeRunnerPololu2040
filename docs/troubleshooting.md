# Troubleshooting


## MicroPython volume files are gone
<b>Problem</b>: Sometimes when you reset the robot or upload a script, the robot MicroPython volume will lose all of its files.  
<b>Solution (screenshots for MacOS)</b>: 
- Turn on the robot in bootloader mode (press Button B and hold the Reset button, then let go of the Reset button). 
<p align="center">
  <img width="400" alt="robot.png" src="https://github.com/user-attachments/assets/8ebb9599-d829-418b-97f3-d3f209e2228b" />
</p>

- The MicroPython volume will turn into "RPI-RP2".
<p align="center">
  <img width="600" alt="Screenshot 2025-01-09 at 1 43 28 PM" src="https://github.com/user-attachments/assets/391f0b4d-afa6-441d-a139-88ce8b208dcd" />
</p>
<br/>

- Download the latest MicroPython firmware .uf2 file for the Pololu 3pi plus 2040 robot [here](https://micropython.org/download/POLOLU_3PI_2040_ROBOT/), then drag the file from the folder you downloaded to into the volume "RPI-RP2". 
<p align="center">
  <img width="600" alt="Screenshot 2025-01-09 at 1 43 46 PM" src="https://github.com/user-attachments/assets/44f50421-bb75-4729-b88c-9652965b1770" />
</p>

<p align="center">
  <img width="600" alt="Screenshot 2025-01-09 at 1 44 17 PM" src="https://github.com/user-attachments/assets/34cfa5bf-6875-4c5d-b505-f5ac757429c9" />
</p>
<br/>

- After the .uf2 file is finished copying, the volume will turn back into "MicroPython" with all the necessary default files, essentially resetting the robot to its initial factory state.
<p align="center">
  <img width="600" alt="Screenshot 2025-01-09 at 1 42 44 PM" src="https://github.com/user-attachments/assets/8d283bce-813d-4e89-aa7f-f47cfe965b48" />
</p>
<br/>
