# Setup

- Download the files from this repo.

- Install Thonny IDE (https://thonny.org/)

- Run -> Configure Interpreter
<img width="1792" alt="Screenshot 2025-01-09 at 1 41 24 PM" src="https://github.com/user-attachments/assets/a60e4c63-f2d9-4dea-aadd-308f56fce8c1" />


- Interpreter settings:
    - What kind of interpreter are you using: MicroPython (Raspberry Pi Pico)

    - Port: The port that connects to the Pololu via the USB cable. Once plugged in, the appropriate port should show up in the dropdown under "Port or WebREPL". Can unplug and then plug in the USB cable again to see which port pops up. In the screenshot below, the port is "Pololu 3pi+ Robot MicroPython @ /dev/cu.usbmoden14201"

<img width="1792" alt="Screenshot 2025-01-09 at 1 41 34 PM" src="https://github.com/user-attachments/assets/519695da-32c0-4009-b33d-139b99d5b6d3" />



- Manual: Move the following folder and files into the MicroPython volume (the file system for the Pololu robot). 
```Folder "pololu_3pi_2040_robot", file "solve.py", file "recall.py"```
(screenshot shows MacOS Finder)

<img width="1792" alt="Screenshot 2025-01-09 at 1 47 13 PM" src="https://github.com/user-attachments/assets/928dfa85-b249-448d-bb77-676ec398a9d5" />


 
