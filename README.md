# Instructions for Testing Code
1. Connect to Raspberry Pi via SSH through Tera Term or PuTTY
   
> The username is *cyosh* & the password is *tppq03iz*

2. Once connected, enter the directory **dronekit-test**
   
3. To run any of the scripts, type `python <script name>.py`.
   - To run on physical systems, append above w/ `--connect=/dev/serial0`
   - To run in simulator, run without additional arguments. It will use the default simulator UDP port.

4. You should it see console ouput that shows the R.Pi connecting to the drone, before executing the script. 
