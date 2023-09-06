# Obstacle avoidance app for Crazyflie 2.X

This folder contains the app layer application for the Crazyflie to avoid obstacles, which can be started and configured from the [cfclient](https://github.com/bitcraze/crazyflie-clients-python). 

See App layer API guide [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

## Build and flash

Make sure that you are in the app_hello_world folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make 
make cload
```

If you want to compile the application elsewhere in your machine, make sure to update ```CRAZYFLIE_BASE``` in the **Makefile**.
This app is tested with the crazyflie-firmware commit b0c72f2a4cb8b432211a2fa38d97c5a1dcef07ff.

## Run

- Turn on drone  
- Connect to it via CF GUI
- Adapt params like max vel, height, etc. if wanted (in the Parameters tab, ToF_FLY_PARAMS)
- Enter a number (in seconds) for how long it should fly (it will anyway land once the battery runs out) in the ToFly parameter
- Press enter, it will take off and start flying around!
- Look at the cmds in the Plotter tab (add a config under "Settings" "logging configurations")