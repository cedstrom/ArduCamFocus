# ArduCamFocus

This plugin controls the focus on an ArduCam Motorized Focus Camera.  https://www.arducam.com/docs/cameras-for-raspberry-pi/motorized-focus-camera/

![screenshot](extras/assets/img/plugins/ArduCamFocus/ControlScreenShot.png)

It uses a custom `@ARDUCAMFOCUSSET FOCUS` command that can be incorporated within your slicer to automatically adjust focus while printing.
You can also send relative adjustments with `@ARDUCAMFOCUS 50` or `@ARDUCAMFOCUS -50`

Example: `@ARDUCAMFOCUSSET 300` will set the focus to 300.

## Video Example

<video  width="800" height="600" src="extras/assets/img/plugins/ArduCamFocus/ArduCamFocusScreenCap.mov"></video>

## Pre-Installation Requirements

Please follow the manufacturer's instructions:

This plugin uses I2C to communicate with the camera.  That is not enabled by default.  The ArduCamFocus plugin will not function until
you enable I2C.
  ssh to your octopi and enter this commands (this only needs to be done once):
```bash
if ! grep -Fxq "^#ArduCamFocus$" /boot/config.txt; then
sudo cat << end_of_file >> /boot/config.txt
#ArduCamFocus
dtparam=i2c_vc=on
dtparam=i2c_arm=on
end_of_file
fi
```
After executing the above command, the file /boot/config.txt should now have the commands to enable I2C.  In addition, you have to enable the I2C
kernel module using raspi-config.  Again, ssh to your octopi, and then enter this command:
```bash
sudo raspi-config
```

1. select "5 Interfacing Options"
2. select "P5 I2C"
3. raspi-config will ask, "Would you like the ARM I2C interface to be enabled?"
4. select "Yes"
5. you should see, "The ARM I2C interface is enabled"
6. select "Finish"

You may also need to install the following packages:

libilmbase-dev
libopenexr-dev
libgstreamer1.0-dev
python-smbus
i2c-tools
libgtk-3-0

After you reboot, the camera should become operational in OctoPrint.

### Disclaimer

Although I use this plugin and it works for me without issues, I take no resposiblity for any damage caused by using this plugin. Your camera version, i2c address, or system configuration may be different from mine.  Please make sure to do your reseach and understand the dangers and please be careful.
