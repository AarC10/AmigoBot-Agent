# Local Forwarding
Using camera locally and publishing as rostopic
```
roslaunch picam_forwarder local_cam.launch
```


# Viewing Topic
```
rosrun image_view image_view image:=/camera/image_raw
```

If you can't open the display, run
```
xhost +local:root
```
on host

# Potential Issues
## Camera not found
- Make sure camera is connected
- Running container with `--device /dev/video0:/dev/video0` flag
- Potentially need to run as root
