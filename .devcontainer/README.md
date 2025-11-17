Quick start:

1. allow local root connections so the container can open GUI windows

```bash
xhost +local:root
```

2.  Start the Docker Compose environment

```bash
docker-compose up -d
```

3. Optional: Open a terminal in PyCharm or exec into the container to build or source:

```bash
docker-compose exec amigo bash
source /opt/ros/noetic/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi
cd /root/catkin_ws
catkin_make
```
