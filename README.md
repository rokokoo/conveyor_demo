# conveyor_demo
Demo for conveyor + manipulator

Download with
```bash
git clone --recurse-submodules https://github.com/rokokoo/conveyor_demo.git
```
Build with catkin and launch the included demo world.

```bash
roslaunch demo_world demo.launch
```

The demo contains a python script, that spawns cubes, red, green and blue. It will also automatically delete the cube, when it falls from the conveyor and reaches a height under 0.05 meters.

Remember to add power to the conveyor, using
```bash
rosservice call /conveyor/control "power: 100.0"
```

The power can be anything between 0 and 100.
