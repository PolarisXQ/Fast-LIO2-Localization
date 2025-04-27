# Fast-LIO2-Localization

Modified Fast-LIO2 for localization in prior map.

![](./reloc.gif)

Blue point is the prior map, white point is current scan. When the white point stop twinkling, icp registration successed, and fast-lio2 starts in relocalization mode.

## Usage

### Mapping

Same as Fast-LIO2.

```bash
ros2 launch fast_lio mapping.launch.py
```

### Relocalization

A example of launch file is provided at `example.launch.py`. It will call the `icp_node` and localization.launch.py to do re-localization. 

**IMPORTANT**

1. CHANGE the `map_path` in `example.launch.py` AND `prior_map_path` in `FAST_LIO/config/fast_lio_relocalization_param.yaml` to the path of the map you want to localize in, THEY SHOULD BE THE SAME.
2. Give an initial guess of the pose of the robot in the map by changing `initial_x`, `initial_y` etc. in `example.launch.py`, or give your initial guess in the rviz using `2D Pose Estimation`.

## Notice 

I'd love to hear from you if you have any suggestions or find any bugs. Please feel free to open an issue or make a pull request or contact me in any way you like.
