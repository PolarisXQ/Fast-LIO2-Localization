﻿# Fast-LIO2-Localization

Modified Fast-LIO2 for localization in prior map.

![](./reloc.gif)

Blue point is the prior map, white point is current scan. When the white point stop twinkling, icp registration successed, and fast-lio2 starts in relocalization mode.

## launch file description

### In FAST_LIO2 package

`mapping.launch.py` is the launch file for Fast-LIO2 Mapping. i.e. the original Fast-LIO2. 

`localization.launch.py` is the launch file for Fast-LIO2 Localization. Change the `map_path` in the `fast_lio_relocalization_param.yaml`, which is under the `config` folder in the `FAST_LIO2` package.

### In icp_relocalization package

`icp.launch.py` is the launch file for ICP Relocalization. 

## Usage

A example of launch file is provided at `example.launch.py`. It will call the `icp_node` and localization.launch.py to do the re-localization. 

Change the `map_path` in the example launch file to the path of the map you want to localize in. i.e. same as the `map_path` in `fast_lio_relocalization_param.yaml`. Additionally, you should give a initial guess of the pose of the robot in the map by changing `initial_x`, `initial_y` etc. in the launch file.

## Notice 

I'd love to hear from you if you have any suggestions or find any bugs. Please feel free to open an issue or make a pull request or contact me in any way you like.
