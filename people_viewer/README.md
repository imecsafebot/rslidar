# RSLiDAR People Tracking Viewer

This viewer displays:
- the original PCD point cloud for each frame
- 3D bounding boxes around tracked people
- person/track IDs
- moving/static state
- speed
- frame number

It reads the PCD sequence and the CSV produced by the people detector.
It does NOT run the detector again; it visualizes the existing results.

## Build

```bash
sudo apt install libpcl-dev
cd ~/Documents/rslidar_people_viewer
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Run

```bash
./rslidar_people_viewer /path/to/pcd_directory /path/to/test_results.csv
```

Example:

```bash
./rslidar_people_viewer ~/Documents/rslidar_pcds ~/Documents/rslidar_people_detector/test_results\(5\).csv
```

If the CSV contains spaces/parentheses, quote the path:

```bash
./rslidar_people_viewer ~/Documents/rslidar_pcds "$HOME/Documents/rslidar_people_detector/test_results(5).csv"
```

## Controls

- SPACE: pause/resume
- N: next frame while paused
- B: previous frame while paused
- M: show/hide static tracks (M = moving only)
- Q or ESC: quit

The viewer expects frame files named like:

```text
scan_frame_0.pcd
scan_frame_1.pcd
...
scan_frame_1050.pcd
```

The CSV parser accepts common names such as:
- frame
- person_id / track_id / id
- center_x / x
- center_y / y
- center_z / z
- width
- depth
- height
- speed
- moving

If the CSV has no center_z, the box is placed with its bottom at z=0.
