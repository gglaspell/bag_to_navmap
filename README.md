# bag_to_navmap

Convert ROS 2 bag files containing LiDAR point clouds into binary NavMap files
for use with [EasyNavigation/NavMap](https://github.com/EasyNavigation/NavMap).

The pipeline runs entirely inside Docker — no ROS 2 installation required on
your host machine.

---

## What This Does

This tool takes a ROS 2 bag file recorded with a LiDAR sensor and produces
three output files:

| File | Description |
|---|---|
| `*_cloud.ply` | Dense 3D point cloud (view in CloudCompare or MeshLab) |
| `*_mesh.obj` | Triangle mesh from Poisson surface reconstruction (view in Blender) |
| `*.navmap` | Binary NavMap file ready to load into EasyNavigation |

**Design philosophy:** Accuracy over speed. Built for offline post-processing
where quality matters more than real-time performance.

---

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) (the only hard requirement)
- A ROS 2 bag file containing `sensor_msgs/PointCloud2` messages
- 8 GB+ RAM (16 GB+ recommended for large datasets)

---

## Quick Start

### 1. Clone the repo

```bash
git clone https://github.com/your-org/bag_to_navmap.git
cd bag_to_navmap
```


### 2. Build the Docker image

```bash
docker build -t bag-to-navmap .
```

This downloads dependencies and compiles the NavMap helper (~5–10 min on first
build). Subsequent builds are fast thanks to Docker layer caching.

### 3. Organize your data

```bash
mkdir -p data/input data/output
# Copy your ROS 2 bag file into data/input/
```


### 4. Run the conversion

**Basic command** (assumes point cloud topic is `/points`):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-navmap \
  /app/data/input/your_bag_file \
  /app/data/output
```

**Recommended command** (with odometry for better alignment):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-navmap \
  /app/data/input/your_bag_file \
  /app/data/output \
  --pc_topic /your/pointcloud/topic \
  --odom_topic /your/odom/topic \
  --icp_fitness_thresh 0.4 \
  --level_floor
```


### 5. Find your outputs

```
data/output/
  your_bag_file_cloud.ply   ← Point cloud
  your_bag_file_mesh.obj    ← Triangle mesh
  your_bag_file.navmap      ← Binary NavMap (load into EasyNavigation)
```


---

## Finding Your Topic Names

If you are unsure what topics are in your bag file, run:

```bash
ros2 bag info /path/to/your_bag_file
```

Look for:

- `sensor_msgs/msg/PointCloud2` — your `--pc_topic` value
- `nav_msgs/msg/Odometry` — your `--odom_topic` value (optional but improves results)

---

## All Parameters

| Parameter | Default | Description |
| :-- | :-- | :-- |
| `bag_path` | required | Path to the ROS 2 bag file |
| `output_dir` | required | Directory to write output files |
| `--pc_topic` | `/points` | PointCloud2 topic name |
| `--odom_topic` | none | Odometry topic (nav_msgs/Odometry) |
| `--voxel_size` | `0.05` | Downsampling resolution in meters |
| `--icp_dist_thresh` | `0.2` | Max ICP point correspondence distance (m) |
| `--icp_fitness_thresh` | `0.6` | Min fraction of points aligned to accept a frame |
| `--enable_loop_closure` | off | Enable loop closure detection |
| `--loop_closure_radius` | `10.0` | Search radius for loop closures (m) |
| `--loop_closure_fitness_thresh` | `0.3` | Min fitness score for a loop closure |
| `--loop_closure_search_interval` | `10` | Check for loop closures every N frames |
| `--level_floor` | off | Rotate the dominant floor plane to +Z |
| `--decimate_target` | none | Reduce triangles: ≤1.0 = ratio, >1 = absolute count |
| `--odom_max_latency` | `0.5` | Max timestamp gap between odom and point cloud (s) |
| `--poisson_depth` | `9` | Octree depth for Poisson reconstruction |
| `--density_trim_percentile` | `0.05` | Bottom density fraction of vertices to remove |
| `--surface_name` | `map` | NavMap surface name embedded in the output file |


---

## Common Use Cases

### Indoor mapping (TurtleBot, small rooms)

```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-navmap \
  /app/data/input/office_scan \
  /app/data/output \
  --pc_topic /scan/points \
  --odom_topic /odom \
  --voxel_size 0.02 \
  --icp_fitness_thresh 0.5 \
  --level_floor
```


### Outdoor mapping (large areas)

```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-navmap \
  /app/data/input/outdoor_survey \
  /app/data/output \
  --pc_topic /velodyne_points \
  --odom_topic /integrated_odom \
  --voxel_size 0.1 \
  --icp_fitness_thresh 0.2 \
  --level_floor
```


### Warehouse / campus with loop closure

```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-navmap \
  /app/data/input/warehouse_full \
  /app/data/output \
  --pc_topic /velodyne_points \
  --odom_topic /integrated_odom \
  --voxel_size 0.1 \
  --icp_fitness_thresh 0.3 \
  --enable_loop_closure \
  --level_floor
```


---

## Speed vs. Quality Guide

| Goal | `--voxel_size` | `--icp_fitness_thresh` | `--icp_dist_thresh` |
| :-- | :-- | :-- | :-- |
| 🚀 Fast preview | `0.1` | `0.3` | `0.5` |
| ⚖️ Balanced | `0.05` | `0.5` | `0.2` |
| 🎨 Maximum quality | `0.01` | `0.7` | `0.1` |


---

## Processing Time Reference

| Environment | Frames | Voxel size | Time (no loop closure) | Peak RAM |
| :-- | :-- | :-- | :-- | :-- |
| Small room | ~500 | 0.02 m | 1–2 min | 4 GB |
| Office floor | ~1500 | 0.05 m | 3–5 min | 8 GB |
| Large warehouse | ~3000 | 0.1 m | 8–12 min | 12 GB |
| Campus outdoor | ~5000 | 0.1 m | 15–25 min | 16 GB |

Loop closure enabled adds roughly 3–8× processing time.

---

## Troubleshooting

### "No messages found for topics"

Your topic name does not match the bag. Run `ros2 bag info` and use the exact
topic name shown.

### "Registration failed: too few accepted frames"

Try lowering `--icp_fitness_thresh` to `0.3` and raising `--icp_dist_thresh`
to `0.5`. Adding `--odom_topic` also helps significantly.

### Point clouds look "stacked" or doubled

Registration is failing silently. Use stricter settings:
`--icp_fitness_thresh 0.7 --icp_dist_thresh 0.15`

### Processing is very slow

Raise `--voxel_size` to `0.1` first — this has the biggest speed impact. If
you have `--enable_loop_closure` on, try removing it unless your path revisits
the same areas.

### "Mesh generation failed"

The merged point cloud is too sparse for Poisson reconstruction. Lower
`--voxel_size` to `0.02` or add `--odom_topic` to improve registration density.

### Floors slope or tilt vertically

Use `--level_floor` for single-floor indoor or flat outdoor environments. Do
not use it for multi-story buildings or terrain with elevation changes.

---

## Loading the `.navmap` File into EasyNavigation

Loading the output back into EasyNavigation will be covered in a future update.

---

## Output File Reference

### `*_cloud.ply`

Dense 3D point cloud. Open with:
[CloudCompare](https://www.cloudcompare.org/) (free) or
[MeshLab](https://www.meshlab.net/) (free).

### `*_mesh.obj`

Triangle mesh from Poisson surface reconstruction. Open with:
[Blender](https://www.blender.org/) (free) or
[3D Viewer Online](https://3dviewer.net/).

### `*.navmap`

Binary NavMap file using ROS 2 CDR serialization. Contains vertices, triangles,
surface definitions, and prebuilt geometry accelerators for use with the
[NavMap C++ API](https://easynavigation.github.io/NavMap/).

---

## Project Files

| File | Purpose |
| :-- | :-- |
| `bag_to_navmap.py` | Main conversion script (meshing + NavMap export) |
| `Dockerfile` | Builds the container (ROS 2 + NavMap + Python deps) |
| `entrypoint.sh` | Sources ROS and workspace overlays, then runs the script |
| `requirements.txt` | Python dependencies |


---

## Built With

[Open3D](https://www.open3d.org/) •
[rosbags](https://github.com/rpng/rosbags) •
[NumPy](https://numpy.org/) •
[SciPy](https://scipy.org/) •
[NavMap](https://github.com/EasyNavigation/NavMap) •
Python 3.10+


