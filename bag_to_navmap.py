#!/usr/bin/env python3
import argparse
import math
import struct
import subprocess
from bisect import bisect_left
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

HELPER_BIN = "/opt/navmap_ws/install/bag_to_navmap_helper/lib/bag_to_navmap_helper/obj_to_navmap"

def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9

def make_transform_xyzquat(x, y, z, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def inverse_tf(T):
    out = np.eye(4, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out

def nearest_time_index(times, t):
    pos = bisect_left(times, t)
    if pos == 0:
        return 0
    if pos == len(times):
        return len(times) - 1
    before = pos - 1
    after = pos
    return before if abs(times[before] - t) <= abs(times[after] - t) else after

def field_offsets(fields):
    out = {}
    for f in fields:
        out[f.name] = (f.offset, f.datatype, f.count)
    return out

def pointcloud2_to_xyz(msg) -> np.ndarray:
    fmap = field_offsets(msg.fields)
    needed = ["x", "y", "z"]
    if not all(name in fmap for name in needed):
        raise ValueError("PointCloud2 does not contain x/y/z fields")

    xoff, xdt, _ = fmap["x"]
    yoff, ydt, _ = fmap["y"]
    zoff, zdt, _ = fmap["z"]

    if not (xdt == ydt == zdt == 7): # FLOAT32
        raise ValueError("Only FLOAT32 x/y/z fields are supported")

    data = bytes(msg.data)
    point_step = int(msg.point_step)
    npts = int(msg.width) * int(msg.height)
    endian = ">" if msg.is_bigendian else "<"

    xyz = np.empty((npts, 3), dtype=np.float32)
    unpack_x = struct.Struct(endian + "f").unpack_from
    unpack_y = struct.Struct(endian + "f").unpack_from
    unpack_z = struct.Struct(endian + "f").unpack_from

    for i in range(npts):
        base = i * point_step
        xyz[i, 0] = unpack_x(data, base + xoff)[0]
        xyz[i, 1] = unpack_y(data, base + yoff)[0]
        xyz[i, 2] = unpack_z(data, base + zoff)[0]

    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz

def load_bag_streams(bag_path: Path, pc_topic: str, odom_topic: str | None):
    point_frames = []
    odom_samples = []

    # Initialize a default type store for standard ROS 2 messages
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        wanted = [
            c for c in reader.connections
            if c.topic == pc_topic or (odom_topic and c.topic == odom_topic)
        ]
        
        if not wanted:
            raise RuntimeError("No matching topics found in bag")

        for connection, _, rawdata in reader.messages(connections=wanted):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == pc_topic:
                t = stamp_to_sec(msg.header.stamp)
                xyz = pointcloud2_to_xyz(msg)
                if xyz.size == 0:
                    continue
                point_frames.append({"time": t, "xyz": xyz})

            elif odom_topic and connection.topic == odom_topic:
                t = stamp_to_sec(msg.header.stamp)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                T = make_transform_xyzquat(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                odom_samples.append((t, T))

    if not point_frames:
        raise RuntimeError(f"No point clouds found on topic {pc_topic}")

    odom_samples.sort(key=lambda x: x[0])
    return point_frames, odom_samples

def attach_initial_odom(point_frames, odom_samples, odom_max_latency):
    if not odom_samples:
        for fr in point_frames:
            fr["odom"] = None
        return

    times = [t for t, _ in odom_samples]
    Ts = [T for _, T in odom_samples]

    for fr in point_frames:
        idx = nearest_time_index(times, fr["time"])
        dt = abs(times[idx] - fr["time"])
        fr["odom"] = Ts[idx] if dt <= odom_max_latency else None

def make_o3d_cloud(xyz: np.ndarray):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return pcd

def preprocess_cloud(xyz: np.ndarray, voxel_size: float):
    pcd = make_o3d_cloud(xyz)
    pcd = pcd.voxel_down_sample(voxel_size)
    if len(pcd.points) == 0:
        return pcd
    radius_normal = max(voxel_size * 2.0, 0.05)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    pcd.normalize_normals()
    return pcd

def icp_register(source, target, init, dist_thresh):
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        dist_thresh,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )
    info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, dist_thresh, result.transformation
    )
    return result, info

def maybe_add_loop_closure(
    idx,
    clouds,
    pose_graph,
    current_pose_guess,
    voxel_size,
    dist_thresh,
    radius,
    search_interval,
    fitness_thresh,
):
    if idx < 2 or idx % search_interval != 0:
        return

    curr_pos = current_pose_guess[:3, 3]
    for j in range(0, idx - 1, search_interval):
        old_pose = np.linalg.inv(pose_graph.nodes[j].pose)
        old_pos = old_pose[:3, 3]
        if np.linalg.norm(curr_pos - old_pos) > radius:
            continue

        source = clouds[idx]
        target = clouds[j]

        init = inverse_tf(old_pose) @ current_pose_guess
        result, info = icp_register(source, target, init, dist_thresh)
        if result.fitness >= fitness_thresh:
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(
                    j, idx, result.transformation, info, uncertain=True
                )
            )

def build_pose_graph(
    frames,
    voxel_size,
    icp_dist_thresh,
    icp_fitness_thresh,
    enable_loop_closure,
    loop_closure_radius,
    loop_closure_fitness_thresh,
    loop_closure_search_interval,
):
    processed = [preprocess_cloud(fr["xyz"], voxel_size) for fr in frames]
    if not processed or len(processed[0].points) == 0:
        raise RuntimeError("First point cloud became empty after preprocessing")

    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.eye(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

    accepted = [0]

    for i in range(1, len(processed)):
        src = processed[i]
        tgt = processed[i - 1]

        if len(src.points) == 0 or len(tgt.points) == 0:
            pose_graph.nodes.append(
                o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
            )
            continue

        odom_prev = frames[i - 1].get("odom")
        odom_curr = frames[i].get("odom")
        if odom_prev is not None and odom_curr is not None:
            init = inverse_tf(odom_prev) @ odom_curr
        else:
            init = np.eye(4)

        result, info = icp_register(src, tgt, init, icp_dist_thresh)

        if result.fitness < icp_fitness_thresh:
            if odom_prev is not None and odom_curr is not None:
                T_prev_curr = inverse_tf(odom_prev) @ odom_curr
                T_curr_prev = inverse_tf(T_prev_curr)
                odometry = odometry @ T_curr_prev
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
                )
            else:
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
                )
            continue

        accepted.append(i)
        odometry = result.transformation @ odometry
        pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
        )
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                i - 1, i, result.transformation, info, uncertain=False
            )
        )

        if enable_loop_closure:
            maybe_add_loop_closure(
                i,
                processed,
                pose_graph,
                inverse_tf(pose_graph.nodes[i].pose),
                voxel_size,
                icp_dist_thresh,
                loop_closure_radius,
                loop_closure_search_interval,
                loop_closure_fitness_thresh,
            )

    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=icp_dist_thresh,
        edge_prune_threshold=0.25,
        preference_loop_closure=1.5 if enable_loop_closure else 0.1,
        reference_node=0,
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )

    return processed, pose_graph, accepted

def merge_global_cloud(frames, pose_graph, voxel_size):
    merged = o3d.geometry.PointCloud()
    for i, fr in enumerate(frames):
        pcd = make_o3d_cloud(fr["xyz"])
        pcd.transform(pose_graph.nodes[i].pose)
        merged += pcd

    merged = merged.voxel_down_sample(voxel_size)
    if len(merged.points) == 0:
        raise RuntimeError("Merged cloud is empty")

    radius_normal = max(voxel_size * 2.5, 0.05)
    merged.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    merged.normalize_normals()
    return merged

def level_cloud_to_floor(cloud: o3d.geometry.PointCloud):
    plane, inliers = cloud.segment_plane(
        distance_threshold=0.03, ransac_n=3, num_iterations=2000
    )
    normal = np.asarray(plane[:3], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    target = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    dot = np.clip(np.dot(normal, target), -1.0, 1.0)
    if dot > 0.999999:
        return cloud

    axis = np.cross(normal, target)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-12:
        return cloud

    axis /= axis_norm
    angle = math.acos(dot)
    R = Rotation.from_rotvec(axis * angle).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    cloud.transform(T)
    return cloud

def poisson_mesh_from_cloud(cloud, poisson_depth, density_trim_percentile, decimate_target):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=poisson_depth
    )
    densities = np.asarray(densities)
    if densities.size == 0:
        raise RuntimeError("Poisson reconstruction returned no density values")

    trim_q = np.quantile(densities, density_trim_percentile)
    keep = densities >= trim_q
    mesh.remove_vertices_by_mask(~keep)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    if decimate_target is not None:
        ntri = len(mesh.triangles)
        if decimate_target <= 1.0:
            target_triangles = max(100, int(ntri * decimate_target))
        else:
            target_triangles = int(decimate_target)
        target_triangles = min(target_triangles, ntri)
        if 0 < target_triangles < ntri:
            mesh = mesh.simplify_quadric_decimation(target_triangles=target_triangles)
            mesh.remove_degenerate_triangles()
            mesh.remove_duplicated_triangles()
            mesh.remove_duplicated_vertices()
            mesh.remove_non_manifold_edges()

    mesh.compute_vertex_normals()
    return mesh

def run_navmap_export(mesh_obj_path: Path, navmap_path: Path, surface_name: str):
    cmd = [HELPER_BIN, str(mesh_obj_path), str(navmap_path), surface_name]
    subprocess.run(cmd, check=True)

def output_stem_for_bag(bag_path: Path) -> str:
    return bag_path.name

def main():
    parser = argparse.ArgumentParser(
        description="Convert a ROS 2 bag with PointCloud2 data into PLY, OBJ, and binary NavMap output."
    )
    parser.add_argument("bag_path", help="Path to ROS 2 bag")
    parser.add_argument("output_dir", help="Output directory")
    parser.add_argument("--pc_topic", default="/points", help="PointCloud2 topic")
    parser.add_argument("--odom_topic", default=None, help="Odometry topic")
    parser.add_argument("--voxel_size", type=float, default=0.05, help="Voxel downsample size in meters")
    parser.add_argument("--icp_dist_thresh", type=float, default=0.2, help="ICP max correspondence distance")
    parser.add_argument("--icp_fitness_thresh", type=float, default=0.6, help="Minimum ICP fitness to accept a frame")
    parser.add_argument("--enable_loop_closure", action="store_true", help="Enable loop closure edges")
    parser.add_argument("--loop_closure_radius", type=float, default=10.0, help="Loop closure search radius")
    parser.add_argument("--loop_closure_fitness_thresh", type=float, default=0.3, help="Loop closure ICP fitness threshold")
    parser.add_argument("--loop_closure_search_interval", type=int, default=10, help="Check loop closures every N frames")
    parser.add_argument("--level_floor", action="store_true", help="Rotate dominant floor plane to +Z")
    parser.add_argument("--decimate_target", type=float, default=None, help="<=1.0 ratio to keep, >1 absolute triangle count")
    parser.add_argument("--odom_max_latency", type=float, default=0.5, help="Max odom-to-pointcloud timestamp gap in seconds")
    parser.add_argument("--poisson_depth", type=int, default=9, help="Poisson reconstruction octree depth")
    parser.add_argument("--density_trim_percentile", type=float, default=0.05, help="Bottom density fraction to remove")
    parser.add_argument("--surface_name", default="map", help="NavMap surface name")
    args = parser.parse_args()

    bag_path = Path(args.bag_path).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not Path(HELPER_BIN).exists():
        raise RuntimeError(f"NavMap helper not found: {HELPER_BIN}")

    print(f"Reading bag: {bag_path}", flush=True)
    frames, odom_samples = load_bag_streams(bag_path, args.pc_topic, args.odom_topic)
    attach_initial_odom(frames, odom_samples, args.odom_max_latency)

    print(f"Extracted {len(frames)} point clouds", flush=True)
    processed, pose_graph, accepted = build_pose_graph(
        frames=frames,
        voxel_size=args.voxel_size,
        icp_dist_thresh=args.icp_dist_thresh,
        icp_fitness_thresh=args.icp_fitness_thresh,
        enable_loop_closure=args.enable_loop_closure,
        loop_closure_radius=args.loop_closure_radius,
        loop_closure_fitness_thresh=args.loop_closure_fitness_thresh,
        loop_closure_search_interval=args.loop_closure_search_interval,
    )

    if len(accepted) < 2:
        raise RuntimeError("Registration failed: too few accepted frames")

    print(f"Accepted {len(accepted)} / {len(frames)} frames", flush=True)

    merged = merge_global_cloud(frames, pose_graph, args.voxel_size)
    if args.level_floor:
        merged = level_cloud_to_floor(merged)

    stem = output_stem_for_bag(bag_path)
    ply_path = output_dir / f"{stem}_cloud.ply"
    obj_path = output_dir / f"{stem}_mesh.obj"
    navmap_path = output_dir / f"{stem}.navmap"

    o3d.io.write_point_cloud(str(ply_path), merged)

    print("Running Poisson reconstruction", flush=True)
    mesh = poisson_mesh_from_cloud(
        merged,
        poisson_depth=args.poisson_depth,
        density_trim_percentile=args.density_trim_percentile,
        decimate_target=args.decimate_target,
    )

    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError("Mesh generation failed")

    o3d.io.write_triangle_mesh(str(obj_path), mesh, write_triangle_uvs=False)
    run_navmap_export(obj_path, navmap_path, args.surface_name)

    print(f"Wrote point cloud: {ply_path}", flush=True)
    print(f"Wrote mesh: {obj_path}", flush=True)
    print(f"Wrote NavMap: {navmap_path}", flush=True)

if __name__ == "__main__":
    main()
