#!/usr/bin/env python3

import time
import numpy as np

from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


# ============================================================
# 1. POINTCLOUD2 DECODING
# ============================================================

def decode_xyz(msg):

    fields = {field.name: field for field in msg.fields}

    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError(
            "PointCloud2 message missing x, y, or z fields"
        )

    for name in ("x", "y", "z"):
        if fields[name].datatype != PointField.FLOAT32:
            raise ValueError(
                f"Expected FLOAT32 for {name}"
            )

    endian = ">" if msg.is_bigendian else "<"

    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + "f4"] * 3,
        "offsets": [
            fields["x"].offset,
            fields["y"].offset,
            fields["z"].offset,
        ],
        "itemsize": msg.point_step,
    })

    raw = memoryview(msg.data)

    rows = []

    for row in range(msg.height):

        start = row * msg.row_step

        row_points = np.frombuffer(
            raw,
            dtype=dtype,
            count=msg.width,
            offset=start,
        )

        xyz = np.column_stack((
            row_points["x"],
            row_points["y"],
            row_points["z"],
        ))

        rows.append(xyz)

    if not rows:
        return np.empty((0, 3), dtype=np.float32)

    points = np.concatenate(rows, axis=0)

    return points[
        np.isfinite(points).all(axis=1)
    ]


# ============================================================
# 2. VOXEL DOWNSAMPLING
# ============================================================

def voxel_downsample(points, voxel_size):

    if len(points) == 0 or voxel_size <= 0:
        return points

    cells = np.floor(
        points / voxel_size
    ).astype(np.int32)

    _, inverse = np.unique(
        cells,
        axis=0,
        return_inverse=True,
    )

    counts = np.bincount(inverse)

    result = np.empty(
        (len(counts), 3),
        dtype=np.float32,
    )

    for axis in range(3):

        sums = np.bincount(
            inverse,
            weights=points[:, axis],
        )

        result[:, axis] = sums / counts

    return result


# ============================================================
# 3. RANSAC FLOOR REMOVAL
# ============================================================

def remove_floor(
    points,
    z_tol=0.025,
    iterations=80,
    min_alignment=0.9,
    max_sample_points=15000,
):

    if len(points) < 3:
        return points

    rng = np.random.default_rng()

    if len(points) > max_sample_points:

        indices = rng.choice(
            len(points),
            max_sample_points,
            replace=False,
        )

        sample_cloud = points[indices]

    else:
        sample_cloud = points

    best_count = 0
    best_normal = None
    best_origin = None

    for _ in range(iterations):

        sample = sample_cloud[
            rng.choice(
                len(sample_cloud),
                3,
                replace=False,
            )
        ]

        normal = np.cross(
            sample[1] - sample[0],
            sample[2] - sample[0],
        )

        norm = np.linalg.norm(normal)

        if norm < 1e-8:
            continue

        normal = normal / norm

        # Floor is approximately horizontal.
        if abs(normal[2]) < min_alignment:
            continue

        distances = np.abs(
            (sample_cloud - sample[0]) @ normal
        )

        count = np.count_nonzero(
            distances < z_tol
        )

        if count > best_count:

            best_count = count
            best_normal = normal.copy()
            best_origin = sample[0].copy()

    if best_normal is None:
        return points

    minimum_inliers = max(
        30,
        int(0.15 * len(sample_cloud)),
    )

    if best_count < minimum_inliers:
        return points

    distances = np.abs(
        (points - best_origin) @ best_normal
    )

    return points[distances >= z_tol]


# ============================================================
# 4. EUCLIDEAN CLUSTERING
# ============================================================

def euclidean_clusters(
    points,
    radius=0.10,
    min_points=12,
    max_points=5000,
    max_edges=1500000,
):

    if len(points) == 0:
        return []

    tree = cKDTree(points)

    pairs = tree.query_pairs(
        radius,
        output_type="ndarray",
    )

    if len(pairs) > max_edges:
        raise RuntimeError(
            f"Clustering graph too dense: "
            f"{len(pairs)} edges"
        )

    if len(pairs) == 0:
        return []

    n = len(points)

    row = np.concatenate((
        pairs[:, 0],
        pairs[:, 1],
    ))

    col = np.concatenate((
        pairs[:, 1],
        pairs[:, 0],
    ))

    data = np.ones(
        len(row),
        dtype=np.uint8,
    )

    graph = coo_matrix(
        (data, (row, col)),
        shape=(n, n),
    ).tocsr()

    count, labels = connected_components(
        graph,
        directed=False,
    )

    sizes = np.bincount(
        labels,
        minlength=count,
    )

    valid_labels = np.where(
        (sizes >= min_points) &
        (sizes <= max_points)
    )[0]

    clusters = [
        points[labels == label]
        for label in valid_labels
    ]

    return clusters


# ============================================================
# 5. VERTICAL CYLINDER FITTING
# ============================================================

def fit_vertical_cylinder(
    points,
    min_radius=0.15,
    max_radius=0.35,
    min_height=0.20,
    max_height=1.20,
    max_circle_error=0.025,
    min_inlier_fraction=0.65,
):

    if len(points) < 3:
        return None, "Rejected: too few points"

    # Cylinders are upright in the base_link frame.
    # Fit a circle to the X-Y projection.

    xy = points[:, :2].astype(np.float64)

    origin = np.mean(xy, axis=0)

    centered = xy - origin

    A = np.column_stack((
        2.0 * centered[:, 0],
        2.0 * centered[:, 1],
        np.ones(len(centered)),
    ))

    b = np.sum(
        centered ** 2,
        axis=1,
    )

    params, _, rank, _ = np.linalg.lstsq(
        A,
        b,
        rcond=None,
    )

    if rank < 3:
        return None, "Rejected: circle fit rank deficient"

    local_cx, local_cy, c = params

    r2 = (
        c +
        local_cx ** 2 +
        local_cy ** 2
    )

    if r2 <= 0:
        return None, "Rejected: invalid radius"

    radius = float(np.sqrt(r2))

    center = origin + np.array([
        local_cx,
        local_cy,
    ])

    distances = np.linalg.norm(
        xy - center,
        axis=1,
    )

    residuals = np.abs(
        distances - radius
    )

    median_error = float(
        np.median(residuals)
    )

    inlier_fraction = float(
        np.mean(residuals < 0.035)
    )

    z_low = float(
        np.percentile(points[:, 2], 5)
    )

    z_high = float(
        np.percentile(points[:, 2], 95)
    )

    height = z_high - z_low

    cz = (z_low + z_high) / 2.0

    stats = (
        f"radius={radius:.3f}m, "
        f"height={height:.3f}m, "
        f"circle_error={median_error:.4f}m, "
        f"inliers={inlier_fraction:.2f}"
    )

    # Radius validation
    if not min_radius <= radius <= max_radius:

        return (
            None,
            stats + " | REJECTED: radius",
        )

    # Height validation
    if not min_height <= height <= max_height:

        return (
            None,
            stats + " | REJECTED: height",
        )

    # Circle fit quality
    if median_error > max_circle_error:

        return (
            None,
            stats + " | REJECTED: circle error",
        )

    # Required fraction of fitted points
    if inlier_fraction < min_inlier_fraction:

        return (
            None,
            stats + " | REJECTED: insufficient inliers",
        )

    result = (
        float(center[0]),
        float(center[1]),
        float(cz),
        radius,
        height,
    )

    return result, stats + " | ACCEPTED"


# ============================================================
# 6. ROS 2 CYLINDER DETECTOR NODE
# ============================================================

class CylinderDetector(Node):

    def __init__(self):

        super().__init__("cylinder_detector")

        # ----------------------------------------
        # Parameters
        # ----------------------------------------

        self.declare_parameter(
            "target_frame",
            "base_link",
        )

        self.declare_parameter(
            "voxel_size",
            0.01,
        )

        self.declare_parameter(
            "cluster_radius",
            0.10,
        )

        self.declare_parameter(
            "min_cluster_points",
            12,
        )

        self.declare_parameter(
            "max_processing_points",
            30000,
        )

        self.declare_parameter(
            "min_range",
            0.15,
        )

        self.declare_parameter(
            "max_range",
            40.0,
        )

        self.declare_parameter(
            "min_height",
            -0.25,
        )

        self.declare_parameter(
            "max_height",
            1.5,
        )

        self.target_frame = self.get_parameter(
            "target_frame"
        ).value

        # ----------------------------------------
        # TF listener
        # ----------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ----------------------------------------
        # Detection publisher
        # ----------------------------------------

        self.publisher = self.create_publisher(
            MarkerArray,
            "/viz/detections",
            10,
        )

        # ----------------------------------------
        # Point cloud subscriber
        # ----------------------------------------

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/camera/points",
            self.on_cloud,
            cloud_qos,
        )

        self.last_count = -1

        self.get_logger().info(
            "Cylinder detector started. "
            "Expected cylinder radius: 0.25m, "
            "height: 1.0m. "
            "Waiting for /camera/points."
        )

    # ========================================================
    # 7. POINT CLOUD CALLBACK
    # ========================================================

    def on_cloud(self, msg):

        start_time = time.perf_counter()

        # ----------------------------------------
        # STEP 1: TRANSFORM LOOKUP
        # ----------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(
                    msg.header.stamp
                ),
            )

        except TransformException as exc:

            self.get_logger().warn(
                f"TF unavailable: {exc}",
                throttle_duration_sec=5.0,
            )

            return

        # ----------------------------------------
        # STEP 2: DECODE CLOUD
        # ----------------------------------------

        try:

            points = decode_xyz(msg)

        except (ValueError, TypeError) as exc:

            self.get_logger().error(
                f"Cannot decode point cloud: {exc}"
            )

            return

        decoded_count = len(points)

        if decoded_count == 0:

            self.publish_markers([], msg)
            return

        # ----------------------------------------
        # STEP 3: CAMERA -> BASE_LINK
        # ----------------------------------------

        t = transform.transform.translation
        q = transform.transform.rotation

        R = Rotation.from_quat([
            q.x,
            q.y,
            q.z,
            q.w,
        ]).as_matrix()

        translation = np.array([
            t.x,
            t.y,
            t.z,
        ])

        points = points @ R.T + translation

        # ----------------------------------------
        # STEP 4: REGION OF INTEREST
        # ----------------------------------------

        dist = np.linalg.norm(
            points[:, :2],
            axis=1,
        )

        min_range = float(
            self.get_parameter("min_range").value
        )

        max_range = float(
            self.get_parameter("max_range").value
        )

        min_height = float(
            self.get_parameter("min_height").value
        )

        max_height = float(
            self.get_parameter("max_height").value
        )

        mask = (
            (dist > min_range) &
            (dist < max_range) &
            (points[:, 2] > min_height) &
            (points[:, 2] < max_height)
        )

        points = points[mask]

        roi_count = len(points)

        if roi_count < 12:

            self.get_logger().warn(
                f"Too few ROI points: {roi_count}",
                throttle_duration_sec=5.0,
            )

            self.publish_markers([], msg)
            return

        # ----------------------------------------
        # STEP 5: VOXEL DOWNSAMPLING
        # ----------------------------------------

        voxel_size = float(
            self.get_parameter("voxel_size").value
        )

        points = voxel_downsample(
            points,
            voxel_size,
        )

        voxel_count = len(points)

        # ----------------------------------------
        # STEP 6: REMOVE FLOOR
        # ----------------------------------------

        points = remove_floor(points)

        floor_count = len(points)

        # ----------------------------------------
        # STEP 7: LIMIT POINT COUNT
        # ----------------------------------------

        max_points = int(
            self.get_parameter(
                "max_processing_points"
            ).value
        )

        if max_points > 0 and len(points) > max_points:

            rng = np.random.default_rng(42)

            indices = rng.choice(
                len(points),
                max_points,
                replace=False,
            )

            points = points[indices]

        # ----------------------------------------
        # STEP 8: CLUSTERING
        # ----------------------------------------

        cluster_radius = float(
            self.get_parameter(
                "cluster_radius"
            ).value
        )

        min_cluster_points = int(
            self.get_parameter(
                "min_cluster_points"
            ).value
        )

        try:

            cluster_list = euclidean_clusters(
                points,
                radius=cluster_radius,
                min_points=min_cluster_points,
            )

        except RuntimeError as exc:

            self.get_logger().warn(
                str(exc),
                throttle_duration_sec=5.0,
            )

            self.publish_markers([], msg)
            return

        # ----------------------------------------
        # STEP 9: FIT CYLINDERS
        # ----------------------------------------

        detected = []

        for i, cluster in enumerate(cluster_list):

            try:

                result, diagnostic = fit_vertical_cylinder(
                    cluster
                )

            except np.linalg.LinAlgError as exc:

                self.get_logger().warn(
                    f"Cluster {i}: fit failed: {exc}",
                    throttle_duration_sec=5.0,
                )

                continue

            self.get_logger().info(
                f"Cluster {i}: "
                f"{len(cluster)} points, "
                f"{diagnostic}",
                throttle_duration_sec=5.0,
            )

            if result is not None:
                detected.append(result)

        # ----------------------------------------
        # STEP 10: PUBLISH MARKERS
        # ----------------------------------------

        self.publish_markers(
            detected,
            msg,
        )

        elapsed = (
            time.perf_counter() - start_time
        )

        self.get_logger().info(
            f"Cloud statistics: "
            f"decoded={decoded_count}, "
            f"ROI={roi_count}, "
            f"voxel={voxel_count}, "
            f"after_floor={floor_count}, "
            f"processing={len(points)}, "
            f"clusters={len(cluster_list)}, "
            f"cylinders={len(detected)}, "
            f"time={elapsed:.3f}s",
            throttle_duration_sec=5.0,
        )

    # ========================================================
    # 8. PUBLISH RVIZ MARKERS
    # ========================================================

    def publish_markers(self, detected, msg):

        array = MarkerArray()

        # Clear markers from previous cloud.
        clear = Marker()

        clear.header.frame_id = self.target_frame
        clear.header.stamp = msg.header.stamp

        clear.action = Marker.DELETEALL

        array.markers.append(clear)

        # Create a marker for every detected cylinder.
        for i, (cx, cy, cz, radius, height) in enumerate(detected):

            marker = Marker()

            marker.header.frame_id = self.target_frame
            marker.header.stamp = msg.header.stamp

            marker.ns = "cylinders"
            marker.id = i

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz

            marker.pose.orientation.w = 1.0

            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = height

            # Green detection markers.
            # This is visualization only.
            # Detection does NOT depend on cylinder color.
            marker.color.r = 0.15
            marker.color.g = 0.9
            marker.color.b = 0.25
            marker.color.a = 0.85

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000

            array.markers.append(marker)

        self.publisher.publish(array)

        if len(detected) != self.last_count:

            self.get_logger().info(
                f"Detected {len(detected)} candidate cylinders"
            )

            self.last_count = len(detected)


# ============================================================
# 9. MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = CylinderDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
