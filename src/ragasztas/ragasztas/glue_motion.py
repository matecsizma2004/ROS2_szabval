"""Mozgásvezérlő: a /glue_map kép alapján ragasztópályát tervez és hajt végre.

Lépések:
  1. megvárja a mock szenzor képét,
  2. soronként megkeresi a fekete szakaszokat (ragasztócsíkokat),
  3. a robotot a konfigban megadott Descartes-kezdőpózba állítja: MoveIt /move_action,
     Pilz PTP tervező (az IK-t és a csuklópályát a MoveIt számolja),
  4. minden csíkra Cartesian pályát kér a MoveIt-tól (/compute_cartesian_path),
     és végrehajtja (/execute_trajectory),
  5. RViz-markerekkel mutatja a lapot, a tervezett és a kész csíkokat.
"""

import time

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetCartesianPath
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ragasztas.glue_path import (cell_center, find_strokes, image_to_grid,
                                 rpy_deg_to_quaternion)

GROUP = 'manipulator'
TCP_LINK = 'tool0'
PTP_PIPELINE = 'pilz_industrial_motion_planner'
PTP_PLANNER = 'PTP'


class GlueMotion(Node):

    def __init__(self):
        super().__init__('glue_motion')
        p = self.declare_parameter
        self.frame_id = p('frame_id', 'world').value
        self.cell = p('cell_size', 0.02).value
        self.sheet_x = p('sheet_x', 0.82).value
        self.sheet_y = p('sheet_y', 0.10).value
        self.sheet_z = p('sheet_z', 0.0).value
        self.glue_z = self.sheet_z + p('glue_height', 0.005).value
        self.lift_z = self.glue_z + p('lift_height', 0.05).value
        rpy = list(p('tool_rpy_deg', [0.0, 180.0, 0.0]).value)
        self.tool_orientation = Quaternion(**dict(zip('xyzw', rpy_deg_to_quaternion(*rpy))))
        self.ready_point = Point(**dict(zip('xyz', p('ready_position', [0.70, 0.0, 0.10]).value)))
        self.max_step = p('max_step', 0.005).value
        self.vel_scaling = p('velocity_scaling', 0.2).value
        self.acc_scaling = p('acceleration_scaling', 0.2).value

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.image = None
        self.create_subscription(Image, 'glue_map', self._on_image, latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'glue_markers', latched)

        self.cartesian_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_cli = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.move_cli = ActionClient(self, MoveGroup, 'move_action')

    # ------------------------------------------------------------------ utils
    def _on_image(self, msg):
        self.image = msg

    def _wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _run_action(self, client, goal):
        handle = self._wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Az action célt elutasították.')
        return self._wait(handle.get_result_async()).result

    def _point(self, row, col, z):
        return Point(**dict(zip('xyz', cell_center(
            row, col, self.sheet_x, self.sheet_y, z, self.cell))))

    def _pose(self, point):
        return Pose(position=point, orientation=self.tool_orientation)

    # ----------------------------------------------------------------- lépések
    def wait_for_interfaces(self):
        self.get_logger().info('Várakozás a /glue_map képre és a MoveIt / controller interfészekre...')
        waits = [
            ('/glue_map kép (fut a glue_map_sensor?)', lambda: self.image is not None),
            ('/compute_cartesian_path (move_group)', self.cartesian_cli.service_is_ready),
            ('/execute_trajectory (move_group)', self.execute_cli.server_is_ready),
            ('/move_action (move_group)', self.move_cli.server_is_ready),
        ]
        start = time.monotonic()
        last_warn = start
        while rclpy.ok():
            missing = [name for name, ready in waits if not ready()]
            if not missing:
                return
            if time.monotonic() - last_warn > 5.0:
                last_warn = time.monotonic()
                self.get_logger().warn(
                    f'{last_warn - start:.0f} s óta várok erre: {", ".join(missing)}')
            rclpy.spin_once(self, timeout_sec=0.2)

    def move_to_pose(self, pose):
        """PTP mozgás egy Descartes-pózba: az IK-t és a pályát a MoveIt (Pilz PTP) számolja."""
        position = PositionConstraint(link_name=TCP_LINK, weight=1.0)
        position.header.frame_id = self.frame_id
        position.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])],
            primitive_poses=[Pose(position=pose.position, orientation=Quaternion(w=1.0))])
        orientation = OrientationConstraint(
            link_name=TCP_LINK, orientation=pose.orientation, weight=1.0,
            absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
            absolute_z_axis_tolerance=0.01)
        orientation.header.frame_id = self.frame_id

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.pipeline_id = PTP_PIPELINE
        req.planner_id = PTP_PLANNER
        req.start_state.is_diff = True
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel_scaling
        req.max_acceleration_scaling_factor = self.acc_scaling
        req.goal_constraints = [Constraints(position_constraints=[position],
                                            orientation_constraints=[orientation])]
        goal.planning_options.plan_only = False  # a move_group végre is hajtja

        result = self._run_action(self.move_cli, goal)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'PTP mozgás sikertelen ({pose.position.x:.3f}, {pose.position.y:.3f}, '
                f'{pose.position.z:.3f}), hibakód: {result.error_code.val}. '
                'Elérhető a pont ezzel az orientációval?')

    def move_to_ready(self):
        self.move_to_pose(self._pose(self.ready_point))

    def plan_stroke(self, stroke):
        row, c0, c1 = stroke
        waypoints = [self._pose(self._point(row, c0, self.lift_z))]
        waypoints += [self._pose(self._point(row, c, self.glue_z)) for c in range(c0, c1 + 1)]
        waypoints.append(self._pose(self._point(row, c1, self.lift_z)))

        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame_id
        req.start_state.is_diff = True
        req.group_name = GROUP
        req.link_name = TCP_LINK
        req.waypoints = waypoints
        req.max_step = self.max_step
        req.avoid_collisions = True
        # Jazzy-ban a szolgáltatás maga időparaméterezi a pályát ezekkel.
        if hasattr(req, 'max_velocity_scaling_factor'):
            req.max_velocity_scaling_factor = self.vel_scaling
            req.max_acceleration_scaling_factor = self.acc_scaling
        res = self._wait(self.cartesian_cli.call_async(req))
        if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < 1.0:
            raise RuntimeError(
                f'Cartesian tervezés sikertelen ({stroke}): fraction={res.fraction:.3f}, '
                f'hibakód={res.error_code.val}. Ellenőrizd a lap pozícióját!')
        return res.solution, res.fraction

    def execute(self, trajectory):
        result = self._run_action(self.execute_cli, ExecuteTrajectory.Goal(trajectory=trajectory))
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Végrehajtás sikertelen, hibakód: {result.error_code.val}')

    # ---------------------------------------------------------------- markerek
    def publish_markers(self, grid, strokes, done):
        def marker(mid, ns, mtype, rgba, scale):
            m = Marker(id=mid, ns=ns, type=mtype, action=Marker.ADD)
            m.header.frame_id = self.frame_id
            m.pose.orientation.w = 1.0
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.scale.x, m.scale.y, m.scale.z = scale
            return m

        rows, cols = len(grid), len(grid[0])
        # A4-es lap (29.7 x 21 cm), középen a rács.
        sheet = marker(0, 'lap', Marker.CUBE, (1.0, 1.0, 1.0, 1.0), (0.297, 0.21, 0.001))
        sheet.pose.position = Point(x=self.sheet_x - rows * self.cell / 2,
                                    y=self.sheet_y - cols * self.cell / 2,
                                    z=self.sheet_z - 0.0005)

        cells = marker(1, 'fekete_cellak', Marker.CUBE_LIST, (0.05, 0.05, 0.05, 1.0),
                       (self.cell * 0.95, self.cell * 0.95, 0.001))
        cells.points = [self._point(r, c, self.sheet_z + 0.0005)
                        for r in range(rows) for c in range(cols) if grid[r][c]]

        def lines(mid, ns, rgba, items, width):
            m = marker(mid, ns, Marker.LINE_LIST, rgba, (width, 0.0, 0.0))
            for r, c0, c1 in items:
                m.points += [self._point(r, c0, self.glue_z), self._point(r, c1, self.glue_z)]
            return m

        planned = lines(2, 'tervezett_csikok', (0.2, 0.4, 1.0, 0.8), strokes, 0.003)
        finished = lines(3, 'kesz_csikok', (0.0, 0.8, 0.0, 1.0), strokes[:done], 0.008)
        self.marker_pub.publish(MarkerArray(markers=[sheet, cells, planned, finished]))

    # ------------------------------------------------------------------- futás
    def run(self):
        self.wait_for_interfaces()
        img = self.image
        if img.encoding != 'mono8':
            raise RuntimeError(f'Nem támogatott képkódolás: {img.encoding}')
        grid = image_to_grid(bytes(img.data), img.width, img.height)
        strokes = find_strokes(grid)
        self.get_logger().info(f'{img.width}x{img.height} kép, {len(strokes)} ragasztócsík: {strokes}')
        self.publish_markers(grid, strokes, 0)

        self.get_logger().info('Kezdőpózba állás...')
        self.move_to_ready()

        t0 = time.monotonic()
        for i, stroke in enumerate(strokes):
            trajectory, fraction = self.plan_stroke(stroke)
            self.get_logger().info(
                f'[{i + 1}/{len(strokes)}] {stroke[0]}. sor, {stroke[1]}-{stroke[2]}. oszlop '
                f'(fraction={fraction:.2f}, {len(trajectory.joint_trajectory.points)} pont)')
            self.execute(trajectory)
            self.publish_markers(grid, strokes, i + 1)
        elapsed = time.monotonic() - t0

        self.get_logger().info('Vissza a kezdőpózba...')
        self.move_to_ready()
        self.get_logger().info(f'Kész: {len(strokes)} csík, ragasztási ciklusidő {elapsed:.1f} s.')


def main():
    rclpy.init()
    node = GlueMotion()
    try:
        node.run()
        # Tovább fut, hogy a (latched) markerek később induló RViz-ben is látszódjanak.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C / launch leállítás
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
