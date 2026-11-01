"""Mozgásvezérlő: a /glue_map kép alapján ragasztópályát tervez és hajt végre.

Lépések:
  1. megvárja a mock szenzor képét,
  2. soronként megkeresi a fekete szakaszokat (ragasztócsíkokat),
  3. a robotot kezdőpózba állítja (joint_trajectory_controller),
  4. minden csíkra Cartesian pályát kér a MoveIt-tól (/compute_cartesian_path),
     és végrehajtja (/execute_trajectory),
  5. RViz-markerekkel mutatja a lapot, a tervezett és a kész csíkokat.
"""

import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from ragasztas.glue_path import cell_center, find_strokes, image_to_grid

JOINT_NAMES = [f'joint_{i}' for i in range(1, 7)]
GROUP = 'manipulator'
TCP_LINK = 'tool0'
# tool0 z tengelye függőlegesen lefelé (180°-os forgatás az y tengely körül).
TOOL_DOWN = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)


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
        self.ready_joints = list(p('ready_joints', [0.0] * 6).value)
        self.ready_duration = p('ready_duration', 4.0).value
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
        self.joint_cli = ActionClient(
            self, FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory')

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
        return Pose(position=point, orientation=TOOL_DOWN)

    # ----------------------------------------------------------------- lépések
    def wait_for_interfaces(self):
        self.get_logger().info('Várakozás a /glue_map képre és a MoveIt / controller interfészekre...')
        while rclpy.ok() and self.image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.cartesian_cli.wait_for_service()
        self.execute_cli.wait_for_server()
        self.joint_cli.wait_for_server()

    def move_to_ready(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        sec = int(self.ready_duration)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=self.ready_joints,
            time_from_start=Duration(sec=sec, nanosec=int((self.ready_duration - sec) * 1e9)))]
        result = self._run_action(self.joint_cli, goal)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Kezdőpózba állás sikertelen: {result.error_string}')

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
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
