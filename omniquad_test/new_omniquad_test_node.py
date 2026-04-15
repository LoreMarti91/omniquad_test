import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import SetBool
from pi3hat_moteus_int_msgs.msg import Counter

from new_trajectory import NewTrajectory

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from datetime import datetime
import math


class OmniquadTest(Node):

    def __init__(self,
                 node_name="omniquad_test_node",
                 period=1/450,
                 test_name='prova',
                 try_test=3,
                 height_target=0.03,
                 Vel_linear=0.5,
                 Vel_angolar=1.0,
                 t_acc=1.0,
                 t_holding_vel=2.0,
                 t_dec=1.0):

        super().__init__(node_name)

        # ─────────────────────────────
        # TRAJECTORY ENGINE
        # ─────────────────────────────
        self.traj = NewTrajectory()

        # ─────────────────────────────
        # TIMING
        # ─────────────────────────────
        self.period = period
        self.t_acc = t_acc
        self.t_holding_vel = t_acc + t_holding_vel
        self.t_dec = t_acc + t_holding_vel + t_dec

        self.height_target = height_target
        self.Vel_linear = Vel_linear
        self.Vel_angolar = Vel_angolar

        # ─────────────────────────────
        # PUBLISHERS
        # ─────────────────────────────
        twist_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_twist = self.create_publisher(
            Twist, '/omni_controller/twist_cmd', twist_qos)

        self.pub_pose = self.create_publisher(
            Pose, '/ik_controller/base_pose', 1)

        self.pub_count = self.create_publisher(
            Counter, 'Counter', 10)

        # ─────────────────────────────
        # SERVICES
        # ─────────────────────────────
        self.stand_client = self.create_client(SetBool, '/omni_controller/stand_srv')

        # ─────────────────────────────
        # STATE MACHINE GET-UP
        # ─────────────────────────────
        self.getup_state = 0
        self.getup_t0 = None

        self.counter_move = 1
        self.try_test = try_test

        self.start_node = self.time_now()

        # ─────────────────────────────
        # TIMER
        # ─────────────────────────────
        self.timer = self.create_timer(period, self.homing_callback)

    # ═════════════════════════════════════
    # TIME
    # ═════════════════════════════════════
    def time_now(self):
        return self.get_clock().now()

    def time_s(self, t):
        sec, ns = t.seconds_nanoseconds()
        return sec + ns * 1e-9

    def dt(self, t0):
        return self.time_s(self.time_now()) - self.time_s(t0)

    # ═════════════════════════════════════
    # PUBLISH SAFE
    # ═════════════════════════════════════
    def publish_zero(self):
        self.publish_twist(0.0, 0.0, 0.0)
        # NOT publishing pose intentionally

    def publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub_twist.publish(msg)

    def publish_pose(self, h):
        msg = Pose()
        msg.position.z = float(h)
        msg.orientation.w = 1.0
        self.pub_pose.publish(msg)

    # ═════════════════════════════════════
    # SERVICE CALL
    # ═════════════════════════════════════
    def call_service(self, client, value: bool):
        if client.service_is_ready():
            req = SetBool.Request()
            req.data = value
            client.call_async(req)

    # ═════════════════════════════════════
    # GET-UP SEQUENCE
    # ═════════════════════════════════════
    def homing_callback(self):

        # ─────────────────────────
        # STATE 0: SILENT 2s
        # ─────────────────────────
        if self.getup_state == 0:

            if self.getup_t0 is None:
                self.getup_t0 = self.time_now()

            self.publish_zero()

            if self.dt(self.getup_t0) > 2.0:
                self.getup_state = 1
            return

        # ─────────────────────────
        # STATE 1: HOMING SERVICE
        # ─────────────────────────
        elif self.getup_state == 1:

            self.get_logger().info("[GET-UP] homing call")

            self.call_service(self.stand_client, True)

            self.getup_t0 = self.time_now()
            self.getup_state = 2
            return

        # ─────────────────────────
        # STATE 2: WAIT 5s
        # ─────────────────────────
        elif self.getup_state == 2:

            self.publish_zero()

            if self.dt(self.getup_t0) > 5.0:
                self.getup_state = 3
            return

        # ─────────────────────────
        # STATE 3: START TEST
        # ─────────────────────────
        elif self.getup_state == 3:

            self.get_logger().info("[GET-UP] DONE → START TEST")

            self.start_node = self.time_now()

            self.timer.destroy()
            self.timer = self.create_timer(
                self.period,
                self.timer_callback_iteration
            )

    # ═════════════════════════════════════
    # TEST LOOP
    # ═════════════════════════════════════
    def timer_callback_iteration(self):

        self.timer_callback_movements()

    def timer_callback_movements(self):

        moves = {
            1: self.move_forward,
            2: self.move_backward,
            3: self.move_left,
            4: self.move_right,
            5: self.rotate_ccw,
            6: self.rotate_cw
        }

        if self.counter_move in moves:
            moves[self.counter_move]()
        else:
            self.counter_move = 1

    # ═════════════════════════════════════
    # MOTION EXECUTION
    # ═════════════════════════════════════
    def exec_move(self, vx_i, vx_f, vy_i, vy_f, w_i, w_f):

        t = self.dt(self.start_node)

        # ACC
        if t < self.t_acc:

            vx = self.traj.linear([vx_i], [vx_f], 0, self.t_acc, t)[0]
            vy = self.traj.linear([vy_i], [vy_f], 0, self.t_acc, t)[0]
            w  = self.traj.linear([w_i],  [w_f],  0, self.t_acc, t)[0]

        # HOLD
        elif t < self.t_holding_vel:

            vx, vy, w = vx_f, vy_f, w_f

        # DEC
        elif t < self.t_dec:

            vx = self.traj.linear([vx_f], [0.0], self.t_holding_vel, self.t_dec, t)[0]
            vy = self.traj.linear([vy_f], [0.0], self.t_holding_vel, self.t_dec, t)[0]
            w  = self.traj.linear([w_f],  [0.0], self.t_holding_vel, self.t_dec, t)[0]

        else:
            self.counter_move += 1
            self.start_node = self.time_now()
            return

        self.publish_twist(vx, vy, w)
        self.publish_pose(self.height_target)

    # ═════════════════════════════════════
    # MOVES
    # ═════════════════════════════════════
    def move_forward(self):
        self.exec_move(0, self.Vel_linear, 0, 0, 0, 0)

    def move_backward(self):
        self.exec_move(0, -self.Vel_linear, 0, 0, 0, 0)

    def move_left(self):
        self.exec_move(0, 0, 0, self.Vel_linear, 0, 0)

    def move_right(self):
        self.exec_move(0, 0, 0, -self.Vel_linear, 0, 0)

    def rotate_ccw(self):
        self.exec_move(0, 0, 0, 0, 0, self.Vel_angolar)

    def rotate_cw(self):
        self.exec_move(0, 0, 0, 0, 0, -self.Vel_angolar)


# ═════════════════════════════════════
# MAIN
# ═════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    node = OmniquadTest(period=1/450)

    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()