import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog


class SlaveFollower(Node):

    def __init__(self):

        super().__init__("slave")

        self.Kp = 1.0
        self.Kv = 0.8

        self.vmax = 3.0

        self.q_master = None
        self.qdot_master = None

        self.q_slave = None
        self.qdot_slave = None

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        self.sub_master = self.create_subscription(
            JointState,
            "/master/joint_states",
            self.master_callback,
            10
        )

        self.sub_slave = self.create_subscription(
            JointState,
            "/slave/joint_states",
            self.slave_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointJog,
            "/servo_server/delta_joint_cmds",
            10
        )

        self.timer = self.create_timer(0.005, self.compute_control)

    def master_callback(self, msg):

        self.q_master = list(msg.position)
        self.qdot_master = list(msg.velocity)

    def slave_callback(self, msg):

        self.q_slave = list(msg.position)
        self.qdot_slave = list(msg.velocity)

    def compute_control(self):

        if self.q_master is None:
            return

        if self.q_slave is None:
            return

        if self.qdot_master is None:
            return

        if self.qdot_slave is None:
            return

        qdot_cmd = []

        for i in range(6):

            vel = (
                self.Kv * self.qdot_master[i]
                + self.Kp * (self.q_master[i] - self.q_slave[i])
            )

            vel = max(min(vel, self.vmax), -self.vmax)

            qdot_cmd.append(vel)

        self.publish_joint_velocity(qdot_cmd)

    def publish_joint_velocity(self, qdot):

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"

        msg.joint_names = self.joint_names
        msg.velocities = qdot

        msg.duration = 0.005

        self.joint_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SlaveFollower()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()