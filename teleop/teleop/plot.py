import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from collections import deque
import threading
import time


class ForceTorquePlotter(Node):

    def __init__(self):
        super().__init__('plot')

        self.window_time = 5.0
        self.max_samples = 2000

        # TOPICS
        self.force_topic = '/force_esp32'
        self.master_topic = '/master/joint_states'
        self.slave_topic = '/slave/joint_states'

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # SUBSCRIBERS
        self.force_sub = self.create_subscription(
            Float32,
            self.force_topic,
            self.force_callback,
            qos
        )

        self.master_sub = self.create_subscription(
            JointState,
            self.master_topic,
            self.master_callback,
            qos
        )

        self.slave_sub = self.create_subscription(
            JointState,
            self.slave_topic,
            self.slave_callback,
            qos
        )

        # TIME
        self.start_time = time.time()
        self.time_buffer = deque(maxlen=self.max_samples)

        # FORCE
        self.fz_buffer = deque(maxlen=self.max_samples)
        self.last_fz = 0.0

        # TORQUES
        self.tau_buffers = [deque(maxlen=self.max_samples) for _ in range(6)]
        self.last_tau = [0.0]*6

        # POSITIONS
        self.last_qm = [0.0]*6
        self.last_qs = [0.0]*6

        # ERROR
        self.error_buffers = [deque(maxlen=self.max_samples) for _ in range(6)]

        self.data_lock = threading.Lock()

        self.setup_plots()

    def current_time(self):
        return time.time() - self.start_time

    def force_callback(self, msg):

        with self.data_lock:

            self.last_fz = float(msg.data)

            t = self.current_time()

            self.time_buffer.append(t)
            self.fz_buffer.append(self.last_fz)

            for i in range(6):
                self.tau_buffers[i].append(self.last_tau[i])

            for i in range(6):
                error = self.last_qm[i] - self.last_qs[i]
                self.error_buffers[i].append(error)

    def master_callback(self, msg):

        with self.data_lock:

            for i in range(min(6,len(msg.position))):
                self.last_qm[i] = msg.position[i]

    def slave_callback(self, msg):

        with self.data_lock:

            for i in range(min(6,len(msg.position))):
                self.last_qs[i] = msg.position[i]

            for i in range(min(6,len(msg.effort))):
                self.last_tau[i] = msg.effort[i]

    def setup_plots(self):

        self.fig = plt.figure(figsize=(12,10), facecolor='#0a0a0f')

        self.fig.suptitle(
            'ROS2 | Fuerza, Torques y Error Teleoperación',
            color='white',
            fontsize=14,
            fontweight='bold'
        )

        bg = '#0d1117'

        self.ax_force = self.fig.add_subplot(3,1,1, facecolor=bg)
        self.ax_tau = self.fig.add_subplot(3,1,2, facecolor=bg)
        self.ax_error = self.fig.add_subplot(3,1,3, facecolor=bg)

        # hacer visibles los números
        for ax in [self.ax_force, self.ax_tau, self.ax_error]:

            ax.tick_params(axis='x', colors='white', labelsize=10)
            ax.tick_params(axis='y', colors='white', labelsize=10)

            for spine in ax.spines.values():
                spine.set_edgecolor('#333')

            ax.grid(True, color='#1e2530', linestyle='--', alpha=0.5)

        colors = [
            '#00FFD0',
            '#FF6B6B',
            '#69FF47',
            '#00BFFF',
            '#FFA500',
            '#A020F0',
            '#FFD700'
        ]

        # FORCE
        self.ax_force.set_title('Fuerza en Z [N]', color='white')
        self.ax_force.set_xlabel('Tiempo [s]', color='white')
        self.ax_force.set_ylabel('Fuerza [N]', color='white')

        self.line_fz, = self.ax_force.plot([],[],color=colors[0],linewidth=2,label='Fz')
        self.ax_force.legend()

        # TORQUES
        self.ax_tau.set_title('Torques Articulares [Nm]', color='white')
        self.ax_tau.set_xlabel('Tiempo [s]', color='white')
        self.ax_tau.set_ylabel('Torque [Nm]', color='white')

        self.torque_lines = []

        for i in range(6):
            line, = self.ax_tau.plot([],[],label=f'tau{i+1}',color=colors[(i+1)%7])
            self.torque_lines.append(line)

        self.ax_tau.legend()

        # ERROR
        self.ax_error.set_title('Error Maestro - Esclavo [rad]', color='white')
        self.ax_error.set_xlabel('Tiempo [s]', color='white')
        self.ax_error.set_ylabel('Error [rad]', color='white')

        self.error_lines = []

        for i in range(6):
            line, = self.ax_error.plot([],[],label=f'e{i+1}',color=colors[(i+2)%7])
            self.error_lines.append(line)

        self.ax_error.legend()

        plt.tight_layout(rect=[0,0.02,1,0.96])

    def animate(self, frame):

        with self.data_lock:

            if len(self.time_buffer) == 0:
                return [self.line_fz] + self.torque_lines + self.error_lines

            t = np.array(self.time_buffer)
            fz = np.array(self.fz_buffer)

            tau = [np.array(self.tau_buffers[i]) for i in range(6)]
            err = [np.array(self.error_buffers[i]) for i in range(6)]

        t_now = t[-1]
        mask = t >= max(0.0,t_now-self.window_time)

        # FUERZA
        self.line_fz.set_data(t[mask],fz[mask])

        self.ax_force.set_xlim(t_now-self.window_time,t_now)
        self.ax_force.relim()
        self.ax_force.autoscale_view(scalex=False)

        if len(fz[mask]) > 0:
            self.ax_force.set_ylim(min(fz[mask])-1, max(fz[mask])+1)

        # TORQUES
        for i in range(6):
            self.torque_lines[i].set_data(t[mask],tau[i][mask])

        self.ax_tau.set_xlim(t_now-self.window_time,t_now)
        self.ax_tau.relim()
        self.ax_tau.autoscale_view(scalex=False)

        # ERROR
        for i in range(6):
            self.error_lines[i].set_data(t[mask],err[i][mask])

        self.ax_error.set_xlim(t_now-self.window_time,t_now)
        self.ax_error.relim()
        self.ax_error.autoscale_view(scalex=False)

        return [self.line_fz] + self.torque_lines + self.error_lines


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):

    rclpy.init(args=args)

    node = ForceTorquePlotter()

    spin_thread = threading.Thread(
        target=ros_spin_thread,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    ani = animation.FuncAnimation(
        node.fig,
        node.animate,
        interval=50,
        blit=False
    )

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
