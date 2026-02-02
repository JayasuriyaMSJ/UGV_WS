#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from functools import partial


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        self.sources = {
            'ekf': {
                'topic': '/odometry/filtered',
                'x': [],
                'y': [],
                'color': 'b',
                'ax': None,
                'line': None,
                'title': 'EKF Odometry'
            },
            'wheel': {
                'topic': '/odom',
                'x': [],
                'y': [],
                'color': 'g',
                'ax': None,
                'line': None,
                'title': 'Wheel Odometry'
            },
        }

        # Subscriptions
        for name, src in self.sources.items():
            self.create_subscription(
                Odometry,
                src['topic'],
                partial(self.odom_callback, name),
                10
            )

        # ─── Matplotlib setup ─────────────────────────────
        plt.ion()
        self.fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

        self.sources['ekf']['ax'] = ax1
        self.sources['wheel']['ax'] = ax2

        for src in self.sources.values():
            src['line'], = src['ax'].plot(
                [], [],
                color=src['color'],
                linewidth=2.0
            )
            src['ax'].set_title(src['title'])
            src['ax'].set_xlabel("X [m]")
            src['ax'].set_ylabel("Y [m]")
            src['ax'].grid(True)
            src['ax'].axis('equal')

        # Timer to update plots at ~10 Hz
        self.create_timer(0.1, self.update_plot)

    def odom_callback(self, source_name, msg):
        src = self.sources[source_name]
        src['x'].append(msg.pose.pose.position.x)
        src['y'].append(msg.pose.pose.position.y)

    def update_plot(self):
        updated = False

        for src in self.sources.values():
            if src['x']:
                src['line'].set_data(src['x'], src['y'])
                src['ax'].relim()
                src['ax'].autoscale_view()
                updated = True

        if updated:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)


def main():
    rclpy.init()
    node = OdomPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()
