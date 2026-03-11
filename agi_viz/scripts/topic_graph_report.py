#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from collections import defaultdict
import fnmatch
import time


class TopicGraphInspector(Node):

    def __init__(self):
        super().__init__("topic_graph_inspector")

        # -------------------------
        # FILTER LIST
        # -------------------------
        self.topic_filter = [
            "/rosout",
            "/clock",
            "/tf",
            "/tf_static",
            "/parameter_events",
            "/fmu/*",
            "/drone0/*",
        ]

        self.graph_lines = []
        self.report_lines = []

        self.wait_for_discovery()

        self.inspect_graph()

    # -------------------------
    # WAIT FOR DDS DISCOVERY
    # -------------------------

    def wait_for_discovery(self):

        self.get_logger().info("Waiting for ROS graph discovery...")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.1)

        self.get_logger().info("Discovery done")

    # -------------------------
    # FILTER
    # -------------------------

    def is_filtered(self, topic_name):

        for pattern in self.topic_filter:
            if fnmatch.fnmatch(topic_name, pattern):
                return True

        return False

    # -------------------------
    # RESOLVE NODE NAME
    # -------------------------

    def resolve_node_name(self, info):

        name = info.node_name
        namespace = info.node_namespace

        if name is None:
            return None

        if name == "" or name == "_NODE_NAME_UNKNOWN_":
            return None

        if namespace and namespace != "/":
            return f"{namespace}/{name}"

        return name

    # -------------------------
    # MAIN
    # -------------------------

    def inspect_graph(self):

        topics = self.get_topic_names_and_types()

        topics_by_subscriber = defaultdict(list)
        topics_by_publisher = defaultdict(list)
        topics_by_type = defaultdict(list)

        self.graph_lines.append("digraph ROS2Topics {")
        self.graph_lines.append("rankdir=LR;")

        for topic_name, types in topics:

            if self.is_filtered(topic_name):
                continue

            msg_type = types[0] if types else "unknown"

            publishers = self.get_publishers_info_by_topic(topic_name)
            subscribers = self.get_subscriptions_info_by_topic(topic_name)

            topics_by_type[msg_type].append(topic_name)

            # topic node
            self.graph_lines.append(
                f'"{topic_name}" [shape=ellipse, color=blue];'
            )

            # -------------------------
            # publishers
            # -------------------------

            for pub in publishers:

                node_name = self.resolve_node_name(pub)

                if node_name is None:
                    continue

                topics_by_publisher[node_name].append(topic_name)

                self.graph_lines.append(
                    f'"{node_name}" [shape=box, color=green];'
                )

                self.graph_lines.append(
                    f'"{node_name}" -> "{topic_name}";'
                )

            # -------------------------
            # subscribers
            # -------------------------

            for sub in subscribers:

                node_name = self.resolve_node_name(sub)

                if node_name is None:
                    continue

                topics_by_subscriber[node_name].append(topic_name)

                self.graph_lines.append(
                    f'"{node_name}" [shape=box, color=red];'
                )

                self.graph_lines.append(
                    f'"{topic_name}" -> "{node_name}";'
                )

        self.graph_lines.append("}")

        self.write_graph()
        self.write_report(
            topics_by_subscriber,
            topics_by_publisher,
            topics_by_type,
        )

    # -------------------------
    # GRAPH OUTPUT
    # -------------------------

    def write_graph(self):

        with open("topic_graph.dot", "w") as f:
            f.write("\n".join(self.graph_lines))

        self.get_logger().info("Graph written to topic_graph.dot")

    # -------------------------
    # REPORT OUTPUT
    # -------------------------

    def write_report(
        self,
        topics_by_subscriber,
        topics_by_publisher,
        topics_by_type,
    ):

        # -------------------------
        # SUBSCRIBER
        # -------------------------

        self.report_lines.append("==== GROUP BY SUBSCRIBER ====\n")

        for node in sorted(topics_by_subscriber.keys()):

            self.report_lines.append(node + ":")

            for t in sorted(topics_by_subscriber[node]):
                self.report_lines.append(f"  - {t}")

            self.report_lines.append("")

        # -------------------------
        # PUBLISHER
        # -------------------------

        self.report_lines.append("\n==== GROUP BY PUBLISHER ====\n")

        for node in sorted(topics_by_publisher.keys()):

            self.report_lines.append(node + ":")

            for t in sorted(topics_by_publisher[node]):
                self.report_lines.append(f"  - {t}")

            self.report_lines.append("")

        # -------------------------
        # TYPE
        # -------------------------

        self.report_lines.append("\n==== GROUP BY TYPE ====\n")

        for ttype in sorted(topics_by_type.keys()):

            self.report_lines.append(ttype + ":")

            for t in sorted(topics_by_type[ttype]):
                self.report_lines.append(f"  - {t}")

            self.report_lines.append("")

        with open("topic_report.txt", "w") as f:
            f.write("\n".join(self.report_lines))

        self.get_logger().info("Report written to topic_report.txt")


# -------------------------
# MAIN
# -------------------------

def main(args=None):

    rclpy.init(args=args)

    node = TopicGraphInspector()

    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()