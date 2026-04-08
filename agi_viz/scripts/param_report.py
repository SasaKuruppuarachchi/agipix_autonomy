#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters

from collections import defaultdict
import fnmatch
import time


SERVICE_TIMEOUT = 0.5
CALL_TIMEOUT = 1.0


class ParamInspector(Node):

    def __init__(self):
        super().__init__("param_inspector")

        self.node_filter = [
            "/rviz*",
            "/fmu/*",
            "/parameter_events",
        ]

        self.report_lines = []

        self.wait_for_discovery()

        self.inspect()

    # -------------------------

    def wait_for_discovery(self):

        self.get_logger().info("Waiting for discovery...")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.1)

    # -------------------------

    def is_filtered(self, name):

        for p in self.node_filter:
            if fnmatch.fnmatch(name, p):
                return True

        return False

    # -------------------------

    def inspect(self):

        nodes = self.get_node_names_and_namespaces()

        by_node = defaultdict(list)
        by_type = defaultdict(list)
        by_name = defaultdict(list)

        for name, namespace in nodes:

            full = namespace + "/" + name if namespace != "/" else "/" + name

            if self.is_filtered(full):
                continue

            self.get_logger().info(f"Checking {full}")

            param_names = self.list_params_safe(full)

            if not param_names:
                continue

            types, values = self.get_param_info_safe(
                full,
                param_names,
            )

            for i, pname in enumerate(param_names):

                ptype = types[i]
                pvalue = values[i]

                entry = f"{full}:{pname} = {pvalue}"

                by_node[full].append(entry)
                by_type[ptype].append(entry)
                by_name[pname].append(entry)

        self.write_report(by_node, by_type, by_name)

    # -------------------------
    # SAFE LIST PARAMS
    # -------------------------

    def list_params_safe(self, node_name):

        srv = node_name + "/list_parameters"

        client = self.create_client(ListParameters, srv)

        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            return []

        req = ListParameters.Request()
        req.depth = 10

        future = client.call_async(req)

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.1)

            if future.done():
                break

            if time.time() - start > CALL_TIMEOUT:
                self.get_logger().warn(
                    f"Timeout list_parameters {node_name}"
                )
                return []

        result = future.result()

        if result is None:
            return []

        return result.result.names

    # -------------------------
    # SAFE PARAM INFO
    # -------------------------

    def get_param_info_safe(self, node_name, names):

        desc_srv = node_name + "/describe_parameters"
        get_srv = node_name + "/get_parameters"

        desc_client = self.create_client(
            DescribeParameters,
            desc_srv,
        )

        get_client = self.create_client(
            GetParameters,
            get_srv,
        )

        if not desc_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT
        ):
            return [], []

        if not get_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT
        ):
            return [], []

        # describe

        dreq = DescribeParameters.Request()
        dreq.names = names

        dfut = desc_client.call_async(dreq)

        # get

        greq = GetParameters.Request()
        greq.names = names

        gfut = get_client.call_async(greq)

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.1)

            if dfut.done() and gfut.done():
                break

            if time.time() - start > CALL_TIMEOUT:
                self.get_logger().warn(
                    f"Timeout params {node_name}"
                )
                return [], []

        if dfut.result() is None:
            return [], []

        if gfut.result() is None:
            return [], []

        types = []
        values = []

        for d, v in zip(
            dfut.result().descriptors,
            gfut.result().values,
        ):

            types.append(str(d.type))
            values.append(self.value_to_string(v))

        return types, values

    # -------------------------

    def value_to_string(self, v):

        t = v.type

        if t == 1:
            return str(v.bool_value)

        if t == 2:
            return str(v.integer_value)

        if t == 3:
            return str(v.double_value)

        if t == 4:
            return v.string_value

        if t == 5:
            return str(v.byte_array_value)

        if t == 6:
            return str(v.bool_array_value)

        if t == 7:
            return str(v.integer_array_value)

        if t == 8:
            return str(v.double_array_value)

        if t == 9:
            return str(v.string_array_value)

        return "?"

    # -------------------------

    def write_report(self, by_node, by_type, by_name):

        self.report_lines.append("==== GROUP BY NODE ====\n")

        for n in sorted(by_node):

            self.report_lines.append(n + ":")

            for p in by_node[n]:
                self.report_lines.append("  " + p)

            self.report_lines.append("")

        self.report_lines.append("\n==== GROUP BY TYPE ====\n")

        for t in sorted(by_type):

            self.report_lines.append(t + ":")

            for p in by_type[t]:
                self.report_lines.append("  " + p)

            self.report_lines.append("")

        self.report_lines.append("\n==== GROUP BY NAME ====\n")

        for n in sorted(by_name):

            self.report_lines.append(n + ":")

            for p in by_name[n]:
                self.report_lines.append("  " + p)

            self.report_lines.append("")

        with open("param_report.txt", "w") as f:
            f.write("\n".join(self.report_lines))

        self.get_logger().info("Report written")


def main():

    rclpy.init()

    node = ParamInspector()

    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()