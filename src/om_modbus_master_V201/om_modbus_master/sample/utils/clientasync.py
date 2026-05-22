#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import ListParameters


class ClientAsync(Node):
    def __init__(self, name=""):
        super().__init__("my_node" + name)
        # self.set_parameters_from_another_node("om_node", [self.param])
        # self.values = self.get_parameters_from_another_node("om_node", ['init_baudrate'])

    def list_parameters_from_another_node(self, node_name):
        client = self.create_client(
            ListParameters, "{node_name}/list_parameters".format_map(locals())
        )

        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service time out")

        request = ListParameters.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # handle response
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError("Exception while callint service of node")

        return response

    def set_parameters_from_another_node(self, node_name, param_name, param_value):
        self.param = Parameter()
        self.param.name = param_name
        self.param.value.type = ParameterType.PARAMETER_INTEGER
        self.param.value.integer_value = param_value

        client = self.create_client(
            SetParameters, "{node_name}/set_parameters".format_map(locals())
        )

        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service time out")

        request = SetParameters.Request()
        request.parameters = [self.param]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # handle response
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError("Exception while callint service of node")

        # results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='')])
        return response

    def get_parameters_from_another_node(self, node_name, parameter_names):
        client = self.create_client(
            GetParameters, "{node_name}/get_parameters".format_map(locals())
        )
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service time out")

        request = GetParameters.Request()
        request.names = parameter_names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # handle response
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError("Exception while callint service of node")

        return_values = []
        for pvalue in response.values:
            if pvalue.type == ParameterType.PARAMETER_BOOL:
                value = pvalue.bool_value
            elif pvalue.type == ParameterType.PARAMETER_INTEGER:
                value = pvalue.integer_value
            elif pvalue.type == ParameterType.PARAMETER_STRING:
                value = pvalue.string_value
            elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
                value = None
            else:
                raise RuntimeError("Unknown parameter type")
            return_values.append(value)

        return return_values
