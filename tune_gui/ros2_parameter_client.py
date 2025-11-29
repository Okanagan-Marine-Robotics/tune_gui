#!/usr/bin/env python3

import time
import rclpy 
from typing import Dict, List, Any
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import ParameterType

class ROS2ParameterClient:
    """Client for interacting with ROS2 node parameters."""
    
    def __init__(self, node: Node, timeout_sec: float = 2.0):
        self.node = node
        self.clients = {}  # Cache service clients
        self.timeout_sec = timeout_sec
        
    def get_node_names(self) -> List[str]:
        """Get list of all active ROS2 nodes."""
        node_names = self.node.get_node_names()
        return sorted([name for name in node_names if not name.startswith('/_')])

    def get_node_parameters(self, node_name: str) -> Dict[str, Dict[str, Any]]:
        """Get all parameters for a specific node."""
        try:
            list_client = self._get_service_client(node_name, ListParameters, 'list_parameters')
            list_req = ListParameters.Request()
            list_resp = self._call_service(list_client, list_req)
            
            if not list_resp.result.names:
                return {}

            param_names = list_resp.result.names

            get_client = self._get_service_client(node_name, GetParameters, 'get_parameters')
            get_req = GetParameters.Request(names=param_names)
            get_resp = self._call_service(get_client, get_req)

            parameters = {}
            for name, p_value in zip(param_names, get_resp.values):
                parameters[name] = {
                    'value': parameter_value_to_python(p_value),
                    'type': self._get_type_name(p_value.type)
                }
            return parameters

        except (TimeoutError, RuntimeError) as e:
            self.node.get_logger().error(f"Error getting params for {node_name}: {e}")
            return {}

    def set_parameter(self, node_name: str, param_name: str, value: Any) -> bool:
        """Set a parameter value for a specific node."""
        try:
            client = self._get_service_client(node_name, SetParameters, 'set_parameters')
            
            param_wrapper = Parameter(param_name, value=value)
            
            req = SetParameters.Request()
            req.parameters = [param_wrapper.to_parameter_msg()]
            
            response = self._call_service(client, req)
            
            if not response.results:
                return False
                
            result = response.results[0]
            if not result.successful:
                self.node.get_logger().error(f"Failed to set {param_name}: {result.reason}")
                return False
                
            return True

        except (TimeoutError, RuntimeError) as e:
            self.node.get_logger().error(f"Error setting param {param_name} on {node_name}: {e}")
            return False

    def _get_service_client(self, node_name: str, srv_type, srv_suffix: str):
        """Generic method to get or create a service client."""
        client_key = f"{node_name}_{srv_suffix}"
        
        if client_key not in self.clients:
            service_name = f"{node_name}/{srv_suffix}"
            client = self.node.create_client(srv_type, service_name)
            
            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError(f"Service {service_name} not available")
                
            self.clients[client_key] = client
            
        return self.clients[client_key]

    def _call_service(self, client, request):
        """Helper to call service synchronously and handle timeouts."""
        future = client.call_async(request)
        
        start_time = time.time()
        
        while not future.done():
            if time.time() - start_time > self.timeout_sec:
                try:
                    future.cancel()
                except Exception:
                    pass
                raise TimeoutError("Service call timed out")
            
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
        return future.result()

    def _get_type_name(self, type_enum: int) -> str:
        """Helper to get string representation of type."""
        return {
            ParameterType.PARAMETER_BOOL: 'bool',
            ParameterType.PARAMETER_INTEGER: 'int',
            ParameterType.PARAMETER_DOUBLE: 'float',
            ParameterType.PARAMETER_STRING: 'str',
            ParameterType.PARAMETER_BYTE_ARRAY: 'byte[]',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'int[]',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'float[]',
            ParameterType.PARAMETER_STRING_ARRAY: 'str[]',
        }.get(type_enum, 'unknown')