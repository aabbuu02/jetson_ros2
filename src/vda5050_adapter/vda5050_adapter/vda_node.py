"""
@file vda_node.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import paho.mqtt.client as mqtt
import json

class VdaAdapterNode(Node):
    def __init__(self):
        super().__init__('vda_adapter_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('broker_address', '127.0.0.1')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('vda_topic', 'vda5050/v1/some_agv/order')

        # Get Parameters
        broker_address = self.get_parameter('broker_address').get_parameter_value().string_value
        broker_port = self.get_parameter('broker_port').get_parameter_value().integer_value
        vda_topic = self.get_parameter('vda_topic').get_parameter_value().string_value

        # Create ROS 2 Client for the brake service
        self.brake_client = self.create_client(SetBool, 'brake_service')
        while not self.brake_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Brake service not available, waiting again...')

        # --- MQTT Client Setup ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.get_logger().info(f'Connecting to MQTT broker at {broker_address}:{broker_port}')
        try:
            self.mqtt_client.connect(broker_address, broker_port, 60)
            self.mqtt_client.loop_start() # Starts a background thread for MQTT
            self.vda_topic = vda_topic
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Successfully connected to MQTT broker. Subscribing to topic: {self.vda_topic}')
            client.subscribe(self.vda_topic)
        else:
            self.get_logger().error(f'Failed to connect to MQTT, return code {rc}\n')

    def on_mqtt_message(self, client, userdata, msg):
        self.get_logger().info(f'VDA5050 message received on topic: {msg.topic}')
        try:
            payload = json.loads(msg.payload.decode())
            
            if 'actions' in payload:
                for action in payload['actions']:
                    action_type = action.get('actionType', '')
                    self.get_logger().info(f'Processing action: {action_type}')

                    # --- This is the translation logic ---
                    if action_type == 'stop':
                        self.get_logger().info('Stop action detected! Calling brake service.')
                        req = SetBool.Request()
                        req.data = True # True to engage the brake
                        self.brake_client.call_async(req)

        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse incoming VDA5050 JSON message.')
        except Exception as e:
            self.get_logger().error(f'An error occurred in on_mqtt_message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VdaAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


