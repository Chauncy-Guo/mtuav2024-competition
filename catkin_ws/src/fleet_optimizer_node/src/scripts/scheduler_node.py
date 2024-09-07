#!/usr/bin/env python
import rospy
from fleet_optimizer_node.msg import VehicleState, DroneState
from fleet_optimizer_node.srv import BindDevices, BindDevicesResponse

class SchedulerNode:
    def __init__(self):
        rospy.init_node('scheduler_node')
        self.vehicle_states = {}
        self.drone_states = {}
        rospy.Subscriber('/vehicle_state', VehicleState, self.vehicle_state_callback)
        rospy.Subscriber('/drone_state', DroneState, self.drone_state_callback)
        self.bind_service = rospy.Service('/bind_devices', BindDevices, self.bind_devices_callback)

    def vehicle_state_callback(self, msg):
        # 保存无人车状态
        self.vehicle_states[msg.vehicle_id] = msg
        rospy.loginfo(f'Received vehicle state: {msg}')

    def drone_state_callback(self, msg):
        # 保存无人机状态
        self.drone_states[msg.drone_id] = msg
        rospy.loginfo(f'Received drone state: {msg}')

    def bind_devices_callback(self, req):
        # 简单的绑定逻辑示例
        if self.vehicle_states and self.drone_states:
            vehicle = next(iter(self.vehicle_states.values()))
            drone = next(iter(self.drone_states.values()))
            # 实际绑定逻辑可以更加复杂
            rospy.loginfo(f'Binding {vehicle.vehicle_id} with {drone.drone_id}')
            return BindDevicesResponse(success=True, message=f'Bound {vehicle.vehicle_id} with {drone.drone_id}')
        return BindDevicesResponse(success=False, message='No available devices to bind')

if __name__ == '__main__':
    try:
        SchedulerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass