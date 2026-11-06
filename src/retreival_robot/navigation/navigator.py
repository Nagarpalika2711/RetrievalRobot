#!/usr/bin/env python3
"""Stop-and-turn differential-drive path follower; idle until /planned_path arrives."""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        for n,v in [('wheel_radius',.1132),('wheel_separation',.76),('linear_speed',1.132),
                    ('max_angular_speed',10.0),('heading_gain',1.8),('distance_tolerance',.16),
                    ('heading_tolerance',.15),('max_wheel_rad_s',10.0)]:self.declare_parameter(n,v)
        self.path=[];self.index=0;self.active=False;self.final_yaw=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(Float64MultiArray,'/wheel_velocity_controller/commands',10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub=self.create_subscription(Path,'/planned_path',self.on_path,qos)
        # Pause immediately when a new task arrives, even before the planner finishes.
        self.task_sub=self.create_subscription(PoseStamped,'/get_task',self.on_new_task,10)
        self.timer=self.create_timer(.1,self.step)
        self.get_logger().info('Navigator idle until new path received.')
    def speed(self,left,right):
        limit=float(self.get_parameter('max_wheel_rad_s').value)
        msg=Float64MultiArray();msg.data=[max(-limit,min(limit,left)),max(-limit,min(limit,right))]
        self.pub.publish(msg)
    def stop(self):self.speed(0.,0.)
    def on_new_task(self,_):
        self.active=False;self.path=[];self.index=0;self.stop()
    def on_path(self,msg):
        self.stop();self.active=False;self.path=[];self.index=0
        if msg.header.frame_id!='map' or not msg.poses:
            self.get_logger().info('Path cleared; waiting for a valid new path.')
            return
        self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.final_yaw=yaw(msg.poses[-1].pose.orientation)
        self.index=1 if len(self.path)>1 else 0
        self.active=True
        self.get_logger().info(f'Following path with {len(self.path)} waypoints')
    def step(self):
        if not self.active:
            self.stop();return
        try:t=self.tf.lookup_transform('map','base_link',Time())
        except TransformException:
            self.stop();return
        x=t.transform.translation.x;y=t.transform.translation.y;heading=yaw(t.transform.rotation)
        while self.index<len(self.path):
            gx,gy=self.path[self.index]
            dist=math.hypot(gx-x,gy-y)
            if dist>=float(self.get_parameter('distance_tolerance').value):break
            self.index+=1
        if self.index>=len(self.path):
            # Ensure final commanded heading is attained.
            diff=wrap(self.final_yaw-heading)
            if abs(diff)<float(self.get_parameter('heading_tolerance').value):
                self.active=False;self.stop();self.get_logger().info('Goal reached; waiting for new path');return
            v=0.;omega=max(-.5,min(.5,1.8*diff))
        else:
            gx,gy=self.path[self.index];diff=wrap(math.atan2(gy-y,gx-x)-heading)
            if abs(diff)>float(self.get_parameter('heading_tolerance').value):
                v=0.;omega=max(-.5,min(.5,float(self.get_parameter('heading_gain').value)*diff))
            else:
                v=float(self.get_parameter('linear_speed').value)
                omega=max(-.2,min(.2,float(self.get_parameter('heading_gain').value)*diff))
        radius=float(self.get_parameter('wheel_radius').value);L=float(self.get_parameter('wheel_separation').value)
        self.speed((v-omega*L/2)/radius,(v+omega*L/2)/radius)
    def destroy_node(self):
        self.stop();return super().destroy_node()

def main():
    rclpy.init();node=Navigator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
