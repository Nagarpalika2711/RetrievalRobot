#!/usr/bin/env python3
"""Publish pre-generated static occupancy grid (/map) with transient-local QoS."""
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        self.declare_parameter('map_yaml', '')
        path=Path(self.get_parameter('map_yaml').value)
        data=yaml.safe_load(path.read_text())
        pgm=(path.parent/data['image']).read_bytes()
        # Generated P5 format: fixed 3-line header, no comments.
        head, size, maxval, body=pgm.split(b'\n',3)
        if head!=b'P5' or maxval!=b'255': raise ValueError('Expected generated P5 PGM')
        w,h=map(int,size.split())
        img=np.frombuffer(body,dtype=np.uint8).reshape((h,w))
        grid=np.where(img<128,100,0).astype(np.int8)
        msg=OccupancyGrid()
        msg.header.frame_id='map'
        msg.info.resolution=float(data['resolution']); msg.info.width=w; msg.info.height=h
        origin=data['origin'];msg.info.origin.position.x=float(origin[0]);msg.info.origin.position.y=float(origin[1])
        msg.info.origin.orientation.w=1.0
        # nav_msgs OccupancyGrid starts bottom-left, PGM rows start top-left.
        msg.data=grid[::-1].flatten().tolist()
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/map',qos)
        self.msg=msg
        self.timer=self.create_timer(1.0,self.publish)
        self.publish()
    def publish(self):
        self.msg.header.stamp=self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

def main():
    rclpy.init();node=MapPublisher()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
