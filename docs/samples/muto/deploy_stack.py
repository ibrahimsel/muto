#!/usr/bin/env python3       
"""
Deploy a stack to MUTO by publishing to the /muto/stack topic.
"""                                                                                                                                                               
import rclpy                                                                                                                                                                                
from rclpy.node import Node                                                                                                                                                                 
from muto_msgs.msg import MutoAction                                                                                                                                                        
import json                                                                                                                                                                                 
import sys                                                                                                                                                                                  
                                                                                                                                                                                            
class StackDeployer(Node):                                                                                                                                                                  
    def __init__(self, stack_file):                                                                                                                                                         
        super().__init__('stack_deployer')                                                                                                                                                  
        self.publisher = self.create_publisher(MutoAction, '/muto/stack', 10)                                                                                                      
                                                                                                                                                                                            
        with open(stack_file) as f:                                                                                                                                                         
            stack = json.load(f)                                                                                                                                                            
                                                                                                                                                                                            
        msg = MutoAction()                                                                                                                                                                  
        msg.method = 'start'                                                                                                                                                                
        msg.payload = json.dumps(stack)                                                                                                                                                     
                                                                                                                                                                                            
        # Wait for publisher to be ready                                                                                                                                                    
        self.create_timer(1.0, lambda: self.publish_and_exit(msg))                                                                                                                          
                                                                                                                                                                                            
    def publish_and_exit(self, msg):                                                                                                                                                        
        self.publisher.publish(msg)                                                                                                                                                         
        self.get_logger().info(f"Published stack: {msg.payload[:100]}...")                                                                                                                  
        raise SystemExit(0)                                                                                                                                                                 
                                                                                                                                                                                            
def main():                                                                                                                                                                                 
    rclpy.init()                                                                                                                                                                            
    node = StackDeployer(sys.argv[1])                                                                                                                                                       
    rclpy.spin(node)                                                                                                                                                                        
                                                                                                                                                                                            
if __name__ == '__main__':                                                                                                                                                                  
    main()                  