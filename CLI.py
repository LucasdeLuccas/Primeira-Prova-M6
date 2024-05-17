from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleCommandPublisher(Node):
    def __init__(self):
        super().__init__('turtle_command_publisher')
        self.command_queue = deque()
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_next_command)

    def append(self, command):
        self.command_queue.append(command)

    def appendleft(self, command):
        self.command_queue.appendleft(command)

    def pop(self):
        if self.command_queue:
            return self.command_queue.pop()
        else:
            return None

    def popleft(self):
        if self.command_queue:
            return self.command_queue.popleft()
        else:
            return None

    def rotate(self, n):
        self.command_queue.rotate(n)

    def publish_next_command(self):
        if self.command_queue:
            command = self.command_queue.popleft()
            twist = Twist()
            twist.linear.x = command.vx
            twist.linear.y = command.vy
            twist.angular.z = command.vtheta
            self.publisher_.publish(twist)
            self.get_logger().info(f'Comando escolhifo {command}')
        else:
            twist = Twist()
            self.publisher_.publish(twist)
            self.get_logger().info("Nenhum comando na fila, colocando zero twist'")



class Command:
    def __init__(self, vx, vy, vtheta, time):
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta
        self.time = time

    def __str__(self):
        return f'vx={self.vx:.2f}, vy={self.vy:.2f}, vtheta={self.vtheta:.2f}, time={self.time}ms'


def main():
    rclpy.init()
    node = TurtleCommandPublisher()

    
    node.append(Command(1.0, 0.0, 0.0, 1000))
    node.appendleft(Command(0.0, 0.0, 1.0, 500))
    node.rotate(1)

 
    command = node.pop()
    if command:
        print(f'Removed command: {command}')
    command = node.popleft()
    if command:
        print(f'Removed command: {command}')

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




    


