import argparse
from meu_workspace.CLI import Command, TurtleCommandPublisher, rclpy


def main():
    rclpy.init() 
    node = TurtleCommandPublisher()

    parser = argparse.ArgumentParser()
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--vt', type=float, default=0.0)
    parser.add_argument('-t', '--time', type=int, default=1000)
    args = parser.parse_args()

    command = Command(args.vx, args.vy, args.vt, args.time)
    node.append(command)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#Acredito que o projeto não esteja rodando e alem disso não to conseguindo commitaaaaaaaaaarrrrrrrrr.