#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from controller_manager_msgs.srv import SwitchController
import pygame


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_pygame')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 基础速度幅值（可用步进键调整）
        self.lin_x = 0.5   # 前后速度基准
        self.lin_y = 0.5   # 左右速度基准
        self.ang_z = 1.0   # 角速度基准 (rad/s)

        # 步长（按 i/k, j/l, u/o 调整）
        self.step_lin_x = 0.05
        self.step_lin_y = 0.05
        self.step_ang_z = 0.05

        # 当前按键集合（pygame key 常量）
        self.pressed = set()

        # 显示信息
        self.get_logger().info('TeleopKeyboard initialized. Use W/S/A/D/Q/E to move, I/K J/L U/O to tune speeds.')

        self.switch_controller_client = self.create_client(SwitchController, 'controller_manager/switch_controller')
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller_manager/switch_controller service...')
        self.switch_controller_request = SwitchController.Request()
        self.switch_controller_request.strictness = SwitchController.Request.BEST_EFFORT

        # For elastic band control
        self.elastic_band_enable_pub = self.create_publisher(Bool, 'elastic_band/enable', 10)
        self.elastic_band_adjust_pub = self.create_publisher(Float64, 'elastic_band/adjust_length', 10)

    def handle_keydown(self, key):
        # 调整速度步长（立即生效）
        if key == pygame.K_i:
            self.lin_x += self.step_lin_x
            self.get_logger().info(f'lin_x increased to {self.lin_x:.3f}')
            return
        if key == pygame.K_k:
            self.lin_x = max(0.0, self.lin_x - self.step_lin_x)
            self.get_logger().info(f'lin_x decreased to {self.lin_x:.3f}')
            return
        if key == pygame.K_j:
            self.lin_y += self.step_lin_y
            self.get_logger().info(f'lin_y increased to {self.lin_y:.3f}')
            return
        if key == pygame.K_l:
            self.lin_y = max(0.0, self.lin_y - self.step_lin_y)
            self.get_logger().info(f'lin_y decreased to {self.lin_y:.3f}')
            return
        if key == pygame.K_u:
            self.ang_z += self.step_ang_z
            self.get_logger().info(f'ang_z increased to {self.ang_z:.3f}')
            return
        if key == pygame.K_o:
            self.ang_z = max(0.0, self.ang_z - self.step_ang_z)
            self.get_logger().info(f'ang_z decreased to {self.ang_z:.3f}')
            return

        # Controller switching
        if key == pygame.K_1:
            self.set_controller_switch(
                start_controllers=[],
                stop_controllers=['rl_controller', 'static_controller']
            )
            return 
        if key == pygame.K_2:
            self.set_controller_switch(
                start_controllers=['static_controller'],
                stop_controllers=['rl_controller']
            )
            return
        if key == pygame.K_3:
            self.set_controller_switch(
                start_controllers=['rl_controller'],
                stop_controllers=['static_controller']
            )
            return
        
        # Elastic band control
        if key == pygame.K_z:
            msg = Bool()
            msg.data = True
            self.elastic_band_enable_pub.publish(msg)
            self.get_logger().info('Elastic band enabled')
            return
        if key == pygame.K_x:
            msg = Bool()
            msg.data = False
            self.elastic_band_enable_pub.publish(msg)
            self.get_logger().info('Elastic band disabled')
            return
        if key == pygame.K_c:
            msg = Float64()
            msg.data = 0.1  # 增加长度
            self.elastic_band_adjust_pub.publish(msg)
            self.get_logger().info('Elastic band length increased by 0.1')
            return
        if key == pygame.K_v:
            msg = Float64()
            msg.data = -0.1  # 减少长度
            self.elastic_band_adjust_pub.publish(msg)
            self.get_logger().info('Elastic band length decreased by 0.1')
            return

        # 方向按键，加入集合
        if key in (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e):
            self.pressed.add(key)

    def handle_keyup(self, key):
        # 松开时从集合移除（若是方向键）
        if key in self.pressed:
            self.pressed.remove(key)

    def compute_twist(self):
        t = Twist()
        # x forward/back
        if pygame.K_w in self.pressed and pygame.K_s in self.pressed:
            t.linear.x = 0.0
        elif pygame.K_w in self.pressed:
            t.linear.x = self.lin_x
        elif pygame.K_s in self.pressed:
            t.linear.x = -self.lin_x
        else:
            t.linear.x = 0.0

        # y left/right (a left, d right)
        if pygame.K_a in self.pressed and pygame.K_d in self.pressed:
            t.linear.y = 0.0
        elif pygame.K_a in self.pressed:
            t.linear.y = self.lin_y
        elif pygame.K_d in self.pressed:
            t.linear.y = -self.lin_y
        else:
            t.linear.y = 0.0

        # angular z (q rotate left/ccw, e rotate right/cw)
        if pygame.K_q in self.pressed and pygame.K_e in self.pressed:
            t.angular.z = 0.0
        elif pygame.K_q in self.pressed:
            t.angular.z = self.ang_z
        elif pygame.K_e in self.pressed:
            t.angular.z = -self.ang_z
        else:
            t.angular.z = 0.0

        return t

    def publish_twist(self, twist):
        self.pub.publish(twist)
        
    def set_controller_switch(self, start_controllers, stop_controllers):
        self.switch_controller_request.activate_controllers = start_controllers
        self.switch_controller_request.deactivate_controllers = stop_controllers
        self.switch_controller_request.strictness = SwitchController.Request.BEST_EFFORT
        self.switch_controller_request.activate_asap = True
        future = self.switch_controller_client.call_async(self.switch_controller_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Switched controllers: started {start_controllers}, stopped {stop_controllers}')
        else:
            self.get_logger().error('Failed to call switch_controller service')

def main(args=None):
    rclpy.init(args=args)
    pygame.init()

    # 创建一个小窗口以接收键盘事件（必要）
    screen = pygame.display.set_mode((420, 200))
    pygame.display.set_caption('ROS2 Teleop Keyboard')

    node = TeleopKeyboard()
    rate_hz = 20.0
    clock = pygame.time.Clock()

    try:
        running = True
        while rclpy.ok() and running:
            # 处理 pygame 事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    else:
                        node.handle_keydown(event.key)
                elif event.type == pygame.KEYUP:
                    node.handle_keyup(event.key)

            # 计算并发布当前速率
            twist = node.compute_twist()
            node.publish_twist(twist)

            # 更新窗口（可选显示文本）
            screen.fill((0, 0, 0))
            # 简单文字显示速度（pygame 字体不必需，但有助于调试）
            try:
                font = pygame.font.Font(None, 40)
                txt = f'lin_x:{node.lin_x:.2f} lin_y:{node.lin_y:.2f} ang_z:{node.ang_z:.2f}'
                surf = font.render(txt, True, (255, 255, 255))
                screen.blit(surf, (10, 80))
            except Exception:
                pass
            pygame.display.flip()

            # 允许 rclpy 做一次处理
            rclpy.spin_once(node, timeout_sec=0)

            clock.tick(rate_hz)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down teleop node.')
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
