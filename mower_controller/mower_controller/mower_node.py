import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8, Bool
import serial
import time

class HD7000W_Advanced_Bridge(Node):
    def __init__(self):
        super().__init__('hd7000w_advanced_bridge')
        
        # --- 1. 파라미터 및 상태 변수 정의 ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.port = self.get_parameter('serial_port').value
        
        # 상태 정의 (0: MANUAL, 1: AUTO, 2: EMERGENCY)
        self.state = 0 
        self.engine_on = 0
        self.last_cmd_time = self.get_clock().now()
        
        # --- 2. 시리얼 통신 설정 ---
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            self.get_logger().info(f'✅ 시리얼 연결 성공: {self.port}')
        except Exception as e:
            self.get_logger().error(f'❌ 시리얼 연결 실패: {e}')

        # --- 3. ROS 2 인터페이스 (구독/발행) ---
        # 수동 조종 (Teleop)
        self.manual_sub = self.create_subscription(Twist, 'cmd_vel', self.manual_callback, 10)
        # 자율 주행 명령 (팀원들의 알고리즘 명령 대역)
        self.auto_sub = self.create_subscription(Twist, 'cmd_vel_auto', self.auto_callback, 10)
        # 상태 변경 명령 (0, 1, 2)
        self.mode_sub = self.create_subscription(Int8, 'mower/set_mode', self.mode_callback, 10)
        # 엔진 명령
        self.eng_sub = self.create_subscription(Bool, 'mower/engine', self.engine_callback, 10)
        
        # 현재 상태를 맥북(Foxglove)에 알려주는 퍼블리셔
        self.status_pub = self.create_publisher(String, 'mower/current_status', 10)
        
        # --- 4. 타이머 설정 ---
        self.create_timer(0.1, self.main_loop) # 10Hz로 시스템 루프 가동

    def mode_callback(self, msg):
        self.state = msg.data
        mode_names = {0: "MANUAL", 1: "AUTO", 2: "EMERGENCY"}
        self.get_logger().warn(f'제어 모드 변경됨: {mode_names.get(self.state, "UNKNOWN")}')

    def engine_callback(self, msg):
        self.engine_on = 1 if msg.data else 0

    def manual_callback(self, msg):
        self.get_logger().info(f"수동 신호 수신, 현재 상태: {self.state}")
        if self.state == 0: # MANUAL 모드일 때만 처리
            self.process_twist(msg)

    def auto_callback(self, msg):
        if self.state == 1: # AUTO 모드일 때만 처리
            self.process_twist(msg)

    def process_twist(self, msg):
        self.last_cmd_time = self.get_clock().now()
        linear = msg.linear.x
        angular = msg.angular.z
        
        lpwm = max(min(int(1500 + (linear - angular) * 500), 2000), 1000)
        rpwm = max(min(int(1500 + (linear + angular) * 500), 2000), 1000)
        self.send_secure_packet(lpwm, rpwm)

    def calculate_checksum(self, data_str):
        """XOR Checksum 계산 함수"""
        checksum = 0
        for char in data_str:
            checksum ^= ord(char)
        return hex(checksum)[2:].upper().zfill(2)

    def send_secure_packet(self, lpwm, rpwm):
        """[1단계] 체크섬이 포함된 보안 패킷 생성 및 전송"""
        # 패킷 구조: $MODE,LPWM,RPWM,ENG,CHECKSUM*
        body = f"{self.state},{lpwm},{rpwm},{self.engine_on}"
        cs = self.calculate_checksum(body)
        packet = f"${body},{cs}*\n"
        
        try:
            self.ser.write(packet.encode())
        except:
            pass

    def main_loop(self):
        # 1. Foxglove 모니터링용 상태 메시지 발행
        mode_names = {0: "MANUAL", 1: "AUTO", 2: "EMERGENCY"}
        msg = String()
        msg.data = f"MODE: {mode_names.get(self.state)}, ENG: {'ON' if self.engine_on else 'OFF'}"
        self.status_pub.publish(msg)

        # 2. 안전 장치: 통신 두절 시 정지
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if self.state != 2 and elapsed > 0.5:
            self.send_secure_packet(1500, 1500)

def main(args=None):
    rclpy.init(args=args)
    node = HD7000W_Advanced_Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_secure_packet(1500, 1500)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()