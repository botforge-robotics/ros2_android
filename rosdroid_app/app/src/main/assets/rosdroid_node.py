#!/usr/bin/env python3
"""
Rosdroid ROS2 Robot Node
Bridges Android hardware sensors, camera, voice, TTS, and biometrics to ROS 2 Jazzy.
Implements the exact ROS interfaces from rio_mini_ros2.
"""

import socket
import json
import threading
import time
import math
import logging

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Standard ROS2 message types
from std_msgs.msg import String, Bool, Header
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Illuminance, Imu, Range, CompressedImage
from geometry_msgs.msg import Twist, Vector3

# Custom RIO ROS2 interfaces
from rio_interfaces.action import TTS, Auth
from rio_interfaces.srv import Camera, Expression, GetExpression

logging.basicConfig(level=logging.INFO, format="%(asctime)s [RosdroidNode] %(levelname)s: %(message)s")
logger = logging.getLogger("RosdroidNode")


class RosdroidNode(Node):
    def __init__(self):
        super().__init__('rosdroid_robot')
        self.cb_group = ReentrantCallbackGroup()

        # Internal state
        self.current_expression = "idle"
        self.android_sock = None
        self.sock_lock = threading.Lock()
        self.running = True

        # Active Action Goals
        self.active_tts_goals = {}
        self.active_auth_goals = {}
        self.active_srv_requests = {}

        # 1. Publishers
        self.pub_battery = self.create_publisher(BatteryState, '/battery', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/gps', 10)
        self.pub_illuminance = self.create_publisher(Illuminance, '/illuminance', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 20)
        self.pub_tof = self.create_publisher(Range, '/tof', 10)
        self.pub_cam_front = self.create_publisher(CompressedImage, '/camera/front/image_raw/compressed', 10)
        self.pub_cam_back = self.create_publisher(CompressedImage, '/camera/back/image_raw/compressed', 10)
        
        # Speech topics
        # Note: hotword_detected published ONLY on trigger!
        self.pub_hotword = self.create_publisher(Bool, '/speech_recognition/hotword_detected', 10)
        self.pub_speech_result = self.create_publisher(String, '/speech_recognition/result', 10)
        self.pub_speech_status = self.create_publisher(String, '/speech_recognition/status', 10)
        self.pub_expression = self.create_publisher(String, '/expression', 10)

        # 2. Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10, callback_group=self.cb_group)
        self.sub_torch = self.create_subscription(
            Bool, '/torch', self.torch_callback, 10, callback_group=self.cb_group)

        # 3. Services
        self.srv_camera = self.create_service(
            Camera, '/enable_camera', self.enable_camera_callback, callback_group=self.cb_group)
        self.srv_set_expr = self.create_service(
            Expression, '/set_expression', self.set_expression_callback, callback_group=self.cb_group)
        self.srv_get_expr = self.create_service(
            GetExpression, '/expression_status', self.get_expression_callback, callback_group=self.cb_group)

        # 4. Action Servers
        self.action_tts = ActionServer(
            self,
            TTS,
            '/tts',
            execute_callback=self.execute_tts_callback,
            callback_group=self.cb_group
        )

        self.action_auth = ActionServer(
            self,
            Auth,
            '/auth',
            execute_callback=self.execute_auth_callback,
            callback_group=self.cb_group
        )

        logger.info("RosdroidNode ROS2 Action Servers and Services ready.")

        # Start background TCP bridge to Android app (localhost:9871)
        self.bridge_thread = threading.Thread(target=self.tcp_bridge_loop, daemon=True)
        self.bridge_thread.start()
        self.cam_thread = threading.Thread(target=self.udp_camera_loop, daemon=True)
        self.cam_thread.start()

    def udp_camera_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('127.0.0.1', 9872))
            logger.info('UDP Camera receiver listening on 127.0.0.1:9872')
            while self.running:
                data, _ = sock.recvfrom(65507)
                if len(data) > 4:
                    is_front = (data[0] == 0)
                    jpeg_bytes = data[1:]
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_front_link' if is_front else 'camera_back_link'
                    msg.format = 'jpeg'
                    msg.data = list(jpeg_bytes)
                    if is_front:
                        self.pub_cam_front.publish(msg)
                    else:
                        self.pub_cam_back.publish(msg)
        except Exception as e:
            logger.warning(f'UDP Camera loop error: {e}')

    def send_to_android(self, data_dict):
        with self.sock_lock:
            if self.android_sock:
                try:
                    payload = (json.dumps(data_dict) + "\n").encode('utf-8')
                    self.android_sock.sendall(payload)
                except Exception as e:
                    logger.warning(f"Error sending to Android: {e}")

    def tcp_bridge_loop(self):
        while self.running:
            try:
                logger.info("Connecting to Android Rosdroid Bridge at 127.0.0.1:9871...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(('127.0.0.1', 9871))
                with self.sock_lock:
                    self.android_sock = s
                logger.info("Connected to Android Rosdroid Bridge!")

                f = s.makefile('r', encoding='utf-8')
                for line in f:
                    if not line:
                        break
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        self.handle_android_message(msg)
                    except Exception as ex:
                        logger.warning(f"Error handling android message: {ex}")

            except Exception as e:
                logger.debug(f"Android bridge connection retry in 2s ({e})")
            finally:
                with self.sock_lock:
                    if self.android_sock:
                        try:
                            self.android_sock.close()
                        except:
                            pass
                        self.android_sock = None
                time.sleep(2.0)

    def handle_android_message(self, msg):
        mtype = msg.get("type", "")

        if mtype == "imu":
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation.w = float(msg.get("qw", 1.0))
            imu_msg.orientation.x = float(msg.get("qx", 0.0))
            imu_msg.orientation.y = float(msg.get("qy", 0.0))
            imu_msg.orientation.z = float(msg.get("qz", 0.0))
            imu_msg.angular_velocity.x = float(msg.get("gx", 0.0))
            imu_msg.angular_velocity.y = float(msg.get("gy", 0.0))
            imu_msg.angular_velocity.z = float(msg.get("gz", 0.0))
            imu_msg.linear_acceleration.x = float(msg.get("ax", 0.0))
            imu_msg.linear_acceleration.y = float(msg.get("ay", 0.0))
            imu_msg.linear_acceleration.z = float(msg.get("az", 9.81))
            self.pub_imu.publish(imu_msg)

        elif mtype == "illuminance":
            ill_msg = Illuminance()
            ill_msg.header.stamp = self.get_clock().now().to_msg()
            ill_msg.header.frame_id = "light_sensor_link"
            ill_msg.illuminance = float(msg.get("lux", 0.0))
            self.pub_illuminance.publish(ill_msg)

        elif mtype == "tof":
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = "tof_link"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.44  # ~25 deg
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = float(msg.get("range", 0.0))
            self.pub_tof.publish(range_msg)

        elif mtype == "gps":
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "gps_link"
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.latitude = float(msg.get("lat", 0.0))
            gps_msg.longitude = float(msg.get("lon", 0.0))
            gps_msg.altitude = float(msg.get("alt", 0.0))
            self.pub_gps.publish(gps_msg)

        elif mtype == "battery":
            bat_msg = BatteryState()
            bat_msg.header.stamp = self.get_clock().now().to_msg()
            pct = float(msg.get("percentage", 100)) / 100.0
            bat_msg.percentage = pct
            bat_msg.voltage = 3.8 + (pct * 0.4)
            is_charging = msg.get("charging", False)
            bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if is_charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            self.pub_battery.publish(bat_msg)

        elif mtype == "hotword_detected":
            # ONLY published on trigger!
            hotword = msg.get("hotword", "Hey Robot")
            logger.info(f"Triggering /speech_recognition/hotword_detected ({hotword})")
            hw_msg = Bool()
            hw_msg.data = True
            self.pub_hotword.publish(hw_msg)

        elif mtype == "speech_result":
            text = msg.get("text", "")
            logger.info(f"Publishing /speech_recognition/result: '{text}'")
            res_msg = String()
            res_msg.data = text
            self.pub_speech_result.publish(res_msg)

        elif mtype == "speech_status":
            stat = msg.get("status", "")
            stat_msg = String()
            stat_msg.data = stat
            self.pub_speech_status.publish(stat_msg)

        elif mtype == "expression":
            expr = msg.get("expression", "idle")
            self.current_expression = expr
            expr_msg = String()
            expr_msg.data = expr
            self.pub_expression.publish(expr_msg)

        # Action TTS Feedback & Result
        elif mtype == "action_tts_feedback":
            gid = msg.get("goal_id")
            if gid in self.active_tts_goals:
                goal_handle = self.active_tts_goals[gid]
                fb = TTS.Feedback()
                fb.start_idx = int(msg.get("start_idx", 0))
                fb.end_idx = int(msg.get("end_idx", 0))
                fb.word = str(msg.get("word", ""))
                fb.expression = str(msg.get("expression", "speaking"))
                goal_handle.publish_feedback(fb)

        elif mtype == "action_tts_result":
            gid = msg.get("goal_id")
            if gid in self.active_tts_goals:
                goal_handle = self.active_tts_goals.pop(gid)
                result = TTS.Result()
                result.success = bool(msg.get("success", True))
                result.expression = str(msg.get("expression", self.current_expression))
                goal_handle.succeed()
                self.active_tts_goals[f"{gid}_result"] = result

        # Action Auth Result
        elif mtype == "action_auth_result":
            gid = msg.get("goal_id")
            if gid in self.active_auth_goals:
                goal_handle = self.active_auth_goals.pop(gid)
                result = Auth.Result()
                result.success = bool(msg.get("success", False))
                result.error_message = str(msg.get("error_message", ""))
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                self.active_auth_goals[f"{gid}_result"] = result

        # Service responses
        elif mtype in ("srv_camera_result", "srv_expression_result"):
            sid = msg.get("srv_id")
            if sid:
                self.active_srv_requests[sid] = msg

    # --- Action Server Execution Callbacks ---

    def execute_tts_callback(self, goal_handle):
        logger.info(f"Executing TTS Action Goal: '{goal_handle.request.text}'")
        gid = f"tts_{int(time.time()*1000)}"
        self.active_tts_goals[gid] = goal_handle

        req = {
            "type": "action_tts_goal",
            "goal_id": gid,
            "text": goal_handle.request.text,
            "voice_output": goal_handle.request.voice_output,
            "start_expression": goal_handle.request.start_expression,
            "end_expression": goal_handle.request.end_expression,
            "expression_sound": goal_handle.request.expression_sound
        }
        self.send_to_android(req)

        # Wait for completion from Android
        timeout = 20.0
        start_t = time.time()
        res_key = f"{gid}_result"
        while res_key not in self.active_tts_goals and (time.time() - start_t < timeout):
            time.sleep(0.05)

        if res_key in self.active_tts_goals:
            res = self.active_tts_goals.pop(res_key)
            return res
        else:
            logger.warning(f"TTS Action {gid} timed out")
            self.active_tts_goals.pop(gid, None)
            goal_handle.abort()
            res = TTS.Result()
            res.success = False
            res.expression = self.current_expression
            return res

    def execute_auth_callback(self, goal_handle):
        logger.info(f"Executing Auth Action Goal: '{goal_handle.request.message}'")
        gid = f"auth_{int(time.time()*1000)}"
        self.active_auth_goals[gid] = goal_handle

        req = {
            "type": "action_auth_goal",
            "goal_id": gid,
            "message": goal_handle.request.message
        }
        self.send_to_android(req)

        # Wait for user FaceID / fingerprint prompt completion
        timeout = 30.0
        start_t = time.time()
        res_key = f"{gid}_result"
        while res_key not in self.active_auth_goals and (time.time() - start_t < timeout):
            time.sleep(0.05)

        if res_key in self.active_auth_goals:
            res = self.active_auth_goals.pop(res_key)
            return res
        else:
            logger.warning(f"Auth Action {gid} timed out / cancelled")
            self.active_auth_goals.pop(gid, None)
            goal_handle.abort()
            res = Auth.Result()
            res.success = False
            res.error_message = "Authentication timed out or cancelled by user"
            return res

    # --- Service Callbacks ---

    def enable_camera_callback(self, request, response):
        logger.info(f"Service /enable_camera called: direction={request.direction}, status={request.status}")
        sid = f"cam_{int(time.time()*1000)}"
        self.send_to_android({
            "type": "srv_camera",
            "srv_id": sid,
            "direction": request.direction,
            "status": request.status
        })
        # Wait up to 2 seconds for ack
        for _ in range(20):
            if sid in self.active_srv_requests:
                res = self.active_srv_requests.pop(sid)
                response.success = res.get("success", True)
                response.message = res.get("message", "Camera configured")
                return response
            time.sleep(0.1)

        response.success = True
        response.message = "Camera switched"
        return response

    def set_expression_callback(self, request, response):
        logger.info(f"Service /set_expression called: {request.expression}")
        self.current_expression = request.expression
        self.send_to_android({
            "type": "srv_expression",
            "expression": request.expression,
            "sound": request.expression_sound
        })
        response.success = True
        return response

    def get_expression_callback(self, request, response):
        response.expression = self.current_expression
        return response

    # --- Topic Subscriber Callbacks ---

    def cmd_vel_callback(self, msg: Twist):
        self.send_to_android({
            "type": "topic_cmd_vel",
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z
        })

    def torch_callback(self, msg: Bool):
        self.send_to_android({
            "type": "topic_torch",
            "data": msg.data
        })


def main(args=None):
    rclpy.init(args=args)
    node = RosdroidNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
