import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import argparse
import time

class EulerAnglePlotter:
    def __init__(self, port, baudrate=115200, max_data_points=500):  # 增加数据点数量
        self.ser = serial.Serial(port, baudrate, timeout=0.01)  # 添加超时设置
        self.max_data_points = max_data_points

        # 初始化数据队列
        self.roll_data = deque(maxlen=max_data_points)
        self.pitch_data = deque(maxlen=max_data_points)
        self.yaw_data = deque(maxlen=max_data_points)
        self.time_data = deque(maxlen=max_data_points)
        self.time_counter = 0

        # 设置图形
        self.fig, self.ax = plt.subplots(figsize=(10, 6))  # 增大图形尺寸
        self.roll_line, = self.ax.plot([], [], 'r-', label='Roll', linewidth=1)
        self.pitch_line, = self.ax.plot([], [], 'g-', label='Pitch', linewidth=1)
        self.yaw_line, = self.ax.plot([], [], 'b-', label='Yaw', linewidth=1)

        self.ax.legend(loc='upper left')
        self.ax.set_ylim(-180, 180)
        self.ax.set_xlim(0, max_data_points)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Angle (degrees)')
        self.ax.set_title('Euler Angles')
        self.fig.tight_layout()

        # 预编译正则表达式以提高效率
        self.roll_pattern = re.compile(r'Roll=([-+]?\d*\.\d+)')
        self.pitch_pattern = re.compile(r'Pitch=([-+]?\d*\.\d+)')
        self.yaw_pattern = re.compile(r'Yaw=([-+]?\d*\.\d+)')

        # 性能监控
        self.last_update_time = time.time()
        self.update_count = 0

    def extract_euler_angles(self, line):
        """从字符串中提取欧拉角"""
        try:
            line_str = line.decode('ascii', errors='ignore').strip()

            # 使用预编译的正则表达式匹配
            roll_match = self.roll_pattern.search(line_str)
            pitch_match = self.pitch_pattern.search(line_str)
            yaw_match = self.yaw_pattern.search(line_str)

            if roll_match and pitch_match and yaw_match:
                roll = float(roll_match.group(1))
                pitch = float(pitch_match.group(1))
                yaw = float(yaw_match.group(1))
                return roll, pitch, yaw
        except Exception as e:
            # 减少错误输出频率
            if self.update_count % 100 == 0:
                print(f"Error parsing line: {e}")
        return None, None, None

    def update(self, frame):
        """更新图表数据"""
        try:
            # 一次性读取所有可用数据，减少IO操作
            data_read = []
            while self.ser.in_waiting > 0:
                line = self.ser.readline()
                if line:
                    data_read.append(line)

            # 只处理最后几条数据，避免积压
            for line in data_read[-5:]:  # 只处理最后5条数据
                roll, pitch, yaw = self.extract_euler_angles(line)
                if roll is not None and pitch is not None and yaw is not None:
                    self.time_counter += 1
                    self.time_data.append(self.time_counter)
                    self.roll_data.append(roll)
                    self.pitch_data.append(pitch)
                    self.yaw_data.append(yaw)

            # 只有在有数据时才更新图表
            if self.time_data:
                self.roll_line.set_data(self.time_data, self.roll_data)
                self.pitch_line.set_data(self.time_data, self.pitch_data)
                self.yaw_line.set_data(self.time_data, self.yaw_data)

                # 动态调整X轴范围
                current_max_time = max(self.time_data) if self.time_data else 0
                self.ax.set_xlim(
                    max(0, current_max_time - self.max_data_points),
                    current_max_time + 10
                )

            # 性能监控
            self.update_count += 1
            if self.update_count % 50 == 0:
                current_time = time.time()
                elapsed = current_time - self.last_update_time
                fps = 50 / elapsed if elapsed > 0 else 0
                print(f"Update rate: {fps:.1f} FPS")
                self.last_update_time = current_time

        except Exception as e:
            if self.update_count % 100 == 0:  # 减少错误输出频率
                print(f"Error: {e}")

        return self.roll_line, self.pitch_line, self.yaw_line

    def run(self):
        """运行图表"""
        # 使用更长的间隔以减少更新频率
        ani = animation.FuncAnimation(
            self.fig, self.update, interval=50, blit=True,  # 增加间隔到50ms
            cache_frame_data=False  # 禁用帧缓存
        )
        plt.show()

    def close(self):
        """关闭串口连接"""
        self.ser.close()

def main():
    parser = argparse.ArgumentParser(description='Euler Angle Serial Plotter')
    parser.add_argument('port', help='Serial port name (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--points', type=int, default=500, help='Max data points to display (default: 500)')

    args = parser.parse_args()

    plotter = EulerAnglePlotter(args.port, args.baud, args.points)

    try:
        print(f"Reading Euler angles from {args.port} at {args.baud} baud")
        print("Press Ctrl+C to exit")
        plotter.run()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        plotter.close()

if __name__ == "__main__":
    main()