import os
import rosbag
from sensor_msgs.msg import Imu

def extract_imu_data(bag_file, output_file, timestamp_limit):
    # 打开 rosbag 文件
    with rosbag.Bag(bag_file, 'r') as bag:
        with open(output_file, 'w') as f:
            for topic, msg, t in bag.read_messages(topics=['/imu/imu_uncompensated']):
                if msg._type == 'sensor_msgs/Imu':
                    # 获取时间戳（秒和纳秒）
                    secs = msg.header.stamp.secs
                    nsecs = msg.header.stamp.nsecs
                    # 获取角速度和线加速度数据
                    ang_vel_x = msg.angular_velocity.x
                    ang_vel_y = msg.angular_velocity.y
                    ang_vel_z = msg.angular_velocity.z
                    lin_acc_x = msg.linear_acceleration.x
                    lin_acc_y = msg.linear_acceleration.y
                    lin_acc_z = msg.linear_acceleration.z

                    # 将数据写入文件
                    f.write(f"IMU {secs + nsecs*1e-9} {ang_vel_x} {ang_vel_y} {ang_vel_z} {lin_acc_x} {lin_acc_y} {lin_acc_z}\n")


    # 关闭 rosbag 文件
    bag.close()

if __name__ == "__main__":
    # 替换成您的 rosbag 文件路径和输出文件路径
    bag_file = '/media/jason/Samsung_T5/slam_data/TRI/combined_kri.bag'
    output_file = '/home/jason/DPGO/FixedLagSmoother/python/imu_data.txt'
    # 设置时间戳限制
    timestamp_limit = 1689804936
    extract_imu_data(bag_file, output_file, timestamp_limit)
    print(f"IMU data before timestamp {timestamp_limit} has been extracted and saved to {output_file}")

