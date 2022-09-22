#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import serial.tools.list_ports
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from message_interfaces.msg import Imu

python_version = platform.python_version()[0]
acc_k = 100.0
key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True]

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


# crc 校验
def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])



# 16 进制转 ieee 浮点数
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str =hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        if python_version == '2':
            ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
        if python_version == '3':
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data



# 处理串口数据
def handleSerialData(raw_data):

    #print(raw_data)
    global acc_k,buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

        

       

    key += 1
    if buff[0] != 0xaa:
        key = 0
        return
    if key < 3:
        return
    if buff[1] != 0x55:
        key = 0
        return
    if key < buff[2] + 5:  # 根据数据长度位的判断, 来获取对应长度数据
        return

    else:
        data_buff = list(buff.values())  # 获取字典所以 value

    

        if buff[2] == 0x2c and pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                angularVelocity = data[1:4]
                acceleration = data[4:7]
                magnetometer = data[7:10]
            else:
                print('校验失败')
            pub_flag[0] = False
        elif buff[2] == 0x14 and pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                angle_degree = data[1:4]
            else:
                print('校验失败')
            pub_flag[1] = False
        else:
            print("该数据处理类没有提供该 " + str(buff[2]) + " 的解析")
            print("或数据错误")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True:
            return
        pub_flag[0] = pub_flag[1] = True
        acc_k = math.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2)

#         print('''
# 加速度(m/s²)：
#     x轴：%.2f
#     y轴：%.2f
#     z轴：%.2f

# 角速度(rad/s)：
#     x轴：%.2f
#     y轴：%.2f
#     z轴：%.2f

# 欧拉角(°)：
#     x轴：%.2f
#     y轴：%.2f
#     z轴：%.2f

# 磁场：
#     x轴：%.2f
#     y轴：%.2f
#     z轴：%.2f
# ''' % (acceleration[0] * -9.8 / acc_k, acceleration[1] * -9.8 / acc_k, acceleration[2] * -9.8 / acc_k,
#        angularVelocity[0], angularVelocity[1], angularVelocity[2],
#        angle_degree[0], angle_degree[1], angle_degree[2],
#        magnetometer[0], magnetometer[1], magnetometer[2]
#       ))


    data = []
    data=open("/home/swy/dev_ws/src/imu/imudata.txt",'w+')
    print("%.2f" %float(acceleration[0] * -9.8 / acc_k), "%.2f" % float(acceleration[1] * -9.8 / acc_k), "%.2f" %  float(acceleration[2] * -9.8 / acc_k),
                "%.2f" %  angularVelocity[0], "%.2f" % angularVelocity[1], "%.2f" % angularVelocity[2],
                "%.2f" %angle_degree[0], "%.2f" % angle_degree[1],  "%.2f" % angle_degree[2],
                "%.2f" % magnetometer[0], "%.2f" % magnetometer[1],"%.2f" % magnetometer[2],file=data)
    data.close()
    


class NodePublisher02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.command_publisher_ = self.create_publisher(Imu,"imu_command", 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        
    
    def timer_callback(self):
        """
        定时器回调函数
        """
        
        #python_version = platform.python_version()[0]
        

    
        find_ttyUSB()
        port = "/dev/ttyUSB1"
        baudrate = 921600


        try:
            hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            if hf_imu.isOpen():
                print("\033[32m串口打开成功...\033[0m")
            else:
                hf_imu.open()
                print("\033[32m打开串口成功...\033[0m")
        except Exception as e:
            print(e)
            print("\033[31m串口打开失败\033[0m")
            exit(0)
        else:
            while True:
                try:
                    buff_count = hf_imu.inWaiting()
                except Exception as e:
                    print("exception:" + str(e))
                    print("imu 失去连接，接触不良，或断线")
                    exit(0)
                else:
                    if buff_count > 0:
                        buff_data = hf_imu.read(buff_count)
                        for i in range(0, buff_count):
                            handleSerialData(buff_data[i])   
                            
                    f = open("/home/swy/dev_ws/src/imu/imudata.txt")

        #读取一行数据

                    byt = f.readline()

                    print(byt)
        
                    #acc_k = math.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2)
                    msg = Imu()
                    print('acc_k=%f' % acc_k)
                    #print("%.2f" % float(acceleration[0] * -9.8 / acc_k))
                    msg.acceleration_x = str("%.2f" % float(acceleration[0] * -9.8 / acc_k))
                    msg.acceleration_y = str("%.2f" % float(acceleration[1] * -9.8 / acc_k))
                    msg.acceleration_z = str("%.2f" % float(acceleration[2] * -9.8 / acc_k))
                    msg.angular_velocity_x = str("%.2f" % angularVelocity[0])
                    msg.angular_velocity_y = str("%.2f" % angularVelocity[1])
                    msg.angular_velocity_z = str("%.2f" % angularVelocity[2])
                    msg.angle_degree_x = str("%.2f" % angle_degree[0])
                    msg.angle_degree_y = str("%.2f" % angle_degree[1])
                    msg.angle_degree_z = str("%.2f" % angle_degree[2])
                    msg.magnetometer_x = str("%.2f" % magnetometer[0])
                    msg.magnetometer_y = str("%.2f" % magnetometer[1])
                    msg.magnetometer_z = str("%.2f" % magnetometer[2])
                    #print("%s" % msg.acceleration_x)
                    #msg.data =str("%.2f" %float(acceleration[0] * -9.8 / acc_k))+" "+str("%.2f" % float(acceleration[1] * -9.8 / acc_k))+" "+str("%.2f" %  float(acceleration[2] * -9.8 / acc_k))+" "+str("%.2f" %  angularVelocity[0])+" "+str("%.2f" % angularVelocity[1])+" "+str("%.2f" % angularVelocity[2])+" "+str("%.2f" %angle_degree[0])+" "+str("%.2f" % angle_degree[1])+" "+str("%.2f" % angle_degree[2])+" "+str("%.2f" % magnetometer[0])+" "+str("%.2f" % magnetometer[1])+" "+str("%.2f" % magnetometer[2])
                    #msg.data = str(byt)
                    self.command_publisher_.publish(msg) 
                    self.get_logger().info(f'''\nacceleration_x: {msg.acceleration_x}\tacceleration_y: {msg.acceleration_y}\tacceleration_z: {msg.acceleration_z}\n
angular_velocity_x: {msg.angular_velocity_x}\tangular_velocity_y: {msg.angular_velocity_y}\tangular_velocity_z: {msg.angular_velocity_z}\n
angle_degree_x: {msg.angle_degree_x}\tangle_degree_y: {msg.angle_degree_y}\tangle_degree_z: {msg.angle_degree_z}\n
magnetometer_x: {msg.magnetometer_x}\tmagnetometer_y: {msg.magnetometer_y}\tmagnetometer_z: {msg.magnetometer_z}\n
OK''')    #打印一下发布的数据





def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodePublisher02("imu_publisher")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy