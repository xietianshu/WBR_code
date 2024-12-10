import serial
import threading
import queue
import time

# 串口初始化
ser = serial.Serial('COM1', 9600, timeout=1)  # 根据你的串口号和波特率调整

# 创建发送和接收队列
send_queue = queue.Queue()
recv_queue = queue.Queue()

# 继承自 threading.Thread 的串口收发线程类
class SerialThread(threading.Thread):
    def __init__(self, mode, ser, send_queue=None, recv_queue=None, send_lock=None, recv_lock=None):
        super().__init__()
        self.mode = mode  # 'send' 或 'recv'
        self.ser = ser    # 串口对象
        self.send_queue = send_queue  # 发送队列
        self.recv_queue = recv_queue  # 接收队列
        self.send_lock = send_lock  # 用于发送锁
        self.recv_lock = recv_lock  # 用于接收锁

    def run(self):
        if self.mode == 'send':
            self.send_data()
        elif self.mode == 'recv':
            self.receive_data()

    def send_data(self):
        while True:
            # 获取要发送的数据
            message = input("请输入要发送的数据：")  # 用户输入要发送的数据
            self.send_queue.put(message)  # 将发送的数据放入队列
            print(f"数据放入发送队列: {message}")

            # 从发送队列中取数据并发送
            while not self.send_queue.empty():
                data = self.send_queue.get()  # 从队列中获取待发送的数据
                
                # 获取锁，发送数据
                with self.send_lock:  # 获取发送锁，确保发送操作互斥
                    self.ser.write(data.encode())  # 发送数据
                    print(f"已发送: {data}")

            time.sleep(1)

    def receive_data(self):
        while True:
            # 检查是否有数据可读取
            if self.ser.in_waiting > 0:  # 检查是否有数据可读
                data = self.ser.read(self.ser.in_waiting)  # 读取所有可用数据
                self.recv_queue.put(data)  # 将接收到的数据放入接收队列
                print(f"数据放入接收队列: {data.decode()}")

            # 从接收队列中取数据并处理
            if not self.recv_queue.empty():
                data = self.recv_queue.get()  # 从队列中获取待处理的接收数据
                print(f"从接收队列获取数据: {data.decode()}")

            time.sleep(0.1)

# 创建两个锁对象
send_lock = threading.Lock()
recv_lock = threading.Lock()

# 创建发送和接收线程对象，分别传入各自的队列和锁
send_thread = SerialThread(mode='send', ser=ser, send_queue=send_queue, send_lock=send_lock)
recv_thread = SerialThread(mode='recv', ser=ser, recv_queue=recv_queue, recv_lock=recv_lock)

# 启动线程
send_thread.start()
recv_thread.start()

# 主线程保持运行，避免退出
while True:
    time.sleep(1)

