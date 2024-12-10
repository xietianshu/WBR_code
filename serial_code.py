import serial
import threading
import queue
import time

# ���ڳ�ʼ��
ser = serial.Serial('COM1', 9600, timeout=1)  # ������Ĵ��ںźͲ����ʵ���

# �������ͺͽ��ն���
send_queue = queue.Queue()
recv_queue = queue.Queue()

# �̳��� threading.Thread �Ĵ����շ��߳���
class SerialThread(threading.Thread):
    def __init__(self, mode, ser, send_queue=None, recv_queue=None, send_lock=None, recv_lock=None):
        super().__init__()
        self.mode = mode  # 'send' �� 'recv'
        self.ser = ser    # ���ڶ���
        self.send_queue = send_queue  # ���Ͷ���
        self.recv_queue = recv_queue  # ���ն���
        self.send_lock = send_lock  # ���ڷ�����
        self.recv_lock = recv_lock  # ���ڽ�����

    def run(self):
        if self.mode == 'send':
            self.send_data()
        elif self.mode == 'recv':
            self.receive_data()

    def send_data(self):
        while True:
            # ��ȡҪ���͵�����
            message = input("������Ҫ���͵����ݣ�")  # �û�����Ҫ���͵�����
            self.send_queue.put(message)  # �����͵����ݷ������
            print(f"���ݷ��뷢�Ͷ���: {message}")

            # �ӷ��Ͷ�����ȡ���ݲ�����
            while not self.send_queue.empty():
                data = self.send_queue.get()  # �Ӷ����л�ȡ�����͵�����
                
                # ��ȡ������������
                with self.send_lock:  # ��ȡ��������ȷ�����Ͳ�������
                    self.ser.write(data.encode())  # ��������
                    print(f"�ѷ���: {data}")

            time.sleep(1)

    def receive_data(self):
        while True:
            # ����Ƿ������ݿɶ�ȡ
            if self.ser.in_waiting > 0:  # ����Ƿ������ݿɶ�
                data = self.ser.read(self.ser.in_waiting)  # ��ȡ���п�������
                self.recv_queue.put(data)  # �����յ������ݷ�����ն���
                print(f"���ݷ�����ն���: {data.decode()}")

            # �ӽ��ն�����ȡ���ݲ�����
            if not self.recv_queue.empty():
                data = self.recv_queue.get()  # �Ӷ����л�ȡ������Ľ�������
                print(f"�ӽ��ն��л�ȡ����: {data.decode()}")

            time.sleep(0.1)

# ��������������
send_lock = threading.Lock()
recv_lock = threading.Lock()

# �������ͺͽ����̶߳��󣬷ֱ�����ԵĶ��к���
send_thread = SerialThread(mode='send', ser=ser, send_queue=send_queue, send_lock=send_lock)
recv_thread = SerialThread(mode='recv', ser=ser, recv_queue=recv_queue, recv_lock=recv_lock)

# �����߳�
send_thread.start()
recv_thread.start()

# ���̱߳������У������˳�
while True:
    time.sleep(1)

