#!/usr/bin/python3
import time
import serial
import struct
import  re
import numpy as np
from threading import Thread,Lock











class UART:
  def __init__(self):
       #定义了一个用于UART通讯的类：      
       self.single_frame_len=72 #通讯帧长度
       self.data_frame_len=64
       self.head_frame_len=4
       self.tail_frame_len=4

  def parce_multi_frame(self,message,msg_length,msg_single_len,head_length,tail_length):
      #从数据流里找出一帧完整数据，并检验是否可用
      #param  message:          bytes  以字节单位输入的信息
      #param msg_length        int     输入消息的字长    
      #param msg_single_length:       int     一帧消息的字长(68)
      #param head_length：  int     帧头字节量(2)
      #param tail_length ：     int     帧尾字节量(2)

      #return:
      # data_frame_total       lists of list   从message中读出的帧list，每个元素即为每一帧发给下位机的顺序命令list(16个float)   
      # frame_num                      int              返还的完整帧数量
      # lefted_bytes                   bytes          如果读到最后没有切片完整，将剩余字段保存，等到下一轮调用再用
      current_idx=0          #当前读的16进制数据的标号
      frame_num=int(0)         #解析帧数目
      data_frame_total = [] #所有信息的lists
      lefted_bytes=b''
      if isinstance(message,bytes):
          message=message.hex()
      while current_idx <2*msg_length:
         #按一帧的长度去找头&尾
          head_frame=message[current_idx:current_idx+2*head_length]
          tail_frame=message[current_idx+2*(msg_single_len-tail_length):current_idx+2*msg_single_len]
          #头帧校验
          if head_frame!='11223344':
            current_idx+=2
            continue

         #切片一帧数据
          curr_message=message[current_idx:current_idx+2*msg_single_len]
          uart_2=UART()
          head_frame,data_frame,tail_frame=uart_2.parce_single_frame(curr_message,head_length,tail_length)
           #防止数据帧不完整
          #数据不完整时一般是在读到帧末尾处
          if len(curr_message)<2*msg_single_len:
            print('------{} frames received successfully,but with imcomplete frame.------'.format(frame_num))
            lefted_bytes=bytes.fromhex(curr_message)#剩余的字段return
            return  data_frame_total,frame_num,lefted_bytes
          
          #尾帧校验
          if tail_frame!='55667788' : 
            current_idx+=2
            continue
          #data_frame_total += data_frame

          data_frame_total.append(data_frame)#结合
          frame_num+=1
          current_idx+=msg_single_len*2
      print('------{}frames received successfully.------'.format(frame_num))
      return data_frame_total,frame_num,lefted_bytes
    
      
  def   parce_single_frame(self,message,head_length, tail_length):               
      #解析一帧完整数据(默认数据是完整的一帧)
      #param  message:             msg    in bytes
      #param head_length：  int     帧头字节量(2)
      #param tail_length ：     int     帧尾字节量(2)
      #return   hex格式的帧头、帧尾；按顺序构成的各参数的float类型 数组
    
    #转16进制
    if isinstance(message,bytes):
          message=message.hex()
     # assert len(message)==msg_single_len*2, "message  length incorrect!"
  
  
    head_frame=message[:head_length*2]
    tail_frame=message[-tail_length*2:]
    data_frame=message[head_length*2:-tail_length*2]
    
    data_frame=re.findall('.{8}',data_frame)                                                 #按float均分数据,每个float由8个16进制数hex表示
    data_frame=[bytes.fromhex(i)for i in data_frame]                                  #将均分后的msg_list转为byte格式
    data_frame=[struct.unpack('<f',i)[0] for i in data_frame]                           #小端字节序解码 byte to float   
    return  head_frame,data_frame,tail_frame    
 
class WBR_State_Rceive:
#针对xx个观察变量创建的类
#num:观察的数目,默认16
 def __init__(self,num=16):
    self.num=num     
    self.pitch_angle=0.0,
    self.pitch_speed=0.0,
    self.yaw_angle=0.0,
    self.yaw_speed=0.0,
    self.Left_Wheel_torque=0.0,
    self.Left_Wheel_velocity=0.0,
    self.Left_Knee_torque=0.0,
    self.Left_Knee_position=0.0,
    self.Left_Hip_torque=0.0,
    self.Left_Hip_position=0.0,
    self.Right_Wheel_torque=0.0,
    self.Right_Wheel_velocity=0.0,
    self.Right_Knee_torque=0.0,
    self.Right_Knee_position=0.0,
    self.Right_Hip_torque=0.0,
    self.Right_Hip_position=0.0

 def State_update(self,arr:list,callback):
    #输入必须为指定长度列表&&参数均为float类型
    #回调函数call back()
    if not isinstance(arr , list) and all(isinstance(x,float) for x in arr):
     raise TypeError("get_val()需要的输入为float类型的list!")
    
    #检查列表长度
    if len(arr)!=self.num:
       raise TypeError("请检查get_val()输入的list长度!")
  
    #赋值
    self.pitch_angle=arr[0]
    self.pitch_speed=arr[1]
    self.yaw_angle=arr[2]
    self.yaw_speed=arr[3]
    self.Left_Wheel_torque=arr[4]
    self.Left_Wheel_velocity=arr[5]
    self.Left_Knee_torque=arr[6]
    self.Left_Knee_position=arr[7]
    self.Left_Hip_torque=arr[8]
    self.Left_Hip_position=arr[9]
    self.Right_Wheel_torque=arr[10]
    self.Right_Wheel_velocity=arr[11]
    self.Right_Knee_torque=arr[12]
    self.Right_Knee_position=arr[13]
    self.Right_Hip_torque=arr[14]
    self.Right_Hip_position=arr[15]
    callback(self)

 def State_show(self):
    print( 'pitch_angle=', self.pitch_angle,
    'pitch_speed=',self.pitch_speed,
    'yaw_angle=',self.yaw_angle,
    'yaw_speed=',self.yaw_speed,'\n',
    'Left_Wheel_toque=',self.Left_Wheel_torque,
    'Left_Wheel_velocity=',self.Left_Wheel_velocity,
    'Left_Knee_torque=',self.Left_Knee_torque,
    'Left_Knee_position=',self.Left_Knee_position,
    'Left_Hip_torque=',self.Left_Hip_torque,
    'Left_Hip_position=' ,self.Left_Hip_position,'\n',
    'Right_Wheel_torque=' ,self.Right_Wheel_torque,
    'Right_Wheel_velocity=' ,self.Right_Wheel_velocity,
    'Right_Knee_torque=' ,self.Right_Knee_torque,
    'Right_Knee_position=' ,self.Right_Knee_position,
    'Right_Hip_torque=',self.Right_Hip_torque,
    'Right_Hip_position=',self.Right_Hip_position,'\n')


if __name__=='__main__':
   '''''
head_frame='5500'
tail_frame='0055'
x=1.5
y=4.6
z=7.5
u=-3
v=-4
w=6.8563
msg_x=struct.pack('<f',x) #小端字节序压缩 float  to  byte
msg_y=struct.pack('<f',y)
msg_z=struct.pack('<f',z)
msg_u=struct.pack('<f',u)
msg_v=struct.pack('<f',v)
msg_w=struct.pack('<f',w)
msg_head=bytes.fromhex(head_frame)
msg_tail=bytes.fromhex(tail_frame)
#msg=msg_head+msg_x+msg_y+msg_z+msg_tail+msg_head+msg_u+msg_v+msg_w+msg_tail+msg_head#2帧完整信息
#构造一个被截断的数据信息
jd1=1.2458
jd2=2.5905
jd3=-103.6
msg_jd=msg_head+struct.pack('<f',jd1)+struct.pack('<f',jd2)+struct.pack('<f',jd3)+msg_tail
msg_jd_part1=msg_jd[:5]
msg_jd_part2=msg_jd[5:] #随便截为两个部分,5可以改为小于len(msg_jd)的任何数
msg1=msg_head+msg_x+msg_y+msg_z+msg_tail+msg_head+msg_u+msg_v+msg_w+msg_tail+msg_head+msg_jd_part1#2帧完整信息+被截断信息
msg2=msg_jd_part2

print(msg1)
lefted_bytes=b''
uart= UART()
data_total=[]


msg1=lefted_bytes+msg1
data,data_num,lefted_bytes=uart.parce_multi_frame(msg1,len(msg1),uart.single_frame_len,len(msg_head),len(msg_tail))
print(data)
msg2=lefted_bytes+msg2
data,data_num,lefted_bytes=uart.parce_multi_frame(msg2,len(msg2),uart.single_frame_len,len(msg_head),len(msg_tail))
print(data)
 '''''
       #以字节方式接受串口信息
  #def UART_receive(self,serial_port:serial.Serial): 
      # activate UART by "sudo chmod 777 /dev/ttyTHS1"
      #serial_port:定义的串口
'''
  import serial
  例子：
  serial_port = serial.Serial(
  port="/dev/ttyTHS1",
  baudrate=115200,
  bytesize=serial.EIGHTBITS,
  parity=serial.PARITY_NONE,
  timeout=20.0              #超时30秒
  )
'''
serial_port = serial.Serial(
port="/dev/ttyTHS0",
baudrate=460800,
bytesize=serial.EIGHTBITS,
parity=serial.PARITY_NONE,
stopbits=1,
timeout=20.0              #超时20秒
)
# Wait a second to let the port initialize
time.sleep(0.5)
  # Send a simple header
serial_port.write("UART Demonstration Program\r\n".encode())
serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
uart=UART()                                                  #串口类
obs=WBR_State_Rceive(16)                                   #返回一个状态观测
#msg_length=4*uart.single_frame_len#一次性至少读272个字节
msg_length=144
data_total=b''
frame_num=0
lefted_bytes=b''
if  not serial_port.is_open:
   serial_port.open()
   # Wait a second to let the port initialize
time.sleep(0.5)                  
while True:
  data =lefted_bytes +serial_port.read(msg_length)
 # time.sleep(0.05)
  if serial_port.in_waiting >0:
    data_total,frame_num,lefted_bytes=uart.parce_multi_frame(data,len(data),uart.single_frame_len,uart.head_frame_len,uart.tail_frame_len)   
    for i in range(frame_num):
      obs.State_update(data_total[i],WBR_State_Rceive.State_show)

