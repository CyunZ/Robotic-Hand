import serial 
import math
import traceback
import struct



def MainPro():
  try:  
      portx="COM5"  #根据实际修改
      bps=115200
      timex=5
      ser=serial.Serial(portx,bps,timeout=timex)
      print("串口详情参数：", ser)
      print(ser.port)#获取到当前打开的串口名
      print(ser.baudrate)#获取波特率
      while True:
          ba = bytearray()
          sttr = []
          if ser.in_waiting:
              byteArr=ser.read(ser.in_waiting )
              

              if len(byteArr)>=18 and byteArr[0] == 0x11 and byteArr[1] == 0x22 and \
              byteArr[2] == 0x33 and byteArr[17]  == 0x11 and byteArr[16] == 0x22 and \
              byteArr[15] == 0x33  :
                  ba.append(byteArr[6])
                  ba.append(byteArr[5])
                  ba.append(byteArr[4])
                  ba.append(byteArr[3])
                  ptich = struct.unpack("!f",ba)[0]
                  sttr.append(1)
                  sttr.append('%.2f' % ptich)
          
                  ba = bytearray()
                  ba.append(byteArr[10])
                  ba.append(byteArr[9])
                  ba.append(byteArr[8])
                  ba.append(byteArr[7])
                  ptich = struct.unpack("!f",ba)[0]
                  sttr.append(2)
                  sttr.append('%.2f' % ptich)

                  ba = bytearray()
                  ba.append(byteArr[14])
                  ba.append(byteArr[13])
                  ba.append(byteArr[12])
                  ba.append(byteArr[11])
                  ptich = struct.unpack("!f",ba)[0]
                  sttr.append(3)
                  sttr.append('%.2f' % ptich)
          
                  print(sttr)
       
      ser.close()#关闭串口


  except Exception as e:
      print(traceback.format_exc())

if __name__ == '__main__':

  MainPro()



