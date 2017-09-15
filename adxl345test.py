#-*- coding: UTF-8 -*-
import sys,os
import spidev
import time,datetime
import math
import socket,struct
import RPi.GPIO as gpio
import numpy as np 
#import matplotlib
#matplotlib.use("Pdf")
#import matplotlib.pyplot as plt
EARTH_GRAVITY_MS2       =  9.80665
SCALE_MULTIPLIER        =  0.00390 # per LSB
OFFSET_SCALE_MULTIPLIER = 63.50000 # 1G / 0.0156 per LSB
THRESH_SCALE_MULTIPLIER = 16.00000 # 1G / 0.0625 per LSB

DEVID                   = 0x00

NO_OP                   = 0x00
WRITE_OP                = 0x00
MB_OP                   = 0x40
READ_OP                 = 0x80

OFFSET_BLOCK            = 0x1E
OFFSET_X                = 0x1E
OFFSET_Y                = 0x1F
OFFSET_Z                = 0x20

ACT_THRESH              = 0x24
INACT_THRESH            = 0x25
INACT_TIME              = 0x26
ACT_INACT_CTL           = 0x27

BW_RATE                 = 0x2C
POWER_CTL               = 0x2D
INT_ENABLE              = 0x2E
INT_MAP                 = 0x2F
INT_SOURCE              = 0x30
DATA_FORMAT             = 0x31
AXES_DATA_BLOCK         = 0x32

BW_RATE_1600HZ          = 0x0F
BW_RATE_800HZ           = 0x0E
BW_RATE_400HZ           = 0x0D
BW_RATE_200HZ           = 0x0C
BW_RATE_100HZ           = 0x0B
BW_RATE_50HZ            = 0x0A
BW_RATE_25HZ            = 0x09

RANGE_2G                = 0x00
RANGE_4G                = 0x01
RANGE_8G                = 0x02
RANGE_16G               = 0x03
FULL_RES                = 0x08

MEASURE                 = 0x08

INACT_ENABLE_Z          = 0x01
INACT_ENABLE_Y          = 0x02
INACT_ENABLE_X          = 0x04

ACT_ENABLE_Z            = 0x10
ACT_ENABLE_Y            = 0x20
ACT_ENABLE_X            = 0x40

INT_ENABLE_INACTIVITY   = 0x08
INT_ENABLE_ACTIVITY     = 0x10

INT_MAP_INACTIVITY_INT1 = 0x00
INT_MAP_INACTIVITY_INT2 = 0x08
INT_MAP_ACTIVITY_INT1   = 0x00
INT_MAP_ACTIVITY_INT2   = 0x10
''' 用一個class把adxl做成一個物件
python 的命名規則
底線在後面 >>避免特殊用詞
前雙底線>>不可以繼承 
	>>我假如要定義另一個物件，這項參數不可以改  
	>>也可以說這項參數是每個物件都要有並且不可以變的
'''
class ADXL345:
    def __init__(self, cs):		#初始化的時候 要輸入cs腳位 >>不一定限制那兩隻
        self.cs = cs  			#設定cs腳位
        self.offset = {"x": 0, "y": 0, "z": 0} #校準剛開始是0
        # Setup GPIO.
        gpio.setmode(gpio.BCM)
        gpio.setup(self.cs, gpio.OUT)
        gpio.output(self.cs, gpio.HIGH)
        self.__OpenSPI() 		# Setup SPI.
        self.__Setup()  		#設定速率範圍還有量測模式
        return
        #我假如要定義另一個物件，這項參數不可以改，也可以說這項參數是每個物件都要有並且不可以變的
    def __OpenSPI(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = 3
        self.spi.max_speed_hz = 5000000 #5MHZ
        return

    def __Setup(self):
        self.__WriteCommand(BW_RATE, [BW_RATE_100HZ])
        self.__WriteCommand(DATA_FORMAT, [FULL_RES | RANGE_16G])
        self.__WriteCommand(POWER_CTL, [MEASURE])
        time.sleep(0.1)
        return

    def __WriteCommand(self, address, data): #寫入reg用的func
        address = (address | WRITE_OP)
        if len(data) > 1:
            address = address | MB_OP
        if isinstance(data, list) or isinstance(data, tuple):
            gpio.output(self.cs, gpio.LOW)
            self.spi.xfer2([address] + data + [NO_OP])
            gpio.output(self.cs, gpio.HIGH)
        return

    def __ReadCommand(self, address, n = 1):
        address = (address | READ_OP)
        if n > 1:
            address = address | MB_OP
        gpio.output(self.cs, gpio.LOW)
        t = self.spi.xfer2([address] + ([NO_OP] * n))
        gpio.output(self.cs, gpio.HIGH)
        return t #回傳xfer2的東西  xfer2回傳[value] 是一個list 

    def Calibrate(self, samples = 10, x = 0.0, y = 0.0, z = 1.0, t = 0.05):
        self.__WriteCommand(OFFSET_BLOCK, [0x00, 0x00, 0x00])
        time.sleep(0.1)
        data = self.GetMeasures(0.0, 0.0, 0.0) #輸入OFFSET
        self.offset['x'] = -data['x'] + x #offect是負的量測到的值
        self.offset['y'] = -data['y'] + y
        self.offset['z'] = -data['z'] + z
        time.sleep(0.1)
        for i in range(1, samples, 1): #writed 取樣10次但是不知道再算什麼
            data = self.GetMeasures(0.0, 0.0, 0.0)
            self.offset['x'] = (self.offset['x'] + -data['x'] + x) / 2.0
            self.offset['y'] = (self.offset['y'] + -data['y'] + y) / 2.0
            self.offset['z'] = (self.offset['z'] + -data['z'] + z) / 2.0
            time.sleep(0.1)
        xo = int(self.offset['x'] * OFFSET_SCALE_MULTIPLIER) #不知道為什麼要乘補償因子
        yo = int(self.offset['y'] * OFFSET_SCALE_MULTIPLIER)
        zo = int(self.offset['z'] * OFFSET_SCALE_MULTIPLIER)
        self.__WriteCommand(OFFSET_BLOCK, [min(127, max(-127, xo)), min(127, max(-127, yo)), min(127, max(-127, zo))])
        time.sleep(0.1)
        calibration = {"x": True, "y": True, "z": True}
        data = self.GetMeasures(0.0, 0.0, 0.0)
        if abs(data['x']) > t:  #假如量出來的值大於0.05，就不採用
            calibration['x'] = False
            self.__WriteCommand(OFFSET_X, [0x00])
        if abs(data['y']) > t:
            calibration['y'] = False
            self.__WriteCommand(OFFSET_Y, [0x00])
        if abs(data['z']) > t:
            calibration['z'] = False
            self.__WriteCommand(OFFSET_Z, [0x00])
        time.sleep(0.1)
        return calibration

    def CombinedAxisSqrtActivity(self, offset_x = 0.0, offset_y = 0.0, offset_z = 0.0):
        data = self.GetMeasures(offset_x, offset_y, offset_z)
        return math.sqrt(data['x'] * data['x'] + data['y'] * data['y'] + data['z'] * data['z'])
        	#平方再
    def GetID(self):
        id = self.__ReadCommand(DEVID)
        return id[1]

    def GetMeasures(self, offset_x = 0.0, offset_y = 0.0, offset_z = 0.0):
        data = self.__ReadCommand(AXES_DATA_BLOCK, 6) #讀輸出資料reg
        x = int((data[2] << 8) | data[1]) #把x的值整理出來
        if x & 0x800: #??1000 0000 0000 若X是這個值 為什麼會有3BYTE?? 
            x = x - 0xFFFF #X減掉1111 1111 1111 
        y = int((data[4] << 8) | data[3])
        if y & 0x800:
            y = y - 0xFFFF
        z = int((data[6] << 8) | data[5])
        if z & 0x800:#0000 1000 0000 0000 0000 0000 
            z = z - 0xFFFF
        x = x * SCALE_MULTIPLIER #lsb = 63.5 mg 
        y = y * SCALE_MULTIPLIER
        z = z * SCALE_MULTIPLIER
        x = x + offset_x
        y = y + offset_y
        z = z + offset_z
        return {"x": x, "y": y, "z": z} #我猜是回傳float

    def GetOffsetsFromCalibration(self):
        return self.offset

    def SetupInterruptAbove(self, gpio_pin, int_pin, axis_x, axis_y, axis_z, thresh):
        gpio.setup(gpio_pin, gpio.IN)
        to = int(thresh * THRESH_SCALE_MULTIPLIER)
        axs_ctl = 0
        if axis_x:
            axs_ctl = axs_ctl | ACT_ENABLE_X
        if axis_y:
            axs_ctl = axs_ctl | ACT_ENABLE_Y
        if axis_z:
            axs_ctl = axs_ctl | ACT_ENABLE_Z
        if int_pin == 1:
            int_map = INT_MAP_ACTIVITY_INT1
        elif int_pin == 2:
            int_map = INT_MAP_ACTIVITY_INT2
        else:
            raise ValueError('Invalid INT pin requested.')
        self.__WriteCommand(ACT_THRESH,    [min(0xFF, max(0x00, to))])
        self.__WriteCommand(ACT_INACT_CTL, [axs_ctl])
        self.__WriteCommand(INT_MAP,       [int_map])
        self.__WriteCommand(INT_ENABLE,    [INT_ENABLE_ACTIVITY])
        time.sleep(0.1)
        return

    def SetupInterruptBelow(self, gpio_pin, int_pin, axis_x, axis_y, axis_z, thresh, t_time = 0):
        gpio.setup(gpio_pin, gpio.IN)
        to = int(thresh * THRESH_SCALE_MULTIPLIER)
        axs_ctl = 0
        if axis_x:
            axs_ctl = axs_ctl | INACT_ENABLE_X
        if axis_y:
            axs_ctl = axs_ctl | INACT_ENABLE_Y
        if axis_z:
            axs_ctl = axs_ctl | INACT_ENABLE_Z
        if int_pin == 1:
            int_map = INT_MAP_ACTIVITY_INT1
        elif int_pin == 2:
            int_map = INT_MAP_ACTIVITY_INT2
        else:
            raise ValueError('Invalid INT pin requested.')
        self.__WriteCommand(INACT_THRESH,  [min(0xFF, max(0x00, to))])
        self.__WriteCommand(ACT_INACT_CTL, [axs_ctl])
        self.__WriteCommand(INT_MAP,       [int_map])
        self.__WriteCommand(INACT_TIME,    [t_time])
        self.__WriteCommand(INT_ENABLE,    [INT_ENABLE_INACTIVITY])
        time.sleep(0.1)
        return

    def ResetInterrupts(self):
        self.__WriteCommand(ACT_THRESH,    [0x00])
        self.__WriteCommand(INACT_THRESH,  [0x00])
        self.__WriteCommand(ACT_INACT_CTL, [0x00])
        self.__WriteCommand(INT_MAP,       [0x00])
        self.__WriteCommand(INACT_TIME,    [0x00])
        self.__WriteCommand(INT_ENABLE,    [0x00])
        time.sleep(0.1)
        return

    def Remove(self):
        self.ResetInterrupts()
        gpio.cleanup()
        return

    def ClearInterrupts(self):
        self.__ReadCommand(INT_SOURCE) # Just reading shall clear the flags.
        return
	########################################自己加
    def WriteCommand(self, address, data):
        address = (address | WRITE_OP)
        if len(data) > 1:
            address = address | MB_OP
        if isinstance(data, list) or isinstance(data, tuple):
            gpio.output(self.cs, gpio.LOW)
            self.spi.xfer2([address] + data + [NO_OP])
            gpio.output(self.cs, gpio.HIGH)
        return

    def ReadCommand(self, address, n = 1):
        address = (address | READ_OP)
        if n > 1:
            address = address | MB_OP
        gpio.output(self.cs, gpio.LOW)
        t = self.spi.xfer2([address] + ([NO_OP] * n))
        gpio.output(self.cs, gpio.HIGH)
        return t	

def ConvertGravityToAcceleration(g):
    return g * EARTH_GRAVITY_MS2

def ConvertAccelerationToGravity(a):
    return a / EARTH_GRAVITY_MS2

ADXL_CS  = 25
INT1_PIN = 20
INT2_PIN = 21

if __name__ == "__main__":
    adxl345 = ADXL345(ADXL_CS)
    print ("Device ID = 0x%x" % (adxl345.GetID()) )
    adxl345.WriteCommand(BW_RATE,[0b1111]) #rate
    bw= bin( adxl345.ReadCommand(BW_RATE)[1] )
    print ("bwreg: " +str (bw) )
    adxl345.WriteCommand(DATA_FORMAT,[0b00001011])
    print ("DATA_FORMAT reg :  "+str(bin( adxl345.ReadCommand(DATA_FORMAT,1)[1])) )
    print('test a time :'+str(adxl345.GetMeasures(0,0,0)))
    yesorno = input("是否要繼續?輸入y")
    if not yesorno== 'y':
    	sys.exit()
Times = 10000
samplerate =0.0

#input_state = gpio.input(INT1_PIN)
#if input_state != False:
#    print ("Interrupt fired!")
#    adxl345.ClearInterrupts()
axis_offset = adxl345.GetOffsetsFromCalibration()

data =np.zeros([3,Times ]) #zeros 預設是float [0~Times] *3
ii =range(Times) #0~Times
#delay_time = 1/300

startTime = time.time()

for i in ii:
	a_data= adxl345.GetMeasures(axis_offset['x'], axis_offset['y'], axis_offset['z'])
	data[0][i]=a_data['x']
	data[1][i]=a_data['y']
	data[2][i]=a_data['z']
	#time.sleep(delay_time)
endTime=time.time()
print('startTime:  %.4f   endTime: %.4f ' %(startTime,endTime))

samplerate=Times/(endTime-startTime)

print('samplerate : %.5f'%(samplerate) )
adxl345.Remove()


#print (type(data['x']))
#print ("x = %.3f" % (a_data['x']))
#print ("y = %.3f" % (data['y']))
#print ("z = %.3f" % (data['z']))
#print ("")
#time.sleep(1.0)
#print(len(data[:Times][0]))
#print(len(range(Times)))

#save_array  data格式 [0~Times] *3 rate >int
np.savez('adxl345.npz', data = data ,samplerate =samplerate)
#rename
dt =str(datetime.datetime.now())
name = time.strftime("%Y_%m_%d_%H_%M_%S_adxl.npz")
os.rename('adxl345.npz', name)

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect(('192.168.43.62',12308))


'''我覺得路徑應該要可以輸出列表再輸入的'''
''' 或者直接用檔案路徑+name '''
filepath =os.getcwd()+'/'+name #(r'/home/pi/test/ftp-sync/tcp/'+name) #input('Please Enter chars:\r\n')		#輸入檔案路境
        #再rpi要用絕對路徑
if os.path.isfile(filepath):							#檔案存在
    print('into if begin send file info ')        
    fileinfo_size=struct.calcsize('!128si') #定义打包规则
	#定义文件头信息，包含文件名和文件大小
    fhead = struct.pack('!128si',bytes(os.path.basename(filepath) , 'utf-8' ),os.stat(filepath).st_size)
    #fhead = struct.pack('!128si',os.path.basename(filepath) ,os.stat(filepath).st_size)
	#把路徑跟大小打包進去 fhead
    s.sendall(fhead) 
    print ('client filepath: ',filepath)
	# with open(filepath,'rb') as fo: 这样发送文件有问题，发送完成后还会发一些东西过去
fo = open(filepath,'rb') #讀取檔案
while True:
#print('into while2')
    filedata = fo.read(1024)
    if not filedata:
        break
    s.send(filedata)
fo.close()
print ('send over...')
s.close()

'''
#set picture
plt.figure(3,figsize=(8,5))
plt.plot(range(Times),data[:Times][0],label='x')
plt.plot(range(Times),data[:Times][1],label='y')
plt.plot(range(Times),data[:Times][2],label='z')
plt.legend(loc='best')
plt.title ( "samplerate: "+str(samplerate) ) 
#save the pic
filename=time.strftime("%Y%m%d_%H%M%S",time.localtime())
dirPath=os.path.dirname(os.path.abspath(sys.argv[0]))
filePath=os.path.join(dirPath,filename+".jpg")
plt.savefig(filePath)
print("file: {}".format(filePath))
'''


'''    axis_health = adxl345.Calibrate(10, 0.0, 0.0, 1.0, 0.05)
    axis_offset = adxl345.GetOffsetsFromCalibration()
    if axis_health['x'] == False:
        print ( "Device x-axis calibration: Failed. Using software offset.") 
    else:
        print ("Device x-axis calibration: Success.")
        axis_offset['x'] = 0.0
    if axis_health['y'] == False:
        print ("Device y-axis calibration: Failed. Using software offset.")
    else:
        print ("Device y-axis calibration: Success.")
        axis_offset['y'] = 0.0
    if axis_health['z'] == False:
        print ("Device z-axis calibration: Failed. Using software offset.")
    else:
        print ("Device z-axis calibration: Success.")
        axis_offset['z'] = 0.0
    if axis_health['x'] or axis_health['y'] or axis_health['z']: #若有一軸是健康的就設定中斷
        print ("Setting up an Interrupt (2G).")
        #adxl345.SetupInterruptAbove(INT1_PIN, 1, axis_health['x'], axis_health['y'], axis_health['z'], 2.0)
        #adxl345.ClearInterrupts()
        try:
            while True:
				#input_state = gpio.input(INT1_PIN)
				#if input_state != False:
				#    print ("Interrupt fired!")
				#    adxl345.ClearInterrupts()
				data = adxl345.GetMeasures(axis_offset['x'], axis_offset['y'], axis_offset['z'])
				print (type(data['x']))
				print ("x = %.3f" % (data['x']))
				print ("y = %.3f" % (data['y']))
				print ("z = %.3f" % (data['z']))
				print ("")
				time.sleep(1.0)
        finally:
            adxl345.Remove()
           
    #else:
        print ("Unable to setup an interrupt. Using manual detection.")
        try:
            data =np.zeros[1000] #zeros 預設是float
            while True:
				#if adxl345.CombinedAxisSqrtActivity(axis_offset['x'], axis_offset['y'], axis_offset['z']) > 1.2:
				#    print ("Movement detected!")
				data = adxl345.GetMeasures(axis_offset['x'], axis_offset['y'], axis_offset['z'])
				#no attribute的問題還沒解決 
				
				print (type(data['x']))
				print ("x = %.3f" % (data['x']))
				print ("y = %.3f" % (data['y']))
				print ("z = %.3f" % (data['z']))
				print( "")
				time.sleep(1.0)
        finally:
            adxl345.Remove()
'''