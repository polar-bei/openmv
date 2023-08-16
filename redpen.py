import pyb
from pyb import Pin
import sensor
from pid import PID
import math

# PID 初始化 -------------------------------------
pan_pid  = PID(p=0.1, i=0.01, d=0, imax=0.3)  # 底盘的PID v4 目前最好的版本
tilt_pid = PID(p=0.1, i=0.01, d=0, imax=0.3)  # 上下的PID v4 目前最好的版本

# 摄像头初始化 ------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 160 x 120 自左到右，自上到下
#sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_vflip(True)  # 垂直翻转
sensor.set_hmirror(True)  # 水平翻转
#sensor.set_auto_gain(False) # 关闭增益（色块识别时必须要关）
#sensor.set_auto_exposure(False, 1000) # 在这里调节曝光度，调节完可以比较清晰地看清激光点
sensor.skip_frames(10)

# 电机对象初始化 -----------------------------------
# pan_servo: -79 ~ 85  左边 ~ 右边
# tilt_servo: -79 ~ 85 上边 ~ 下边

pan_servo = pyb.Servo(1)   # create a servo object on position P7 #在P7引脚创建servo对象
tilt_servo = pyb.Servo(2)   # create a servo object on position P8 #在P8引脚创建servo对象

pan_servo.calibration(500,2500,1500)
tilt_servo.calibration(500,2500,1500)

# LED 初始化 -------------------------------------
redLed = pyb.LED(1)  # 初始化红灯
redLed.off()  # 关闭灯光

# 变量定义 ----------------------------------------
subject = 1  # 当前功能, 可选值: 1, 2, 3, 4, 默认值应当为 1
red_threshold  = (66, 100, 19, 127, -16, 127)  # 黑线外部红色激光笔的阈值
#red_threshold  = (20, 50, 20, 70, 10, 60)  # 黑线内部的红色激光笔的阈值
#red_threshold  = (44, 100, -81, -41, 5, 73)  # 绿色激光笔的阈值
black_threshold = [(33, 0, -128, 87, -46, 127)]  # 黑色矩形的阈值
rectangleAngle = []  # 保存矩形角的横纵坐标
max_rect = None  # 最大矩形
max_blob = None  # 最大色块
preSubject = 1  # 即将选择的题目
rectCnt = 0  # 用于记录矩形识别的一段时间
PIDCnt = 0  # PID 计数器
# 铅笔框所需的变量
pencilCnt = 0
pencilPoint = 0
pencilStep = 0
pencilStepTotal = 8
# 固定黑色矩形所需的变量
fixedBlackRectCnt = 0
fixedBlackRectPoint = 0
fixedBlackRectStep = 0
fixedBlackRectStepTotal = 4
# 移动黑色矩形所需的变量
laser_point_stable_n = 0  # 激光稳定需要的点数
moveBlackRectPoint = 0
moveBlackRectStep = 0
moveBlackRectStepTotal = 10

# 常量定义 ----------------------------------------
origin = [1, -59]  # 原点
top_left     =  [origin[0]+12,origin[1]-10]     # 框线 - 左上角
top_right    =  [origin[0]-11,origin[1]-8]     # 框线 - 右上角
bottom_right =  [origin[0]-8,origin[1]+17]     # 框线 - 右下角
bottom_left  =  [origin[0]+12,origin[1]+17] # 框线 - 左下角
top_left_third      = [origin[0]+7,origin[1]-3]   # 黑色边框 - 左上角
top_right_third     = [origin[0]-7,origin[1]-2.2]   # 黑色边框 - 右上角
bottom_right_third  = [origin[0]-6,origin[1]+8]   # 黑色边框 - 右下角
bottom_left_third   = [origin[0]+7,origin[1]+8]   # 黑色边框 - 左下角
angleList = [top_left, top_right, bottom_right, bottom_left]
fixedBlackRectAngleList =  [top_left_third, top_right_third, bottom_right_third, bottom_left_third]

# 引脚初始化 --------------------------------
resetKey =  Pin('P0', Pin.IN, Pin.PULL_UP)  # 复位键输入
pausekey =  Pin('P1', Pin.IN, Pin.PULL_UP)  # 暂停键输入
selectKey =  Pin('P2', Pin.IN, Pin.PULL_UP)  # 选择键输入
confirmKey =  Pin('P3', Pin.IN, Pin.PULL_UP)  # 确认键键输入

# 寻找最大黑色矩形 --------------------------------
def find_max_rect(rects):
    max_size = 0
    max_rect = []
    for rect in rects:
        if rect.magnitude() > max_size:
            max_rect = rect
            max_size = rect.magnitude()
    return max_rect

# 寻找最大色块 ----------------------------------
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

# 定义安全范围 ------------------------------------
# 控制左右范围
def panMove(panAngle):
    global top_left
    global top_right
    global bottom_left
    global bottom_right
    # 左端取最大值
    if top_left[0] > bottom_left[0]:
        left = top_left[0]
    else:
        left = bottom_left[0]
    # 右端取最小值
    if top_right[0] < bottom_right[0]:
        right = top_right[0]
    else:
        right = bottom_right[0]
    if panAngle<=right:
        panAngle=right
    if panAngle>=left:
        panAngle=left
    return panAngle

# 控制上下范围
def tiltMove(tiltAngle):
    global top_left
    global top_right
    global bottom_right
    global bottom_left
    # 上端取最小值
    if top_left[1] < top_right[1]:
        top = top_left[1]
    else:
        top = top_right[1]
    # 下端取最大值
    if bottom_left[1] > bottom_right[1]:
        bottom = bottom_left[1]
    else:
        bottom = bottom_right[1]
    if tiltAngle<=top:
        tiltAngle=top
    if tiltAngle>=bottom:
        tiltAngle=bottom
    return tiltAngle

# 按键消抖 ----------------------------------------
def keyPressOn(pin):
    if pin.value() == 0:
        pyb.delay(20)
        if pin.value() == 1:
            return 1
    return 0

# LED 灯处理程序 -----------------------------------
def LightLED(nums):
    for l in range(0, nums, 1):
        redLed.on()
        pyb.delay(200)
        redLed.off()
        pyb.delay(200)

# 移动云台 -------------------------------------
def moveYunTai(target_x, target_y, origin_x, origin_y):
    # 计算偏差
    #print("origin:",origin_x,origin_y)
    #print("target:",target_x,target_y)
    pan_error = target_x - origin_x
    tilt_error = target_y - origin_y
    # PID计算
    pan_output = pan_pid.get_pid(pan_error,1)
    tilt_output = tilt_pid.get_pid(tilt_error,1)
    #print("output:",pan_output,tilt_output)
    # PID 输出限幅
    limit = 1.9
    if pan_output >= limit : pan_output = limit
    if pan_output <= -limit: pan_output = -limit
    if tilt_output >= limit : tilt_output = limit
    if tilt_output <= -limit: tilt_output = -limit
    # 保存最终角度
    pan_angle = pan_servo.angle() - pan_output
    tilt_angle = tilt_servo.angle() + tilt_output
    # 保证安全范围
    pan_angle=panMove(pan_angle)
    tilt_angle=tiltMove(tilt_angle)
    # 控制执行单元
    pan_servo.angle(pan_angle)
    tilt_servo.angle(tilt_angle)

# 定时器 -----------------------------------------
# TIM2 - 基本定时 0.5ms
def tick2(timer):
    global PIDCnt
    PIDCnt += 1
    #global x,y,max_blob
    #if len(rectangleAngle) > 0 and max_blob is not None:
        #moveYunTai(x, y, max_blob.cx(),max_blob.cy())

tim2 = pyb.Timer(2, freq=1000)
tim2.callback(tick2)

# main 函数 --------------------------------------
def main():
    global subject
    global pan_servo
    global tilt_servo
    # 铅笔框变量
    global angleList
    global pencilPoint
    global pencilStep
    global pencilStepTotal
    global pencilCnt
    # 固定黑色边框变量
    global fixedBlackRectCnt
    global fixedBlackRectPoint
    global fixedBlackRectStep
    global fixedBlackRectStepTotal
    if subject == 1:
        # 第一个功能, 实现复位
        pan_servo.angle(origin[0])
        tilt_servo.angle(origin[1])
    elif subject == 2:
        # 第二题代码
        #print("pencilPoint:",pencilPoint)
        if pencilPoint == 3:  # 最后一定点，需要和第一个点作差
            pan_angle = (angleList[0][0] - angleList[3][0]) / pencilStepTotal *pencilStep + angleList[3][0]
            tilt_angle = (angleList[0][1] - angleList[3][1]) / pencilStepTotal *pencilStep + angleList[3][1]
        else:
            pan_angle = (angleList[pencilPoint + 1][0] - angleList[pencilPoint][0]) / pencilStepTotal *pencilStep + angleList[pencilPoint][0]
            tilt_angle = (angleList[pencilPoint + 1][1] - angleList[pencilPoint][1]) / pencilStepTotal *pencilStep + angleList[pencilPoint][1]
        # 移动云台
        #print("angle:",pan_angle,tilt_angle)
        pan_servo.angle(pan_angle)
        tilt_servo.angle(tilt_angle)
        # 判断识别条件
        if pencilStep > pencilStepTotal:   # 每边加满
            pencilStep = 0  # 步数清零
            if pencilPoint >= 3:
                pencilPoint = 0
            else:
                pencilPoint += 1
        # 添加延时进入下一个点
        if pencilCnt >= 2000:
            pencilCnt = 0
            pencilStep += 1
        pencilCnt += 1
    elif subject == 3:
        # 第三题代码
        if fixedBlackRectPoint == 3:  # 最后一定点，需要和第一个点作差
            pan_angle = (fixedBlackRectAngleList[0][0] - fixedBlackRectAngleList[3][0]) / fixedBlackRectStepTotal *fixedBlackRectStep + fixedBlackRectAngleList[3][0]
            tilt_angle = (fixedBlackRectAngleList[0][1] - fixedBlackRectAngleList[3][1]) / fixedBlackRectStepTotal *fixedBlackRectStep + fixedBlackRectAngleList[3][1]
        else:
            pan_angle = (fixedBlackRectAngleList[fixedBlackRectPoint + 1][0] - fixedBlackRectAngleList[fixedBlackRectPoint][0]) / fixedBlackRectStepTotal *fixedBlackRectStep + fixedBlackRectAngleList[fixedBlackRectPoint][0]
            tilt_angle = (fixedBlackRectAngleList[fixedBlackRectPoint + 1][1] - fixedBlackRectAngleList[fixedBlackRectPoint][1]) / fixedBlackRectStepTotal *fixedBlackRectStep + fixedBlackRectAngleList[fixedBlackRectPoint][1]
        # 移动云台
        pan_servo.angle(pan_angle)
        tilt_servo.angle(tilt_angle)
        #print("angle:",pan_angle,tilt_angle)
        # 判断识别条件
        if fixedBlackRectStep > fixedBlackRectStepTotal:   # 每边加满
            fixedBlackRectStep = 0  # 步数清零
            if fixedBlackRectPoint >= 3:
                fixedBlackRectPoint = 0
            else:
                fixedBlackRectPoint += 1
        # 添加延时进入下一个点
        if fixedBlackRectCnt >= 3500:
            fixedBlackRectCnt = 0
            fixedBlackRectStep += 1
        fixedBlackRectCnt += 1
    elif subject == 4:
        # 第四题代码
        global max_rect
        global max_blob
        global rectangleAngle
        global rectCnt
        global laser_point_stable_n # 用于判断激光点是否稳定的计数
        global moveBlackRectStepTotal # 每个边细分成多少份进行移动
        global moveBlackRectPoint  # 存储当前执行到第几个角
        global moveBlackRectStep   # 存储在当前边的第几步
        global PIDCnt
        # 识别
        img = sensor.snapshot().lens_corr(1.8)  # 调 1.0 或者 1.8, 1.8 是防畸变
        # 第一步: 识别黑色矩形
        if rectCnt < 120:
            #sensor.set_auto_exposure(True)
            # 先移到原点
            pan_servo.angle(origin[0])
            tilt_servo.angle(origin[1])
            for blob in img.find_blobs(black_threshold, pixels_threshold=1000, roi=(60, 21, 199, 205),\
                                        area_threshold=1000):
                if blob.density() > 0.05:  # 调
                    # 保存四个角的点
                    rectangleAngle = blob.min_corners()
                    # 画图
                    img.draw_rectangle(blob.rect(),color=(0, 255, 0))
                    img.draw_cross(rectangleAngle[0],color=(255, 0, 0))  # 画出第1个点
                    img.draw_cross(rectangleAngle[1],color=(255, 0, 0))  # 画出第2个点
                    img.draw_cross(rectangleAngle[2],color=(255, 0, 0))  # 画出第3个点
                    img.draw_cross(rectangleAngle[3],color=(255, 0, 0))  # 画出第4个点
                    img.draw_string(rectangleAngle[0][0],rectangleAngle[0][1],"1")
                    img.draw_string(rectangleAngle[1][0],rectangleAngle[1][1],"2")
                    img.draw_string(rectangleAngle[2][0],rectangleAngle[2][1],"3")
                    img.draw_string(rectangleAngle[3][0],rectangleAngle[3][1],"4")
                    # 增加计数器
                    rectCnt = rectCnt + 1
        ## 第二步: 识别红色激光笔并移动
        if rectCnt >= 120:
            #sensor.set_auto_exposure(False, 500)#在这里调节曝光度，调节完可以比较清晰地看清激光点
            # 识别红色激光
            redBlobsOut = img.find_blobs([(66, 100, 19, 127, -16, 127)])
            #redBlobsOut = img.find_blobs([(15, 20, 20, 60, 20, 50)])  # 低曝光
            redBlobs = img.find_blobs([(20, 50, 20, 70, 10, 60)],roi=(53,33,200,171))
            #redBlobs = img.find_blobs([(20, 60, 20, 60, 20, 30)],roi=(53,33,200,171))  # 低曝光
            if redBlobsOut:
                # 识别黑线外的红色激光笔
                max_blob = find_max(redBlobsOut)
                # 画出色块
                img.draw_rectangle(max_blob.rect())
                img.draw_cross(max_blob.cx(), max_blob.cy())
            elif redBlobs:
                # 识别黑线内的红色激光笔
                max_blob = find_max(redBlobs)
                # 画出色块
                img.draw_rectangle(max_blob.rect())
                img.draw_cross(max_blob.cx(), max_blob.cy())
            # 移动云台
            if len(rectangleAngle) > 0 and max_blob is not None:
                # 首先计算得到需要移动到的横纵坐标
                if moveBlackRectPoint == 3:  # 最后一定点，需要和第一个点作差
                    x = (rectangleAngle[0][0] - rectangleAngle[3][0]) / moveBlackRectStepTotal *moveBlackRectStep + rectangleAngle[3][0]
                    y = (rectangleAngle[0][1] - rectangleAngle[3][1]) / moveBlackRectStepTotal *moveBlackRectStep + rectangleAngle[3][1]
                else:
                    x = (rectangleAngle[moveBlackRectPoint + 1][0] - rectangleAngle[moveBlackRectPoint][0]) / moveBlackRectStepTotal *moveBlackRectStep + rectangleAngle[moveBlackRectPoint][0]
                    y = (rectangleAngle[moveBlackRectPoint + 1][1] - rectangleAngle[moveBlackRectPoint][1]) / moveBlackRectStepTotal *moveBlackRectStep + rectangleAngle[moveBlackRectPoint][1]
                # 每隔一段时间移动一次云台
                if PIDCnt >= 500:
                    PIDCnt = 0
                    moveYunTai(x, y, max_blob.cx(),max_blob.cy())
                if moveBlackRectStep > moveBlackRectStepTotal:   # 每边加满
                    moveBlackRectStep = 0  # 步数清零
                    moveBlackRectPoint += 1    # 边序号加一
                    if moveBlackRectPoint > 3: # 边序号满
                        moveBlackRectPoint = 0 # 边序号清零
                if math.fabs(max_blob.cx() - x) < 15 and \
                   math.fabs(max_blob.cy() - y) < 15:
                    laser_point_stable_n += 1
                if laser_point_stable_n > 25:  # 原来是25
                    # 0->1->2->3->0
                    moveBlackRectStep += 1  # 每边细分的步骤加1
                    laser_point_stable_n = 0  # 记住清零
    else:
        # 暂停状态, 什么也不做
        pass

# 主程序 ------------------------------------------
while True:
    if keyPressOn(resetKey) == 1:
        # 复位键按下的代码 ↓
        print("复位开始")
        # 暂存之前舵机角度(局部变量)
        last_pan_value = pan_servo.angle()
        last_tilt_value = tilt_servo.angle()
        # 激光笔移动到中点
        pan_servo.angle(origin[0])
        tilt_servo.angle(origin[1])
        LightLED(2)
        while True:
            if keyPressOn(resetKey) == 1:
                # 之前的角度执行回去
                pan_servo.angle(last_pan_value)
                tilt_servo.angle(last_tilt_value)
                print("复位结束")
                LightLED(1)
                break
    elif keyPressOn(pausekey) == 1:
        # 暂停键按下的代码 ↓
        print("暂停了")
        LightLED(2)
        while True:
            if keyPressOn(pausekey) == 1:
                # 等待再次按下暂停键后运行
                print("暂停结束")
                LightLED(1)
                break
    elif keyPressOn(selectKey) == 1:
        # 选择键按下
        if preSubject == 4:
            preSubject = 1
        else:
            preSubject = preSubject + 1
        print("选择",preSubject)
        LightLED(preSubject)
    elif keyPressOn(confirmKey) == 1:
        # 确认键按下
        subject = preSubject
        print("运行",subject)
        LightLED(1)
        # 激光笔移动到中点
        pan_servo.angle(origin[0])
        tilt_servo.angle(origin[1])
    else:
        # 没有按键按下, 正常运行
        main()
