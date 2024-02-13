import cv2
import numpy as np
from keras.models import load_model
from time import sleep
import serial
import math

ser = serial.Serial('COM5', 9800, timeout=1)
model = load_model('MyModel1.h5')
# Doing some Face Recognition with the webcam
video_capture = cv2.VideoCapture(1)


h1 = 5.5  # cm
h2 = 5.5
sensor_data = None

red_light=False
yellow_light = False
green_light=False


frameCount = 0
frameSkip = 5
stopCount=10000


stop_cascade = cv2.CascadeClassifier("stop_sign.xml")
light_cascade = cv2.CascadeClassifier("traffic_light.xml")
cars_classifier=cv2.CascadeClassifier('cars.xml')
body_classifier=cv2.CascadeClassifier('haarcascade_fullbody.xml')

d_sensor_thresh = 30
d_stop_light_thresh = 25
d_car_thresh = 25
d_pedestrian_thresh = 25
d_stop_sign = d_stop_light_thresh
d_light = d_stop_light_thresh
d_car = d_car_thresh
d_pedestrian = d_pedestrian_thresh
stop_start = 0  # start time when stop at the stop sign
stop_finish = 0
stop_time = 0
drive_time_after_stop = 0

def RCControl(steering_angle):
    if ( -0.01 <= steering_angle <= 0.01 ):
        flag = 1
        ser.write(chr(1).encode())
        #print("Calculated frame {}".format(frameCount))
        
    elif ( steering_angle <= -0.01):
        flag = 2
        ser.write(chr(7).encode())
        #print("Calculated frame {}".format(frameCount))
        
    elif ( steering_angle >= 0.01):
        flag = 3
        ser.write(chr(6).encode())
        #print("Calculated frame {}".format(frameCount))
    else:
        ser.write(chr(0).encode())
    print(flag)

def Stop():
    ser.write(chr(0).encode())

def ObjectDetection(cascade_classifier, gray_image, image,case):
    # y camera coordinate of the target point 'P'
        v = 0
        #global red_light,green_light

        # minimum value to proceed traffic light state validation
        threshold = 150

        # detection
        cascade_obj = cascade_classifier.detectMultiScale(
            gray_image,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30))

        # draw a rectangle around the objects
        for (x_pos, y_pos, width, height) in cascade_obj:
            cv2.rectangle(image, (x_pos + 5, y_pos + 5), (x_pos + width - 5, y_pos + height - 5), (255, 255, 255), 2)
            v = y_pos + height - 5
            # print(x_pos+5, y_pos+5, x_pos+width-5, y_pos+height-5, width, height)

            # stop sign
            if case == 1:
                cv2.putText(image, 'STOP', (x_pos, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # traffic lights
            elif case == 2:
                roi = gray_image[y_pos + 10:y_pos + height - 10, x_pos + 10:x_pos + width - 10]
                mask = cv2.GaussianBlur(roi, (25, 25), 0)
                (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)

                # check if light is on
                if maxVal - minVal > threshold:
                    cv2.circle(roi, maxLoc, 5, (255, 0, 0), 2)

                    # Red light
                    if 1.0 / 8 * (height - 30) < maxLoc[1] < 4.0 / 8 * (height - 30):
                        cv2.putText(image, 'Red', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        red_light = True

                    # Green light
                    elif 5.5 / 8 * (height - 30) < maxLoc[1] < height - 30:
                        cv2.putText(image, 'Green', (x_pos + 5, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),2)
                        green_light = True

                    # yellow light
                    # elif 4.0/8*(height-30) < maxLoc[1] < 5.5/8*(height-30):
                    #    cv2.putText(image, 'Yellow', (x_pos+5, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    #    self.yellow_light = True
            
            if (case == 3):
                cv2.putText(image, 'CAR', (x_pos, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if (case == 4):
                cv2.putText(image, 'PEDESTRIAN', (x_pos, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return v

def DistanceToCamera(v,h,x_shift,frame):
    alpha = 8.0 * math.pi / 180    # degree measured manually
    v0 = 119.865631204             # from camera matrix
    ay = 332.262498472 
    d = h / math.tan(alpha + math.atan((v - v0) / ay))
    if d > 0:
            cv2.putText(frame, "%.1fcm" % d,
                        (frame.shape[1] - x_shift, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return d

def img_preprocess(img):
    img = img[60:135,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255
    return img

stop_flag = False
stop_sign_active = True
while True:
    
    
    
    _, frame = video_capture.read()
    
    frameCount+=1
    if (frameCount % frameSkip != 0):
        continue
    
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    roi = gray[int(height/2):height, :]
    v_param1 = ObjectDetection(stop_cascade, gray, frame,1)
    v_param2 = ObjectDetection(light_cascade, gray, frame,2)
    v_param3 = ObjectDetection(cars_classifier, gray, frame,3)
    v_param4 = ObjectDetection(body_classifier, gray, frame,4)
    if v_param1 > 0 or v_param2 > 0 or v_param3 > 0 or v_param4 > 0:
                        d1 = DistanceToCamera(v_param1, h1, 300, frame)
                        d2 = DistanceToCamera(v_param2, h2, 100, frame)
                        d3 = DistanceToCamera(v_param1, h1, 300, frame)
                        d4 = DistanceToCamera(v_param1, h1, 300, frame)
                        d_stop_sign = d1
                        d_light = d2
    cv2.imshow('image', frame)
    image = np.asarray(frame)
    print(image.shape)
    image = img_preprocess(image)
    image = np.array([image])   
    steering_angle = float(model.predict(image))
    
    if 0 < d_stop_sign < d_stop_light_thresh and stop_sign_active:
                        print("Stop sign ahead")
                        Stop()
                        stop_sign_active=False
                        
                        # stop for 5 seconds
                        if stop_flag is False:
                            '''for i in range (1,stopCount):
                                continue'''
                            sleep(5)
                            stop_flag=True
                        stop_finish = cv2.getTickCount()
                        
                        stop_time = (stop_finish - stop_start) / cv2.getTickFrequency()
                        print("Stop time: %.2fs" % stop_time)

                        # 5 seconds later, continue driving
                        if stop_time >= 5:
                            print("Waited for 5 seconds")
                            stop_flag = False
                            stop_sign_active = False
                        

    elif 0 < d_light < d_stop_light_thresh:
                        
                        # print("Traffic light ahead")
                        if red_light:
                            print("Red light")
                            Stop()
                        elif green_light:
                            print("Green light")
                            pass
                        elif yellow_light:
                            print("Yellow light flashing")
                            pass

                        d_light = d_stop_light_thresh
                        red_light = False
                        green_light = False
                        yellow_light = False
                                           
    elif 0 < d_car < d_car_thresh and v_param3:
                        print("Car detected")
                        Stop()
                        continue
    
    elif 0 < d_pedestrian < d_pedestrian_thresh and v_param4:
                        print("Person detected")
                        Stop()
                        continue
                    
    else:
                        RCControl(steering_angle)
                        '''
                        stop_start = cv2.getTickCount()
                        d_stop_sign = d_stop_light_thresh
                        if stop_sign_active is False:
                            drive_time_after_stop = (stop_start - stop_finish) / cv2.getTickFrequency()
                            if drive_time_after_stop > 5:
                                stop_sign_active = True
                         '''       
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(chr(0).encode())
        ser.close()
        break
video_capture.release()
cv2.destroyAllWindows()