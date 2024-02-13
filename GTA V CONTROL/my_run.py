import numpy as np
import cv2
import time
from draw_lanes import draw_lanes
from grabscreen import grab_screen
from controls import straight,left,right,slow
from keras.models import load_model

model = load_model('MyModel1.h5')
loopCount=1
loopMax=5

def roi(img, vertices):
    
    #blank mask:
    mask = np.zeros_like(img)   
    
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, 255)
    
    #returning the image only where mask pixels are nonzero
    masked = cv2.bitwise_and(img, mask)
    return masked
def process_img(image):
    original_image = image
    # convert to gray
    processed_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # edge detection
    processed_img =  cv2.Canny(processed_img, threshold1 = 200, threshold2=300)
    
    processed_img = cv2.GaussianBlur(processed_img,(5,5),0)
    
    vertices = np.array([[10,500],[10,300],[300,200],[500,200],[800,300],[800,500],
                         ], np.int32)

    processed_img = roi(processed_img, [vertices])

    # more info: http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
    #                                     rho   theta   thresh  min length, max gap:        
    lines = cv2.HoughLinesP(processed_img, 1, np.pi/180, 180,      20,       15)
    m1 = 0
    m2 = 0
    try:
        l1, l2, m1,m2 = draw_lanes(original_image,lines)
        cv2.line(original_image, (l1[0], l1[1]), (l1[2], l1[3]), [0,255,0], 30)
        cv2.line(original_image, (l2[0], l2[1]), (l2[2], l2[3]), [0,255,0], 30)
    except Exception as e:
        print(str(e))
        pass
    try:
        for coords in lines:
            coords = coords[0]
            try:
                cv2.line(processed_img, (coords[0], coords[1]), (coords[2], coords[3]), [255,0,0], 3)
                
                
            except Exception as e:
                print(str(e))
    except Exception as e:
        pass

    return processed_img,original_image, m1, m2

def img_preprocess(img):
    img = img[60:135,:,:]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255
    return img

def main():
    global loopCount
    for i in list(range(8))[::-1]:
        print(i+1)
        time.sleep(1)
    last_time = time.time()

    while True:
        screen = grab_screen(region=(0,40,800,640))
        print('Frame took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
        new_screen,original_image, m1, m2 = process_img(screen)
        #cv2.imshow('window', new_screen)
        original_image = cv2.resize(original_image, (80,60))
        cv2.imshow('window2',cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB))
        screen = cv2.resize(screen, (320,160))
        image = np.asarray(screen)
        print(image.shape)
        image = img_preprocess(image)
        image = np.array([image])   
        steering_angle = float(model.predict(image))
        print(steering_angle)
        straight()
        if ( -0.01 <= steering_angle <= 0.01 ):
            print("straight")
            straight()
        elif ( steering_angle <= -0.01):
            print("left")
            left()
        elif ( steering_angle >= 0.01):
            print("right")
            right()
            
        loopCount=loopCount+1
        if(loopCount % loopMax == 0):
            slow()
        #cv2.imshow('window',cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
main()