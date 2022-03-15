import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Image, 'RobotImage', 10)
        self.bridge = CvBridge()
       
    def publish(self,cv_image):  
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv_image), "bgr8"))

       
rclpy.init()
my_publisher = MyPublisher()

#use opencv to covert the depth image to RGB image for displaying purpose
import cv2
import numpy as np

#using realsense to capture the color and depth image
import pyrealsense2 as rs

#multi-threading is used to capture the image in real time performance
import threading
import time

class Camera():
    def __init__(self):
        super(Camera, self).__init__()
        
        #configure the color and depth sensor
        self.pipeline = rs.pipeline()
        self.configuration = rs.config()  
        
        #set resolution for the color camera
        self.color_width = 640
        self.color_height = 480
        self.color_fps = 30
        self.configuration.enable_stream(rs.stream.color, self.color_width, self.color_height, rs.format.bgr8, self.color_fps)

        #set resolution for the depth camera
        self.depth_width = 640
        self.depth_height = 480
        self.depth_fps = 30
        self.configuration.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_fps)

        #flag to control the thread
        self.thread_runnning_flag = False
        
        #start the RGBD sensor
        self.pipeline.start(self.configuration)
        self.pipeline_started = True
        time.sleep(3)

        frames = self.pipeline.wait_for_frames()

        #start capture the first color image
        color_frame = frames.get_color_frame()   
        image = np.asanyarray(color_frame.get_data())
        self.color_value = image

        #start capture the first depth image
        depth_frame = frames.get_depth_frame()           
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        self.depth_value = depth_colormap   

    def _capture_frames(self):
        while(self.thread_runnning_flag==True): 
            frames = self.pipeline.wait_for_frames() #receive data from RGBD sensor
            
            color_frame = frames.get_color_frame() #get the color image
            image = np.asanyarray(color_frame.get_data()) #convert color image to numpy array
            self.color_value = image #assign the numpy array image to the color_value variable 

            resized_img=cv2.resize(self.color_value,(160,120))
            my_publisher.publish(resized_img)

            depth_frame = frames.get_depth_frame() #get the depth image           
            depth_image = np.asanyarray(depth_frame.get_data()) #convert depth data to numpy array
            #conver depth data to BGR image for displaying purpose
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            self.depth_value = depth_colormap #assign the color BGR image to the depth value
    
    def start(self): #start the data capture thread
        if self.thread_runnning_flag == False: #only process if no thread is running yet
            self.thread_runnning_flag=True #flag to control the operation of the _capture_frames function
            self.thread = threading.Thread(target=self._capture_frames) #link thread with the function
            self.thread.start() #start the thread

    def stop(self): #stop the data capture thread
        if self.thread_runnning_flag == True:
            self.thread_runnning_flag = False #exit the while loop in the _capture_frames
            self.thread.join() #wait the exiting of the thread       

def bgr8_to_jpeg(value):#convert numpy array to jpeg coded data for displaying 
    return bytes(cv2.imencode('.jpg',value)[1])

#create a camera object
RGBcamera = Camera()
RGBcamera.start() # start capturing the data
print('camera started')
# display_color.value = bgr8_to_jpeg(cv2.resize(image,(320,240)))
# display_depth.value = bgr8_to_jpeg(cv2.resize(camera.depth_value,(320,240)))

while(True):
    time.sleep(0.001)
    # cv2.imshow('frame',RGBcamera.color_value)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

RGBcamera.stop() #stop data capturing thread
