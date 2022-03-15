import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.sub = self.create_subscription(Image, 'RobotImage',self.callback,10)
        self.bridge = CvBridge()
        self.image1 = np.zeros((120, 160, 3), np.uint8)
        self.empty = np.zeros((120, 160, 3), np.uint8)
        cv2.putText(self.empty,"not in use", (0,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
        self.merge = np.zeros((120, 160, 3), np.uint8)
        #flag to control the thread
        self.thread_runnning_flag = False
        self.t1 = 0 

    def callback(self, image_message):
        self.t1 = cv2.getTickCount()
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        cv2.putText(image,"1", (60,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
        self.image1 = image

    def _display_frames(self):
        while(self.thread_runnning_flag==True):
            # cv_image=cv2.resize(cv_image,(320,240)) 
            if((cv2.getTickCount()-self.t1)/cv2.getTickFrequency()>1):
                self.image1 = np.zeros((120, 160, 3), np.uint8)
                cv2.putText(self.image1,"1", (60,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)

            cv2.imshow("Robot", self.image1)
            cv2.waitKey(1)

    def start(self): #start the data capture thread
        if self.thread_runnning_flag == False: #only process if no thread is running yet
            self.thread_runnning_flag=True #flag to control the operation of the _capture_frames function
            self.thread = threading.Thread(target=self._display_frames) #link thread with the function
            self.thread.start() #start the thread
    
    def stop(self): #stop the data capture thread
        if self.thread_runnning_flag == True:
            self.thread_runnning_flag = False #exit the while loop in the _capture_frames
            self.thread.join() #wait the exiting of the thread    

rclpy.init()
my_subscriber = MySubscriber()
my_subscriber.start()

rclpy.spin(my_subscriber)

my_subscriber.destroy_node()
rclpy.shutdown()

