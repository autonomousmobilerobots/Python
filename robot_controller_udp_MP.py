import warnings
warnings.simplefilter("ignore", FutureWarning)
import multiprocessing
import serial
import socket
import time
import math
import select
import _thread
import pyrealsense2 as rs
import apriltag
import os
import numpy as np
from PIL import Image
from threading import *

debug = False

"""To make sure that the apriltag package is in the path"""
def _get_demo_searchpath():
    
    return [
        os.path.join(os.path.dirname(__file__), '../build/lib'),
        os.path.join(os.getcwd(), '../build/lib')
    ]


"""Function grabs the serial port mutex lock and sends [data] to 
the [ser_port]"""
def send_message(data, ser_port, serialLock):
    #first grab mutex lock
    serialLock.acquire()
    successful = False
    while not successful:
        try:
            ser_port.write(data)
            successful = True
        except select.error:
            pass

    #release the mutex lock
    serialLock.release()


""" Get DHCP assigned IP"""
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        TCP_IP = s.getsockname()[0]
    except:
        TCP_IP = '127.0.1.1'
    finally:
        s.close()
    return TCP_IP




"""[get_distance] receives the command from the host computer regarding the 
[num_points] to sample from the image, and [height] is the angle in degrees to 
that the depth reading. [depth_frame] is returned from the camera, in a 640*480
array of distance data. [num_points] is taken at regular intervals across a 
horizontal row of the depth image. The left and the right-most points of the 
image is always sampled, and the remaining points are interpolated across the 
row. The distance returned is after taking average of 16 points in the region 
specified (4*4 pixels). 



Requires (enforced by the MATLAB code): 

	- 2<=[num_points]<=9

	- 1<=[height]<=40

"""
def get_distance(depth_frame, num_points, height):
    ret = ""
    pix_per_deg = 12
    intervals = num_points-1
    pix_per_interval = int((640-4*num_points)/intervals)
    for j in range(num_points):
        start = j*pix_per_interval
        sum = 0
        for y in range((40-height)*pix_per_deg-4, (40-height)*pix_per_deg):
           for k in range(start, start+4):
               sum +=depth_frame.get_distance(k, y)
        avg = sum/16
        ret += str(avg)
        ret += " "
    return ret





"""[get_tag] scans the current camera video image and returns the apriltag
information if any is detected. [color_frame] is the RGB image captured by the
RealSense camera, [depth_frame] is the current depth frame captured by the 
camera. [params] is the RealSense camera parameters necessary for the apriltag 
package to calculate the pose of the tag in view."""
def get_tag(color_frame, depth_frame, params):
    ret = ""
	#Initialize apriltage detector
 
    fov = 0.466 #in radians, half angle
    detector = apriltag.Detector(searchpath=_get_demo_searchpath())

    #initialize image
    img = np.asanyarray(color_frame.get_data())
    #convert to grayscale
    pil_img = Image.fromarray(img)
    gray = np.array(pil_img.convert('L'))
    result = detector.detect(gray)
    num_detections = len(result)
    if num_detections==0:
        return "no tags detected"
 	
    i=0
    for detection in result:
        ret += str(i)
        ret += " "
        x, y = detection.center
        id = detection.tag_id
        ret += str(id)
        ret += " "

	#Pose of the apriltags in camera view is returned from the apriltag 
	#package as a homogeneous transform matrix. The tag position in the H 
	#matrix is the (x, y, z) position of the center of the tag with respect 
	#to the left center of the camera frame. 
	# The third argument into the detector.detection_pose() function is the 
	# size of the apriltags used in meters. The pose calculations are very
	# sensitive to this value
        pose, e0, e1 = detector.detection_pose(detection, params, 0.166)
        z_dist = pose[2, 3]
        x_dist = pose[0, 3]-math.tan(fov)*z_dist
        yaw = math.atan2(pose[1, 0],pose[0, 0])
        ret += str(z_dist)
        ret += " "
        ret += str(x_dist)
        ret += " "
        ret += str(yaw)
        ret += " "
              
        i+=1
    return ret


""" Function for the camera to be called in a separate process """
def camera_worker(Host_IP):
    print("Camera process started")
    
    #configure UDP ports
    Dist_UDP_Port = 8833
    Tag_UDP_Port = 8844
    
    s_Dist = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s_Tag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

 
    # Configure depth and color streams
    print ("waiting for camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start camera streaming
    try:
        pipeline.start(config)
        print("Camera started and streaming!")
        camera = True
    except: 
        print ("Camera not connected!")
        camera = False   
    
    #align depth and color images
    align_to = rs.stream.color
    align = rs.align(align_to)

    #Get the camera parameters from the RealSense
    if camera==True:
        profile = pipeline.get_active_profile()
        rgb_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        rgb_intrinsics = rgb_profile.get_intrinsics()
        params = [rgb_intrinsics.fx, rgb_intrinsics.fy, 0, 0]

    num_points = 9 #(data[4])
    height = 20    #(data[5:])
    

    try:

        while 1:
    
            #calculate and broadcast distance and tag
            if camera==True:
   
                #print("Waiting for frames from camera...")
                frames = pipeline.wait_for_frames()
                #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue
 
                dist_data = get_distance(depth_frame, int(num_points), int(height))
                s_Dist.sendto(str.encode(dist_data), (Host_IP, Dist_UDP_Port))

                tag_data = get_tag(color_frame, depth_frame, params)
                s_Tag.sendto(str.encode(tag_data), (Host_IP, Tag_UDP_Port))

            else: 
                s_Dist.sendto(bytes([99]), (Host_IP, Dist_UDP_Port))
                s_Tag.sendto(bytes([99]), (Host_IP, Tag_UDP_Port))
   
    finally:

        #close UDP ports
        s_Dist.close()
        s_Tag.close()
        if camera==True:
            pipeline.stop() 


"""[main()] first establishes connection with host computer. Once the TCPIP
	connection is established, the function sets the RealSense camera stream
	parameters and starts the camera stream pipeline. Any exceptions caught 

	in this function would cause the program to terminate. 
	Once all connections are set and the camera started, the function listens
	to commands from the host computer MATLAB toolbox and executes them 
	accordingly. """
def main():
    try:
        #set the output serial property for create
        ser_port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=0.5)
        print ("Serial port set!")
    except:
        print ("Serial port not connected")
        quit()

    # configure communication
    TCP_IP = get_ip()
    print('Raspberry Pi address: ' + TCP_IP)
    TCP_PORT = 8865
    BUFFER_SIZE = 32
    debug = False
    serialLock = _thread.allocate_lock()
    

    # start the tcp socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print("Waiting for connection with Host...")

    conn, addr = s.accept()
    Host_IP = addr[0]
    print ('Host Computer address: ' + Host_IP)

    #start camera process  
    cam_process=multiprocessing.Process(target=camera_worker, args=(Host_IP,)) 
    cam_process.start() 

#    start = time.time()

    try:
        
        print ("Ready for Commands!") 
        while 1:

            #print (time.time()-start)
            
           # check for command from the host
            command = b''
            dataAvail = select.select([conn],[],[],0)[0]
            if dataAvail:
                command = (conn.recv(BUFFER_SIZE))
                
                if command == b'stop':
                # Case where the host computer asks to stop the script 
                    quit()

                else: 
                # Case where the host computer command is meant for the iRobot
                #write data to robot
                    send_message(command, ser_port, serialLock)
            
            #no command
            else:       
                #Read data from robot and send to Host
                BytesToRead = ser_port.inWaiting()
                if BytesToRead:
                    x = ser_port.read(BytesToRead)
                    conn.send(x)

    finally:
        print("I was told to Stop, or something went wrong")
        #close TCP connection
        conn.shutdown(1)
        conn.close()
        s.close()
       #close the serial connection
        ser_port.close()
        #kill the camera process
        cam_process.terminate()
        cam_process.join()


main()
