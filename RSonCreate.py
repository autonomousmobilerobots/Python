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


""" Returns DHCP assigned IP"""
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



""" get_depth
    depth_frame is a 640x480 Realsense depth frame
    x_pixel and y_pixel are the coordinates to get depth for
    R determines the number of pixels around x,y that are used to get the average
    R == 0 - only 1 pixel
    R == 1 - 3x3
    R == 2 - 5x5
    in general (R+1+R)^2 

    Returns depth reading as float
"""
def get_depth(depth_frame, x_pixel, y_pixel, R):
    
    # No averaging, return depth of 1 pixel
    if R==0:
        return depth_frame.get_distance(x_pixel, y_pixel)
    
    else:
        # check that all pixels remain within the frame
        if x_pixel < R:
            x_pixel=R
        elif x_pixel > 640-R:
            x_pixel = 640-R
        
        if y_pixel < R:
            y_pixel=R
        elif y_pixel > 480-R:
            y_pixel = 480-R

        # calculate margins for depth average
        left = x_pixel-R    
        right = x_pixel+R+1 #range is exclusive of last point
        bottom = y_pixel-R
        top = y_pixel+R+1 
        num_of_pixels = (right-left)*(top-bottom)

        # sum and divide to get the average
        sum = 0
        for y in range(bottom, top):
           for k in range(left, right):
               sum += depth_frame.get_distance(k, y)
        
        return sum/num_of_pixels



"""
[get_distance] receives a RealSense depth frame and 
returns the distance to 9 points equally spaced horizontally

[depth_frame] is returned from the camera, in a 640*480 array of depth data
640 pixels divided into 8 equal intervals
The height is the middle of 480 pixels. 
The distance returned is after taking average of (R+1+R)^2 points around 
the point specified.  
"""

def get_distance(depth_frame):
    num_points = 9
    Y = 240                #480/2
    pix_per_interval = 80  #640/(num_points-1)
    R = 1    

    dist_l = [""]
    for point in range(num_points):
        X = point*pix_per_interval
        dist_l.append(str(get_depth(depth_frame, X, Y, R))[0:6])
    
    return dist_l



"""[get_tag] scans the current camera video image and returns the apriltag
information if any is detected. [color_frame] is the RGB image captured by the
RealSense camera, [depth_frame] is the current depth frame captured by the 
camera. 
"""

def get_tag(color_frame, depth_frame, x_fov_rad, detector):
    
    #initialize image
    img = np.asanyarray(color_frame.get_data())
    
    #convert to grayscale
    pil_img = Image.fromarray(img)
    gray = np.array(pil_img.convert('L'))

    # find tags in gray image
    tag_detections = detector.detect(gray)
        
    if len(tag_detections) == 0:
        return ["", "no tags detected"]
 	
    # Clculate depth of detected tags
    det = 1
    tag_l = [""]
    R = 1
    for tag in tag_detections:
        
        x, y = tag.center
        id = tag.tag_id
        
        z_depth = get_depth(depth_frame, int(x), int(y), R)
        px_to_m = z_depth*math.tan(x_fov_rad/2)/320
        x_depth = (320-x)*px_to_m
        
        # Add to the list
        tag_l.append(str(det))
        tag_l.append(str(id))
        tag_l.append(str(z_depth)[0:6])
        tag_l.append(str(x_depth)[0:6])
        tag_l.append(str(0)[0:6])

        det+=1
    
    return tag_l 


""" Worker function for udp broadcast called in separate process
    Reads data from Queue and broadcasts over udp
    Computes delay from fresh data to time of transmission and 
    adds it to the delay coming from the queue
"""
def udp_broadcast(camera, Host_IP, UDP_Port, queue):
    
    try:

        #configure udp port
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        comp_dt = 0
        queue_start = 0
        data_l = ['0', '0']
 
        while 1:
            
            if camera:  
                # if there is new data in the queue read it
                if not queue.empty():

                    queue_start = time.time()
                    data_l = queue.get()
                    comp_dt = float(data_l[0])
                    
            
                # calculate additional delay from queue update 
                queue_dt = time.time()-queue_start
                total_dt = str(comp_dt + queue_dt) 
                data_l[0] = total_dt[0:6]
                data = ' '.join(data_l)
            
                #broadcast
                s.sendto(str.encode(data), (Host_IP, UDP_Port))
            else:
                # Tell the host there is no camera, and sleep a little
                s.sendto(bytes([99]), (Host_IP, UDP_Port))
                time.sleep(1)

    finally:
        
        #close UDP port
        s.close()


""" Camera function to be called in a separate process 
    Sets up the camera stream and aligns images
    Sets up and starts the UDP broadcasting processes
    Calls Tag and Dist functions and writes the data into the queue 
"""
def camera_worker(Host_IP):
    
    #configure UDP ports
    Dist_UDP_Port = 8833
    Tag_UDP_Port = 8844
   
    #Initialize apriltag detector
    detector = apriltag.Detector(searchpath=_get_demo_searchpath())

    #create queues
    tag_queue = multiprocessing.SimpleQueue()
    dist_queue = multiprocessing.SimpleQueue()
 
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
        #params = [rgb_intrinsics.fx, rgb_intrinsics.fy, 0, 0]
        
        # get FOV from the camera
        fov= rs.rs2_fov(rgb_intrinsics)
        x_fov_rad = math.radians((fov)[0])
 

    #start the broadcasting processes
    tag_broadcast = multiprocessing.Process(target=udp_broadcast, args=(camera, Host_IP,Tag_UDP_Port, tag_queue))
    tag_broadcast.daemon = True
    tag_broadcast.start() 
    
    dist_broadcast = multiprocessing.Process(target=udp_broadcast, args=(camera, Host_IP,Dist_UDP_Port, dist_queue)) 
    dist_broadcast.daemon = True
    dist_broadcast.start() 

    dist_data_l = [""]
    tag_data_l = [""]
    
    try:
        
        while 1:
    
            if camera==True:
                
                frames = pipeline.wait_for_frames()
                cam_start = time.time() 
               
                #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue
 
                # Call distance function, add delay and write to queue
                dist_data_l = get_distance(depth_frame)
                dist_dt = str(time.time()-cam_start)
                dist_data_l[0] = dist_dt[0:6]
                dist_queue.put(dist_data_l)
                
                # Call tag function, add delay and write to queue
                tag_data_l = get_tag(color_frame, depth_frame, x_fov_rad, detector)
                tag_dt = str(time.time()-cam_start)
                tag_data_l[0] = tag_dt[0:6]
                tag_queue.put(tag_data_l)
            
            # No camera - sleep to reduce load
            else:
                time.sleep(1)

    finally:
        
        # Terminte broadcasting processes
        tag_broadcast.terminate()
        tag_broadcast.join()
        dist_broadcast.terminate()
        dist_broadcast.join()
        
        # Stop camera
        if camera==True:
            pipeline.stop() 


"""[main()] first establishes connection with host computer. Once the TCPIP
	connection is established, the function starts the camera in a separate process.
    Any exceptions caught in this function would cause the program to terminate. 
	Once all connections are set, the function listens to commands from the host 
    computer MATLAB toolbox and sends them to the robot, then sends the reply back to the host
"""
def main():

    try:
        # Set the output serial property for create
        ser_port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=0.5)
        print ("Serial port set!")
    except:
        print ("Serial port not connected")
        quit()

    # Configure communication
    TCP_IP = get_ip()
    print('Raspberry Pi address: ' + TCP_IP)
    TCP_PORT = 8865
    BUFFER_SIZE = 32
    debug = False
    serialLock = _thread.allocate_lock()
    

    # Start the tcp socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print("Waiting for connection with Host...")

    conn, addr = s.accept()
    Host_IP = addr[0]
    print ('Host Computer address: ' + Host_IP)

    # Start camera process  
    cam_process=multiprocessing.Process(target=camera_worker, args=(Host_IP,)) 
    cam_process.start() 

    try:
        
        print ("Ready for Commands!") 
        while 1:
            
           # Check for command from the host
            command = b''
            dataAvail = select.select([conn],[],[],0.01)[0]
            if dataAvail:
                command = (conn.recv(BUFFER_SIZE))
                
                if command == b'stop':
                # The host computer asks to stop the script 
                    quit()

                else: 
                # The host computer command is meant for the iRobot
                # write data to serial port
                    send_message(command, ser_port, serialLock)
            
            # No command
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
