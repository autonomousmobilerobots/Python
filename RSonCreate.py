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


"""
To make sure that the apriltag package is in the path called by the Detector init
"""
def get_searchpath():
    
    return [
        os.path.join(os.path.dirname(__file__), '../build/lib'),
        os.path.join(os.getcwd(), '../build/lib')
    ]


"""
send_message grabs the serial port mutex [serialLock] and sends [data] to 
the [ser_port]
"""
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

""" 
get_robot_name returns the robot name based on the IP address
"""
def get_robot_name(IP):

    if   IP == "128.253.194.117": return "Test-Pi"
    elif IP == "10.253.194.101" : return "Wall-E"
    elif IP == "10.253.194.102" : return "EVE"
    elif IP == "10.253.194.103" : return "R2D2"    
    elif IP == "10.253.194.104" : return "BB8"
    elif IP == "10.253.194.105" : return "C3PO"    
    elif IP == "10.253.194.106" : return "Dotmatrix"
    elif IP == "10.253.194.107" : return "Max"    
    elif IP == "10.253.194.108" : return "Baymax"
    elif IP == "10.253.194.109" : return "Rachel"    
    elif IP == "10.253.194.110" : return "Motoko"
    else                        : return "Unknown"    


""" 
get_ip returns the local DHCP assigned IP
"""
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.1.1'
    finally:
        s.close()
    return IP



"""
get_depth calculates the average depth around a target pixel
[depth_frame] is a 640x480 Realsense depth frame
[x_pixel] and [y_pixel] are the pixel coordinates to get depth for
[R] determines the number of pixels around x,y that are used to get the average
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
        for j in range(bottom, top):
           for k in range(left, right):
               sum += depth_frame.get_distance(k, j)
        
        return sum/num_of_pixels



"""
get_distance receives a RealSense depth frame and 
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
    pix_per_interval = 79  #640/(num_points-1)-1 to get away from edges
    R = 1                  # size of target to average, see get_depth   
    
    dist_l = [""]
    for point in range(num_points):
        X = point*pix_per_interval + 4
        dist_l.append(str(get_depth(depth_frame, X, Y, R))[0:6])
        
    return dist_l



"""
get_tag scans the current camera image and returns the apriltag
information if any is detected. 
[color_frame] is the RGB image captured by the RealSense camera.
[depth_frame] is the current depth frame captured by the camera. 
[x_fov_rad] is the camera FOV in radians
[detector] is an initialized Apriltag Detector object
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
 	
    # Calculate data for the detected tags
    detNum = 1
    tag_l = [""]
    R = 1          # size of target to average, see get_depth
	
    for tag in tag_detections:
        
        center_x, center_y = tag.center
        id = tag.tag_id
        
        # Depth to tag center
        tag_z = get_depth(depth_frame, int(center_x), int(center_y), R)
        
        # Calculate X
        px_to_m = tag_z*math.tan(x_fov_rad/2)/320
        tag_x = (320-center_x)*px_to_m
        
        # Add tag information to the list
        tag_l.append(str(detNum))
        tag_l.append(str(id))
        tag_l.append(str(tag_z)[0:6])
        tag_l.append(str(tag_x)[0:6])
        tag_l.append(str(0)[0:6]) # not calculting yaw but leaving this here so the Matlab function doesn't change

        detNum+=1
    
    return tag_l 


""" 
udp_broadcast is a worker function for udp broadcast called in separate process
Reads data from Queue and broadcasts over udp
Computes delay from fresh data to time of transmission and 
adds it to the delay coming from the queue
[camera] is a boolean thats is True if the camera is present
[Host_IP] is the IP of the host PC controlling the robot
[queue] is a multiprocessing queue object containing data to broadcast
"""
def udp_broadcast(camera, Host_IP, UDP_Port_Type, queue):
    
    Dist_UDP_Port_Number = 8833
    Tag_UDP_Port_Number = 8844

    try:

        #configure udp port
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	print(UDP_Port_Type + " UDP Port is Set")
        
        comp_dt = 0
        queue_start = 0
        data_l = ['0', '0']
 
        while True:
            
            if camera:  
                # if there is new data in the queue read it
                if not queue.empty():

                    queue_start = time.time()
                    data_l = queue.get()
                    comp_dt = float(data_l[0])
                    
            
                # Calculate additional delay from queue update 
                queue_dt = time.time()-queue_start
                total_dt = str(comp_dt + queue_dt) 
                data_l[0] = total_dt[0:6]
                
                # Build packet to broadcast
                data = ' '.join(data_l)
                packet = str.encode(data)
               
            else:
                # Tell the host there is no camera, and sleep a little
                packet = bytes([99])
                time.sleep(1)
            
            # Broadcast
	    if UDP_Port_Type == "Tag":
                s.sendto(packet, (Host_IP, Tag_UDP_Port_Number))
	    if UDP_Port_Type == "Dist":
		s.sendto(packet, (Host_IP, Dist_UDP_Port_Number))
    
    except socket.error:
        # Turn off green LED
        os.system("echo none > /sys/class/leds/led0/trigger")
	print("Could Not Set " + UDP_Port_Type + " UDP Port")
    
    finally:
        print("Stopping UDP Process!")
        #close UDP port
        s.close()


""" 
camera_worker is the Camera control function to be called in a separate process 
Sets up the camera stream and aligns images
Sets up and starts the UDP broadcasting processes
Calls Tag and Distance functions and writes the data into the queue
[Host_IP] is the IP of the host PC controlling the robot
"""
def camera_worker(Host_IP):
    
    #Initialize apriltag detector
    detector = apriltag.Detector(searchpath=get_searchpath())

    #create queues
    tag_queue = multiprocessing.SimpleQueue()
    dist_queue = multiprocessing.SimpleQueue()
 
    # Configure depth and color streams
    print ("Initializing camera...")
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
    tag_broadcast = multiprocessing.Process(target=udp_broadcast, args=(camera, Host_IP,"Tag", tag_queue))
    tag_broadcast.daemon = True
    tag_broadcast.start() 
    
    dist_broadcast = multiprocessing.Process(target=udp_broadcast, args=(camera, Host_IP,"Dist", dist_queue)) 
    dist_broadcast.daemon = True
    dist_broadcast.start() 

    dist_data_l = [""]
    tag_data_l = [""]
    
    
        
    while True:
    
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

   


"""
main first establishes connection with the host computer. Once the TCPIP
connection is established, the function starts the camera control process.
Any exceptions caught in this function would cause the program to terminate. 
once all connections are set, the function listens to commands from the host 
computer and sends them to the robot, then sends the reply back to the host
"""
def main():
    
    TCP_PORT = 8865
    BUFFER_SIZE = 32
    serialLock = _thread.allocate_lock()

    # Configure communication
    my_IP = get_ip()
    print("Hello! My name is " + get_robot_name(my_IP) + ". My Raspberry Pi address is: " + my_IP)

    # Configure serial port to Create
    try:
        ser_port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=0.5)
        print ("Serial port to Create is set!")
    except:
        print ("Serial port to Create is not connected. Stopping!")
        quit()
    
    # Start the tcp socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((my_IP, TCP_PORT))
        s.listen(1)
        print("Waiting for TCP connection with Host...")

        # Create new socket bound to the port
        conn, addr = s.accept()
        Host_IP = addr[0]
        print ('TCP Port set. Host Computer address: ' + Host_IP)
    
       #Close the original socket 
        s.shutdown(1)
        s.close()
    
    except:
        print("Could not open TCP Port")
        quit()  
    
    else:
    
        # Start camera process  
        cam_process=multiprocessing.Process(target=camera_worker, args=(Host_IP,)) 
        cam_process.start() 
 
        # Heartbeat on green LED
        os.system("echo heartbeat > /sys/class/leds/led0/trigger")

        print ("Ready for Commands!")

        try:
        
            while True:
            
               # Check for command from the host
                command = b''
                dataAvail = select.select([conn],[],[],0.01)[0]
                if dataAvail:
                    command = (conn.recv(BUFFER_SIZE))
                
                    if command == b'stop':
                    # The host computer asks to stop the script 
                        print("Received a Stop command from Host")
                        break

                    else:
                    # The host computer command is meant for the iRobot Create
                    # write command to the serial port
                        send_message(command, ser_port, serialLock)
                
                # No command
                else:
                    #If there is data from the robot, send it to the Host computer
                    BytesToRead = ser_port.inWaiting()
                    if BytesToRead:
                        packet = ser_port.read(BytesToRead)
                        conn.send(packet)
    
        except:
            print("Problems communicating with the Create")
        
    finally:
        # Turn off green LED
        os.system("echo none > /sys/class/leds/led0/trigger")
        
        print("Stopping!")
        
        # Close the serial connection
        ser_port.close()
        
        # Terminate the camera process
        cam_process.terminate()
        cam_process.join(timeout=1)
        
        # Close TCP connection
        conn.shutdown(1)
        conn.close()


# Let's go
main()
