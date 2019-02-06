import warnings
warnings.simplefilter("ignore", FutureWarning)
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

#To make sure that the apriltag package is in the path
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
        #print(start) 
        dist = depth_frame.get_distance(start, height*pix_per_deg)
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
    #print ("Tag detector started!")
	#initialize image
    img = np.asanyarray(color_frame.get_data())
	#convert to grayscale
    pil_img = Image.fromarray(img)
    gray = np.array(pil_img.convert('L'))
    result = detector.detect(gray)
    num_detections = len(result)
    if num_detections==0:
        return "no tags detected"
 	
    #print('Detected {} tags in {}\n'.format(
    #num_detections, 'camera image'))
    #print(result)
    i=0
    for detection in result:
        ret += str(i)
        ret += " "
        x, y = detection.center
        id = detection.tag_id
        ret += str(id)
        ret += " "
        dist_to_center = depth_frame.get_distance(int(x), int(y))
        #print( 'Detection {} of {}:'.format(i+1, num_detections))
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

        #print("pose of the tag is: ")
        #print(pose)
        #print(detection.tostring(indent=2))

        
        #print('Distance to center of apriltag is: ')
        #print(dist_to_center)
        
        i+=1
    return ret


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

    # set the static IP address of this Python server
    TCP_IP = get_ip()
    print('Raspberry Pi address: ' + TCP_IP)
    TCP_PORT = 8865
    BUFFER_SIZE = 128
    debug = False
    serialLock = _thread.allocate_lock()
    
    # start the socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print("Waiting for connection with Host...")

    conn, addr = s.accept()
    print ('Host Computer address: ' + addr[0])
    
	# Configure depth and color streams
    print ("waiting for camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

	# Start streaming
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
    
    start = time.time()

    try:
        
        print ("Ready for Commands!") 
        while 1:
            #print (time.time()-start)
            #print ("Waiting for command..." )
            #dataAvail = select.select([conn],[],[],0.25)[0]
            #while not dataAvail:
            
            # Check for data from the robot 
            #bytesToRead = ser_port.inWaiting()
            #if bytesToRead:
                #x = ser_port.read(bytesToRead)
                #print("data received from robot:")
                #print(x)
                #conn.send(x)
            
            # check for command from the host
            data = b''
            dataAvail = select.select([conn],[],[],0)[0]
            if dataAvail:
                data = (conn.recv(BUFFER_SIZE))
            #print ("received data:")
            #print (data)
  			
            # Command received from the host computer, matches with each of the 
			# following cases to handle accordingly. 
            
            if data[:4] == b'dist':
            # Case where the host computer asks for distance readings
                if camera==False:
                    print("No camera connected, cannot call this function")
                    conn.send(bytes([99]))
                    continue
			
                #print("Waiting for frames from camera...")
                frames = pipeline.wait_for_frames()
                #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    #print("waiting...")
                    continue
                #print ("Got frames!")
                data = data.decode('utf-8')
                chars = len(data)-4
                num_points = (data[4])
                height = (data[5:])
                #print(height)
                dist_data = get_distance(depth_frame, int(num_points), int(height))
                #print("SENDING TO COMPUTER")
                conn.send(str.encode(dist_data))
                #print("DATA SENT TO COMPUTER")

            elif data == b'tag':
            # Case where the host computer asks for tag information in the RGB stream
                if camera==False:
                    print("No camera connected, cannot call this function")
                    conn.send(bytes([99]))
                    continue
			
                frames = pipeline.wait_for_frames()
                #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

				#Get the camera parameters from the RealSense
                profile = pipeline.get_active_profile()
                rgb_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
                rgb_intrinsics = rgb_profile.get_intrinsics()

                params = [rgb_intrinsics.fx, rgb_intrinsics.fy, 0, 0]
                if not color_frame or not depth_frame:
                    #print("waiting...")
                    continue
                #print ("Got frames!")

                #Call get_tag function to get the tag information from the camera
                tag_data = get_tag(color_frame, depth_frame, params)
                #print("Sending data to computer...")
                conn.send(str.encode(tag_data))
                #print("Data sent!")

            elif data == b'color':
			# Case where the host computer asks the current image from the camera
                if camera==False:
                    print("No camera connected, cannot call this function")
                    conn.send(bytes([99]))                    
                    continue

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                pil_image = Image.fromarray(color_image)
				#Convert to grayscale image before sending to host computer
                gray = np.array(pil_image.convert('L'))

                for i in range(480):
                    for j in range(640): 
                         conn.send(gray[i, j])

            elif data == b'depth':
			# Case where the host computer asks the depth image from the camera
                if camera==False:
                    print("No camera connected, cannot call this function")
                    conn.send(bytes([0]))
                    conn.send(bytes([99]))
                    continue

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                depth_image = np.asanyarray(depth_frame.get_data())
                for i in range(480):
                    for j in range(640): 
                         conn.send(depth_image[i, j])

            elif data == b'stop':
            # Case where the host computer asks to stop the script 
                quit()

            else: 
			# Case where the host computer command is meant for the iRobot
		        #write data to port
                if dataAvail:
                    send_message(data, ser_port, serialLock)
                #print ("data sent to the robot")
                #time.sleep(0.1)
                
                BytesToRead = ser_port.inWaiting()
                if BytesToRead:
                    x = ser_port.read(BytesToRead)
                    #print("data received from robot:")
                    #print(x)
                    conn.send(x)

    finally:
        print("I was told to Stop, or something went wrong")
        #close TCP connection
        conn.shutdown(1)
        conn.close()
        #close the serial connection
        ser_port.close()
        if camera==True:
            pipeline.stop()

main()
