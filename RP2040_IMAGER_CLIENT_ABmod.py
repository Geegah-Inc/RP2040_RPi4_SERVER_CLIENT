# -*- coding: utf-8 -*-
"""
Created on Thu Apr 13 13:01:17 2023

Client program to run on local laptop
Must be connected to the Raspberry Pi Zero W Wifi network

Note that this script uses OPENCV!

Need to wait until Raspberry Pi has finished starting up and initializing the RP2040
I think this takes ~2 minutes

You can keep running this script without resetting host
host can just be powered up once

Based on the following tutorials:
https://www.digitalocean.com/community/tutorials/python-socket-programming-server-client
https://pythonprogramming.net/client-chatroom-sockets-tutorial-python-3/
https://pythonprogramming.net/buffering-streaming-data-sockets-tutorial-python-3/ 


@author: Justin Kuo
"""


#%% import libraries

#opencv is used to plot images faster than Matplotlib

import socket
#for matrices
import numpy as np
#for sleep
import time
#for file writing
import os


#%% setup folder to write saved data to
#top directory where files will be stored

savedirname = "C:/Users/anujb/Downloads/RP0_wifi_test1/" #change this to whatever you want your directory name to be
if not os.path.exists(savedirname):
    os.makedirs(savedirname)

#folder to dump raw .dat files in 
rawdata_save_dir = savedirname + "rawdata/"
if not os.path.exists(rawdata_save_dir):
    os.makedirs(rawdata_save_dir)
    
img_save_dir = savedirname + "images/"
if not os.path.exists(img_save_dir):
    os.makedirs(img_save_dir)
#%% Set up wifi

#this is the IP address of the Raspberry Pi Zero W
#this should never change
# host ="raspberrypi.local"
host ="169.254.202.26"
#port opened on the RPi for communication
port = 5560  # initiate port no above 1024
SIZE = 65536 #16384 #8192 
client_socket = socket.socket()  # instantiate
client_socket.connect((host, port))  # connect to the server

#commands
#command for doing 2 dummy reads to clear RP2040 buffer
cmd_dummy = 'senddmy'
#command for reading 1 set of I and Q frames
cmd_frame = 'sendfrm'
#command send to indicate done
#we don't use this in this script
cmd_done = 'zzzdone'

cmd_mode2 = 'mmmmmm2'
cmd_mode1 = 'mmmmmm1'

timing425 = 'tttt425'

f1800 = 'a180000'

# print("Changing mode to 2")
# client_socket.send(cmd_mode2.encode())  #send command

# print("Changing mode to 1")
# client_socket.send(cmd_mode1.encode())  #send command

# print("Changing timing to 425")
# client_socket.send(timing425.encode())  #send command

print("Changing frequency to 1800.00")
client_socket.send(f1800.encode())  #send command


#send first dummy read to clear RP2040 buffers
print("Doing Dummy Read 1")
client_socket.send(cmd_dummy.encode())  #send command
data = client_socket.recv(SIZE)  # receive response
print(data.decode())
print("press the button")
#a = input("Press Enter") #note that this is not really needed after you do this once

#%%
# send second dummy read to clear RP2040 buffers
print("Doing Dummy Read 2")
client_socket.send(cmd_dummy.encode()) #send command
data = client_socket.recv(SIZE)  # receive response
print(data.decode())


# Miscellaneous functions

#for writing binary array to file
def writeFile(file_name, byte_data):
    # write to file
    f = open(file_name, "wb")
    f.write(byte_data)
    f.close()

#for reading a frame from the imager
def readFrame():
    #packet size
    exp_length = 65536
    # send command to take an image
    message = cmd_frame 
    # receive data back
    client_socket.send(message.encode()) 
    
    #for receiving frame data separated over multiple packets
    split_list = []
    #length of currently received data
    count = 0
    while (count < exp_length):
        data = client_socket.recv(SIZE)  # receive response
        datalen = len(data)
        count= count + datalen #update the number of received bytes
        message = 'rx_rcved ' + str(datalen).zfill(3) #acknowledge message
        #append received frame segments to list
        split_list.append(data)
        if (count <exp_length):
            client_socket.send(message.encode())  # send message
        
    #combine all the frame segments
    combined_bytes = split_list[0]
    for count in range(1,len(split_list)):
        combined_bytes = combined_bytes + split_list[count]
    #return the full frame
    return combined_bytes


#take raw byte_data and convert to ADC output (output is in ADC units)
#convert raw ADC data to bit-shift corrected ADC data and convert to voltage
def convertADCToVolts(I_IMAGE, Q_IMAGE):
    I_IMAGE_ADC = I_IMAGE #correct bit shift
    Q_IMAGE_ADC = Q_IMAGE #correct bit shift
    I_IMAGE_VOLTS = I_IMAGE_ADC*1e-3 #convert to volts
    Q_IMAGE_VOLTS = Q_IMAGE_ADC*1e-3 #convert to volts
    return I_IMAGE_ADC, Q_IMAGE_ADC, I_IMAGE_VOLTS, Q_IMAGE_VOLTS


#take raw byte_data and convert to ADC output (bit shift is NOT corrected)
def convertToIQImage(byte_data):
    import numpy as np
    wi = 0
    imgBytesI = np.zeros(128*128)
    imgBytesQ = np.zeros(128*128)
    for row in range (128):
        for col in range(128):
            wi = row*128 + col
            iwrd = (byte_data[4 * wi + 1] + 256*byte_data[4 * wi + 0])
            qwrd = (byte_data[4 * wi + 3] + 256*byte_data[4 * wi + 2])
            
            imgBytesI[wi] = iwrd
            imgBytesQ[wi] = qwrd
            
            
            
    J_MYIMAGE_I=imgBytesI.reshape([128,128])
    J_MYIMAGE_Q=imgBytesQ.reshape([128,128])
    return J_MYIMAGE_I, J_MYIMAGE_Q


#%% read air data
print("Doing AIR READ")
combined_bytes = readFrame() #receive a frame
AIR_I, AIR_Q = convertToIQImage(combined_bytes) #separate I and Q out
I_A_ADC, Q_A_ADC, I_A_VOLTS, Q_A_VOLTS = convertADCToVolts(AIR_I, AIR_Q)

Mag_A = np.sqrt(np.square(I_A_VOLTS)+np.square(Q_A_VOLTS))
Phase_A = np.arctan2(I_A_VOLTS, Q_A_VOLTS)
air_file_name = rawdata_save_dir + "air_data.dat" #save binary data
writeFile(air_file_name, combined_bytes) #save binary data
print("Received Air Data")


#%%
#number of frames to image
numFrames = 100
numDigits = len(str(numFrames)) #this is for file save names
import matplotlib.pyplot as plt

fig2,ax2 = plt.subplots(1)
mytitle = fig2.suptitle('Out of phase (V):  ')
im2 = np.flipud(np.rot90(Q_A_VOLTS,1))
pos201 = ax2.imshow(im2, vmin = -0.15, vmax = 0.15, cmap = 'Spectral', interpolation = 'gaussian')
fig2.colorbar(pos201)
base_title ='Real-time Ultrasonic image (V):  '
#time how long this takes
ml = []
t1 = time.time()

for cf in range(numFrames):
    #read an image
    combined_bytes = readFrame()
    
    # frame_list.append(combined_bytes)
    F_IMG_I, F_IMG_Q = convertToIQImage(combined_bytes);
    I_S_ADC, Q_S_ADC, I_S_VOLTS, Q_S_VOLTS = convertADCToVolts(F_IMG_I, F_IMG_Q)
    #write file
    #img_file_name = f"frame{cf}.dat"
    img_file_name = rawdata_save_dir+"frame"+str(cf).zfill(numDigits)+".dat"
    writeFile(img_file_name, combined_bytes)
    #plot only Q

    Mag_S = np.sqrt(np.square(I_S_VOLTS)+np.square(Q_S_VOLTS))
   
    #update title to reflect frame count
    mytitle.set_text(base_title+str(cf))
    # sub_image = np.flipud(np.rot90(Mag_S-Mag_A,1))
    sub_image = np.flipud(np.rot90(Mag_S - Mag_A,1))

    # sub_image = Q_del2 
    pos201.set_data(sub_image)
    # pos201.set_clim(-0.001,0.001)
    pos201.set_clim(-0.04,0.04)

    # pos201.set_clim(sub_image.min(),sub_image.max())
   
    plt.pause(0.001)
    
    #redraw
    fig2.canvas.draw_idle()
    fig2.savefig(img_save_dir+'plot'+str(cf)+'.png')
    
plt.close("all")
plt.plot(ml)
plt.show()

t2 = time.time() #end time
#after finished imaging frames    
client_socket.close()  # close the connection
framerate = 100/(t2-t1)
print("Frame rate: ", framerate)
#prompt to close windows
#a = input("Enter to close windows")


