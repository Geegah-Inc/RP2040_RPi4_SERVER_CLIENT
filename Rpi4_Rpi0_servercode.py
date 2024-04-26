#!usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 14:11:44 2023

Server program to run on Raspberry Pi Zero W at bootup


Setup runs at startup
Need to take 2 dummy frames after setup
Then press the reset button on the imager


Based on Serhan and Scott's original code

Also based on the following tutorial:
https://pythonprogramming.net/server-chatroom-sockets-tutorial-python-3/

@author: Justin Kuo
"""


#import Serhan's original library
import picoDAQ_Lib   # custom functions for DataAcquisition

#import Justin's modification
#main modification is that functions that relied on numpy arrays are using binary arrays now
#3X improvement in acquisition speed
import picoDAQ_Lib_JMOD1

#raspberry pi GPIO
import RPi.GPIO as GPIO

#for sleep
import time

#for file operations
#not used right now, but would be good to have option to save log files
#import os

#following two libraries are used for socket operations
import socket
import select

import serial

gmode=1
gadc=1
gtiming=125
gfreq=1853

def serial_setup(mode=1, adc=1, timing=125):
    ser = serial.Serial('/dev/ttyACM0', 115200)
    print("Found serial port")
    packet = str(str(mode)+'.'+str(timing)+'.'+str(adc)+'\n')
    print(packet)
    ser.write(packet.encode())
    ser.close()

serial_setup(gmode, gadc, gtiming)

#%% initial GPIO setup --> this part will run at boot
#this section is unmodified

GPIO_PINNO_TEST = 27

# VCO, ad4351 PINS, SPI0, DEV0
GPIO_NUM_VCO_LE = 7  #SPI0_CE1 from Rpi
#GPIO_NUM_VCO_CE = 8 # Note that SPI GPIO8 is connected to VCO-LE and VCO CE-is floating, since it is pulled high all the time on the board
#GPIO_NUM_VCO_CLK = 11
#GPIO_NUM_VCO_DATA = 10

## Raspberry PI, RP PICO, SPI1, DEV0
#GPIO_NUM_PICO_CE = 18 # GPIO 28 @ PIN12
#GPIO_NUM_PICO_CLK = 21 # GPIO 21 @ PIN40
#GPIO_NUM_PICO_MISO = 19  # GPIO 19 @ PIN35

# SPI DAC new device, Configured on SPI0 with dedicated GPIO for C#
GPIO_NUM_DAC_MOSI = 10
GPIO_NUM_DAC_MISO = 9
GPIO_NUM_DAC_CLK = 11
GPIO_NUM_DAC_CE = 8 # STANDARD SPI0, dev0 CE, not used for DAC
GPIO_NUM_DAC_CE0B =22 # Dedicated DAC for


##GPIO assignments;
#gpDacMosi=10
#gpDacMiso=9
#gpDacSck=11
#gpDacCe0=8
#gpTest=25
#gpDacCe0b = 22
#gpTest = 27
#
#gpVcoLe = 7
#gpVCOCE=8
#gpVCOCLK=11
#gpVCODATA=10

# Connections
# RPI           Target
# Pin15         CE ----> DAC----> right connector
# Pin19         Data ----> DAC----> right connector
# Pin21         Data ----> PICO----> PICO connector
# Pin23         CK ----> DAC & CK-----PICO : ----> shorted externally
# Pin24         CS ----> PICO----> PICO connector
# Pin26         LE ----> VCO----> Right connector


pinDict_Main = dict(gpio_num_PINNO_TEST = GPIO_PINNO_TEST,
                    gpio_num_DAC_CE0B = GPIO_NUM_DAC_CE0B,
                    gpio_num_VCO_LE = GPIO_NUM_VCO_LE);


# CLKn frequency for SPI in Hz
CLK_FREQ_SPI_PICO =int(6e6)


# SEtup the GPIO
picoDAQ_Lib.setup_GPIO(pinDict_arg = pinDict_Main)

#%% More startup sequence
#also unmodified

#Create the sPI object for VCO
# SPI-0, DEV-1, with
# Get VCO SPi object
spi_obj_vco = picoDAQ_Lib.get_spi_vco()


def set_frequency(freq_val=1853):
    global spi_obj_vco
    # VCO settings and registers
    freq_target_MHz = 1853
    OUTEN_user = 1
    PSET_user = 3
    regs=picoDAQ_Lib.calc_vco_reg_values(freq_target_MHz,OUTEN_user ,PSET_user)

    regshx=[]
    for i in range(len(regs)):
        regshx.append("{:#010x}".format(regs[i]))
    print("The registers are:", regshx)


    # Create list from regs
    # This is basically same as regbytesbyte
    regs_list_of_ints  = list(map(picoDAQ_Lib.int2bytes, regs))


    #Start writing from the re5 down to reg0
    for i in range(len(regs)-1,-1,-1):
        regToWrite_list =regs_list_of_ints[i]

        print("writing register",i,regToWrite_list)
        # Because we connected to LE to CE pin of the pi, and keep CE of the VCO on the board floating
        # LE pin needs to be lowered and made high to load the registers
        #GPIO.output(GPIO_NUM_VCO_LE, 0)
        spi_obj_vco.writebytes(regToWrite_list)
        #GPIO.output(GPIO_NUM_VCO_LE, 1)

    print(f"Frequency set to {freq_val}")

set_frequency(gfreq)

#%% More setup
#DAC update
# Setup SPI for DAC MAXIM 5252, Bus0, DEV 0 but with dedicated CS GPIO pin at GPIO_NUM_DAC_CE0B
DAC_val = 3815
spi_obj_dac = picoDAQ_Lib.get_spi_dac_MAXIM5123()
picoDAQ_Lib.writeDAC_MAXIM5123(spi_obj_dac, DAC_val, GPIO_NUM_DAC_CE0B)
#spi_obj_dac.close()

#GET PICO SPI object
#also unmodifiedi
spi_obj_pico = picoDAQ_Lib.get_spi_pico(CLK_FREQ_SPI_PICO)

# Block read Size
n_bytes_block_arg=1<<12 #was 1

#%% Server setup


# set the hostname to the RPi IP address
host ="169.254.202.26"
#host ="raspberrypi.local"
# port for WiFi communication
# make sure this one is not blocked by ufw firewall
port = 5560  # initiate port - any port above 1024 should be ok

# socket packet size for TCP transfers
# one frame just happens to be ok
SIZE = 65536


#segments to divide the transmit into
#fortunately we are able to use a packet size that fits one frame
#so this variable is 1
#setting larger will slice each frame into multiple packets for transmission
#which slows down the acquisition speed
num_segments = 1


#commands
#command for doing 2 dummy reads to clear RP2040 buffer
cmd_dummy = 'senddmy'
#command for reading 1 set of I and Q frames
cmd_frame = 'sendfrm'
#command send to indicate done
#z in front is because this is an ASCII character that we will never see from a 12 bit ADC value
#but we are not actually doing anything with this on the client end
cmd_done = 'zzzdone'

cmd_mode2 = 'mmmmmm2'
cmd_mode1 = 'mmmmmm1'
cmd_mode0 = 'mmmmmm0'

# cmd_serial = 'a'

#flags
sendFrameCount = 0

#initialize variable
#this is used for slicing up frame data over multiple transmissions
split_list = []


# Create a socket
# socket.AF_INET - address family, IPv4, some otehr possible are AF_INET6, AF_BLUETOOTH, AF_UNIX
# socket.SOCK_STREAM - TCP, conection-based, socket.SOCK_DGRAM - UDP, connectionless, datagrams, socket.SOCK_RAW - raw IP packets
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# SO_ - socket option
# SOL_ - socket option level
# Sets REUSEADDR (as a socket option) to 1 on socket
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)



# Bind, so server informs operating system that it's going to use given IP and port
# For a server using 0.0.0.0 means to listen on all available interfaces, useful to connect locally to 127.0.0.1 and remotely to LAN interface IP
# Try/Except/Else just in case WiFi access point is not fully initialized yet
while True:
    try:
        server_socket.bind((host, port))
        print("trying hard")
    except:
        time.sleep(5) #wait 5 seconds before trying again
        print("still waiting")
    else:
        print("Succesfully bound socket")
        break

# This makes server listen to new connections
server_socket.listen()

# List of sockets for select.select()
sockets_list = [server_socket]

#print that this setup is done
print(f'Listening for connections on {host}:{port}...')

#%% Miscellaneous functions

#function for reading 2 dummy frames
#this uses the original numpy based code
#since speed is not important for this
def dummyRead(spi_obj_pico_VAR):
    #read two dummy frames
    #this is to clear garbage data out of the buffer
    frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = picoDAQ_Lib.getFrames_I_and_Q(spi_ob_arg=spi_obj_pico_VAR, nFrames = 2)


#function for reading a single frame
#this uses the faster binary array based code
def readFrame(spi_obj_pico_VAR):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib_JMOD1.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico_VAR,\
                                                                  n_bytes_block_arg=n_bytes_block_arg)
    return frames_data_nD_baseline

# Handles receiving from client
# these should all be commands in ASCII
def receive_message(client_socket):

    try:

        # Receive message from client
        message_data = client_socket.recv(SIZE)

        # If we received no data, client gracefully closed a connection, for example using socket.close() or socket.shutdown(socket.SHUT_RDWR)
        if not len(message_data):
            return False

        #decode to ASCII
        message_data_stripped = message_data.decode().strip()

        # Return an object of message header and message data
        return message_data_stripped

    except:

        # If we are here, client closed connection violently, for example by pressing ctrl+c on his script
        # or just lost his connection
        # socket.close() also invokes socket.shutdown(socket.SHUT_RDWR) what sends information about closing the socket (shutdown read/write)
        # and that's also a cause when we receive an empty message
        return False



#%% main server loop
# this part runs forever
# until user powers off the raspberry pi

#run foreer
while True:

    # Calls Unix select() system call or Windows select() WinSock call with three parameters:
    #   - rlist - sockets to be monitored for incoming data
    #   - wlist - sockets for data to be send to (checks if for example buffers are not full and socket is ready to send some data)
    #   - xlist - sockets to be monitored for exceptions (we want to monitor all sockets for errors, so we can use rlist)
    # Returns lists:
    #   - reading - sockets we received some data on (that way we don't have to check sockets manually)
    #   - writing - sockets ready for data to be send thru them
    #   - errors  - sockets with some exceptions
    # This is a blocking call, code execution will "wait" here and "get" notified in case any action should be taken
    read_sockets, _, exception_sockets = select.select(sockets_list, [], sockets_list)

    #initialize/reset received data variable
    rx_message = ''

    #reset/initialize flags
    #these flags are used to determine if the client has send a dummy read command or a send frame command
    dummyFlag = False
    sendFrameFlag = False
    mode2Flag = False
    mode1Flag = False
    mode0Flag = False
    timingFlag = False
    freqFlag = False

    # Iterate over notified sockets
    for notified_socket in read_sockets:

        # If notified socket is a server socket - new connection, accept it
        if notified_socket == server_socket:

            # Accept new connection
            # That gives us new socket - client socket, connected to this given client only, it's unique for that client
            # The other returned object is ip/port set
            client_socket, client_address = server_socket.accept()


            rx_message = receive_message(client_socket)

            # If False - client disconnected before he sent his name
            if rx_message is False:
                continue

            # Add accepted socket to select.select() list
            sockets_list.append(client_socket)

            # this line maybe not needed?
            notified_socket = client_socket

            #print notification
            print('Accepted new connection from {}:{}'.format(*client_address))

        # Else existing socket is sending a message
        else:

            # Receive message
            rx_message = receive_message(notified_socket)

            # If False, client disconnected, cleanup
            if rx_message is False:
                print('Closed connection')

                # Remove from list for socket.socket()
                sockets_list.remove(notified_socket)



                continue
            #uncomment as needed
            #print('Received Message')
            #print(rx_message)


        #set flags
        #if received command to take dummy frames
        if (rx_message == cmd_dummy):
            dummyFlag = True
        elif (len(rx_message)>8): #sometimes the message comes on the end of a previous message
            if (rx_message[-7:]==cmd_dummy):
                dummyFlag=True
        # if received command to take an image frame
        if (rx_message==cmd_frame):
            sendFrameFlag = True
        elif (len(rx_message)>8): #somtimes message comes at the end of a previous messsage
            if (rx_message[-7:]==cmd_frame):
                sendFrameFlag=True

        if (rx_message==cmd_mode2):
            mode2Flag = True
        elif (len(rx_message)>8): #somtimes message comes at the end of a previous messsage
            if (rx_message[-7:]==cmd_mode2):
                mode2Flag=True

        if (rx_message==cmd_mode1):
            mode1Flag = True
        elif (len(rx_message)>8): #somtimes message comes at the end of a previous messsage
            if (rx_message[-7:]==cmd_mode1):
                mode1Flag=True

        if (rx_message==cmd_mode0):
            mode0Flag = True
        elif (len(rx_message)>8): #somtimes message comes at the end of a previous messsage
            if (rx_message[-7:]==cmd_mode0):
                mode0Flag=True

        # tttt425 - format for timing
        if rx_message[:4] == 'tttt' and rx_message[-3:].isdigit():
            timingFlag = True
            gtiming = int(rx_message[-3:])
        elif len(rx_message) > 7 and rx_message[-7:-3] == 'tttt' and rx_message[-3:].isdigit():
            timingFlag = True
            gtiming = int(rx_message[-3:])

        # a185300 - format for frequency
        if rx_message[0] == 'a' and rx_message[-6:].isdigit():
            freqFlag = True
            freqval = int(rx_message[-6:])
            gfreq = freqval/100
            print("freq flag set")
        elif len(rx_message) > 7 and rx_message[-7] == 'a' and rx_message[-6:].isdigit():
            freqFlag = True
            freqval = int(rx_message[-6:])
            gfreq = freqval/100
            print("freq flag set")

        #if the flags are set, do the appropriate action

        #to do dummy read
        if (dummyFlag):
            #read two dummy frames
            dummyRead(spi_obj_pico)
            #send acknowledge
            notified_socket.send(cmd_done.encode())
            #reset the flag
            dummyFlag = False

        #to take an image
        if (sendFrameFlag):
            #Read frame from imager
            dataFrame=readFrame(spi_obj_pico)

            #split to segments if needed
            split_list = []
            #it just so happens that the data is evenly divisible exactly into chunks of 8192
            for count in range(num_segments):
                split_list.append(dataFrame[(count*SIZE):((count+1)*SIZE)])
            sendFrameCount = num_segments #this variable is named poorly, should be segmentCount

        if mode2Flag:
            gmode=2
            serial_setup(gmode, gadc, gtiming)
            mode2Flag = False

        if mode1Flag:
            gmode=1
            serial_setup(gmode, gadc, gtiming)
            mode1Flag = False

        if mode0Flag:
            gmode=0
            serial_setup(gmode, gadc, gtiming)
            mode1Flag = False

        if timingFlag:
            serial_setup(gmode, gadc, gtiming)
            timingFlag = False

        if freqFlag:
            set_frequency(gfreq)
            print("called set_frequency")
            freqFlag = False

        # send each frame segment to the client
        if (sendFrameCount > 0):
            sendIndex = num_segments-sendFrameCount #frame segment to send
            sendData = split_list[sendIndex] #frame segment data
            notified_socket.send(sendData) #send frame segment data
            sendFrameCount = sendFrameCount-1 #number of remaining frame segments to send

    # It's not really necessary to have this, but will handle some socket exceptions just in case
    for notified_socket in exception_sockets:

        # Remove from list for socket.socket()
        sockets_list.remove(notified_socket)


# #%% the code should never reach this point!
# #because the user will power off before this point

spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()

print("Reached End of Server Script")