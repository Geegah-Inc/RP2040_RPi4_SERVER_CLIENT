#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 27 17:45:20 2022

@author: Serhan Ardanuc

Library To Read Data From  PICO
"""
import spidev
import RPi.GPIO as GPIO
import fractions
import math # for ceil function
import time
import datetime

import numpy as np
import os

# Array parameters
N_rows = 128
N_cols = 128



#Let us start the action
def setup_GPIO(pinDict_arg):
    GPIO.setmode(GPIO.BCM) #address the GPIOs via their GPIO number
    GPIO.setup(pinDict_arg["gpio_num_PINNO_TEST"], GPIO.OUT)
    GPIO.setup(pinDict_arg["gpio_num_DAC_CE0B"], GPIO.OUT)
    #GPIO.setup(pinDict_arg["gpio_num_VCO_LE"], GPIO.OUT) #LE for VCO

    print('Finished setting up GPIO')
    

def get_spi_pico(CLK_FREQ_SPI_PICO):             
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 0
        spi.open(bus, device)            
        spi.max_speed_hz = int(CLK_FREQ_SPI_PICO)   #PICO implementation has max frequency of clock as 10MHz (100ns period)
        spi.mode = 0b00
        spi.lsbfirst = False 
        spi.bits_per_word = 8
            
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi

#
##Setup SPI for the Pico
#spi = spidev.SpiDev()
#spi.open(0,0)      
#spi.max_speed_hz = 1000000   #AD4351 has max frequency of clock as 10MHz (100ns period)
#spi.mode = 0b00  #mode is different for DAC and VCO
#spi.lsbfirst = False
#


def get_spi_vco():             
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 1
        spi.open(bus, device)            
        spi.max_speed_hz = int(5e5)   #AD4351 has max frequency of clock as 10MHz (100ns period) #change to 10e5
        spi.mode = 0b00
        spi.lsbfirst = False
        
        
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi

# MAXIM 5123
def get_spi_dac_MAXIM5123():             
    spi = None
    
    try: 
        spi = spidev.SpiDev()
        bus = 0
        device = 0
        spi.open(bus, device)            
        spi.max_speed_hz = int(1e6)   #AD4351 has max frequency of clock as 10MHz (100ns period)
        spi.mode = 0b10
        spi.lsbfirst = False
        
        
    except Exception as e:
        print("Error is:",e)
        #GPIO.cleanup()
        if spi:
            spi.close()
            spi = None
    
    return spi

# Load DAC input and DAC registers simultainously: 
# Set bits 15-->13 to 010, bit 0 to 0, then 12-bit data in bits 12-->1.
def writeDAC_MAXIM5123(spi_obj, val, gpio_num_DAC_CE0B):  #Value argument in decimal 0-4095.
    if val>4095:
        print("Error - value too big for DAC")
    else:
        GPIO.output(gpio_num_DAC_CE0B,0)  #Enable the chip select of the DAC
        print('Updating DAC with value %i and expected voltage of %4.3f V'%( val,round(get_DACVoltage_fromValue_5123(val),3)),)
        DACValLower = (val<<1)&0xfe
        DACValUpper = (val >> 7) | 0x40
        #print(DACValUpper)
        #print(DACValLower)
        spi_obj.writebytes([DACValUpper,DACValLower])
        GPIO.output(gpio_num_DAC_CE0B,1)  #Disable the chip select of the DAC
    return()


def get_DACVoltage_fromValue(val):
    """
    convert DAC value to calibrated 
    """
    if val>4095:
        print("Error - value too big for DAC")
        return None
    zero_value = 0.549
    full_value = 2.924
    full_code = 4000
    zero_code = 750
    dac_voltage  =  (val-zero_code)/(1.0*(full_code-zero_code))*(full_value-zero_value)+zero_value
    return dac_voltage

def get_DACVoltage_fromValue_5123(val):
    """
    convert DAC value to calibrated
    Use the  amplifier feedback resistor values
    """
    if val>4095:
        print("Error - value too big for DAC")
        return None
    Vref = 1.25
    Rf = 12000
    Ri = 8450
    Av = 1+Rf/Ri
    dac_voltage = val/4095*(Av)*Vref
    return dac_voltage

def int2bytes(x):
    return (x>>24) & 0xff, (x>>16) & 0xff , (x>>8) & 0xff, x & 0xff


def convert_TwoBytes_To_Int(byte_1, byte_0):
    """
    Converts 2bytes data 
    byte_1 : MSB ,1 byte
    byte_0 : LSB , 1 byte
    """
    
    return (byte_1&0xFF)<<8 | (byte_0 & 0xFF)
    


#
#def writeDAC(val, GPIO_PINNO_DAC_CLK, DAC_ORDERED_PIN_LIST):
#    if val>4096:
#        print("Error - value too big for DAC")
#    else:
#        val_binary = '{0:b}'.format(val).zfill(12)   #removes the 0b in front and pads with zeros in front
#        print("val in binary=", val_binary,"  orignal value=", val)
#        for tempIdx in range(12):
#            bit_cur=val_binary[11-tempIdx]
#            #print("bit=",int(bit), " i=",i)
#            GPIO.output(DAC_ORDERED_PIN_LIST[tempIdx], int(bit_cur))
#        GPIO.output(GPIO_PINNO_DAC_CLK,0)    
#        GPIO.output(GPIO_PINNO_DAC_CLK,1)   #write a 1 on clock to load the values
#        GPIO.output(GPIO_PINNO_DAC_CLK,0)   #write a zero to bring back for next writing        
#    return()       
    
def calc_vco_reg_values(freq_des_user,OUTEN_user,PSET_user):
    import math
    
    freq_des = freq_des_user
    OUTEN = OUTEN_user
    PSET = PSET_user# GET PICO SPI object


    #keep OUTEN in bounds
    if (OUTEN > 1):
        OUTEN = 0
    #keep PSET in bounds
    if (PSET > 3):
        PSET = 0
    
    #this is fixed, not user specifiable
    freq_res = 0.1 # in MHz
    
    #make sure it is in the correct resolution
    freq_actual = freq_res * round(freq_des / freq_res)
    
    #calculate divider setting
    myDIVSEL = math.ceil(math.log(math.ceil(2200/freq_actual))/math.log(2))
    myDIV = 2**(myDIVSEL);
    
    
    
    #reference oscillator
    refin = 10 # in MHz
    #VCO PFD (phase frequency detector) frequency
    myPFD = 10 # in MHz
    
    myVCO = freq_actual*myDIV
    mylumped = myVCO/myPFD #N in the ADI software  
    
    
    myINT = math.floor(mylumped)
    
    
    print("my DIVSEL is ",myDIVSEL)
    print("my VCO is ", myVCO)
    
    
    #mod goes from 2 to 4095
    #frac goes from 0 to mod-1
    #if 0 for frac, use 2 for mod
    
    #MOD = Refin/Fres
    #fdes_dec = freq_actual - math.floor(freq_actual)
    if ((mylumped - myINT) == 0):
        myMOD = 2
        myFRAC = 0        
    else:
        myMOD = refin/freq_res
        myFRAC = myMOD * (mylumped-math.floor(mylumped))
        myMOD = round(myMOD) #if some decimal stuff left
        myFRAC = round(myFRAC)
        
        while(math.gcd(myMOD,myFRAC)>1):
            mygcd = math.gcd(myMOD,myFRAC)
            myMOD = round(myMOD/mygcd)
            myFRAC = round(myFRAC/mygcd)
        
    #structure of register 0
    #0 at db31, 16 bit INT, 12 bit FRAC, 3 control bits 000
    reg0 = (myFRAC*(2**3)) + (myINT*(2**15))
    #structure of register 1
    reg1 = 0x8008001  + (myMOD * (2**3))
    
    #calculate reg4
    reg4 = (2**23) #fundamental feedback select
    reg4 = reg4 + (myDIVSEL*(2**20)) #divider select
    reg4 = reg4 + (2**2) #control bits
    reg4 = reg4 + (PSET*(2**3)) #power setting
    reg4 = reg4 + (OUTEN*(2**5)) #output enable setting
    reg4 = reg4 + ((80)*(2**12)) #band select clock divider value
    #registers that are unchanged
    reg2 = 0x4E42
    reg3 = 0x4B3
    reg5 = 0x580005

    return reg0, reg1, reg2, reg3, reg4, reg5


# Used to send square across the test pin
def gpio_testroutine_with_testpin(GPIO_PINNO_TEST, flag_include_delay=True):
    print("Starting Test Routine with flag_delay", flag_include_delay)
    count=100
    delay_sec = 0.001
    for tempIdx in range(count):
        GPIO.output(GPIO_PINNO_TEST, 1)
        if flag_include_delay:
            time.sleep(delay_sec)
        GPIO.output(GPIO_PINNO_TEST, 0)
        if flag_include_delay:
            time.sleep(delay_sec)
    print("Finsihed Test Routine with count %i"%count)
        


#
#def ramp_DAC_test():
#    
#    print("... Starting DAC Test")
#    dac_delay = 0.01
#    # Ramp DAC output with 
#    for DAC_val in range(2**12):
#        writeDAC(DAC_val)
#        print("Writing %i to DAC: You should read %3.2f V"%(DAC_val, 0.3+(1-DAC_val/(2**12))*2.7))
#        time.sleep(dac_delay)
#    print("... Ending DAC Test")
#    


# Number of 16bit transfer per frame
n_16bit_transfer_perFrame  =128*128*2     # 128 row X 128 col X 2 modes(I and Q) + 1sync indicator

N_BYTES_BLOCK = 1<<1 #  2**10 bytes for every Block
# REad large number of bytes as block
def blockRead_SPI( spi_obj_arg, num_bytes_to_read_arg, n_bytes_block_arg = N_BYTES_BLOCK):
    # Read 
    #num_bytes_to_read_actual =  1<<num_bytes_to_read_arg.bit_length()
    
    #allocate space
    data_out_nD =  np.zeros((num_bytes_to_read_arg,), dtype='uint16')
    
    _n_bytes_block = n_bytes_block_arg
    
    # the minimum size to be read is at leat N_BYTES_BLOCK
    num_bytes_to_read_temp  = max(num_bytes_to_read_arg, _n_bytes_block)
    n_blocks= math.floor(num_bytes_to_read_temp/_n_bytes_block)
    n_bytes_remainder = num_bytes_to_read_arg%_n_bytes_block
    
    
    #num_bytes_to_read_actual = n_blocks*N_BYTES_BLOCK
    #data_all = []
    for blockIdx in range(n_blocks):
        
        cur_slice = np.s_[blockIdx*_n_bytes_block:(blockIdx+1)*_n_bytes_block]
        #print("Reading block: %i"%blockIdx)
        data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(_n_bytes_block), dtype = 'uint16')

    
    if n_bytes_remainder >0:
        
        cur_slice = np.s_[n_blocks*_n_bytes_block:]
        # Read the remainder
        data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(n_bytes_remainder), dtype='uint16')

    
    print("Requested n_bytes = %i, n_blocks=%i, Returned n_bytes = %i"%(num_bytes_to_read_arg,n_blocks,(data_out_nD.size)))
    return data_out_nD
    

# REad large number of bytes as block
def blockRead_SPI_with2byteconversion( spi_obj_arg, num_bytes_to_read_arg, n_bytes_block_arg = N_BYTES_BLOCK, flag_verbose=False):
    # Read 
    #num_bytes_to_read_actual =  1<<num_bytes_to_read_arg.bit_length()
    
    # Make sure the odd numbers are converted to one higher
    num_bytes_to_read_actual = int(2*np.ceil(num_bytes_to_read_arg/2))
    
    
    #allocate space
    data_out_nD =  np.zeros((num_bytes_to_read_actual,), dtype='uint16')
    
    _n_bytes_block = n_bytes_block_arg
    
    # the minimum size to be read is at leat N_BYTES_BLOCK
    num_bytes_to_read_temp  = max(num_bytes_to_read_actual, _n_bytes_block)
    n_blocks= math.floor(num_bytes_to_read_temp/_n_bytes_block)
    n_bytes_remainder = num_bytes_to_read_actual%_n_bytes_block
    
    
    #num_bytes_to_read_actual = n_blocks*N_BYTES_BLOCK
    #data_all = []
    for blockIdx in range(n_blocks): 
        
        cur_slice = np.s_[blockIdx*_n_bytes_block:(blockIdx+1)*_n_bytes_block]
        #print("Reading block: %i"%blockIdx)
        data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(_n_bytes_block), dtype = 'uint16')

    
    if n_bytes_remainder >0:
        
        cur_slice = np.s_[n_blocks*_n_bytes_block:]
        # Read the remainder
        data_out_nD[cur_slice] =  np.array(spi_obj_arg.readbytes(n_bytes_remainder), dtype='uint16')

    
    # Now convert to two bytes
    
    data_out_nD = convert_TwoBytes_To_Int(data_out_nD[0::2], data_out_nD[1::2])
    if flag_verbose:   
        print("Requested n_bytes = %i, n_blocks=%i, Returned n_bytes = %i as %i uint16"%(num_bytes_to_read_arg,n_blocks,int (2*data_out_nD.size),data_out_nD.size))
    return data_out_nD
    


# This one is for before block read to make sure we are synchronized to the 
# the buffer signature bytes 0xEFFF and 0xFFFF) 
# Do not use
def read_data_until_synchronized(spi_obj_arg, bufferMarker_List =[0xEFFF, 0xFFFF] ):
    """
    Read 2 bytes by 2 bytes  until you hit the buffer signatures 0xEFFF and 0xFFFF
    and then read 128*128*2*2 bytes all at once
    """
    


    """
    read a frame from the spi_obj, eaxh transfer is 16 bit long
    Both I and Q data in a single frame as 1D array
    TODO:  Update it to block read once the FW supports it
    """
    
    # Returnn buffer Signature to indicate which buffer you are reading From
    bufferSignature = -1

    # First two bytes ie frame sync transfer will be either #0xFFFF or #0xEFFF based on which buffer you are reading
    # Then it will spit 128*128*2 = n_16bit_transfer_perFrame
    
    N_BYTES_BLOB = 2
    flagSignatureFound = False
    
    # Keep track of unsyncronized data
    missedBlobCount = 0
    
    # TODO call with timeout
    while (missedBlobCount < n_16bit_transfer_perFrame+10):
        # Read 2 Bytes
        curData_TwoByte_List = spi_obj_arg.readbytes(N_BYTES_BLOB)
        #print(curData_TwoByte_List)
        blobValue = convert_TwoBytes_To_Int(*curData_TwoByte_List)
        # If the flagSignatureFound is True,we are synchronized and the next 128*128*2 (byte pairs) are good    
        flagSignatureFound = blobValue in bufferMarker_List
        if flagSignatureFound:            # Then the following n_16bit_transfer_perFrame will be the frame
            break
        else:
            missedBlobCount = missedBlobCount+1
            # display every 1000 value
            if missedBlobCount%1000 ==0:
                print(missedBlobCount,"{:X}".format(blobValue))
                
            # dellater
            if missedBlobCount<5:
                print(missedBlobCount,"{:X}".format(blobValue))
    
    # Save in case you want to check for any trouble
    bufferSignature = blobValue
    

    return  flagSignatureFound, bufferSignature, missedBlobCount


def read_block_singleOrMultiple_frames_at_unsynchronized_state(spi_obj_arg,nFrames = 1, n_bytes_block_arg=1<<1,   flag_verbose = False, bufferMarker_List =[0xEFFF, 0xFFFF] ):
    

    
    """
    Read 2 bytes by 2 bytes  until you hit the buffer signatures 0xEFFF and 0xFFFF
    and then read 128*128*2*2 bytes all at once
    """
    
    missedBlobCount_1D = -1*np.ones((nFrames,))
    bufferSignature_1D = -1*np.ones((nFrames,))
    flagSignatureFound_1D = np.array([False]*nFrames, dtype=bool)
    #Also store the timestamp
    timeStamp_1D = np.array([datetime.datetime.now()]*nFrames)
    
    # Note that initally we will store each and every bit
    # But we will do the final conversion at the end

    # DIM 0: nFrames
    # DIM 1: 2   for I,Q
    # DIM 2: picoDAQ_Lib.N_rows
    # DIM 3: picoDAQ_Lib.N_cols
    frames_data_nD = np.zeros((nFrames,2,N_rows,N_cols), dtype="uint16")
    
#    frames_data_nD = 0
    # Total block read bytes:
    num_bytes_to_blockread = n_16bit_transfer_perFrame*2
    
#    flagSignatureFound, bufferSignature, missedBlobCount = read_data_until_synchronized(spi_obj_arg = spi_obj_arg, \
#                                                                                         bufferMarker_List =[0xEFFF, 0xFFFF] )
    
        # if you could not find the searched signature bytes issue a warning
#    if missedBlobCount > picoDAQ_Lib.n_16bit_transfer_perFrame:
#        print("Could not find the signatures in the first run.")
    
#    flagSignatureFound = bufferSignature in bufferMarker_List
#    if flagSignatureFound:
#        print("Signature found! Starting block read!")
#    else:
#        print("Signature not found")
    
    # Some initializations
    flagSignatureFound = False
    blobValue = -1
    missedBlobCount = -1
    
    
    frame_count = 0
    
    for frameIdx in range(nFrames):

            # Get the rest of the frames
        
    
        if flag_verbose:        
            # Comment the following out later
            print("Reading frame %i of %i:"%(frameIdx, nFrames) )
        
        if flagSignatureFound:
            # We got the frame
            bufferSignature = blobValue
            missedBlobCount = 0
        else:
            if flag_verbose:   
                print("Searching for the signature for frameIdx %i. "%frameIdx)
            
            
            flagSignatureFound, bufferSignature, missedBlobCount = read_data_until_synchronized(spi_obj_arg = spi_obj_arg,\
                                                                                     bufferMarker_List =[0xEFFF, 0xFFFF] )
                

        if flagSignatureFound:
        # perform the block read            
            frame_data_1D = blockRead_SPI_with2byteconversion( spi_obj_arg=spi_obj_arg,\
                                      num_bytes_to_read_arg = num_bytes_to_blockread, n_bytes_block_arg = n_bytes_block_arg)
            
            if flag_verbose:   
                print("Length of bytes collected is %i"%(frame_data_1D.size))  
            
            if (frame_data_1D.size)==n_16bit_transfer_perFrame:
                if flag_verbose:   
                    print("Adding frame {} ".format(frame_count))
                frame_count+=1
            
            # convertfrom 1D byte pairs to the actual numbers
            # From size 
#            frames_data_1D = np.apply_along_axis (lambda x: picoDAQ_Lib.convert_TwoBytes_To_Int(*x), axis=1, arr = np.reshape(bytes_data_1D,(picoDAQ_Lib.n_16bit_transfer_perFrame,2)))
            # add to the main data after
            # Make sure you permute the axis since I/Q is in C order
            frames_data_nD [frameIdx,...] = np.moveaxis(np.reshape(frame_data_1D, (N_rows,N_cols,2)),-1,0)
        
        
        
        
        
        else:
            if flag_verbose:   
                print("No signature found for frame {}. Assuming all -1's".format(frameIdx))
            frame_data_1D = -1*np.ones((n_16bit_transfer_perFrame,), dtype = 'uint16')
        
        
        
        # You Update timestamp
        timeStamp_1D [frameIdx] = datetime.datetime.now()
        
        # You have your first signature
        bufferSignature_1D [frameIdx] = bufferSignature
        # also record the previous missedBlobCount
        missedBlobCount_1D [frameIdx] = missedBlobCount
        # Sto0re the flagSignatureFoung
        flagSignatureFound_1D [frameIdx] = flagSignatureFound
        
        
        if flag_verbose:   
            print("bufferSignature for frameIdx {} is {:X}".format( frameIdx,bufferSignature))
            print("missedBlobCount for frameIdx {} is {:d}".format( frameIdx,missedBlobCount))
            print("flagSignatureFound for frameIdx {} is {}".format( frameIdx,flagSignatureFound))
        
        # Do the following only if this was not the last frame, just to save time
        if frameIdx<nFrames-1:
            # Now read 2 bytes this should be the signature for the next frame        
            # Read 2 Bytes
            curData_TwoByte_List = spi_obj_arg.readbytes(2)
            #print(curData_TwoByte_List)
            blobValue = convert_TwoBytes_To_Int(*curData_TwoByte_List)
            # If the flagSignatureFound is True,we are synchronized and the next 128*128*2 (byte pairs) are good    
            flagSignatureFound = blobValue in bufferMarker_List
        
    # Time for numpy conversion: First get every 2 bytes
        
    #frames_data_nD = np.reshape(np.array(frame_bytes_ParentList_ofList, dtype = "uint16"),(frame_count, 2, picoDAQ_Lib.N_rows,picoDAQ_Lib.N_cols,2))

    return frames_data_nD,timeStamp_1D, flagSignatureFound_1D, bufferSignature_1D, missedBlobCount_1D





def read_single_frame(spi_obj_arg):
    """
    read a frame from the spi_obj, eaxh transfer is 16 bit long
    Both I and Q data in a single frame as 1D array
    TODO:  Update it to block read once the FW supports it
    """
    
    # Returnn buffer Signature to indicate which buffer you are reading From
    bufferSignature = -1
    frameDataOut=[]
    # First two bytes ie frame sync transfer will be either #0xFFFF or #0xEFFF based on which buffer you are reading
    # Then it will spit 128*128*2 = n_16bit_transfer_perFrame
    
    N_BYTES_BLOB = 2
    
    
    # Keep track of unsyncronized data
    missedBlobCount = 0
    # TODO call with timeout
    while (missedBlobCount < n_16bit_transfer_perFrame+10):
        # Read 2 Bytes
        curData_TwoByte_List = spi_obj_arg.readbytes(N_BYTES_BLOB)
        #print(curData_TwoByte_List)
        blobValue = convert_TwoBytes_To_Int(*curData_TwoByte_List)
        
        if (blobValue == 0xEFFF) | (blobValue == 0xFFFF):            # Then the following n_16bit_transfer_perFrame will be the frame
            break
        else:
            missedBlobCount = missedBlobCount+1
            # display every 1000 value
            if missedBlobCount%1000 ==0:
                print(missedBlobCount,"{:X}".format(blobValue))
    
    # Save in case you want to check for any trouble
    bufferSignature = blobValue
    
    # TODO: Update this to block read later
    for byteIdx in range(n_16bit_transfer_perFrame):

        blobValue = convert_TwoBytes_To_Int(*spi_obj_arg.readbytes(N_BYTES_BLOB))
        frameDataOut.append(blobValue)
    
    
    return frameDataOut, bufferSignature, missedBlobCount

#%% Plotter function for a single frame
def plot_I_Q_Mag_imageFrame(frames, frameIdx = 0, vmin_arg=-100, vmax_arg=100):
    import matplotlib.pyplot as plt
    """
    plots the frame for I, !, and mag
    frames 4D:
    # DIM0: Number of frames, size: N_frames
    # DIM1: I or Q, 0:I, 1:Q size:2
    # DIM2: N_rows
    # DIM3: N_cols 
    
            
    """
    
    fig01,ax01 = plt.subplots(nrows=1, ncols=3, figsize=(12,8))
    
    ax01_a,ax01_b,ax01_c = ax01
    
    # plot the I 
    h_plot_a = ax01_a.imshow(frames[frameIdx,0,:,:],cmap="inferno", vmin=vmin_arg, vmax=vmax_arg)
    #colorboar
    h_colorbar_a = fig01.colorbar(h_plot_a, ax=ax01_a)
    ax01_a.set_title("Baseline Subtracted I")
    
    # plot the Q 
    h_plot_b = ax01_b.imshow(frames[frameIdx,1,:,:],cmap="inferno",vmin=vmin_arg, vmax=vmax_arg)
    #colorboar
    h_colorbar_b = fig01.colorbar(h_plot_b, ax=ax01_b)
    ax01_b.set_title("Baseline Subtracted Q")
    
    
    # Calculate the magnitude
    # Note that the below is nRows*nCols, 128x128
    frame_mag_2D = np.sqrt(np.power(frames[frameIdx,0,:,:].astype("float"),2)+np.power(frames[frameIdx,1,:,:].astype("float"),2))

        
    print("frame I mean", np.mean(frames[frameIdx,0,:,:]))
    print("frame Q mean", np.mean(frames[frameIdx,1,:,:]))
    print("frame_mag_2D mean", np.mean(frame_mag_2D))
    
    # plot the mag, min is 0
    h_plot_c = ax01_c.imshow(frame_mag_2D,cmap="inferno", vmin=0, vmax=vmax_arg)
    #colorboar
    h_colorbar_c = fig01.colorbar(h_plot_c, ax=ax01_c)
    ax01_c.set_title("Baseline Subtracted Mag")
    
    
    h_plot_list = [h_plot_a,h_plot_b, h_plot_c]
    h_colorbar_list = [h_colorbar_a, h_colorbar_b, h_colorbar_c]
    
    return fig01, ax01, h_plot_list, h_colorbar_list


def getFrames_I_and_Q_merged_and_LivePlot(spi_ob_arg=None,  nFrames = 1, frames_baseline_avg = np.zeros((1,2,N_rows, N_cols), dtype='uint16'), flag_livePlot=True, vmin_arg=None, vmax_arg=None):
    import matplotlib.pyplot as plt
    
    """
    Live plot and return I and Q together as numpy arrays of uint16
    
    Return Frame has dimensions of following 
        plots the frame
    frames 4D:
    # DIM0: Number of frames, size: N_frames
    # DIM1: I or Q, 0:I, 1:Q size:2
    # DIM2: N_rows
    # DIM3: N_cols 
    
    # NOTE THAT the PLOT IS DONE with Baseline subtraction, but the returned array is without subtraction to keep the raw data
    """
    
    # initialize for return
    fig01=None
    
    missedBlobCount_1D = 0*np.zeros((nFrames,))
    bufferSignature_1D = 0*np.zeros((nFrames,))
    
    frames_sample = np.zeros((nFrames,2,N_rows,N_cols), dtype="uint16")
    
    #Plot if desired
    if flag_livePlot:
        
        # Turn the interactive mode on
        # https://www.geeksforgeeks.org/how-to-update-a-plot-in-matplotlib/
        plt.ion()
        fig01, ax01, h_plot_list, h_colorbar_list = plot_I_Q_Mag_imageFrame(frames_sample, frameIdx = 0, vmin_arg=vmin_arg, vmax_arg=vmax_arg )
        h_suptitle = fig01.suptitle("Frame:%i"%(0))
        # Non blocking show
        plt.show(block=False)

        
    # Get the baseline Average
    baseline_I_mean_allpixels =np.mean(frames_baseline_avg[0,0,...])
    baseline_Q_mean_allpixels =np.mean(frames_baseline_avg[0,1,...])
    
    flagMissedBlob = False
    firstMissedBlobCount = 0
    for frameIdx in range(nFrames):
        # Get the single Frame
        frameDataOut, bufferSignature, missedBlobCount = read_single_frame(spi_ob_arg)
        # Check if any Frame is Missed
        if not flagMissedBlob & (missedBlobCount>0):
            flagMissedBlob = True
            firstMissedBlobCount = missedBlobCount
     
        # ASsume the first I and then Q comes
        frames_sample[frameIdx,0,:,:] = np.reshape(np.array(frameDataOut[::2], dtype='uint16'), (N_rows,N_cols))
        frames_sample[frameIdx,1,:,:] = np.reshape(np.array(frameDataOut[1::2], dtype='uint16'), (N_rows,N_cols))
       
        missedBlobCount_1D[frameIdx] = missedBlobCount
        bufferSignature_1D[frameIdx] = bufferSignature
        

        # Update the figure if liveplot is enabled
        if flag_livePlot:
            

            if True:
            # subtract the background   [frame frame_delta_cur is 2x128x128]         
                frame_delta_cur = frames_sample[frameIdx,...].astype('int32') - frames_baseline_avg[0,...].astype('int32')
                
                # Calculate the magnitude
                # Note that the below is nRows*nCols, 128x128
                frame_mag_2D_cur = np.sqrt(np.power(frame_delta_cur[0,:,:].astype("float"),2)+np.power(frame_delta_cur[1,:,:].astype("float"),2))

            
            # Update the data of the plot for I
            h_plot_list[0].set_data(frame_delta_cur[0,:,:])
            # Update the data of the plot for Q
            h_plot_list[1].set_data(frame_delta_cur[1,:,:])
            
            # Update the data of the plot for magnitude
            h_plot_list[2].set_data(frame_mag_2D_cur)
            
            # Update the title
            frame_cur_I_mean_allpixels =np.mean(frames_sample[frameIdx,0,...])
            frame_cur_Q_mean_allpixels =np.mean(frames_sample[frameIdx,1,...])
            
            annotation_text_02 = "Frame Averages: Baseline_I={%3.2f}, Baseline_Q={%3.2f}, CurFrame_I={%3.2f}, CurFrame_Q={%3.2f}"\
            %(baseline_I_mean_allpixels,baseline_Q_mean_allpixels,frame_cur_I_mean_allpixels, frame_cur_Q_mean_allpixels)
            h_suptitle.set_text("Frame:%i @ %s\n %s"%(frameIdx, datetime.datetime.now(), annotation_text_02))
            
            

            
            # update the figure
            # https://www.geeksforgeeks.org/how-to-update-a-plot-in-matplotlib/
            fig01.canvas.draw()
            fig01.canvas.flush_events()
            
            #plt.show()
            
            print("Frame:%i is plotted"%(frameIdx))

        
    
    if firstMissedBlobCount > 0:
        print("WARNING: Missed Blob: FirstMissedBlobCount %i"%(firstMissedBlobCount))
    print("Captured %i frames!"%nFrames)
    return frames_sample , bufferSignature_1D, missedBlobCount_1D, fig01
    
    
    
    
    
    
    
    
    
        
def getFrames_I_and_Q(spi_ob_arg=None, nFrames = 1):
    # Grabs nFrames and 
    # Returns numpy arrays of the data read from the 
    # create images
    frames_I = np.zeros((nFrames,N_rows, N_cols),dtype='uint16')
    frames_Q = np.zeros((nFrames,N_rows, N_cols),dtype='uint16')
    missedBlobCount_1D = 0*np.zeros((nFrames,))
    bufferSignature_1D = 0*np.zeros((nFrames,))
 
    flagMissedBlob = False
    firstMissedBlobCount = 0
    for frameIdx in range(nFrames):
        # Get the single Frame
        frameDataOut, bufferSignature, missedBlobCount = read_single_frame(spi_ob_arg)
        # Check if any Frame is Missed
        if not flagMissedBlob & (missedBlobCount>0):
            flagMissedBlob = False
            firstMissedBlobCount = missedBlobCount
     
        # ASsume the first I and then Q comes
        frames_I[frameIdx,:,:] = np.reshape(np.array(frameDataOut[::2], dtype='uint16'), (N_rows,N_cols))
        frames_Q[frameIdx,:,:] = np.reshape(np.array(frameDataOut[1::2], dtype='uint16'), (N_rows,N_cols))
        missedBlobCount_1D[frameIdx] = missedBlobCount
        bufferSignature_1D[frameIdx] = bufferSignature
    
    if firstMissedBlobCount > 0:
        print("WARNING: Missed Blob: FirstMissedBlobCount %i"%(firstMissedBlobCount))
    print("Captured %i frames!"%nFrames)
    return frames_I, frames_Q , bufferSignature_1D, missedBlobCount_1D
    
    
    # Poll until you read 

def unload_And_Load_SPI_Aux():

    print("----Unloading and Reloading module---")
    commandList = ["echo 'here we go!'",
            'sudo rmmod spi_bcm2835aux',
    'sudo modprobe spi_bcm2835aux']
    
    for command in commandList:
        print("Executing command:", command)
        returnCode = os.system(command)
        print("Command Returned", returnCode)
    print("Done loading the command")
