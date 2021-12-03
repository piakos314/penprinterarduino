import numpy as np
import time
from numpy.lib.function_base import angle
import serial
from serial.serialutil import XOFF
from serial.win32 import DTR_CONTROL_DISABLE

ser = serial.Serial('COM3',9600)

## global functions ###

def deg_rad(x):
    return x*(np.pi/180)

def rad_deg(x):
    return x*(180/np.pi)

def deg_freq(a1, a2):
    a1 =int(((a1+90)*(1625/180))+645)     # attachment correction + servo error correction (range 645 - 2270 microsecond) 
    a2 =int(((180-a2)*(1660/180))+730)    # attachment correction + servo error correction (range 675 - 2267 microsecond) 
    return a1, a2

def rad_freq(a1,a2):
    a1 =int(((a1+(np.pi/2))*(1625/np.pi))+645)     # attachment correction + servo error correction (range 645 - 2270 microsecond) 
    a2 =int(((np.pi-a2)*(1660/np.pi))+730)    # attachment correction + servo error correction (range 675 - 2267 microsecond) 
    return a1, a2

def cartesian_to_polar(x,y):
    r = np.sqrt((x**2 + y**2))
    theta = np.arctan2(y,x)    # in radian
    return r, theta

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def position_of_base_cartesian(nx,ny,arm_length):
    nr,ntheta = cartesian_to_polar(nx,ny)
    midlength = np.sqrt(arm_length**2 -( nr/2)**2) # pythagoras
    midx = nx/2
    midy = ny/2
    midtheta = ntheta - np.pi/2
    xmidlength = midlength * np.cos(midtheta)
    ymidlength = midlength * np.sin(midtheta)
    x = midx+xmidlength
    y = midy+ymidlength
    return x, y
def position_of_base_polar(nx,ny,arm_length):
    xx, yy = position_of_base_cartesian(nx,ny,arm_length)
    xx, yy = cartesian_to_polar(xx,yy)
    return xx, yy
def angle_th1_phi(nx,ny,arm_length):
    temp, theta1 = position_of_base_polar(nx,ny,arm_length)
    temp, vec_theta = cartesian_to_polar(nx,ny)
    phi = vec_theta + (vec_theta-theta1)
    return theta1, phi
def angle_th1_th2(nx,ny,arm_length):
    th1, phi = angle_th1_phi(nx,ny,arm_length)
    th2 = phi - th1
    return th1, th2

def base_angle_to_cartesian(th1, arm_length):
    x1 = arm_length * np.cos(th1)
    y1 = arm_length * np.sin(th1)
    return x1, y1
def angle_to_cartesian_of_vector(th1, th2, arm_length):
    phi = th1 + th2
    x1, y1 = base_angle_to_cartesian(th1, arm_length)
    x2 = arm_length * np.cos(phi)
    y2 = arm_length * np.sin(phi)
    x = x1 + x2
    y = y1 + y2
    return x, y

# defines theta2 range with x bounds. Needs y bound while looping
# y bound is too complex to code as a general solution
def theta2_range(theta1,x_min,x_max,arm_length):
    x1, y1 = base_angle_to_cartesian(theta1, arm_length)
    phi_min = np.arccos((x_max-x1)/arm_length)
    phi_max = np.arccos((x_min - x1)/arm_length)
    th2_min = phi_min - theta1
    th2_max = phi_max - theta1
    return th2_min, th2_max

def xy_scaling(xpos, ypos, x_offset, y_offset, frame_width, frame_height, data_width):
    xpos = xpos - x_offset # sets origin to zero
    ypos = ypos - y_offset # for scanning the origin file array
    xpos = int(xpos * (data_width/frame_width)) #scaled and rounded up for array value extraction
    ypos = int(ypos * (data_width/frame_height))
    return xpos, ypos




def send_data(xpulse, ypulse, zangle, delay):
    xpulse = int(xpulse)
    ypulse = int(ypulse)
    print(xpulse, ypulse)
    x1 = int(xpulse/100)   # separate the integer
    x2 = xpulse - (x1*100)  # to make it fit in one byte
    ser.write(x1.to_bytes(1,'little'))  #send 1 byte piece to arduino
    ser.write(x2.to_bytes(1,'little'))
    y1 = int(ypulse/100)
    y2 = ypulse - (y1*100)
    ser.write(y1.to_bytes(1,'little'))
    ser.write(y2.to_bytes(1,'little'))
    time.sleep(delay)  # wait for arm to move to position
    ser.write(zangle.to_bytes(1,'little'))
 

##################
## Fixed Params ##
##################
# length units are in mm
# degree are in radian
arm_length = 85
frame_height = 90
frame_width = 90
y_offset = 20
x_offset = 30
step_size_radial = deg_rad(0.4)
step_size = deg_rad(0.3) # defines the accuracy as well
pixel_per_line = 300      # self explainatory

print('Calculating fixed parameters')
# direct dependent parameters
# corners, th1_start, th1_end, x_min, x_max
#
corners = [] # counter_clockwise from frame origin
corners.append([x_offset,y_offset])
corners.append([x_offset+frame_width,y_offset])
corners.append([x_offset+frame_width,y_offset+frame_height])
corners.append([x_offset,y_offset+frame_height])

print('Corners calculated')
th1_start = np.arcsin((y_offset-arm_length)/arm_length)
print('theta1 start calculated')
xlbase, ylbase = position_of_base_cartesian(corners[3][0],corners[3][1],arm_length)
xrbase, yrbase = position_of_base_cartesian(corners[2][0],corners[2][1],arm_length)
if ylbase >= yrbase:
    temp, th1_stop = position_of_base_polar(corners[3][0],corners[3][1],arm_length)
else:
    temp, th1_stop = position_of_base_polar(corners[2][0],corners[2][1],arm_length)
print('theta1 stop calculated')
x_min = x_offset
x_max = x_offset + frame_width
y_min = y_offset
y_max = y_offset + frame_height

print('All fixed parameters are fine.')

###############################
### image to txt conversion ###
###############################

from PIL import Image
import os


wide = pixel_per_line   # characters generated per line

ASCII_CHARS = [ 'x', 'x', 'x', 'x', 'x', 'x', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a']

'''
def scale_image(image, new_width=wide):
    """Resizes an image preserving the aspect ratio.
    """
    (original_width, original_height) = image.size
    aspect_ratio = original_height/float(original_width)
    new_height = int(aspect_ratio * new_width)

    new_image = image.resize((new_width, new_height))
    return new_image'''

def scale_image(image, new_width=wide):
    """Squares images
    """
    new_height = new_width

    new_image = image.resize((new_width, new_height))
    return new_image

def convert_to_grayscale(image):
    return image.convert('L')

def map_pixels_to_ascii_chars(image, range_width=25):
    """Maps each pixel to an ascii char based on the range
    in which it lies.

    0-255 is divided into 11 ranges of 25 pixels each.
    """

    pixels_in_image = list(image.getdata())
    pixels_to_chars = [ASCII_CHARS[pixel_value//range_width] for pixel_value in
            pixels_in_image]

    return "".join(pixels_to_chars)

def convert_image_to_ascii(image, new_width=wide):
    image = scale_image(image)
    image = convert_to_grayscale(image)

    pixels_to_chars = map_pixels_to_ascii_chars(image)
    len_pixels_to_chars = len(pixels_to_chars)

    image_ascii = [pixels_to_chars[index: index + new_width] for index in
            range(0, len_pixels_to_chars, new_width)]

    return "\n".join(image_ascii)

def handle_image_conversion(image_filepath):
    image = None
    try:
        image = Image.open(image_filepath)
    except Exception as e:
        print("Unable to open image file {image_filepath}.".format(image_filepath=image_filepath))
        print(e)
        return

    image_ascii = convert_image_to_ascii(image)
    f = open(os.path.splitext(image_filepath)[0]+'.txt','w')
    f.write(image_ascii)
    f.close()

import sys
print(' ___________________________ ')
print('|                           |')
print('|      Image to Text        |')
print('|         (.jpeg)           |')
print('|___________________________|')


fname = input('\nEnter file name (without the extention): ')
ffname = fname+'.jpeg'


image_file_path = sys.argv[0]
handle_image_conversion(ffname)

print('\nFile converted...\n')

#########################
###    File reader    ###
#########################


data = open(fname+'.txt','r+')
print('Data open')

##############################
## store .txt data in array ##
##############################

data_width = pixel_per_line # actual number i.e not in index
flag = []
items = data.read()
for iy in range(data_width):
    rowt = []
    for ix in range(data_width):
        value = items[((iy)*(data_width+1))+ix]
        rowt.append(value)
    flag.append(rowt)
print('Data extracted')
data.close()
os.remove(fname+'.txt')


####################
### printing ########
####################

print(' ___________________________ ')
print('|                           |')
print('|        Printing...        |')
print('|___________________________|')
# the arm scans everywhere within its range
# when the position is within the printing frame... else, it skips to next position
# it scans for a dot in the flag[][] array
# when it sees a key, it dots
# otherwise, it just moves there
# in the end of each line, it offsets the arm shortly for more accuracy in the next line

th1 = th1_start
th1x = 0
th2x = 0
while th1 <= th1_stop:
    newline = 1
    th2_start, th2_stop = theta2_range(th1, x_min, x_max, arm_length)
    th2 = th2_stop
    while th2 >= th2_start:
        isdot = 0 # key for dot or not
        xpos, ypos = angle_to_cartesian_of_vector(th1, th2, arm_length)
        if (xpos > x_min and xpos < x_max) and (ypos > y_min and ypos < y_max): #in range of frame check
            scanx, scany = xy_scaling(xpos,ypos,x_offset,y_offset,frame_width,frame_height,data_width)
            if (flag[scany][data_width-1-scanx]=='x'): # in range and is a dot
                isdot = 1
            else:
                isdot = 0
        th1x = rad_deg(th1)
        th2x = rad_deg(th2)
        f1, f2 = deg_freq(th1x, th2x)

        if newline: #newline makes a dot - for even line separation
            newline = 0
            send_data(f1, f2, 90, 0.5) # move it and dot it
            time.sleep(0.1)
            send_data(f1, f2, 100, 0) # lift it
            time.sleep(0.13)

        if isdot:
            send_data(f1, f2, 90, 0.01) # move it and dot it
            time.sleep(0.1)
            send_data(f1, f2, 100, 0) # lift it
            time.sleep(0.13)
        else:
            send_data(f1, f2, 100, 0)
        th2 = th2 - step_size
    
    time.sleep(0.1)
    print('New Radial')
    send_data(f1+100,f1,100,0) # shift slightly for larger arm movement
    time.sleep(0.2)             # because it cannot shift too small degree
    th1 = th1 + step_size_radial #end of line update

    
print(' ___________________________ ')
print('|                           |')
print('|      Print Complete.      |')
print('|___________________________|')


input('\nPress enter to exit')