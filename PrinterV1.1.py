import numpy as np
import time
import serial

##################
## Serial connection initiation
#################

ser = serial.Serial('COM3',9600) #start serial com


# note that all distance are in mm
###########################
##     core functions    ##
###########################

def deg_rad(x):
    return x*(np.pi/180)

def rad_deg(x):
    return x*(180/np.pi)

def cartesian_to_polar(x,y):
    r = np.sqrt((x**2 + y**2))
    theta = np.arctan2(y,x)    # in radian
    return r, theta

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def second_position(ntheta, theta1):
    result = (ntheta + (ntheta-theta1))
    return result

def correction_for_device(a1,a2):
    a2 = a2 - a1
    a1 =int(((a1+90)*(1625/180))+645)     # attachment correction + servo error correction (range 645 - 2270 microsecond) 
    a2 =int(((180-a2)*(1592/180))+675)    # attachment correction + servo error correction (range 675 - 2267 microsecond) 
    return a1, a2 # output is pulse, send to arduino

def position(nx,ny,arm_length): # any position can give equilateral triangle in a circle of radius arm length
    # first position is calculated first
    nx = (nx*100000)/100000
    ny = (ny*100000)/100000
    arm_length = (arm_length*100000)/100000
    midpoint_nx = nx/2
    midpoint_ny = ny/2
    nr, ntheta = cartesian_to_polar(nx,ny)
    perp_length = np.sqrt(arm_length**2 - (nr/2)**2)
    perp_theta = ntheta-(np.pi/2)
    perp_xlength = perp_length*np.cos(perp_theta)
    perp_ylength = perp_length*np.sin(perp_theta)
    new_x = midpoint_nx + perp_xlength
    new_y = midpoint_ny + perp_ylength
    new_r, new_theta = cartesian_to_polar(new_x, new_y)
    ### passing values for the second position
    new_theta2 = second_position(ntheta, new_theta)
    ### correction
    new_theta = rad_deg(new_theta)
    new_theta2 = rad_deg(new_theta2)
    new_theta, new_theta2 = correction_for_device(new_theta, new_theta2)
    return new_theta, new_theta2

def send_data(xpulse, ypulse, zangle, delay):
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
    
vr = 85.000  # defines the arm length,

###############################
## initial corner check move ##
###############################

x=[45,45,45,0,0,90,90,45]
y=[45,45,45,0,90,90,0,45]

for i in range(8):
    x[i] = x[i]+31
    y[i] = y[i]+27

for i in range(8):
    a1, a2 = (position(x[i],y[i],vr))
    a3=100
    send_data(a1,a2,a3,0)
    time.sleep(1)


###############################
### image to txt conversion ###
###############################

from PIL import Image
import os

pixel_per_line = 90     # self explainatory
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

########################
### Printing section ###
########################

print(' ___________________________ ')
print('|                           |')
print('|        Printing...        |')
print('|___________________________|')

for y_pos in range(data_width):
    time.sleep(0.1)
    for x_pos in range(data_width):
        if flag[y_pos][x_pos] == 'x':
            tx,ty = position((89-x_pos)+27,y_pos+23,vr) # 31 and 11 are origin offset
            send_data(tx,ty,91,0.4) # move and imprint
            time.sleep(0.13)         # wait for imprint
            send_data(tx,ty,100,0) # lift
            time.sleep(0.13)


print(' ___________________________ ')
print('|                           |')
print('|      Print Complete.      |')
print('|___________________________|')

data.close()

input('\nPress enter to exit')



