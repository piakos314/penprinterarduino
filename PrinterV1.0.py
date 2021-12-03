import numpy as np
import time
import serial

##################
## Serial connection initiation
#################
ser = serial.Serial('COM3',9600) #start serial com
xtemp = 75
ytemp = 55
ztemp = 100
for i in range(5):  # this is needed because it serial com ignores first 2 datas
    ser.write(xtemp.to_bytes(1,'little'))
    ser.write(ytemp.to_bytes(1,'little'))
    ser.write(ztemp.to_bytes(1,'little'))
    time.sleep(0.1)


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
    a1 =int(((a1+90)*(158/180))+11)     # attachment correction + servo error correction (range 11 - 169)
    a2 =int(((180-a2)*(154/180))+14)    # attachment correction + servo error correction (range 14 - 168)
    return a1, a2

def position(nx,ny,arm_length): # any position can give equilateral triangle in a circle of radius arm length
    # first position is calculated first
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


vr = 85.0  # defines the arm length,

############################
## initial corner check move
############################

x=[45,45,45,0,0,90,90,45]
y=[45,45,45,0,90,90,0,45]

for i in range(8):
    x[i] = x[i]+30
    y[i] = y[i]+10

for i in range(8):
    a1, a2 = (position(x[i],y[i],vr))
    ser.write(a1.to_bytes(1,'little'))
    ser.write(a2.to_bytes(1,'little'))
    a3 = 100
    ser.write(a3.to_bytes(1,'little'))
    time.sleep(1)


#############################
### image to txt conversion #
#############################

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
print('|___________________________|')


fname = input('\nEnter file name (.jpeg): ')
ffname = fname+'.jpeg'


image_file_path = sys.argv[0]
handle_image_conversion(ffname)

print('\nFile converted...\n')


#########################
###    File reader    ###
#########################


data = open(fname+'.txt','r+')
print('Data open')
####################
# print sequence

def send_data(xangle, yangle, zangle, delay):
    ser.write(xangle.to_bytes(1,'little'))
    ser.write(yangle.to_bytes(1,'little'))
    time.sleep(delay)  # wait for arm to move to position
    ser.write(zangle.to_bytes(1,'little'))

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

####################
### Printing section
####################

print(' ___________________________ ')
print('|                           |')
print('|        Printing...        |')
print('|___________________________|')
for y_pos in range(data_width):
    for x_pos in range(data_width):
        if flag[y_pos][x_pos] == 'x':
            tx,ty = position((89-x_pos)+31,y_pos+11,vr)
            send_data(tx,ty,90,0.4) # move and imprint
            time.sleep(0.2)         # wait for imprint
            send_data(tx,ty,100,0) # lift
            time.sleep(0.1)

print(' ___________________________ ')
print('|                           |')
print('|      Print Complete.      |')
print('|___________________________|')

input('\nPress enter to exit')

data.close()

