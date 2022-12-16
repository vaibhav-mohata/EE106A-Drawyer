#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

#NOTE: original file from lab 6. path: ...lab6/src/segmentation/src/segementation.py

#filepaths
this_file = os.getcwd()
IMG_DIR = this_file + '/img/'

#argument parsing for testing
# import argparse
# parser = argparse.ArgumentParser(description='Convert raw images to edge-detected image.')
# parser.add_argument('-f', '--filename', default='face', help='Image file to conduct edge detection on.')
# args = parser.parse_args()
# if args.filename != "":
#     filename = args.filename
# else:
#     print("Unable to get file " + args.filename + " - Aborting!")
#     quit()

def read_image(img_name, grayscale=False):
    """ reads an image

    Parameters
    ----------
    img_name : str
        name of image
    grayscale : boolean
        true if image is in grayscale, false o/w
    
    Returns
    -------
    ndarray
        an array representing the image read (w/ extension)
    """

    if not grayscale:
        img = cv2.imread(img_name)
    else:
        img = cv2.imread(img_name, 0)

    return img

def write_image(img, img_name):
    """writes the image as a file
    
    Parameters
    ----------
    img : ndarray
        an array representing an image
    img_name : str
        name of file to write as (make sure to put extension)
    """

    cv2.imwrite(img_name, img)

def show_image(img_name, title='Fig', grayscale=False):
    """show the  as a matplotlib figure
    
    Parameters
    ----------
    img_name : str
        name of image
    tile : str
        title to give the figure shown
    grayscale : boolean
        true if image is in grayscale, false o/w
    """
    plt.ion()

    if not grayscale:
        plt.imshow(img_name)
        plt.title(title)
        plt.show()
    else:
        plt.imshow(img_name, cmap='gray')
        plt.title(title)
        plt.show()

def edge_detect_canny(gray_img, lower=0, upper=255):
    """perform Canny edge detection

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array

    Returns
    -------
    ndarray
        gray_img with edges outlined
    """
    edges = cv2.medianBlur(gray_img, 15)
    # edges = cv2.bilateralFilter(gray_img,7,75,75) #supposedly keeps edges sharp and removes noise, but this function is too finicky for me to figure out rn
    edges = cv2.Canny(edges, lower, upper, apertureSize = 3) #aperture size must be an odd number between 3 and 7
    return edges

def to_grayscale(rgb_img):
    return np.dot(rgb_img[... , :3] , [0.299 , 0.587, 0.114])

def show_edge_canny(img, lower, upper):
    edges = edge_detect_canny(img, lower, upper)
    show_image(edges, title='edge canny', grayscale=True)
    return edges

def main(filename):
    test_img = read_image(IMG_DIR + filename + ".jpg", grayscale=True)
    # test_img_color = read_image(IMG_DIR + filename)

    v = np.median(test_img)
    sigma = 0.40
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edge_img = show_edge_canny(test_img, lower, upper)

    save_img = False
    while save_img == False:
        draw_img = input("Do you want to draw this image? Enter 'yes' or 'no'. (If you want to exit the program, press 'x'.)")
        img_name = filename + "_edges.jpg"
        path_to_img = os.path.join(IMG_DIR , img_name)
        
        if draw_img == 'yes':
            write_image(edge_img, path_to_img)
            save_img = True
        elif draw_img == 'no':
            draw_img_final = False
            while draw_img_final == False:
                change_thresh = input("'w' to increase lower threshold. 's' to decrease lower threshold. 'a' to lower upper threshold. 'd' to increase upper threshold. 'y' to draw image. 'x' to exit.")
                if change_thresh == 's':
                    if lower == 5:
                        print("Lower threshold is at limit. Cannot decrease threshold.")
                        pass
                    else:
                        lower -= 10
                        edge_img = show_edge_canny(test_img, lower, upper)
                elif change_thresh == 'd':
                    if upper == 255:
                        print("Upper threshold is at limit. Cannot increase threshold.")
                        pass
                    else:
                        upper += 10
                        edge_img = show_edge_canny(test_img, lower, upper)
                elif change_thresh == 'w':
                    if lower + 10 == upper:
                        print("Lower threshold must be lower than upper limit. Cannot increase threshold.")
                    else:
                        lower += 10
                        edge_img = show_edge_canny(test_img, lower, upper)
                elif change_thresh == 'a':
                    if upper - 10 == lower:
                        print("Upper threshold must be higher than lower limit. Cannot decrease threshold.")
                    else:
                        upper -= 10
                        edge_img = show_edge_canny(test_img, lower, upper)         
                elif change_thresh == 'y':
                    write_image(edge_img, path_to_img)
                    draw_img_final = True
                    save_img = True
                elif change_thresh == 'x':
                    print("Exiting program...")
                    quit()
                else:
                    print("Please enter a valid response.")
                
        elif draw_img == 'x':
            print("Exiting program...")
            quit()
        else:
            print("Please enter a valid response.")
    return edge_img
