import cv2
import numpy as np
from class_object import *

# The function loads the wanted image and processes it with the necessary steps for further analysis
def load_img(img_path, ksize):
    """
    :param img_path: path for the location, where the image is stored
    :param ksize: kernel size for the filters
    :return: three pictures: the original image, the filtered image and the image in hsv colors
    """
    img_original = cv2.imread(img_path)
    # img_original = cv2.resize(img_original, (0,0), fx= 0.25, fy= 0.25)      # -> resizes the image (not always necessary)
    img_original = cv2.resize(img_original, (0, 0), fx=1, fy=1)  # -> don't resize the image for the detector, otherwise the distance estimation will be wrong
    img = cv2.GaussianBlur(img_original, (ksize, ksize), cv2.BORDER_DEFAULT)         # -> blurs the image for better object detection
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((ksize, ksize), np.uint8))  # -> removes background noise
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # -> transforms the image into hsv colors
    return img_original, img, hsv

# The function detects objects (balls and cubes) in a picture
def object_detection (img_path, ksize):
    """
    :param img_path: path for the location, where the image is stored
    :param ksize: kernel size for the filters
    :return: -> the original image in which all detected objects and their center points are marked
             -> a list where all detected objects are saved, including their attributes
    """
    # load image with the load_img function
    img_original, img, hsv = load_img(img_path, ksize)
    # create a list in which all objects will later be saved
    balls = []
    cubes = []
    # go through every color in the color dictionary by creating masks
    for color,(lower,upper) in color_dic_hsv.items():

        lower_bound = np.array(lower)
        upper_bound = np.array(upper)
        # create mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # detect all balls in the picture
        balls, img_original = circle_detection(balls, color, mask, img_original)
        # detect all cubes in the picture
        cubes, img_original = cube_detection(cubes, color, mask, img_original)

    return img_original, balls, cubes

# The function detects all balls in a given image
def circle_detection(balls, color, mask, img_original):
    """
    :param balls: list of all detected balls
    :param color: the color of the object
    :param mask: mask of the image
    :param img_original: the original image
    :return: list with every detected balls object; the original image with an outline of every detected ball
    """
    # find circles
    # adjusted parameter based on original size of image instead of resized img
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.6, minDist=100, param1=300, param2=30, minRadius=10,
                               maxRadius=200)

    if circles is not None:
        # round values
        circles = np.uint16(np.around(circles))

        # iterate over every circle that was found
        for circle in circles[0, :]:
            # define center point and radius
            x, y, radius = circle
            # outline circle and mark center point
            cv2.circle(img_original, (x, y), radius, (0, 255, 0), 2)
            cv2.circle(img_original, (x, y), 2, (0, 0, 255), 2)
            # append balls list with a personalized object
            balls.append(Ball((int(x), int(y)), radius, color))

    return balls, img_original

# The function detects all cubes in a given image
def cube_detection(cubes, color, mask, img_original):
    """
    :param cubes: list of all detected cubes
    :param color: the color of the object
    :param mask: mask of the image
    :param img_original: the original image
    :return: list with every detected cube object; the original image with an outline of every detected cube
    """
    # find all contours in the image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        # calculate the corner points of the found objects
        approx = cv2.approxPolyDP(contour, epsilon, True)

        corner_points = len(approx)

        # create an empty list to save the calculated distances
        distances = []
        if corner_points == 6 or corner_points == 4:
            # calculate every distance between found corner points
            for i in range(corner_points):
                x1, y1 = approx[i][0]
                x2, y2 = approx[(i + 1) % len(approx)][0]
                distances.append(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
            # the minimal distance is the side length
            side_length = min(distances)

            # find the center point by calculating the smallest possible
            # circle around the cube
            (x, y), radius = cv2.minEnclosingCircle(contour)

            # outline cube and mark center point, append cube list
            cv2.drawContours(img_original, [approx], -1, (0, 255, 0), 2)
            cv2.circle(img_original, (int(x), int(y)), 2, (0, 0, 255), 2)
            cubes.append(Cube((int(x), int(y)), int(side_length), color))

    return cubes, img_original


