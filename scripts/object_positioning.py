import math

# The function to calculate the focal length
def cal_focal(distance, h_pic, h_real):
# one distance is required to calculate the focal length once (calibration)
    """
    Parameters
    ------------------
    distance : the distance from the camera to the objekt in mm
    h_pic : the height of the object in pixel
    h_real : the height of the object in mm

    Returns
    ------------------
    returns the focal length of the used camera in pixel
    """
    return distance * (h_pic / h_real)

# The function calculates the distance between the camera and the object
def cal_distance(h_pic, h_real, focal):
    """
    Parameters
    ------------------
    h_pic : the height of the object in pixel
    h_real : the height of the object in mm
    focal : the focal length of the camera in pixel

    Returns
    ------------------
    returns the distance from the camera to the objekt in mm
    """
    return focal * (h_real / h_pic)

# The function calculates the horizontal and vertical angle between the object and the camera
def cal_angle(x_pic, y_pic, focal, camera_frame=[1080, 720]):
    # the coordinate original point has to be clarified! e.g. middle point or left-up point.
    """
    Parameters
    -------------------
    camera_frame: the image width and height, the default value is (1080x720)
    x_pic : the x coordinate of the object in pixel, originally left-up point
    y_pic : the y coordinate of the object in pixel, originally left-up point
    focal : the focal length of the camera in pixel

    Returns
    --------------------
    horizontal_angle : the deflection angle in the horizon plane of the camera coordinate, positive value means left
    vertical_angle : the deflection angle in the vertical plane of the camera coordinate, positive value means upon
    """
    err_x = camera_frame[0] / 2 - x_pic   # the error of pixel in the direction of x
    err_y = camera_frame[1] / 2 - y_pic  # the error of pixel in the direction of y
    horizontal_angle = math.atan(err_x/focal)
    vertical_angle = math.atan(err_y/focal)
    return horizontal_angle, vertical_angle

# The function calculates the position of the object in the world coordinate system
def cal_position(x_cam_world, y_cam_world, z_cam_world, angle_h, angle_v, d):
    """
    Parameters
    ----------------------
    x_cam_world: x coordinate of the camera in mm
    y_cam_world: y coordinate of the camera in mm
    z_cam_world: z coordinate of the camera in mm
    angle_h: horizontal angle between the camera and the object in degrees
    angle_v: vertical angle between the camera and the object in degrees
    d: distance from the camera to the object in mm

    Returns
    ----------------------
    returns the position of the object in mm
    """
    x_object_world = float(x_cam_world + d)
    y_object_world = y_cam_world + (math.tan(angle_h) * d)
    z_object_world = z_cam_world + (math.tan(angle_v) * d)

    return x_object_world, y_object_world, z_object_world

# open cv

def main():
    # calibration of the focal length with a known object:
    print('calibrating the camera focal length:')
    focal_cal = cal_focal(distance=20, h_pic=1080, h_real=6.5)
    print("Focal length: ", focal_cal)

    # calculate the distance of unknown object 1
    print('\ndetecting the object 1: ')
    distance = cal_distance(h_pic=760, h_real=6.5, focal=focal_cal)
    # angle_h, angle_v = cal_angle(distance, h_pic, h_real)
    angle_h, angle_v = cal_angle(x_pic=915, y_pic=1725, focal=focal_cal, camera_frame=[1872,4032])
    x_cam, y_cam, z_cam = 100, 150, 200
    x,y,z = cal_position(x_cam, y_cam, z_cam, angle_h, angle_v, distance)
    print("Distance: " , distance)
    print("Horizontal angle: ", angle_h)
    print("Vertical angle: " , angle_v)
    print("Position: " , x, y, z)

    # calculate the distance of unknown object 2
    print('\ndetecting the object 2: ')
    distance = cal_distance(h_pic=433, h_real=6.5, focal=focal_cal)
    # angle_h, angle_v = cal_angle(distance, h_pic, h_real)
    angle_h, angle_v = cal_angle(x_pic=1161, y_pic=1319, focal=focal_cal, camera_frame=[1872,4032])
    x_cam, y_cam, z_cam = 100, 150, 200
    x,y,z = cal_position(x_cam, y_cam, z_cam, angle_h, angle_v, distance)
    print("Distance: " , distance)
    print("Horizontal angle: ", angle_h)
    print("Vertical angle: " , angle_v)
    print("Position: " , x, y, z)

if __name__ == '__main__':
    main()


