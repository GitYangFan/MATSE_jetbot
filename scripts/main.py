import cv2
from object_detection import object_detection  # import from root dictionary directly, all uebungen will combine a whole project


def main():
    # image_balls, list_balls, list_cubes = object_detection('../data/detection_ideal/all_mix_color.png', 3)
    image_detected, list_balls, list_cubes = object_detection('../data/detection_ideal/all_mix_color.png', 3)   # be care of the name of variables to avoid misunderstanding
    # image_cubes, list_cubes = object_detection_cubes('pictures/50cm.jpg', 3)
    for ball in list_balls:
        # print(ball.color)
        # print(ball.radius)
        # print(ball.center_point)
        print('detected ball:', ball.color, ball.radius, ball.center_point) # print in a more clear way
    for cube in list_cubes:
        # print(cube.color)
        # print(cube.side_length)
        # print(cube.center_point)
        print('detected cube:', cube.color, cube.side_length, cube.center_point) # print in a more clear way

    # add a resize processing before display
    scale = 0.5
    size_display = (int(scale*image_detected.shape[1]),int(scale*image_detected.shape[0]))
    image_detected_resized = cv2.resize(image_detected, size_display)
    cv2.imshow('image_detected', image_detected_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
