"""hello Github"""

import numpy as np
# import cv2, PIL
from cv2 import ROTATE_180, aruco
import cv2
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

def marker_to_int(marker_num):
    img = aruco.drawMarker(aruco_dict, marker_num, 8)
    cv2.rotate(img, ROTATE_180, img)
    output = 0
    shift = 0

    for line in img[1:-1]:
        for pixel in line[1:-1]:
            
            if pixel == 255:
                output |= (1 << shift)
                print(output)
            shift += 1



    return output

def main():
    fig = plt.figure()
    nx = 1
    ny = 1
    for i in range(103, nx*ny+103):
        ax = fig.add_subplot(ny,nx, 1)
        img = aruco.drawMarker(aruco_dict,i, 700)
        plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
        ax.axis("off")

    print("marker_to_int")
    print(marker_to_int(3))

    

    plt.savefig("one_marker.pdf")
    plt.show()


if __name__ == "__main__":
    main()
