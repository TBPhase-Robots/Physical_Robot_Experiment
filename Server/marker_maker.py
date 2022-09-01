"""hello Github"""

import numpy as np
# import cv2, PIL
from cv2 import ROTATE_180, aruco
import cv2
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

# Outputs a binary representation of an aruco marker
def marker_to_int(marker_id):
    # Creates the marker
    img = aruco.drawMarker(aruco_dict, marker_id, 8)

    # Rotates the marker, to compensate for the stack being rotated on the robots
    cv2.rotate(img, ROTATE_180, img)

    output = 0
    shift = 0

    # Loops through each pixel
    # Ignores the outside of the marker, as it is always black
    for line in img[1:-1]:
        for pixel in line[1:-1]:
            
            # Sets the current bit of the output to 1 if the marker pixel is white 
            if pixel == 255:
                output |= (1 << shift)

            # Moves to the next bit of the output
            shift += 1

    return output

def main():
    # The id of the marker to display
    marker_id = 103

    # Creates a matplotlib plot
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # Draws the marker
    img = aruco.drawMarker(aruco_dict, marker_id, 700)

    # Displays the marker on the plot
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

    # Prints the marker's binary representation
    print(f'the binary representation of marker {marker_id} is {marker_to_int(marker_id)}')

    # Outputs the marker to PDF
    plt.savefig("marker.pdf")

    # Shows the marker image
    plt.show()

if __name__ == "__main__":
    main()

