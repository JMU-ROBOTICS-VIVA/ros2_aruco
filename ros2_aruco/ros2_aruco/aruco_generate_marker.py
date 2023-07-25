"""
Script for generating Aruco marker images.

Author: Nathan Sprague
Version: 10/26/2020
"""

import argparse
import cv2
import numpy as np


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter):
    """ Trick to allow both defaults and nice formatting in the help. """
    pass


def main():
    parser = argparse.ArgumentParser(formatter_class=CustomFormatter,
                                     description="Generate a .png image of a specified maker.")
    parser.add_argument('--id', default=1, type=int,
                        help='Marker id to generate')
    parser.add_argument('--size', default=200, type=int,
                        help='Side length in pixels')
    dict_options = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
    option_str = ", ".join(dict_options)
    dict_help = "Dictionary to use. Valid options include: {}".format(option_str)
    parser.add_argument('--dictionary', default="DICT_5X5_250", type=str,
                        choices=dict_options,
                        help=dict_help, metavar='')
    args = parser.parse_args()

    dictionary_id = cv2.aruco.__getattribute__(args.dictionary)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    image = np.zeros((args.size, args.size), dtype=np.uint8)
    image = dictionary.generateImageMarker(args.id, args.size, image, 1)
    cv2.imwrite("marker_{:04d}.png".format(args.id), image)


if __name__ == "__main__":
    main()
