#!/usr/bin/env python3
"""Generate the board image matching this tool's default detector settings."""

from pathlib import Path

import cv2
import numpy as np


SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_MM = 30.0
MARKER_LENGTH_MM = 15.0
DICTIONARY_NAME = 'DICT_5X5_100'
PIXELS_PER_MM = 10  # 254 dpi
MARGIN_MM = 15


def main() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, DICTIONARY_NAME)
    )
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM,
        MARKER_LENGTH_MM,
        dictionary,
    )
    board_width = round(SQUARES_X * SQUARE_LENGTH_MM * PIXELS_PER_MM)
    board_height = round(SQUARES_Y * SQUARE_LENGTH_MM * PIXELS_PER_MM)
    board_image = board.generateImage((board_width, board_height), marginSize=0)
    margin = round(MARGIN_MM * PIXELS_PER_MM)
    canvas = np.full(
        (board_height + 2 * margin, board_width + 2 * margin), 255, dtype=np.uint8
    )
    canvas[margin:margin + board_height, margin:margin + board_width] = board_image
    output = Path(__file__).resolve().parent / 'charuco_7x5_30mm_15mm.png'
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f'Failed to write {output}')
    print(output)
    print('Print at exactly 254 dpi / 100% scale; then physically measure square size.')


if __name__ == '__main__':
    main()
