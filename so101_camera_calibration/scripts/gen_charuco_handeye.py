# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate a ChArUco handeye target PNG sized for A4 at 300 DPI.

Print actual size (no scale-to-fit). Verify with a ruler: one checker = 15 mm.
"""
import cv2
import cv2.aruco as aruco
import numpy as np

SQUARES_X = 4
SQUARES_Y = 5
SQUARE_MM = 15.0
MARKER_MM = 11.0
DICT = aruco.DICT_4X4_50
DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
MARGIN_MM = 15.0  # white margin around the board for cutting

mm2px = DPI / 25.4
board_w_px = int(SQUARES_X * SQUARE_MM * mm2px)
board_h_px = int(SQUARES_Y * SQUARE_MM * mm2px)

aruco_dict = aruco.getPredefinedDictionary(DICT)
if hasattr(aruco, "CharucoBoard_create"):
    board = aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_MM / 1000.0, MARKER_MM / 1000.0, aruco_dict
    )
    board_img = board.draw((board_w_px, board_h_px), 0, 1)
else:
    board = aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_MM / 1000.0, MARKER_MM / 1000.0, aruco_dict
    )
    board_img = board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)

page_w_px = int(A4_W_MM * mm2px)
page_h_px = int(A4_H_MM * mm2px)
page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)
off_x = (page_w_px - board_w_px) // 2
off_y = (page_h_px - board_h_px) // 2
page[off_y:off_y + board_h_px, off_x:off_x + board_w_px] = board_img

# Crop marks just outside each board corner for clean cutting.
m = int(MARGIN_MM * mm2px / 3)
for (x, y) in [(off_x, off_y), (off_x + board_w_px, off_y),
               (off_x, off_y + board_h_px), (off_x + board_w_px, off_y + board_h_px)]:
    cv2.line(page, (x - m, y), (x + m, y), 0, 2)
    cv2.line(page, (x, y - m), (x, y + m), 0, 2)

out_png = "/tmp/charuco_handeye_A4.png"
cv2.imwrite(out_png, page)

# PDF with embedded A4 page size — print dialog will show "Actual size" option.
out_pdf = "/tmp/charuco_handeye_A4.pdf"
try:
    from PIL import Image
    Image.fromarray(page).save(
        out_pdf, "PDF", resolution=float(DPI)
    )
    print(f"Saved {out_png} and {out_pdf}")
except ImportError:
    print(f"Saved {out_png}  (install Pillow for PDF: pip install Pillow)")
print(f"board={SQUARES_X}x{SQUARES_Y} sq={SQUARE_MM}mm mk={MARKER_MM}mm dict=DICT_4X4_50")
