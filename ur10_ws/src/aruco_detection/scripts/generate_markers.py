#!/usr/bin/env python3
"""
Generate ArUco markers for printing.
Creates marker images for DICT_4X4_50 dictionary.
"""

import cv2
import numpy as np
import os


def generate_markers(output_dir='aruco_markers', num_markers=20, marker_size_px=200):
    """Generate ArUco marker images."""
    os.makedirs(output_dir, exist_ok=True)

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    print(f"Generating {num_markers} ArUco markers (DICT_4X4_50)...")
    print(f"Output directory: {output_dir}/")
    print(f"Marker size: {marker_size_px}x{marker_size_px} pixels")

    for marker_id in range(num_markers):
        # Generate marker
        marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

        # Add white border (20% on each side)
        border = int(marker_size_px * 0.2)
        marker_with_border = cv2.copyMakeBorder(
            marker_image,
            border, border, border, border,
            cv2.BORDER_CONSTANT,
            value=255
        )

        # Add text label below
        label_height = 50
        full_image = np.ones((marker_with_border.shape[0] + label_height, marker_with_border.shape[1]), dtype=np.uint8) * 255
        full_image[:marker_with_border.shape[0], :] = marker_with_border

        # Add ID text
        text = f"ID: {marker_id}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 1, 2)[0]
        text_x = (full_image.shape[1] - text_size[0]) // 2
        text_y = marker_with_border.shape[0] + 35
        cv2.putText(full_image, text, (text_x, text_y), font, 1, 0, 2)

        # Save
        filename = f"{output_dir}/aruco_marker_{marker_id:03d}.png"
        cv2.imwrite(filename, full_image)
        print(f"  Generated: {filename}")

    print(f"\nGenerated {num_markers} markers")
    print(f"\nPRINT INSTRUCTIONS:")
    print(f"  1. Print markers at exactly 5cm x 5cm (inner black square)")
    print(f"  2. Use thick paper or cardboard for rigidity")
    print(f"  3. Ensure white border is visible")
    print(f"  4. Avoid glossy paper (causes glare)")
    print(f"\nIf using different size, update launch parameter:")
    print(f"  marker_size:=0.05  # Change to your size in meters")


def generate_marker_sheet(output_file='aruco_sheet.png', markers_per_row=4, num_markers=16):
    """Generate a single sheet with multiple markers."""
    marker_size_px = 200
    border_px = 20
    spacing_px = 40

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    # Calculate sheet size
    rows = (num_markers + markers_per_row - 1) // markers_per_row
    sheet_width = markers_per_row * (marker_size_px + 2*border_px + spacing_px) + spacing_px
    sheet_height = rows * (marker_size_px + 2*border_px + spacing_px + 50) + spacing_px

    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255

    print(f"Generating marker sheet with {num_markers} markers...")
    print(f"Layout: {rows} rows x {markers_per_row} columns")

    for i in range(num_markers):
        row = i // markers_per_row
        col = i % markers_per_row

        # Generate marker
        marker_image = cv2.aruco.drawMarker(aruco_dict, i, marker_size_px)
        marker_with_border = cv2.copyMakeBorder(
            marker_image,
            border_px, border_px, border_px, border_px,
            cv2.BORDER_CONSTANT,
            value=255
        )

        # Position on sheet
        x_pos = spacing_px + col * (marker_size_px + 2*border_px + spacing_px)
        y_pos = spacing_px + row * (marker_size_px + 2*border_px + spacing_px + 50)

        # Place marker
        sheet[y_pos:y_pos+marker_with_border.shape[0], x_pos:x_pos+marker_with_border.shape[1]] = marker_with_border

        # Add label
        text = f"ID: {i}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 0.7, 2)[0]
        text_x = x_pos + (marker_with_border.shape[1] - text_size[0]) // 2
        text_y = y_pos + marker_with_border.shape[0] + 30
        cv2.putText(sheet, text, (text_x, text_y), font, 0.7, 0, 2)

    cv2.imwrite(output_file, sheet)
    print(f"Saved: {output_file}")
    print(f"\nPrint this sheet on A4 paper and cut out individual markers")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Generate ArUco markers')
    parser.add_argument('--num', type=int, default=20, help='Number of markers to generate')
    parser.add_argument('--size', type=int, default=200, help='Marker size in pixels')
    parser.add_argument('--output', type=str, default='aruco_markers', help='Output directory')
    parser.add_argument('--sheet', action='store_true', help='Generate single sheet instead of individual files')

    args = parser.parse_args()

    if args.sheet:
        generate_marker_sheet('aruco_sheet.png', markers_per_row=4, num_markers=args.num)
    else:
        generate_markers(args.output, args.num, args.size)
