# SVG Animation Script

## What it does

This script (`convert_svg.py`) processes the handwritten notes SVG and adds a "writing" animation that makes it look like you're drawing the content in real-time.

## Features

1. **10-second animation**: The entire drawing animates over 10 seconds
2. **Top-to-bottom, left-to-right ordering**: Elements are animated in reading order:
   - First by Y-coordinate (top to bottom)
   - Then by X-coordinate (left to right within each horizontal level)
3. **Grid lines fade in**: Background grid lines (grey) fade in at low opacity at the start
4. **All strokes animated**:
   - 513 pen strokes (main handwriting)
   - 88 connector lines (blue tree-like lines)
   - 57 markers (colored arrows/shapes)
5. **Dark mode support**: Automatically adapts colors based on system preference

## How to use

1. Place your `Flatland XR.svg` file in this directory
2. Run the script:
   ```bash
   python3 convert_svg.py
   ```
3. Open `flatland-book-notes.svg` in a browser or use `test-animation.html` to preview

## Animation technique

The script uses CSS animations with `stroke-dasharray` and `stroke-dashoffset` to create a "drawing" effect where each stroke appears to be drawn from start to finish.

## Customization

You can adjust the animation duration by changing `ANIMATION_DURATION` in the script (currently 10 seconds).
