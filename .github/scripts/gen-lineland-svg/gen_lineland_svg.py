import os
import requests
import base64
from io import BytesIO
from PIL import Image

def get_hd_emoji():
    """Downloads the 512x512 Sunglasses Emoji."""
    url = "https://raw.githubusercontent.com/googlefonts/noto-emoji/main/png/512/emoji_u1f60e.png"
    print(f"Downloading HD Emoji from {url}...")
    response = requests.get(url)
    if response.status_code == 200:
        return Image.open(BytesIO(response.content)).convert("RGBA")
    raise Exception(f"Failed to download emoji. Status: {response.status_code}")

def pil_to_base64(img):
    """Converts a PIL Image to a base64 string for SVG embedding."""
    buff = BytesIO()
    img.save(buff, format="PNG")
    img_str = base64.b64encode(buff.getvalue()).decode("utf-8")
    return f"data:image/png;base64,{img_str}"

def main():
    # 1. Setup
    raw_img = get_hd_emoji()

    # Crop and Center
    bbox = raw_img.getbbox()
    if bbox:
        raw_img = raw_img.crop(bbox)

    # Create the base image
    w = 150
    base_img = raw_img.resize((w, w), Image.Resampling.LANCZOS)

    # 2. Define the Animation Steps
    # We add a few duplicate 1s at the end to make it pause on the "Line" view
    target_heights = [150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 3, 1]

    frames_data = [] # Will hold the base64 strings

    print(f"Processing {len(target_heights)} frames...")

    for h in target_heights:
        # A. DOWNSCALE (Sensor Simulation - BOX filter averages light)
        sensor_view = base_img.resize((w, h), resample=Image.Resampling.BOX)

        # B. UPSCALE (Visualization - NEAREST keeps it pixelated)
        # We ensure minimum visibility of 5px so it doesn't vanish completely visually
        vis_h = max(h, 5)
        final_view = sensor_view.resize((w, vis_h), resample=Image.Resampling.NEAREST)

        # C. CANVAS (Paste onto transparent 200x200)
        final_frame = Image.new("RGBA", (w, w), (0, 0, 0, 0))
        y_offset = (w - vis_h) // 2
        final_frame.paste(final_view, (0, y_offset))

        # D. Convert to Base64
        frames_data.append(pil_to_base64(final_frame))

    # 3. Generate the SVG
    total_frames = len(frames_data)
    total_duration = 3.0    # Total loop time
    pause_duration = 2.0    # Time for the final frame
    anim_duration = total_duration - pause_duration # Time for the collapse (1.0s)

    # Calculate how long each "fast" frame lasts
    # We have (total_frames - 1) fast frames, and 1 slow frame
    fast_frame_count = total_frames - 1
    time_per_fast_frame = anim_duration / fast_frame_count

    # Calculate duty cycles (percentage of total time visible)
    pct_fast = (time_per_fast_frame / total_duration) * 100
    pct_slow = (pause_duration / total_duration) * 100

    svg_content = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {w}" width="{w}" height="{w}">',
        '<defs>',
        '<style>',

        # Class 1: Fast Frames (The collapse)
        f'@keyframes showFast {{',
        f'  0% {{ opacity: 1; }}',
        f'  {pct_fast:.2f}% {{ opacity: 1; }}',        # Visible duration
        f'  {pct_fast + 0.5:.2f}% {{ opacity: 0; }}',  # +0.5% Overlap to fix flicker
        f'  100% {{ opacity: 0; }}',
        f'}}',

        # Class 2: Slow Frame (The pause at the end)
        f'@keyframes showSlow {{',
        f'  0% {{ opacity: 1; }}',
        f'  {pct_slow:.2f}% {{ opacity: 1; }}',        # Visible duration
        f'  {pct_slow + 0.5:.2f}% {{ opacity: 0; }}',  # +0.5% Overlap
        f'  100% {{ opacity: 0; }}',
        f'}}',

        # Apply animations
        f'.fast-frame {{ animation: showFast {total_duration}s linear infinite; opacity: 0; }}',
        f'.slow-frame {{ animation: showSlow {total_duration}s linear infinite; opacity: 0; }}',

        '</style>',
        '</defs>'
    ]

    # Generate Image Tags
    for i, b64_data in enumerate(frames_data):
        is_last = (i == total_frames - 1)

        if not is_last:
            # Logic for Fast Frames
            delay = i * time_per_fast_frame
            css_class = "fast-frame"
        else:
            # Logic for the Final Frame
            # Starts exactly when the animation duration ends
            delay = anim_duration
            css_class = "slow-frame"

        img_tag = (
            f'<image href="{b64_data}" x="0" y="0" width="{w}" height="{w}" '
            f'class="{css_class}" style="animation-delay: {delay:.3f}s" />'
        )
        svg_content.append(img_tag)

    svg_content.append('</svg>')

    # 4. Save to File
    filename = "lineland-xr-emoji.svg"
    with open(filename, "w") as f:
        f.write("\n".join(svg_content))

    print(f"✅ Generated animated SVG: {filename}")

if __name__ == "__main__":
    main()
