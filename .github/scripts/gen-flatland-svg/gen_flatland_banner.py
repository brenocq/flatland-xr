import os
import base64
from io import BytesIO
from PIL import Image, ImageDraw, ImageFont
import cairosvg

def png_to_base64(filename, target_size):
    """Loads local PNG and converts to base64 string for embedding."""
    print(f"Attempting to load local image: {filename}...")
    if not os.path.exists(filename):
        raise FileNotFoundError(f"❌ Error: The file '{filename}' was not found.")

    try:
        img = Image.open(filename).convert("RGBA")
        # Resize to the target size for high resolution
        if img.size != (target_size, target_size):
             print(f"⚠️ Resizing input from {img.size} to ({target_size}, {target_size})")
             img = img.resize((target_size, target_size), Image.Resampling.LANCZOS)

        buff = BytesIO()
        img.save(buff, format="PNG")
        img_str = base64.b64encode(buff.getvalue()).decode("utf-8")
        print("✅ Image loaded and encoded successfully.")
        return f"data:image/png;base64,{img_str}"
    except Exception as e:
        raise Exception(f"❌ Failed to process image. Error: {e}")

def generate_emoji_at_stage(img_b64, size, stage_percent):
    """
    Generate SVG for emoji at a specific stage.
    stage_percent: 100 (full), 66, 33, 0 (just a line)
    Returns the SVG content for that emoji state.
    """
    if stage_percent == 100:
        # Full emoji - no mask
        return f'<image href="{img_b64}" x="0" y="0" width="{size}" height="{size}" />'

    elif stage_percent == 0:
        # Just a line at Y=65 (the critical line)
        target_y = 65
        return f'''<image href="{img_b64}" x="0" y="0" width="{size}" height="{size}" mask="url(#squeeze-mask-{stage_percent})" />
  <defs>
    <mask id="squeeze-mask-{stage_percent}">
      <rect x="0" y="{target_y}" width="{size}" height="1" fill="white" />
    </mask>
  </defs>'''

    else:
        # Partial visibility (66% or 33%)
        target_y = 65
        # Calculate visible height based on percentage
        visible_height = int(size * stage_percent / 100)
        # Center around target_y
        y_start = target_y - (visible_height // 2)

        return f'''<image href="{img_b64}" x="0" y="0" width="{size}" height="{size}" mask="url(#squeeze-mask-{stage_percent})" />
  <defs>
    <mask id="squeeze-mask-{stage_percent}">
      <rect x="0" y="{y_start}" width="{size}" height="{visible_height}" fill="white" />
    </mask>
  </defs>'''

def calculate_target_y(emoji_size):
    """Calculate the target Y position scaled to emoji size."""
    # Original Y=65 was for 150px emoji
    # Scale proportionally
    return int(65 * emoji_size / 150)

def main():
    # --- CONFIGURATION ---
    input_filename = "flatland-emoji.png"
    output_svg_filename = "github-banner.svg"
    output_png_filename = "github-banner.png"

    # Banner dimensions
    banner_width = 1280
    banner_height = 640

    # GitHub safe zone (recommended 40pt border)
    safe_border = 40

    # Title
    title_text = "Flatland XR"

    # Calculate emoji size - 5 emojis with nice padding
    # Add safe border to padding
    padding = 60 + safe_border
    title_emoji_gap = 30  # Gap between title and emojis

    available_width = banner_width - (2 * padding)

    # Calculate emoji size to fit 5 emojis with spacing
    num_emojis = 5
    spacing_between = 30  # Space between emojis
    total_spacing = spacing_between * (num_emojis - 1)
    emoji_size = int((available_width - total_spacing) / num_emojis)

    print(f"ℹ️ Safe border: {safe_border}px (GitHub recommendation)")
    print(f"ℹ️ Total padding (including safe border): {padding}px")
    print(f"ℹ️ Calculated emoji size: {emoji_size}x{emoji_size}px")

    # Catppuccin Mocha background color
    bg_color = "#1e1e2e"

    # Catppuccin Mocha text color
    text_color = "#cdd6f4"  # Catppuccin Mocha "text" color

    # Stages to show: 100%, 75%, 50%, 25%, 0%
    stages = [100, 75, 50, 25, 0]
    # ---------------------

    try:
        img_b64 = png_to_base64(input_filename, emoji_size)
    except Exception as e:
        print(e)
        return

    # Calculate vertical layout
    # Available vertical space after top and bottom padding
    available_height = banner_height - (2 * padding)

    # Title font size (make it prominent)
    title_font_size = 96

    # Calculate positions
    title_y = padding + title_font_size  # Baseline for title text
    emoji_y = title_y + title_emoji_gap + emoji_size / 2  # Top of emojis

    # Build the SVG content
    svg_content = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {banner_width} {banner_height}" width="{banner_width}" height="{banner_height}">
  <!-- Catppuccin Mocha Background -->
  <rect width="{banner_width}" height="{banner_height}" fill="{bg_color}" />

  <!-- Title -->
  <text x="{banner_width / 2}" y="{title_y}"
        font-family="'Inter', 'Segoe UI', system-ui, sans-serif"
        font-size="{title_font_size}"
        font-weight="700"
        fill="{text_color}"
        text-anchor="middle"
        letter-spacing="-1">
    {title_text}
  </text>

  <!-- Define all masks in defs section -->
  <defs>'''

    # Add mask definitions for partial stages
    target_y = calculate_target_y(emoji_size)

    for stage in stages:
        if stage == 0:
            svg_content += f'''
    <mask id="squeeze-mask-{stage}">
      <rect x="0" y="{target_y}" width="{emoji_size}" height="1" fill="white" />
    </mask>'''
        elif stage != 100:
            visible_height = int(emoji_size * stage / 100)
            y_start = target_y - (visible_height // 2)
            svg_content += f'''
    <mask id="squeeze-mask-{stage}">
      <rect x="0" y="{y_start}" width="{emoji_size}" height="{visible_height}" fill="white" />
    </mask>'''

    svg_content += '''
  </defs>

  <!-- Emojis at different stages -->
'''

    # Position emojis - center them horizontally and vertically
    y_position = emoji_y

    # Add each emoji
    for i, stage in enumerate(stages):
        x_position = padding + i * (emoji_size + spacing_between)

        svg_content += f'''  <g transform="translate({x_position}, {y_position})">
'''

        if stage == 100:
            # Full emoji - no mask
            svg_content += f'''    <image href="{img_b64}" x="0" y="0" width="{emoji_size}" height="{emoji_size}" />
'''
        else:
            # Masked emoji
            svg_content += f'''    <image href="{img_b64}" x="0" y="0" width="{emoji_size}" height="{emoji_size}" mask="url(#squeeze-mask-{stage})" />
'''

        svg_content += '''  </g>
'''

    svg_content += '''</svg>'''

    # Save SVG first
    with open(output_svg_filename, "w") as f:
        f.write(svg_content)

    print(f"✅ Generated SVG: {output_svg_filename}")

    # Convert SVG to PNG using cairosvg
    try:
        print(f"🔄 Converting SVG to PNG...")
        cairosvg.svg2png(
            bytestring=svg_content.encode('utf-8'),
            write_to=output_png_filename,
            output_width=banner_width,
            output_height=banner_height
        )
        print(f"✅ Generated PNG banner: {output_png_filename}")
    except Exception as e:
        print(f"⚠️ Could not generate PNG (cairosvg not available): {e}")
        print(f"💡 Install with: pip install cairosvg")

    print(f"ℹ️ Banner dimensions: {banner_width}x{banner_height}px")
    print(f"ℹ️ Safe border: {safe_border}px")
    print(f"ℹ️ Emoji stages: {stages}")
    print(f"ℹ️ Background: Catppuccin Mocha ({bg_color})")

if __name__ == "__main__":
    main()
