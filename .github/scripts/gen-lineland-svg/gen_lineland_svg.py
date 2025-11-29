import os
import base64
from io import BytesIO
from PIL import Image

def png_to_base64(filename):
    """Loads local PNG and converts to base64 string for embedding."""
    print(f"Attempting to load local image: {filename}...")
    if not os.path.exists(filename):
        raise FileNotFoundError(f"❌ Error: The file '{filename}' was not found.")

    try:
        img = Image.open(filename).convert("RGBA")
        # Ensure it's 150x150 for this specific SVG setup
        if img.size != (150, 150):
             print(f"⚠️ Resizing input from {img.size} to (150, 150)")
             img = img.resize((150, 150), Image.Resampling.LANCZOS)

        buff = BytesIO()
        img.save(buff, format="PNG")
        img_str = base64.b64encode(buff.getvalue()).decode("utf-8")
        print("✅ Image loaded and encoded successfully.")
        return f"data:image/png;base64,{img_str}"
    except Exception as e:
        raise Exception(f"❌ Failed to process image. Error: {e}")

def main():
    # --- CONFIGURATION ---
    input_filename = "lineland-emoji.png"
    output_filename = "animated-lineland-emoji.svg"

    # Canvas size
    Size = 150
    # Animation duration in seconds
    Duration = "5s"

    # THE CRITICAL SETTING: Which Y row ends up being visible?
    # Range: 0 (top row) to 149 (bottom row).
    # Try changing this! E.g., 75 is the middle, 100 is near the mouth.
    TARGET_Y = 65
    # ---------------------

    try:
        img_b64 = png_to_base64(input_filename)
    except Exception as e:
        print(e)
        return

    # We use an f-string to build the SVG content.
    # We use SVG SMIL (<animate>) for smoother attribute animation than CSS.
    svg_content = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {Size} {Size}" width="{Size}" height="{Size}">
  <defs>
    <mask id="squeeze-mask">
      <rect x="0" width="{Size}" fill="white">

        <animate
            attributeName="height"
            values="{Size}; {Size}; 1; 1; {Size}"
            keyTimes="0; 0.2; 0.5; 0.975; 1"
            dur="{Duration}"
            repeatCount="indefinite" />

        <animate
            attributeName="y"
            values="0; 0; {TARGET_Y}; {TARGET_Y}; 0"
            keyTimes="0; 0.2; 0.5; 0.975; 1"
            dur="{Duration}"
            repeatCount="indefinite" />

      </rect>
    </mask>
  </defs>

  <image
      href="{img_b64}"
      x="0" y="0"
      width="{Size}" height="{Size}"
      mask="url(#squeeze-mask)"
  />

  </svg>"""

    with open(output_filename, "w") as f:
        f.write(svg_content)

    print(f"✅ Generated masked SVG: {output_filename}")
    print(f"ℹ️ The final visible line will be at Y position: {TARGET_Y}")

if __name__ == "__main__":
    main()
