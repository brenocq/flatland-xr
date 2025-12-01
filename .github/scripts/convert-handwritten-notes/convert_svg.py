import xml.etree.ElementTree as ET
import re
from collections import defaultdict

# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------
INPUT_FILENAME = "Flatland XR.svg"
OUTPUT_FILENAME = "flatland-book-notes.svg"
ANIMATION_DURATION = 10  # seconds

# The Color Dictionary
#
# Key = The color found in the SVG (Light Mode)
# Value = The color it should become in Dark Mode
COLOR_MAPPING = {
    # The Pen (Dark Grey -> Light Grey)
    "#50514f": "#E0E6ED",

    # The Grid/Background lines (Grey -> Darker Grey for contrast)
    "#93999b": "#93999b",

    # Text Color (Black -> White/Light Grey)
    "#1f2328": "#D1D7E0",

    # Markers (Red, Green, Blue)
    "#ed5d47": "#ed5d47",
    "#38ac54": "#38ac54",
    "#4378c3": "#4378c3"
}

def get_dark_color(light_color):
    """Helper to match case-insensitive hex codes."""
    for k, v in COLOR_MAPPING.items():
        if k.lower() == light_color.lower():
            return v
    return light_color # Return original if no mapping found

def extract_path_bounds(d_attr):
    """Extract bounding box from path d attribute."""
    coords = re.findall(r'[-+]?[0-9]*\.?[0-9]+', d_attr)
    if len(coords) < 2:
        return None

    xs = [float(coords[i]) for i in range(0, len(coords), 2)]
    ys = [float(coords[i]) for i in range(1, len(coords), 2)]

    return {
        'min_x': min(xs),
        'max_x': max(xs),
        'min_y': min(ys),
        'max_y': max(ys),
        'center_x': (min(xs) + max(xs)) / 2,
        'center_y': (min(ys) + max(ys)) / 2,
    }

def is_vertical_connector(bounds):
    """Check if a path is a vertical connector (tree-like line)."""
    if not bounds:
        return False
    height = bounds['max_y'] - bounds['min_y']
    width = bounds['max_x'] - bounds['min_x']
    # Vertical if height is much greater than width
    return height > width * 3

def is_horizontal_connector(bounds):
    """Check if a path is a horizontal connector."""
    if not bounds:
        return False
    height = bounds['max_y'] - bounds['min_y']
    width = bounds['max_x'] - bounds['min_x']
    # Horizontal if width is much greater than height
    return width > height * 3

def group_elements_by_hierarchy(elements_with_bounds):
    """
    Group elements into hierarchical levels based on Y position.
    Returns ordered list of elements for DFS-like animation.
    """
    # Sort primarily by Y (top to bottom), then by X (left to right)
    sorted_elements = sorted(elements_with_bounds,
                            key=lambda e: (e['bounds']['center_y'], e['bounds']['center_x']))

    # Group by approximate Y levels (within 30 units tolerance)
    Y_TOLERANCE = 30
    levels = []
    current_level = []
    current_y = None

    for elem in sorted_elements:
        y = elem['bounds']['center_y']

        if current_y is None or abs(y - current_y) < Y_TOLERANCE:
            current_level.append(elem)
            current_y = y if current_y is None else current_y
        else:
            if current_level:
                levels.append(sorted(current_level, key=lambda e: e['bounds']['center_x']))
            current_level = [elem]
            current_y = y

    if current_level:
        levels.append(sorted(current_level, key=lambda e: e['bounds']['center_x']))

    # Flatten levels for animation order
    ordered = []
    for level in levels:
        ordered.extend(level)

    return ordered

def process_svg():
    # 1. Setup XML parsing
    ET.register_namespace('', "http://www.w3.org/2000/svg")
    ET.register_namespace('xlink', "http://www.w3.org/1999/xlink")

    try:
        tree = ET.parse(INPUT_FILENAME)
        root = tree.getroot()
    except FileNotFoundError:
        print(f"Error: Could not find '{INPUT_FILENAME}'. Make sure the file is in the same folder.")
        return

    # We will store unique styles here to generate CSS later
    # Format: {'class_name': {'prop': 'stroke', 'light': '#abc', 'dark': '#xyz'}}
    styles_to_generate = {}

    # Collect path elements with their positions and types
    pen_strokes = []  # Main handwritten content
    connectors = []   # Connecting lines (blue, vertical/horizontal)
    grid_lines = []   # Background grid
    markers = []      # Colored markers/arrows

    CONNECTOR_COLOR = '#009ef1'
    GRID_COLOR = '#93999b'
    PEN_COLOR = '#50514f'
    MARKER_COLORS = ['#ed5d47', '#38ac54', '#4378c3']

    # 2. Walk through every element in the SVG
    for elem in root.iter():
        # Track and categorize path elements
        if elem.tag.endswith('path'):
            stroke = elem.attrib.get('stroke', '')
            d_attr = elem.attrib.get('d', '')
            bounds = extract_path_bounds(d_attr)

            if bounds:
                elem_data = {'elem': elem, 'bounds': bounds, 'stroke': stroke}

                if stroke == GRID_COLOR:
                    grid_lines.append(elem_data)
                elif stroke == CONNECTOR_COLOR:
                    connectors.append(elem_data)
                elif stroke == PEN_COLOR:
                    pen_strokes.append(elem_data)
                elif stroke in MARKER_COLORS:
                    markers.append(elem_data)

        # --- Handle Strokes ---
        if 'stroke' in elem.attrib:
            original_c = elem.attrib['stroke']
            if original_c.startswith('#'):
                # Create a class name based on the hex (e.g., s-50514f)
                clean_hex = original_c.replace('#', '').lower()
                class_name = f"s-{clean_hex}"

                # Add class to element
                current_classes = elem.attrib.get('class', '')
                if class_name not in current_classes:
                    elem.attrib['class'] = (current_classes + ' ' + class_name).strip()

                # Save style info for CSS generation
                styles_to_generate[class_name] = {
                    'prop': 'stroke',
                    'light': original_c,
                    'dark': get_dark_color(original_c)
                }

                # Remove the inline attribute so CSS can take over
                del elem.attrib['stroke']

        # --- Handle Fills ---
        if 'fill' in elem.attrib:
            original_c = elem.attrib['fill']
            if original_c.startswith('#'):
                clean_hex = original_c.replace('#', '').lower()
                class_name = f"f-{clean_hex}"

                current_classes = elem.attrib.get('class', '')
                if class_name not in current_classes:
                    elem.attrib['class'] = (current_classes + ' ' + class_name).strip()

                styles_to_generate[class_name] = {
                    'prop': 'fill',
                    'light': original_c,
                    'dark': get_dark_color(original_c)
                }

                del elem.attrib['fill']

    # 2.5. Order elements for animation
    print(f"Found {len(pen_strokes)} pen strokes, {len(connectors)} connectors, {len(markers)} markers, {len(grid_lines)} grid lines")

    # Combine ALL content (pen + markers + connectors + grid) for animation
    # Do NOT distinguish between layers - thread them all together
    # Order: top-to-bottom, left-to-right within each level
    animatable_elements = pen_strokes + connectors + markers + grid_lines
    ordered_elements = group_elements_by_hierarchy(animatable_elements)

    print(f"Ordered {len(ordered_elements)} elements for animation (all layers threaded together)")

    # 2.6. Add animation classes to ordered elements
    total_duration = ANIMATION_DURATION
    if ordered_elements:
        duration_per_element = total_duration / len(ordered_elements)

        for i, elem_data in enumerate(ordered_elements):
            elem = elem_data['elem']
            anim_class = f"anim-path-{i}"
            current_classes = elem.attrib.get('class', '')
            elem.attrib['class'] = (current_classes + ' ' + anim_class).strip()

            # Add pathLength attribute for consistent animation
            elem.attrib['pathLength'] = "1"

    # 3. Construct the CSS content
    css_content = []

    # A. Default Text Style (if any text elements exist)
    css_content.append("/* Base Text Style */")
    css_content.append("text { font-family: Arial, sans-serif; font-weight: bold; font-size: 20px; fill: #1F2328; opacity: 0; }")

    # B. Default Colors (Light Mode) - Applied to everyone
    css_content.append("\n/* Default (Light Mode) Colors */")
    for cls, data in styles_to_generate.items():
        css_content.append(f".{cls} {{ {data['prop']}: {data['light']}; }}")

    # C. Dark Mode Overrides - Applied only when prefer-color-scheme is dark
    css_content.append("\n/* Dark Mode Overrides */")
    css_content.append("@media (prefers-color-scheme: dark) {")

    # Override Text
    css_content.append("    text { fill: #D1D7E0; }")

    # Override Paths
    for cls, data in styles_to_generate.items():
        # Only write an override if the color actually changes
        if data['light'].lower() != data['dark'].lower():
            css_content.append(f"    .{cls} {{ {data['prop']}: {data['dark']}; }}")

    css_content.append("}")

    # D. Path Drawing Animations
    if ordered_elements:
        css_content.append("\n/* Path Drawing Animations (DFS Order: Top-to-Bottom, Left-to-Right) */")
        duration_per_element = total_duration / len(ordered_elements)

        for i in range(len(ordered_elements)):
            start_time = i * duration_per_element
            css_content.append(f".anim-path-{i} {{")
            css_content.append(f"    stroke-dasharray: 1;")
            css_content.append(f"    stroke-dashoffset: 1;")
            css_content.append(f"    animation: draw-path-{i} {duration_per_element:.3f}s ease-out {start_time:.3f}s forwards;")
            css_content.append(f"}}")
            css_content.append(f"@keyframes draw-path-{i} {{")
            css_content.append(f"    to {{ stroke-dashoffset: 0; }}")
            css_content.append(f"}}")

    # 4. Inject Style Tag into SVG
    style_elem = ET.Element("style")
    style_elem.text = "\n".join(css_content)
    root.insert(0, style_elem)

    # 5. Save
    tree.write(OUTPUT_FILENAME, encoding="UTF-8", xml_declaration=True)
    print(f"Done! Processed {len(styles_to_generate)} unique colors.")
    print(f"Saved to: {OUTPUT_FILENAME}")

if __name__ == "__main__":
    process_svg()
