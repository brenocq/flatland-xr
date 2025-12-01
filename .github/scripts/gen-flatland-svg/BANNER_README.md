# GitHub Banner Generator

## Overview

This script generates a GitHub profile banner (1280×640px) showing the Flatland emoji at 4 different animation stages side by side.

## Banner Layout

```
┌───────────────────────────────────────────────────┐
│                                                   │
│                    Flatland XR                    │
│                                                   │
│   [Full]    [75%]     [50%]     [25%]    [Line]   │
│   emoji     emoji     emoji     emoji     only    │
│                                                   │
└───────────────────────────────────────────────────┘
     100%      75%       50%       25%        0%
```

## Features

- **Dimensions**: 1280×640px (GitHub recommended banner size)
- **Safe Border**: 40px border around important content (GitHub recommendation)
- **Title**: "Flatland XR" centered at the top
- **Background**: Catppuccin Mocha dark theme (#1e1e2e)
- **Text Color**: Catppuccin Mocha text (#cdd6f4)
- **High Resolution**: Emojis are rendered at 192×192px each (calculated dynamically)
- **Output Format**: PNG (for best compatibility and smaller file size)
- **5 Emoji States**:
  - 100% - Full emoji visible
  - 75% - Slightly squeezed
  - 50% - Half squeezed
  - 25% - Mostly squeezed
  - 0% - Just the critical line

## Usage

```bash
cd .github/scripts/gen-flatland-svg
python3 gen_flatland_banner.py
```

This will generate `github-banner.svg` and `github-banner.png` in the same directory.
