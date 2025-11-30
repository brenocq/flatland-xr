// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include "imgui.h"
#include <Eigen/Dense>

namespace gui {

class Color : public Eigen::Vector4f {
  public:
    Color(float r, float g, float b, float a = 1.0f) : Eigen::Vector4f(r, g, b, a) {}

    operator ImVec4() const { return ImVec4(r(), g(), b(), a()); }
    explicit Color(const ImVec4& c) : Eigen::Vector4f(c.x, c.y, c.z, c.w) {}

    float r() const { return x(); }
    float g() const { return y(); }
    float b() const { return z(); }
    float a() const { return w(); }
    float& r() { return x(); }
    float& g() { return y(); }
    float& b() { return z(); }
    float& a() { return w(); }

    static Color Auto() { return Color(0.0f, 0.0f, 0.0f, -1.0f); }
    static Color Black() { return Color(0.0f, 0.0f, 0.0f); }
    static Color Blue() { return Color(0.0f, 0.0f, 1.0f); }
    static Color Green() { return Color(0.0f, 1.0f, 0.0f); }
    static Color Cyan() { return Color(0.0f, 1.0f, 1.0f); }
    static Color Red() { return Color(1.0f, 0.0f, 0.0f); }
    static Color Magenta() { return Color(1.0f, 0.0f, 1.0f); }
    static Color Yellow() { return Color(1.0f, 1.0f, 0.0f); }
    static Color White() { return Color(1.0f, 1.0f, 1.0f); }
    static Color Transparent() { return Color(0.0f, 0.0f, 0.0f, 0.0f); }
    static Color Random(size_t index);

    // Catppuccin Mocha color palette
    // Base colors (backgrounds)
    static Color CatBase() { return Color(0.117f, 0.117f, 0.172f); }     // #1e1e2e
    static Color CatMantle() { return Color(0.109f, 0.109f, 0.156f); }   // #181825
    static Color CatCrust() { return Color(0.071f, 0.071f, 0.109f); }    // #11111b
    static Color CatSurface0() { return Color(0.200f, 0.207f, 0.286f); } // #313244
    static Color CatSurface1() { return Color(0.247f, 0.254f, 0.337f); } // #3f4056
    static Color CatSurface2() { return Color(0.290f, 0.301f, 0.388f); } // #4a4d63
    static Color CatOverlay0() { return Color(0.396f, 0.403f, 0.486f); } // #65677c
    static Color CatOverlay1() { return Color(0.447f, 0.454f, 0.537f); } // #72748a
    static Color CatOverlay2() { return Color(0.576f, 0.584f, 0.654f); } // #9399b2
    // Text colors
    static Color CatText() { return Color(0.803f, 0.815f, 0.878f); }     // #cdd6f4
    static Color CatSubtext1() { return Color(0.725f, 0.741f, 0.823f); } // #b9bfd2
    static Color CatSubtext0() { return Color(0.639f, 0.658f, 0.764f); } // #a3a8c3
    // Accent colors
    static Color CatRosewater() { return Color(0.956f, 0.878f, 0.878f); } // #f4dbd6
    static Color CatFlamingo() { return Color(0.956f, 0.823f, 0.866f); }  // #f2cdcd
    static Color CatPink() { return Color(0.956f, 0.749f, 0.854f); }      // #f5bde6
    static Color CatMauve() { return Color(0.796f, 0.698f, 0.972f); }     // #cba6f7
    static Color CatRed() { return Color(0.956f, 0.619f, 0.639f); }       // #f38ba8
    static Color CatMaroon() { return Color(0.921f, 0.650f, 0.698f); }    // #eba0ac
    static Color CatPeach() { return Color(0.980f, 0.709f, 0.572f); }     // #fab387
    static Color CatYellow() { return Color(0.980f, 0.913f, 0.596f); }    // #f9e2af
    static Color CatGreen() { return Color(0.650f, 0.890f, 0.631f); }     // #a6e3a1
    static Color CatTeal() { return Color(0.580f, 0.886f, 0.819f); }      // #94e2d5
    static Color CatSky() { return Color(0.537f, 0.850f, 0.972f); }       // #89dceb
    static Color CatSapphire() { return Color(0.458f, 0.784f, 0.878f); }  // #74c7ec
    static Color CatBlue() { return Color(0.533f, 0.698f, 0.976f); }      // #89b4fa
    static Color CatLavender() { return Color(0.709f, 0.764f, 0.980f); }  // #b4befe
};

} // namespace gui
