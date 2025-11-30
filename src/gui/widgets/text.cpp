// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include <cstdarg>
#include <gui/widgets/text.hpp>

namespace gui::widgets {

void Text(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
}

void BoldText(const char* fmt, ...) {
    // Push bold font (loaded as second font, index 1)
    ImGuiIO& io = ImGui::GetIO();
    if (io.Fonts->Fonts.Size > 1) {
        ImGui::PushFont(io.Fonts->Fonts[1]);
    }

    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);

    if (io.Fonts->Fonts.Size > 1) {
        ImGui::PopFont();
    }
}

void ColoredText(const Color& color, const char* fmt, ...) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(color));

    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);

    ImGui::PopStyleColor();
}

void TextWrapped(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    // Get the formatted string
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    ImGui::TextWrapped("%s", buffer);
}

void TextLinkOpenURL(const char* label, const char* url) { ImGui::TextLinkOpenURL(label, url); }

} // namespace gui::widgets
