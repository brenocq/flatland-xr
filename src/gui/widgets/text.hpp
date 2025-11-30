// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/color.hpp>
#include <string>

namespace gui::widgets {

/// Display text
void Text(const char* fmt, ...) IM_FMTARGS(1);

/// Display bold text
void BoldText(const char* fmt, ...) IM_FMTARGS(1);

/// Display colored text
void ColoredText(const Color& color, const char* fmt, ...) IM_FMTARGS(2);

/// Display wrapped text (for long text that should wrap to multiple lines)
void TextWrapped(const char* fmt, ...) IM_FMTARGS(1);

/// Display text with a hyperlink that opens a URL
void TextLinkOpenURL(const char* label, const char* url);

} // namespace gui::widgets
