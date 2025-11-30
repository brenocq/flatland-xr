// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <filesystem>
#include <string>

namespace core {

/// Get the path to a resource file
/// Tries multiple locations:
/// 1. Relative to current directory (build directory): ./resources/...
/// 2. Source directory (compile-time path): ${SOURCE_DIR}/resources/...
inline std::string get_resource_path(const std::string& relative_path) {
    namespace fs = std::filesystem;

    // Try relative to current directory (for running from build dir)
    fs::path path1 = fs::path("resources") / relative_path;
    if (fs::exists(path1)) {
        return path1.string();
    }

// Try source directory (compile-time path)
#ifdef FLATLAND_XR_SOURCE_DIR
    fs::path path2 = fs::path(FLATLAND_XR_SOURCE_DIR) / "resources" / relative_path;
    if (fs::exists(path2)) {
        return path2.string();
    }
#endif

    // Fallback: return the relative path and hope for the best
    return path1.string();
}

} // namespace core
