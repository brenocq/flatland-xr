// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <gui/ui_state.hpp>

namespace gui {

/// Panel for displaying error metrics
class ErrorMetricsPanel {
  public:
    ErrorMetricsPanel() = default;

    void set_ui_state(UIState::SharedPtr ui_state) { _ui_state = std::move(ui_state); }

    /// Render the panel
    void render();

  private:
    UIState::SharedPtr _ui_state;
};

} // namespace gui
