// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

#include <string>

namespace gui {

/// Physics bench to configure and run simulations
class Bench {
  public:
    Bench(const std::string& name);

    /// Render the bench window
    virtual void render();

    const std::string& name() const { return _name; }

  private:
    std::string _name;
};

} // namespace gui
