// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <gui/benches/bench.hpp>
#include <gui/widgets/text.hpp>

namespace gui {

Bench::Bench(const std::string& name) : _name(name) {}

void Bench::render() {
    if (ImGui::Begin(_name.c_str())) {
        widgets::Text("Bench %s was not setup yet", _name.c_str());
    }
    ImGui::End();
}

} // namespace gui
