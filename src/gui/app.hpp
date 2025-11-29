// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

class App {
public:
    App();

    void startup();
    void shutdown();

    /// Update app logic and render frame
    void update();
};
