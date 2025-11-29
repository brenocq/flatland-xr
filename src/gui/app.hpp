// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#pragma once

class App {
public:
    App();

    /// Startup app internal data structures
    void startup();

    /// Shutdown the app and cleanup internal data structures
    void shutdown();

    /// Update app logic and render frame
    void update();
};
