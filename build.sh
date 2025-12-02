#!/bin/bash
# Build and run script for Flatland XR

set -e

# Detect if we should use colors (only if stdout is a terminal)
if [ -t 1 ]; then
    # Colors
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    GRAY='\033[0;90m'
    BOLD='\033[1m'
    NC='\033[0m' # No Color
else
    # No colors
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    GRAY=''
    BOLD=''
    NC=''
fi

# Default values
BUILD_TYPE="Release"
BUILD_DIR="build"
RUN_TESTS=false
RUN_APP=false
CLEAN=false
JOBS=$(nproc)
STRICT_WARNINGS=false
ENABLE_TESTS=true
VERBOSE=false
WEB_BUILD=false

# Print usage
usage() {
    printf "${BOLD}😎 Flatland XR - Build and Run Script${NC}\n\n"

    printf "${BOLD}USAGE:${NC}\n"
    printf "    build.sh [OPTIONS]\n\n"

    printf "${BOLD}OPTIONS:${NC}\n"
    printf "    ${GREEN}-h, --help${NC}              Show this help message\n"
    printf "\n"
    printf "    ${YELLOW}Build Options:${NC}\n"
    printf "    ${GREEN}-d, --debug${NC}             Build in Debug mode (default: Release)\n"
    printf "    ${GREEN}-r, --release${NC}           Build in Release mode\n"
    printf "    ${GREEN}-c, --clean${NC}             Clean build directory before building\n"
    printf "    ${GREEN}-j, --jobs <N>${NC}          Number of parallel jobs (default: auto-detect)\n"
    printf "    ${GREEN}-s, --strict${NC}            Enable strict warnings and treat as errors\n"
    printf "    ${GREEN}-v, --verbose${NC}           Verbose build output\n"
    printf "    ${GREEN}--no-tests${NC}              Don't build tests\n"
    printf "    ${GREEN}--web${NC}                   Build for WebAssembly (requires Emscripten)\n"
    printf "\n"
    printf "    ${YELLOW}Run Options:${NC}\n"
    printf "    ${GREEN}-t, --test${NC}              Run tests after building\n"
    printf "    ${GREEN}-x, --run${NC}               Run the application after building\n"
    printf "\n"
    printf "    ${YELLOW}Directory:${NC}\n"
    printf "    ${GREEN}-b, --build-dir <DIR>${NC}   Build directory (default: build)\n\n"

    printf "${BOLD}EXAMPLES:${NC}\n"
    printf "    ${GRAY}# Build in release mode${NC}\n"
    printf "    ./build.sh\n\n"
    printf "    ${GRAY}# Build in debug mode and run tests${NC}\n"
    printf "    ./build.sh --debug --test\n\n"
    printf "    ${GRAY}# Clean build with strict warnings${NC}\n"
    printf "    ./build.sh --clean --strict\n\n"
    printf "    ${GRAY}# Build and run the application${NC}\n"
    printf "    ./build.sh --run\n\n"
    printf "    ${GRAY}# Fast debug build with 8 jobs${NC}\n"
    printf "    ./build.sh -d -j 8\n\n"
    printf "    ${GRAY}# Build for WebAssembly${NC}\n"
    printf "    ./build.sh --web --run\n\n"
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        -c|--clean)
            CLEAN=true
            shift
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -s|--strict)
            STRICT_WARNINGS=true
            shift
            ;;
        -t|--test)
            RUN_TESTS=true
            shift
            ;;
        -x|--run)
            RUN_APP=true
            shift
            ;;
        -b|--build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --no-tests)
            ENABLE_TESTS=false
            shift
            ;;
        --web)
            WEB_BUILD=true
            BUILD_DIR="build-web"
            ENABLE_TESTS=false
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        *)
            echo -e "${RED}❌ Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Print header
if [ "$WEB_BUILD" = true ]; then
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  😎🌐 Flatland XR - WebAssembly Build${NC}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
    echo ""

    # Check if Emscripten is available
    if ! command -v emcc &> /dev/null; then
        echo -e "${RED}❌ Emscripten not found!${NC}"
        echo ""
        echo "Install Emscripten:"
        echo -e "  ${YELLOW}# Arch Linux${NC}"
        echo -e "  yay -S emscripten"
        echo ""
        echo -e "  ${YELLOW}# Or manually${NC}"
        echo -e "  git clone https://github.com/emscripten-core/emsdk.git"
        echo -e "  cd emsdk"
        echo -e "  ./emsdk install latest"
        echo -e "  ./emsdk activate latest"
        echo -e "  source ./emsdk_env.sh"
        echo ""
        exit 1
    fi

    echo -e "${GREEN}✅ Emscripten found:${NC} $(emcc --version | head -1)"
    echo ""
else
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  😎 Flatland XR - Build Script${NC}"
    echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
    echo ""
fi

# Print configuration
echo -e "${BLUE}Configuration:${NC}"
echo -e "  Build Type:       ${YELLOW}${BUILD_TYPE}${NC}"
echo -e "  Build Directory:  ${YELLOW}${BUILD_DIR}${NC}"
echo -e "  Parallel Jobs:    ${YELLOW}${JOBS}${NC}"
echo -e "  Build Tests:      ${YELLOW}${ENABLE_TESTS}${NC}"
echo -e "  Strict Warnings:  ${YELLOW}${STRICT_WARNINGS}${NC}"
echo -e "  Clean Build:      ${YELLOW}${CLEAN}${NC}"
echo ""

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo -e "${YELLOW}🧹 Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
    echo -e "${GREEN}✅ Build directory cleaned${NC}"
    echo ""
fi

# Configure CMake
echo -e "${YELLOW}⚙️  Configuring CMake...${NC}"

if [ "$WEB_BUILD" = true ]; then
    # Web build with Emscripten
    if ! emcmake cmake -B "$BUILD_DIR" -DFLATLAND_XR_BUILD_TESTS=OFF; then
        echo -e "${RED}❌ CMake configuration failed${NC}"
        exit 1
    fi
else
    # Native build
    CMAKE_ARGS=(
        -B "$BUILD_DIR"
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    )

    if [ "$STRICT_WARNINGS" = true ]; then
        CMAKE_ARGS+=(-DFLATLAND_XR_STRICT_WARNINGS=ON)
    fi

    if [ "$ENABLE_TESTS" = false ]; then
        CMAKE_ARGS+=(-DFLATLAND_XR_BUILD_TESTS=OFF)
    fi

    if ! cmake "${CMAKE_ARGS[@]}"; then
        echo -e "${RED}❌ CMake configuration failed${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✅ CMake configuration complete${NC}"
echo ""

# Build
echo -e "${YELLOW}🔨 Building...${NC}"

if [ "$WEB_BUILD" = true ]; then
    # Web build
    if ! emmake cmake --build "$BUILD_DIR" --parallel; then
        echo -e "${RED}❌ Build failed${NC}"
        exit 1
    fi
else
    # Native build
    BUILD_ARGS=(
        --build "$BUILD_DIR"
        --parallel "$JOBS"
    )

    if [ "$VERBOSE" = true ]; then
        BUILD_ARGS+=(--verbose)
    fi

    if ! cmake "${BUILD_ARGS[@]}"; then
        echo -e "${RED}❌ Build failed${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✅ Build complete${NC}"
echo ""

# Copy compile_commands.json to root for IDE support (native only)
if [ "$WEB_BUILD" = false ] && [ -f "$BUILD_DIR/compile_commands.json" ]; then
    # Check if it's already a symlink pointing to the right place
    if [ ! -L "compile_commands.json" ] || [ "$(readlink -f compile_commands.json)" != "$(readlink -f "$BUILD_DIR/compile_commands.json")" ]; then
        # Remove existing file/symlink if it exists
        rm -f compile_commands.json
        # Create symlink instead of copying
        ln -s "$BUILD_DIR/compile_commands.json" compile_commands.json
        echo -e "${GREEN}✅ Linked compile_commands.json to root${NC}"
    else
        echo -e "${GREEN}✅ compile_commands.json already linked${NC}"
    fi
    echo ""
fi

# Show web build output files
if [ "$WEB_BUILD" = true ]; then
    echo -e "${BLUE}📦 Generated files:${NC}"
    ls -lh "$BUILD_DIR"/flatland-xr.* 2>/dev/null | awk '{printf "   %s  %s\n", $5, $9}' || echo "   (files not found)"
    echo ""
fi

# Run tests if requested (native only)
if [ "$RUN_TESTS" = true ]; then
    if [ "$WEB_BUILD" = true ]; then
        echo -e "${YELLOW}⚠️  Tests not available for web builds${NC}"
        echo ""
    elif [ "$ENABLE_TESTS" = false ]; then
        echo -e "${YELLOW}⚠️  Tests not built (--no-tests was used)${NC}"
    else
        echo -e "${YELLOW}🧪 Running tests...${NC}"
        echo ""
        if ! ctest --test-dir "$BUILD_DIR" --output-on-failure; then
            echo -e "${RED}❌ Tests failed${NC}"
            exit 1
        fi
        echo ""
        echo -e "${GREEN}✅ All tests passed${NC}"
        echo ""
    fi
fi

# Run application if requested
if [ "$RUN_APP" = true ]; then
    if [ "$WEB_BUILD" = true ]; then
        echo -e "${GREEN}🌐 Starting local server...${NC}"
        echo -e "   Open: ${YELLOW}http://localhost:8000/flatland-xr.html${NC}"
        echo -e "   Press Ctrl+C to stop"
        echo ""
        cd "$BUILD_DIR"
        python3 -m http.server 8000
    else
        echo -e "${YELLOW}🎮 Running Flatland XR...${NC}"
        echo ""

        APP_PATH="$BUILD_DIR/flatland-xr"
        if [ ! -f "$APP_PATH" ]; then
            echo -e "${RED}❌ Application not found at: $APP_PATH${NC}"
            exit 1
        fi

        # Run from build directory so resources are found
        cd "$BUILD_DIR"
        ./flatland-xr "$@"
        cd - > /dev/null
    fi
fi

# Summary
echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✨ Done!${NC}"
echo ""
if [ "$WEB_BUILD" = true ]; then
    echo -e "${BLUE}💡 Next steps:${NC}"
    if [ "$RUN_APP" = false ]; then
        echo -e "   • Test locally:  ${YELLOW}$0 --web --run${NC}"
        echo -e "   • Or serve:      ${YELLOW}cd $BUILD_DIR && python3 -m http.server${NC}"
    fi
    echo -e "   • Deploy:        ${YELLOW}git push${NC} (GitHub Actions will deploy)"
else
    echo -e "${BLUE}Next steps:${NC}"
    if [ "$RUN_TESTS" = false ] && [ "$ENABLE_TESTS" = true ]; then
        echo -e "  • Run tests:     ${YELLOW}$0 --test${NC}"
    fi
    if [ "$RUN_APP" = false ]; then
        echo -e "  • Run app:       ${YELLOW}$0 --run${NC}"
    fi
    echo -e "  • Build dir:     ${YELLOW}$BUILD_DIR/${NC}"
    echo -e "  • Executable:    ${YELLOW}$BUILD_DIR/flatland-xr${NC}"
fi
echo -e "${BOLD}════════════════════════════════════════════════════════════════${NC}"
