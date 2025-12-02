#!/bin/bash
# Format all C++ and CMake files in the project

set -e

echo "🎨 Formatting all C++ and CMake files..."

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Find and format all C++ files
echo -e "${YELLOW}📝 Formatting C++ files...${NC}"
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" -o -name "*.cc" \) \
    ! -path "*/build/*" \
    ! -path "*/_deps/*" \
    ! -path "*/imgui/*" \
    ! -path "*/implot/*" \
    -exec clang-format -i {} \; \
    -exec echo "   Formatted: {}" \;

# Find and format all CMake files
echo -e "${YELLOW}📝 Formatting CMake files...${NC}"
find . -type f \( -name "CMakeLists.txt" -o -name "*.cmake" \) \
    ! -path "*/build/*" \
    ! -path "*/_deps/*" \
    -exec cmake-format -i {} \; \
    -exec echo "   Formatted: {}" \;

echo -e "${GREEN}✅ All files formatted!${NC}"
echo ""
echo "Run 'git diff' to see the changes"
