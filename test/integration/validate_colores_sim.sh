#!/bin/bash
# Validation script for colores full game simulation setup
# Checks that all required files and directories are in place

set -e

echo "================================"
echo "Colores Simulation Validation"
echo "================================"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    FAILED=1
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

FAILED=0

# Check script location
echo "Checking files..."
if [ -f "simulate_colores_full_game.py" ]; then
    check_pass "simulate_colores_full_game.py exists"
else
    check_fail "simulate_colores_full_game.py not found"
fi

if [ -x "simulate_colores_full_game.py" ]; then
    check_pass "simulate_colores_full_game.py is executable"
else
    check_warn "simulate_colores_full_game.py not executable (chmod +x needed)"
fi

if [ -f "README_COLORES_SIM.md" ]; then
    check_pass "README_COLORES_SIM.md exists"
else
    check_fail "README_COLORES_SIM.md not found"
fi

# Check docker-compose file
echo ""
echo "Checking docker-compose..."
if [ -f "../../docker-compose.colores-sim.yml" ]; then
    check_pass "docker-compose.colores-sim.yml exists"
else
    check_fail "docker-compose.colores-sim.yml not found"
fi

# Check output directory
echo ""
echo "Checking output directory..."
OUTPUT_DIR="/home/alono/EmorobCare/games_src/ui_developer_manifests"
if [ -d "$OUTPUT_DIR" ]; then
    check_pass "Output parent directory exists: $OUTPUT_DIR"
else
    check_warn "Output directory doesn't exist: $OUTPUT_DIR (will be created automatically)"
fi

if [ -w "$OUTPUT_DIR" ] 2>/dev/null; then
    check_pass "Output directory is writable"
elif [ ! -d "$OUTPUT_DIR" ]; then
    check_warn "Output directory doesn't exist yet (will be created)"
else
    check_fail "Output directory is not writable: $OUTPUT_DIR"
fi

# Check Python syntax
echo ""
echo "Checking Python syntax..."
if python3 -m py_compile simulate_colores_full_game.py 2>/dev/null; then
    check_pass "Python syntax is valid"
else
    check_fail "Python syntax errors found"
fi

# Check Docker
echo ""
echo "Checking Docker..."
if command -v docker &> /dev/null; then
    check_pass "Docker is installed"
    DOCKER_VERSION=$(docker --version | cut -d' ' -f3 | cut -d',' -f1)
    echo "  Version: $DOCKER_VERSION"
else
    check_fail "Docker is not installed"
fi

if command -v docker-compose &> /dev/null || docker compose version &> /dev/null 2>&1; then
    check_pass "docker-compose is available"
    if command -v docker-compose &> /dev/null; then
        COMPOSE_VERSION=$(docker-compose --version | cut -d' ' -f4 | cut -d',' -f1)
        echo "  Version: $COMPOSE_VERSION (standalone)"
    else
        COMPOSE_VERSION=$(docker compose version --short)
        echo "  Version: $COMPOSE_VERSION (plugin)"
    fi
else
    check_fail "docker-compose is not available"
fi

# Check documentation
echo ""
echo "Checking documentation..."
if [ -f "../../QUICKSTART_COLORES_SIM.md" ]; then
    check_pass "QUICKSTART_COLORES_SIM.md exists"
else
    check_warn "QUICKSTART_COLORES_SIM.md not found"
fi

# Check test directory
echo ""
echo "Checking test infrastructure..."
if [ -f "../mocks/mock_decision_making.py" ]; then
    check_pass "mock_decision_making.py exists"
else
    check_fail "mock_decision_making.py not found"
fi

if [ -f "../mocks/mock_generic_ui.py" ]; then
    check_pass "mock_generic_ui.py exists"
else
    check_fail "mock_generic_ui.py not found"
fi

# Summary
echo ""
echo "================================"
if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}All checks passed!${NC}"
    echo ""
    echo "Ready to run simulation:"
    echo "  cd ../../"
    echo "  docker compose -f docker-compose.colores-sim.yml up --build --abort-on-container-exit"
    exit 0
else
    echo -e "${RED}Some checks failed!${NC}"
    echo ""
    echo "Please fix the issues above before running the simulation."
    exit 1
fi
