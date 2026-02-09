#!/bin/bash
# Run isolated integration tests
#
# These tests run the game_controller with mock services,
# completely isolated from external dependencies.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "================================"
echo "Isolated Integration Tests"
echo "================================"
echo ""
echo "Running game_controller tests with mock services..."
echo "No external dependencies required!"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Clean up any previous runs
echo -e "${YELLOW}Cleaning up previous test runs...${NC}"
docker compose -f docker-compose.isolated.yml down -v 2>/dev/null || true

# Create results directory
mkdir -p test/results

# Run tests
echo -e "${YELLOW}Starting mock services and running tests...${NC}"
if docker compose -f docker-compose.isolated.yml up --build --abort-on-container-exit; then
    echo ""
    echo -e "${GREEN}✓ Tests completed successfully!${NC}"
    exit_code=0
else
    echo ""
    echo -e "${RED}✗ Tests failed!${NC}"
    exit_code=1
fi

# Show logs from test runner
echo ""
echo "================================"
echo "Test Runner Logs"
echo "================================"
docker compose -f docker-compose.isolated.yml logs test_runner

# Cleanup
echo ""
echo -e "${YELLOW}Cleaning up...${NC}"
docker compose -f docker-compose.isolated.yml down -v

# Check if results file exists
if [ -f "test/results/isolated_results.xml" ]; then
    echo ""
    echo -e "${GREEN}Test results saved to: test/results/isolated_results.xml${NC}"
fi

exit $exit_code
