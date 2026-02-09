#!/bin/bash
# Runner script for integration test + manifest organization
#
# Usage:
#   ./test/integration/run_and_organize.sh
#
# This script:
# 1. Runs the integration test (generates manifest_log.jsonl)
# 2. Runs the post-processor (organizes manifests for UI developers)
# 3. Reports completion

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "================================================"
echo "Colores Integration Test + Manifest Organization"
echo "================================================"
echo ""

# Step 1: Run integration test
echo "Step 1: Running integration test..."
echo "  This will generate manifest_log.jsonl and decision_state_log.jsonl"
echo ""

cd "${PROJECT_ROOT}"
docker compose -f docker-compose.gc_dm_integration.yml up --build --abort-on-container-exit

TEST_EXIT_CODE=$?

if [ ${TEST_EXIT_CODE} -ne 0 ]; then
    echo ""
    echo "ERROR: Integration test failed with exit code ${TEST_EXIT_CODE}"
    echo ""
    echo "Check the logs:"
    echo "  docker compose -f docker-compose.gc_dm_integration.yml logs"
    exit ${TEST_EXIT_CODE}
fi

echo ""
echo "Step 1: COMPLETE"
echo ""

# Step 2: Run post-processor
echo "Step 2: Organizing manifests for UI developers..."
echo ""

cd "${PROJECT_ROOT}"
python3 test/integration/organize_manifests_for_ui.py

ORGANIZE_EXIT_CODE=$?

if [ ${ORGANIZE_EXIT_CODE} -ne 0 ]; then
    echo ""
    echo "ERROR: Manifest organization failed with exit code ${ORGANIZE_EXIT_CODE}"
    exit ${ORGANIZE_EXIT_CODE}
fi

echo ""
echo "Step 2: COMPLETE"
echo ""

# Step 3: Report completion
echo "================================================"
echo "SUCCESS! All steps completed."
echo "================================================"
echo ""
echo "Integration test results:"
echo "  - Manifest log: ${PROJECT_ROOT}/test/results/manifest_log.jsonl"
echo "  - Decision state log: ${PROJECT_ROOT}/test/results/decision_state_log.jsonl"
echo "  - Report: ${PROJECT_ROOT}/test/results/report.md"
echo ""
echo "UI developer manifests:"
echo "  - Location: /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/"
echo "  - README: /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/README.md"
echo "  - Index: /home/alono/EmorobCare/games_src/ui_developer_manifests/colores_full_game/index.md"
echo ""
echo "Next steps:"
echo "  1. Review the README for an overview"
echo "  2. Browse the index to see all states"
echo "  3. Explore individual state directories"
echo ""
