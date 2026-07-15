#!/bin/bash
# analyze_bag.sh - Analyze FAST-LIO2 debug bag
#
# Usage: ./analyze_bag.sh <bag_path>
# Example: ./analyze_bag.sh ~/bags/fastlio_debug_20250714_101234

set -e

BAG_PATH="$1"

if [ -z "$BAG_PATH" ]; then
    echo "Usage: $0 <bag_path>"
    echo "Example: $0 ~/bags/fastlio_debug_20250714_101234"
    exit 1
fi

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: Bag directory not found: $BAG_PATH"
    exit 1
fi

echo "=========================================="
echo "FAST-LIO2 Bag Analysis"
echo "=========================================="
echo "Bag path: $BAG_PATH"
echo ""

# Info
echo "1. Bag Info:"
ros2 bag info "$BAG_PATH"
echo ""

# Topic statistics
echo "2. Topic Statistics:"
echo "   Message counts per topic:"
ros2 bag info "$BAG_PATH" | grep -E '^\s+[0-9]+\s+' | head -20
echo ""

# Duration
echo "3. Duration:"
ros2 bag info "$BAG_PATH" | grep -i duration
echo ""

# Size
echo "4. Size:"
du -sh "$BAG_PATH"
echo ""

# Create analysis directory
ANALYSIS_DIR="${BAG_PATH}_analysis"
mkdir -p "$ANALYSIS_DIR"

echo "5. Extracting data to: $ANALYSIS_DIR"

# List available topics for manual analysis
echo ""
echo "6. Available topics for analysis:"
ros2 bag info "$BAG_PATH" | grep -E '^\s+[a-z]' | awk '{print "   -", $1}'

echo ""
echo "7. Quick playback command:"
echo "   ros2 bag play $BAG_PATH --clock --rate 0.5"
echo ""

echo "=========================================="
echo "Analysis complete!"
echo "=========================================="
echo "Next steps:"
echo "   - Play bag: ros2 bag play $BAG_PATH --clock"
echo "   - Inspect: ros2 topic echo /lio/odometry"
echo "   - Visualize: rviz2 -d src/fast_lio/config/fast_lio.rviz"
echo ""
