#!/bin/bash
# play_bag_debug.sh - Optimized bag playback for FAST-LIO2 debugging
#
# Usage: ./play_bag_debug.sh <bag_path> [options]
# Options:
#   -r, --rate     Playback rate (default: 0.5)
#   -s, --start    Start offset in seconds (default: 0)
#   -l, --loop     Loop playback
#   --no-clock     Don't publish /clock
#   --pause        Start paused

set -e

RATE=0.5
START=0
LOOP=""
CLOCK="--clock"
PAUSE=""

# Parse arguments
BAG_PATH=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--rate)
            RATE="$2"
            shift 2
            ;;
        -s|--start)
            START="$2"
            shift 2
            ;;
        -l|--loop)
            LOOP="--loop"
            shift
            ;;
        --no-clock)
            CLOCK=""
            shift
            ;;
        --pause)
            PAUSE="--pause"
            shift
            ;;
        -*)
            echo "Unknown option: $1"
            exit 1
            ;;
        *)
            BAG_PATH="$1"
            shift
            ;;
    esac
done

if [ -z "$BAG_PATH" ]; then
    echo "Usage: $0 <bag_path> [options]"
    echo ""
    echo "Options:"
    echo "  -r, --rate     Playback rate (default: 0.5)"
    echo "  -s, --start    Start offset in seconds (default: 0)"
    echo "  -l, --loop     Loop playback"
    echo "  --no-clock     Don't publish /clock"
    echo "  --pause        Start paused"
    echo ""
    echo "Examples:"
    echo "  $0 ~/bags/fastlio_debug_20250714_101234"
    echo "  $0 ~/bags/fastlio_debug_20250714_101234 -r 0.1 --pause"
    exit 1
fi

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: Bag directory not found: $BAG_PATH"
    exit 1
fi

echo "=========================================="
echo "FAST-LIO2 Bag Playback"
echo "=========================================="
echo "Bag:      $BAG_PATH"
echo "Rate:     ${RATE}x"
echo "Start:    ${START}s"
echo "Loop:     ${LOOP:-no}"
echo "Clock:    ${CLOCK:-no}"
echo "Paused:   ${PAUSE:-no}"
echo "=========================================="
echo ""
echo "Controls during playback:"
echo "  SPACE    - Pause/resume"
echo "  S        - Step one message (when paused)"
echo "  /        - Search for topic"
echo "  +/-      - Change playback rate"
echo "  Q        - Quit"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Build command
CMD="ros2 bag play \"$BAG_PATH\" --rate $RATE --start-offset $START $CLOCK $LOOP $PAUSE"

echo "Executing: $CMD"
echo ""
eval $CMD
