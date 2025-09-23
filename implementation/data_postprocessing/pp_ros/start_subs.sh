#!/bin/bash

#start cleanup
cleanup() {
    echo "Stopping all processes..."
    kill $(jobs -p) 2>/dev/null
    wait
    exit 0
}

# cleanup at STGR+C
trap cleanup SIGINT SIGTERM

python3 sub_pose.py &
python3 sub_cameraInfo.py &
python3 sub_images.py &

echo "Data capturing started. STRG+C to stop."

wait