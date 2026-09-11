#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/a/2026-HL-FMA-VTD-JG/hlfma_ws/install/setup.bash
export ROS_DOMAIN_ID=43
OUT=/tmp/rosgraph
rm -rf $OUT; mkdir -p $OUT
echo "노드 수집..."
ros2 node list 2>/dev/null | sort -u > $OUT/nodes.txt
wc -l < $OUT/nodes.txt
while read -r n; do
  [ -z "$n" ] && continue
  f=$(echo "$n" | tr '/' '_')
  timeout 8 ros2 node info "$n" > "$OUT/info$f.txt" 2>/dev/null
done < $OUT/nodes.txt
echo "수집 완료: $(ls $OUT/info* 2>/dev/null | wc -l) 개"
