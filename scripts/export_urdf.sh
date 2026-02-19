#!/bin/bash
# Export URDF and STL meshes from Onshape into the my_robot_description ROS2 package.
# Run this script from INSIDE the Docker container.
#
# Usage:
#   /home/ubuntu/scripts/export_urdf.sh
#
# Prerequisites:
#   - ONSHAPE_API, ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY must be set (via .env / docker-compose)
#   - onshape-to-robot must be installed (already in the Docker image)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PACKAGE_DIR="/home/ubuntu/ros2_workspaces/src/sec26ros/my_robot_description"
WORK_DIR="$(mktemp -d)"

echo "==> Exporting URDF from Onshape..."
echo "    Work directory: $WORK_DIR"

# Verify credentials
if [ -z "$ONSHAPE_ACCESS_KEY" ] || [ -z "$ONSHAPE_SECRET_KEY" ]; then
    echo "ERROR: ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY must be set."
    echo "       They are passed automatically via docker-compose from the root .env file."
    exit 1
fi

# Copy config into working directory and run export
cp "$SCRIPT_DIR/onshape-config.json" "$WORK_DIR/config.json"
cd "$WORK_DIR"
onshape-to-robot .

# Clear and repopulate meshes directory
echo "==> Clearing $PACKAGE_DIR/meshes/ ..."
rm -rf "$PACKAGE_DIR/meshes"
mkdir -p "$PACKAGE_DIR/meshes"
echo "==> Copying meshes to $PACKAGE_DIR/meshes/ ..."
cp "$WORK_DIR/assets/"*.stl "$PACKAGE_DIR/meshes/"

# Rewrite URDF mesh paths from raw export format to ROS2 package:// format
echo "==> Fixing URDF mesh paths..."
sed \
    -e 's|package://assets\\|package://my_robot_description/meshes/|g' \
    -e 's|package://assets/|package://my_robot_description/meshes/|g' \
    "$WORK_DIR/robot.urdf" > "$PACKAGE_DIR/urdf/robot.urdf"

rm -rf "$WORK_DIR"

echo ""
echo "==> Export complete!"
echo "    Updated:"
echo "      $PACKAGE_DIR/urdf/robot.urdf"
echo "      $PACKAGE_DIR/meshes/*.stl"
echo ""
echo "    Rebuild the package inside the container:"
echo "      cd /home/ubuntu/ros2_workspaces"
echo "      colcon build --packages-select my_robot_description"
echo "      source install/setup.bash"
