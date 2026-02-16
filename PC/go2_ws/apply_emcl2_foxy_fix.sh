#!/bin/bash
# Parches de compatibilidad Foxy para emcl2
# Ejecutar desde: /home/unitree/Go2-Software/PC/go2_ws

EMCL2_DIR="$(cd "$(dirname "$0")" && pwd)/src/emcl2_ros2"
SRC="$EMCL2_DIR/src/emcl2_node.cpp"

if [ ! -f "$SRC" ]; then
  echo "Error: no se encontró $SRC"
  exit 1
fi

echo "Aplicando parches en: $EMCL2_DIR"

# Fix 1: create_callback_group (Foxy solo acepta 1 argumento)
sed -i 's/create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)/create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)/g' "$SRC"

# Fix 2 y 3: cbSimpleReset debe retornar void en Foxy
sed -i 's/^bool EMcl2Node::cbSimpleReset/void EMcl2Node::cbSimpleReset/' "$SRC"
sed -i 's/return simple_reset_request_ = true;/simple_reset_request_ = true;/' "$SRC"

# Fix 4: declaración en header
sed -i 's/^\tbool cbSimpleReset/\tvoid cbSimpleReset/' "$EMCL2_DIR/include/emcl2/emcl2_node.h"

# Fix 5: CreateTimerROS en Foxy solo acepta 2 argumentos (sin callback group)
sed -i 's/get_node_timers_interface(),[[:space:]]*$/get_node_timers_interface());/' "$SRC"
sed -i '/create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive/d' "$SRC"

# Fix 6: create_service en Foxy requiere lambda con firma exacta (SharedPtr, no ConstSharedPtr)
sed -i 's|std::bind(&EMcl2Node::cbSimpleReset, this, std::placeholders::_1, std::placeholders::_2)|[this](std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) { simple_reset_request_ = true; }|' "$SRC"

echo "Listo. Ejecuta: colcon build --packages-select emcl2 --symlink-install"
