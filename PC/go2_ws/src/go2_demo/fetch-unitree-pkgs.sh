#!/usr/bin/bash
WS_SRC_DIR=$(realpath ..)
if [[ "${WS_SRC_DIR}" != */src ]]; then
	echo -e "\e[1m\e[31mERROR:\e[0m\e[31m Failed to download unitree API\e[0m"
	echo -e "\tYour package must be inside a some_ws/src directory structure."
	exit 0
fi


if [ ! -d "${WS_SRC_DIR}/unitree_api" ] || [ ! -d "${WS_SRC_DIR}/unitree_go" ] || [ ! -d "${WS_SRC_DIR}/unitree_hg" ]; then
	echo "Downloading unitree_ros2"
	git clone -q -n --depth 1 --filter=tree:0 https://github.com/unitreerobotics/unitree_ros2 tmp
	cd tmp
	git sparse-checkout set --no-cone /LICENSE /cyclonedds_ws/src/unitree &> /dev/null
	find . -type f -exec chmod -x {} \;
	cp LICENSE cyclonedds_ws/src/unitree/unitree_api
	cp LICENSE cyclonedds_ws/src/unitree/unitree_go
	mv LICENSE cyclonedds_ws/src/unitree/unitree_hg
	mv cyclonedds_ws/src/unitree/* ${WS_SRC_DIR} && cd .. && rm -rf tmp
	echo -e "\e[32mUnitree API downloaded to ${WS_SRC_DIR}\e[0m"
else
	echo -e "\e[32mFound unitree API on ${WS_SRC_DIR}\e[0m"
fi
