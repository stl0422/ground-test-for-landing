#!/bin/bash
set -e

LOG_DIR=~/landing/start_online_logs

if compgen -G "${LOG_DIR}/*.pid" >/dev/null 2>&1; then
  for pid_file in "${LOG_DIR}"/*.pid; do
    pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ "${pid}" =~ ^[0-9]+$ ]]; then
      pkill -TERM -P "${pid}" 2>/dev/null || true
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
fi

pkill -f '[r]oslaunch px4 livox_custom.launch' || true
pkill -f '[r]oslaunch fast_lio mapping_mid360.launch' || true
pkill -f '[r]oslaunch fast_lio_global_grid_map global_grid_map.launch' || true
pkill -f '[r]oslaunch safeland safeland.launch' || true
pkill -f '[r]oslaunch mpc_control control_sim_iros2025.launch' || true
pkill -f '[r]oslaunch super_planner iros_real.launch' || true
pkill -f '[r]oslaunch state_machine iros_state_machine_simple_sim.launch' || true

pkill -f '[r]osmaster' || true
pkill -f '[r]osout' || true
pkill -f '[g]zserver' || true
pkill -f '[g]zclient' || true
pkill -f '[m]avros_node' || true
pkill -f '[f]astlio_mapping' || true
pkill -f '[f]astlio_px4' || true
pkill -f '[g]lobal_grid_map_node' || true
pkill -f '[s]afeland_node' || true
pkill -f '[g]rid_map_visualization' || true
pkill -f '[t]racking_real_iros2025' || true
pkill -f '[f]sm_node' || true
pkill -f '[i]ros_state_machine_simple' || true
pkill -f 'PX4_Firmware/build/px4_sitl_default/bin/px4' || true
pkill -f '[p]x4-simulator' || true
pkill -f '[r]ostopic echo -n 1 /mavros/local_position/odom' || true
pkill -f '[r]ostopic echo -n 1 /mavros/local_position/pose' || true

echo "[stop_online] stopped online simulation processes"
