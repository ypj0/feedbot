-- Feedbot pure localization against an existing Cartographer pbstream state.
-- Requires load_state_filename to point at a mapping output.

include "feedbot_2d_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

-- 放宽全局重定位搜索窗，让 Cartographer 自己全图找回位姿
-- （覆盖 feedbot_2d_mapping.lua 末尾的 1m/30°）。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)

return options
