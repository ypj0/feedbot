-- Feedbot pure localization against an existing Cartographer pbstream state.
-- Requires load_state_filename to point at a mapping output.

include "feedbot_2d_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
