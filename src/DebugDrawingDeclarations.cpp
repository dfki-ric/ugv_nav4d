#ifdef ENABLE_DEBUG_DRAWINGS

#include <vizkit3d_debug_drawings/DebugDrawing.hpp>

// planner debug channels
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_obstacle_error");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_env_startPos");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_env_goalPos");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_expandStarts");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_successors");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_allowedAngles");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_expandFailStepHeight");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_check_start_goal_start");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_check_start_goal_goal");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_obst_map");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_trajectory");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_greedyPath");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_check_fail_goal");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_obs_check_fail_goal");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_check_fail_start");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_obs_check_fail_start");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_goalBox");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_startBox");
V3DD_DECLARE_DEBUG_DRAWING_CHANNEL("ugv_nav4d_primitives");

#endif
