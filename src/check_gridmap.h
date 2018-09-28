#ifndef __check_gridmap_h__
#define __check_gridmap_h__

#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>

#include <gridmap_utils/gridmap.h>
#include <gridmap_utils/gridmap_util.h>
#include <interfaces/map3d_interface.h>

#include <lcmtypes/bot_core_planar_lidar_t.h>

#include <lcmtypes/ripl_goal_list_t.h>
#include <lcmtypes/ripl_navigator_plan_t.h>
#include <lcmtypes/ripl_navigator_status_t.h>
#include <lcmtypes/obs_rect_list_t.h>
#include <lcmtypes/obs_track_list_t.h>
#include <lcmtypes/ripl_failsafe_t.h>

#include <lcmtypes/bot2_core.h>

#include "message_buffer.h"


#ifdef __cplusplus
extern "C" {
#endif

#define CHECK_GRIDMAP_RENDER_OBS_CHECK 1
#define CHECK_GRIDMAP_RENDER_DEBUG 2

#define NUM_OBST_LUTS 6
typedef struct {
    lcm_t *lcm;
    bot_lcmgl_t *lcmgl;
    bot_lcmgl_t *lcmgl_debug;
    bot_lcmgl_t *lcmgl_obs_check;
    BotParam *param;
    BotFrames *frames;
    int64_t tile_generation;
    gboolean paused;
    gboolean sensing_only_local;
    gboolean sensing_only_small;
    gboolean publish_gridmap;
    gboolean clear_person;
    gboolean ignore_local;

    bot_core_planar_lidar_t *front_laser;
    bot_core_planar_lidar_t *rear_laser;

    // sources for gridmap
    // "new" variants are the most recently received message.
    // non-"new" variants are those sampled from "new" currently
    // being used by the render thread.
    message_buffer_t    *nav_status_buffer;
    message_buffer_t    *nav_plan_buffer;
    message_buffer_t    *rects_buffer;
    message_buffer_t    *sim_rects_buffer;
    message_buffer_t    *map_rects_buffer;
    message_buffer_t    *tracks_buffer;
    message_buffer_t    *sim_tracks_buffer;
    message_buffer_t    *sim_lanes_buffer;
    message_buffer_t    *goals_buffer;
    message_buffer_t    *person_buffer;

    // used to create vehicle footprint
    double overall_width;
    double overall_length;

    // when testing the gridmaps, what offsets from the front axle
    // should we use? (positive = towards front, negative = towards
    // back)
    int                  convolve_line_searches;     // how many line searches?
    double               convolve_total_width;       // total footprint width
    double               convolve_radius;            // invariant: convolve_total_width / convolve_line_searches / 2.0

    // gridmap sizes (set to values specified in BotParam if available, else #define in check_gridmap.c)
    double               range;                      // width and height of the gridmap
    double               forward_offset;             // how far ahead of robot to center gridmap
    double               resolution;                 // gridmap resolution

    double               obsmap_offsets[2];

    // rate at which uncertain lanes dilate

    gridmap_lut_t       *obst_luts[NUM_OBST_LUTS]; // e.g. rects
    gridmap_lut_t       *gridmap_lut;
    gridmap_lut_t       *self_feasibility_lut;     // used to carve out infeasibility on our vehicle

    // we use two threads: one is the glib thread and listens to goals
    // and obstacles, and prepares gridmaps. The second thread processes
    // another problem. The mutex must be acquired when:
    //
    // x1) Switching the two buffers (to start a new problem)
    // 2) Putting new data into the "preparing" problem.
    pthread_mutex_t      obs_gridmap_mutex;
    pthread_mutex_t      global_map_gridmap_mutex;

    pthread_cond_t       signal_render;
    int                  signal_render_count;
    pthread_mutex_t      signal_render_mutex;
    pthread_t            render_thread;

    gridmap_t            *obsmap, *new_obsmap;

    int64_t              last_gridmap_send_utime;

    int                  failsafe;
    double               failsafe_timer;
    int64_t              failsafe_utime;

    ripl_map_t global_map;
    uint8_t *complete_global_map;

} check_gridmap_t;

//=========================================================================
// one-time init/cleanup
check_gridmap_t * check_gridmap_create_laser(const int constraints, gboolean render_traj,
                                       gboolean sensing_only_local,
                                       gboolean sensing_only_small,
                                       gboolean publish_gridmap,
                                       gboolean clear_person,
                                       gboolean ignore_local,
                                       gboolean clear_using_laser,
                                       double width_buffer);

check_gridmap_t * check_gridmap_create(const int constraints, gboolean render_traj,
                                       gboolean sensing_only_local, gboolean sensing_only_small,
                                       gboolean publish_gridmap,
                                       gboolean clear_person,
                                       gboolean ignore_local,
                                       double width_buffer);

void check_gridmap_destroy(check_gridmap_t *cg);

// start stop processing
int check_gridmap_on(check_gridmap_t *cg);
int check_gridmap_off(check_gridmap_t *cg);


// called when you want to switch to the most recent gridmap
int check_gridmap_update(check_gridmap_t *cg);

struct check_path_result
{
    double cost;          // line integral of max(obs_cost, lane_cost)
    double restricted;    // line integral of or(obs_restricted, lane_restricted)
    double max;           // maximum along line of max(obs_cost, lane_cost)

    double obs_cost;      // obs_cost along query
    double obs_restricted;
    double obs_max;       // maximum instantaneous cost for obstacles
};

// compatibility wrapper
//
// failsafe - when failsafe >= 1, reduce vehicle width by failsafe * FAILSAFE_WIDTH_FACTOR
void
check_gridmap_check_path (check_gridmap_t *self, int is_forward, int failsafe,
                          double x0, double y0, double theta0,
                          double x1, double y1, double theta1,
                          struct check_path_result *res);

void check_gridmap_check_path_ex(check_gridmap_t *self,
                                 gridmap_t *gm,
                                 double x0, double y0, double theta, double distance,
                                 double xmin, double xmax,
                                 double w0, double wstep_size, int wsteps,
                                 double *cost, double *restricted, double *max);

int check_gridmap_check_point(check_gridmap_t *self,
                              double x0, double y0);


void check_gridmap_check_speed(check_gridmap_t *self, int is_forward,
			       double x0, double y0, double theta0,
			       double x1, double y1, double theta1,
			       double *max_cost, double *speed_limit);

int check_gridmap_get_slower_region(check_gridmap_t *self,
                                    double *xy,
                                    double *range, double *speed_limit);


double
check_gridmap_convolve_radius(check_gridmap_t *self);


#ifdef __cplusplus
}
#endif

#endif
