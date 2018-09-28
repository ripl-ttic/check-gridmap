#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <glib.h>
#include <errno.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/obstacle_detector_lcmtypes.h>

#include "message_buffer.h"
#include "check_gridmap.h"

#define SELF_FEASIBILITY_COST    252

///////////////////////////////////////////////////////////////////

#define GLOBAL_MAP_TIMEOUT 30

// These aren't used
#define TWO_WAY_DILATION_RATIO 0.1
#define TWO_WAY_DILATION_MAX 9

// These aren't used
#define ONE_WAY_DILATION_RATIO 0.2
#define ONE_WAY_DILATION_MAX 13.5

// These aren't used
#define FAILSAFE_DILATION_RATIO 0.5
#define FAILSAFE_DILATION_MIN 5
#define FAILSAFE_DILATION_MAX 15

// Whan check_gridmap_check_path is called with failsafe >= 1, the vehicle
// width is reduced by failsafe*FAILSAFE_WIDTH_FACTOR
#define FAILSAFE_WIDTH_FACTOR 0.1

// WANNA SEE EVERY ONE? make it much bigger than 10
// (WILL ONLY SEND AS FAST AS RENDER THREAD)
#define GRIDMAP_SEND_HZ   20.0

// Parameters for full- and small-size gridmaps
// These settings are overridden by those set in BotParam file (if any)
#define GRIDMAP_RANGE       20.0
#define GRIDMAP_FORWARD_OFFSET  5.0
#define GRIDMAP_RESOLUTION   0.05//0.10

#define GRIDMAP_RANGE_SMALL       6.0 //4.0
#define GRIDMAP_FORWARD_OFFSET_SMALL  1.0
#define GRIDMAP_RESOLUTION_SMALL   0.05//0.10


// Reduce the offset that accounts for the distance from the
// body origin to the back/front for forward/reverse motion
// Only comes into play when failsafe>1
#define SKIP_CHECK_LENGTH_TAIL 0.0  // [m] <-- can be always as big
#define SKIP_CHECK_LENGTH_HEAD 0.0  // [m] <-- likely to hit obstacles if large

#define STATIONARY_FRONT_STANDOFF 5.0
#define TRACK_FORWARD_PROPAGATION_TIME 2.0
#define TRACK_ZONE_PROPAGATION_TIME 2.0

#define CARVE_LOCAL_RADIUS_SIZE 3.0 //5.0 //3.0
#define OBS_THRESHOLD 1
#define CARVE_LOCAL_RESOLUTION 36

#define SQ(x) ((x)*(x))

int get_local_pos (BotFrames *frames, double pos[3])
{
   double pos_body[3] = { 0, 0, 0 };
   return bot_frames_transform_vec(frames, "body", "local", pos_body, pos);
}

// Added from original timestamp.c, which is currently not in the libbot
// version of timestamp.c
static void timestamp_to_timespec(int64_t v, struct timespec *ts)
{
    ts->tv_sec  = bot_timestamp_seconds(v);
    ts->tv_nsec = bot_timestamp_useconds(v)*1000;
}


static void request_render(check_gridmap_t *self)
{
    pthread_mutex_lock(&self->signal_render_mutex);
    self->signal_render_count++;
    pthread_mutex_unlock(&self->signal_render_mutex);
}

static void render_request_wait(check_gridmap_t *self, int64_t timeout_us)
{
    pthread_mutex_lock(&self->signal_render_mutex);

    if (self->signal_render_count == 0) {
        double utime_timeout = bot_timestamp_now() + timeout_us;
        struct timespec ts;
        timestamp_to_timespec(utime_timeout, &ts);

        pthread_cond_timedwait(&self->signal_render, &self->signal_render_mutex, &ts);
    }

    self->signal_render_count = 0;

    pthread_mutex_unlock(&self->signal_render_mutex);
}

void on_planar_lidar(const lcm_recv_buf_t *rbuf, const char * channel,
                     const bot_core_planar_lidar_t * msg, void * user)
{
    check_gridmap_t *s = (check_gridmap_t *) user;

    if(!strcmp(channel, "SKIRT_FRONT")){
        if (s->front_laser != NULL)
            bot_core_planar_lidar_t_destroy(s->front_laser);
        s->front_laser = bot_core_planar_lidar_t_copy(msg);
    }

    if(!strcmp(channel, "SKIRT_REAR")){
        if (s->rear_laser != NULL)
            bot_core_planar_lidar_t_destroy(s->rear_laser);
        s->rear_laser = bot_core_planar_lidar_t_copy(msg);
    }
}

polygon2d_t *free_lidar_poly(check_gridmap_t *s, double max_radius){

    //go through the front laser points and create a polygon point list
    if(!s->front_laser && !s->rear_laser)
        return NULL;

    int skip_points = 3;

    int size = s->front_laser->nranges / skip_points;

    polygon2d_t *poly = g_slice_new(polygon2d_t);
    poly->nlists = 1;

    pointlist2d_t *plist =  pointlist2d_new(size);

    poly->pointlists = plist;

    bot_core_planar_lidar_t *laser = s->front_laser;

    double front_sensor_to_body[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (s->frames, "SKIRT_FRONT",
                                                  "body", laser->utime,
                                                  front_sensor_to_body)) {
        fprintf (stderr, "Error getting bot_frames transformation from SKIRT_FRONT to body!\n");
        polygon2d_free(poly);
        return NULL;
    }

    double body_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (s->frames, "body",
                                                  "local", laser->utime,
                                                  body_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from body to local!\n");
        polygon2d_free(poly);
        return NULL;

    }

    int count = 0;

    for(int i=0; i < laser->nranges; i +=skip_points){
        double theta = laser->rad0 +
            (double)i * laser->radstep;

        double pos[3] = {.0,.0,.0};

        double r = laser->ranges[i];

        pos[0] = r*cos(theta);
        pos[1] = r*sin(theta);

        double b_pos[3] = {.0,.0,.0};
        bot_vector_affine_transform_3x4_3d (front_sensor_to_body, pos, b_pos);

        double dist = hypot(b_pos[0], b_pos[1]);

        //might want to fill these as well
        if(dist < 0.5 || dist > max_radius){
            continue;
            /*r = max_radius;

            pos[0] = r*cos(theta);
            pos[1] = r*sin(theta);

            bot_vector_affine_transform_3x4_3d (front_sensor_to_body, pos, b_pos);  */
        }

        double l_pos[3] = {.0,.0,.0};
        bot_vector_affine_transform_3x4_3d (body_to_local, b_pos, l_pos);

        plist->points[count].x = l_pos[0];
        plist->points[count].y = l_pos[1];
        count++;
    }

    pointlist2d_resize(plist, count);

    //fprintf(stderr,"Orig : %d Size : %d\n", size, count);
    return poly;
}


// Defines a polygon approximation to a half circle of radius max_radius
// for LIDAR with coordinate frame given by name
polygon2d_t *free_semicircle_poly(check_gridmap_t *s, char *name, double max_range){

    double fov = 180;
    double dtheta = 1;
    int size = floor (180/1 + 1);
    polygon2d_t *poly = g_slice_new(polygon2d_t);
    poly->nlists = 1;

    pointlist2d_t *plist =  pointlist2d_new(size);

    poly->pointlists = plist;

    double lidar_to_body[12];
    if (!bot_frames_get_trans_mat_3x4 (s->frames, name,
                                       "body", lidar_to_body)) {
        fprintf (stderr, "Error getting bot_frames transformation from %s to body!\n", name);
        polygon2d_free(poly);
        return NULL;
    }

    double body_to_local[12];
    if (!bot_frames_get_trans_mat_3x4 (s->frames, "body",
                                                  "local", body_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from body to local!\n");
        polygon2d_free(poly);
        return NULL;

    }

    double theta = 0;
    for(int i=0; i < size; i++){

        double pos[3] = {.0,.0,.0};

        pos[0] = max_range*cos(bot_to_radians(theta));
        pos[1] = max_range*sin(bot_to_radians(theta));

        double b_pos[3] = {.0,.0,.0};
        bot_vector_affine_transform_3x4_3d (lidar_to_body, pos, b_pos);

        double l_pos[3] = {.0,.0,.0};
        bot_vector_affine_transform_3x4_3d (body_to_local, b_pos, l_pos);

        plist->points[i].x = l_pos[0];
        plist->points[i].y = l_pos[1];
        theta += i*dtheta;
    }

    //pointlist2d_resize(plist, count);

    return poly;
}


int get_local_pose(check_gridmap_t *self, bot_core_pose_t *pose)
{

    BotTrans *result = calloc(1,sizeof(BotTrans));
    int status = bot_frames_get_trans(self->frames, "body", "local", result);

    // Get position.
    //double *pos = (double*) calloc (3, sizeof(double));
    //memcpy(pos, &result->trans_vec, sizeof(double)*3);
    pose->pos[0] = result->trans_vec[0];
    pose->pos[1] = result->trans_vec[1];
    pose->pos[2] = result->trans_vec[2];

    pose->orientation[0] = result->rot_quat[0];
    pose->orientation[1] = result->rot_quat[1];
    pose->orientation[2] = result->rot_quat[2];
    pose->orientation[3] = result->rot_quat[3];

    free(result);

    return status;
}



int get_local_pos_heading(check_gridmap_t *self, double pos[3], double *heading)
{

  BotTrans *result = calloc(1,sizeof(BotTrans));
    int status = bot_frames_get_trans(self->frames, "body", "local", result);

    // Get position.
    memcpy(pos, result->trans_vec, 3 * sizeof(double));

    // Get heading.
    double rpy[3];
    bot_quat_to_roll_pitch_yaw(result->rot_quat, rpy);
    *heading = rpy[2];
    free(result);
    return status;
}

//------------------------------------------
//--- LCM CALLBACKS
//------------------------------------------
static void
on_goals(const lcm_recv_buf_t * rbuf, const char *channel,
         const ripl_goal_list_t *msg, void *user)
{
    check_gridmap_t *self = (check_gridmap_t*) user;
    message_buffer_give(self->goals_buffer, ripl_goal_list_t_copy(msg));
}

static
void guide_pos_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
		       const ripl_guide_info_t * msg,
		       void * user  __attribute__((unused)))
{
    check_gridmap_t *self = (check_gridmap_t*) user;
    message_buffer_give(self->person_buffer, ripl_guide_info_t_copy(msg));
}

// static void
// on_nav_plan(const lcm_recv_buf_t * rbuf, const char *channel,
//             const ripl_navigator_plan_t *msg, void *user)
// {
//     check_gridmap_t *self = (check_gridmap_t*) user;
//     message_buffer_give(self->nav_plan_buffer, ripl_navigator_plan_t_copy(msg));
// }

// static void
// on_nav_status(const lcm_recv_buf_t * rbuf, const char *channel,
//               const ripl_navigator_status_t *msg, void *user)
// {
//     check_gridmap_t *self = (check_gridmap_t*) user;
//     message_buffer_give(self->nav_status_buffer, ripl_navigator_status_t_copy(msg));
// }

static void
on_sim_rects(const lcm_recv_buf_t * rbuf, const char *channel,
             const obs_rect_list_t *msg, void *user)
{
    check_gridmap_t *self = (check_gridmap_t*) user;
    if (!strcmp (channel, "MAP_SERVER_RECTS"))
        message_buffer_give(self->map_rects_buffer, obs_rect_list_t_copy(msg));
    else
        message_buffer_give(self->sim_rects_buffer, obs_rect_list_t_copy(msg));
}

static void
on_obstacles(const lcm_recv_buf_t * rbuf, const char *channel,
             const obs_obstacle_list_t * msg, void * user)
{
    check_gridmap_t *self = (check_gridmap_t*) user;
    message_buffer_give(self->rects_buffer, obs_rect_list_t_copy(&msg->rects));
    //message_buffer_give(self->tracks_buffer, obs_track_list_t_copy(&msg->tracks));

    request_render(self);
}

static void
on_failsafe(const lcm_recv_buf_t * rbuf, const char *channel,
            const ripl_failsafe_t *msg, void *user)
{
    check_gridmap_t *self = (check_gridmap_t*) user;

    if (msg->mode != self->failsafe) {
        // edge-sensitive stuff goes here
    }

    double dt = (msg->utime - self->failsafe_utime)/1000000.0;
    self->failsafe = msg->mode;
    self->failsafe_utime = msg->utime;
    if (msg->timer >= self->failsafe_timer)
        self->failsafe_timer = fmin(msg->timer,50.0);
    else if (dt > 0)
        self->failsafe_timer = fmax(0, self->failsafe_timer - dt);
}


//------------------------------------------
//--- Utilities
//------------------------------------------
struct lineseg
{
  double xy0[2], xy1[2];
};

//-------------------------------------------------
//-------------------------------------------------
// Static obstacles
static void draw_obs_gridmap(check_gridmap_t *self, gridmap_t *gm)
{

    double pos[3], heading;
    if (!get_local_pos_heading(self, pos, &heading))
        return;

    //ripl_navigator_status_t *nav_status = message_buffer_get(self->nav_status_buffer);

    // Find goal position
    //ripl_navigator_plan_t *nav_plan = message_buffer_get(self->nav_plan_buffer);


    int64_t ti = bot_timestamp_now();

    int obst_lut_idx = 2; // amply enlarged
    if (self->failsafe_timer > 10)
        obst_lut_idx = 1; // slightly enlarged
    if (self->failsafe_timer > 20)
        obst_lut_idx = 0; // actual size


    bot_core_pose_t bot_pose;

    if (!get_local_pose (self, &bot_pose))
        return;

    ripl_guide_info_t * p_msg = NULL;

    double person_pos[2] = {0,0};

    gboolean clear_person = FALSE;

    if(self->clear_person){
        p_msg = message_buffer_get(self->person_buffer);

        if(p_msg && p_msg->tracking_state == 1){ //we have a track of the person
            clear_person = TRUE;
            double bot_rpy[3];
            bot_quat_to_roll_pitch_yaw (bot_pose.orientation, bot_rpy);

            //calculate the person's position
            person_pos[0] = bot_pose.pos[0] + p_msg->pos[0] * cos(bot_rpy[2]) -  p_msg->pos[1] * sin(bot_rpy[2]);

            person_pos[1] = bot_pose.pos[1] +  p_msg->pos[0] * sin(bot_rpy[2]) +
                p_msg->pos[1] * cos(bot_rpy[2]);
        }
    }
    //-------------------------
    //--- Static rects
    obs_rect_list_t *rects = message_buffer_get(self->rects_buffer);

    for (int i=0; rects && i<rects->num_rects;i++) {
        obs_rect_t *rect = &rects->rects[i];
        double pos_r[2]={rects->xy[0]+rect->dxy[0],rects->xy[1]+rect->dxy[1]};
        double size[2]={rect->size[0],rect->size[1]};

        double dist = hypot(pos_r[0] - bot_pose.pos[0],
                            pos_r[1] - bot_pose.pos[1]);
        //if(dist > CARVE_LOCAL_RADIUS_SIZE + OBS_THRESHOLD){
        //    fprintf (stdout, "dist = %.2f\n", dist);
        //    continue;
        //}

        //calculate the distance - to the center???
        if(clear_person){
            double p_dist = hypot(pos_r[0] - person_pos[0],
                                  pos_r[1] - person_pos[1]);

            if(p_dist < 0.7){
                continue;
            }

        }
        // render actual obstacle
        gridmap_render_rectangle_restricted(gm, pos_r[0], //rects->xy[0]+rect->dxy[0],
                                            pos_r[1],//rects->xy[1]+rect->dxy[1],
                                            rect->size[0], rect->size[1],
                                            rect->theta,
                                            self->obst_luts[obst_lut_idx]);


    }

    //-------------------------
    //--- Sim rects
    // can't guarentee sim_rects have theta =0

    rects = message_buffer_get(self->sim_rects_buffer);

    for (int i=0; rects && i<rects->num_rects;i++) {
        obs_rect_t *rect = &rects->rects[i];
        double pos_r[2]= {rects->xy[0]+rect->dxy[0],rects->xy[1]+rect->dxy[1]};
        double size[2]={rect->size[0],rect->size[1]};

        //calculate the distance - to the center???
        if(clear_person){
            double p_dist = hypot(pos_r[0] - person_pos[0],
                                  pos_r[1] - person_pos[1]);

            if(p_dist < 0.5){
                continue;
            }

        }

        // render actual obstacle
        gridmap_render_rectangle_restricted(gm, pos_r[0], pos_r[1], size[0], size[1],
                                            rect->theta,
                                            self->obst_luts[obst_lut_idx]);
    /*int ix0, iy0;
    gridmap_to_ix_iy(gm, pos[0], pos[1], &ix0, &iy0);

    fprintf(stderr,"Obs Value : %d\n", gm->data[iy0*gm->width + ix0]);
    */
    }

    //-------------------------
    //--- Sim rects included in the map
    //
    // These are reported in the GLOBAL reference frame.
    // can't guarentee sim_rects have theta =0

    rects = message_buffer_get(self->map_rects_buffer);

    // Transform fromt GLOBAL to LOCAL coordinate frame
    for (int i=0; rects && i<rects->num_rects;i++) {
        obs_rect_t *rect = &rects->rects[i];
        double pos_local[3], rpy_local[3], quat_local[4];
        double pos_global[3]= {rects->xy[0]+rect->dxy[0],rects->xy[1]+rect->dxy[1], 0};
        double size_global[3]={rect->size[0],rect->size[1], 0};
        double rpy_global[3] = {0, 0, rect->theta};
        double quat_global[4];

        // Determine the position and size in the local frame
        bot_frames_transform_vec (self->frames, "global", "local", pos_global, pos_local);
        //no need to transform the size

        // Determine the orientation
        BotTrans global_to_local;
        bot_roll_pitch_yaw_to_quat (rpy_global, quat_global);
        bot_frames_get_trans (self->frames, "global", "local", &global_to_local);
        bot_quat_mult (quat_local, global_to_local.rot_quat, quat_global);
        bot_quat_to_roll_pitch_yaw (quat_local, rpy_local);

        // render actual obstacle
        gridmap_render_rectangle_restricted(gm, pos_local[0], pos_local[1], size_global[0], size_global[1],
                                            rpy_local[2],
                                            self->obst_luts[obst_lut_idx]);
    }


    //-------------------------
    //--- tracks
    if(!self->clear_person){
        obs_track_list_t *tracks = message_buffer_get(self->tracks_buffer);
        for (int i = 0; tracks && i < tracks->ntracks; i++) {

            //not sure what will happen here

            obs_track_t *track = &tracks->tracks[i];
            // render actual obstacle
            gridmap_render_rectangle_restricted(gm, track->pos[0], track->pos[1],
                                                track->size[0], track->size[1],
                                                track->theta,
                                                self->obst_luts[0]);
        }

        //-------------------------
        //--- sim tracks
        obs_track_list_t *simtracks = message_buffer_get(self->sim_tracks_buffer);
        for (int i = 0; simtracks && i < simtracks->ntracks; i++) {
            obs_track_t *track = &simtracks->tracks[i];
            // render actual obstacle
            gridmap_render_rectangle_restricted(gm, track->pos[0], track->pos[1],
                                                track->size[0], track->size[1],
                                                track->theta,
                                                self->obst_luts[0]);
        }
    }

    // make the vehicle's current position feasible (to clip overlapping objects.)
    if (1) {
        double s, c;
        bot_fasttrig_sincos(heading, &s, &c);
        double length_pad = 0.35;
        double vmid = (self->obsmap_offsets[0] + self->obsmap_offsets[1])/2.0;
        double vsz = fabs(self->obsmap_offsets[0] - self->obsmap_offsets[1]) + self->convolve_radius*2 + length_pad;
        gridmap_render_rectangle_min(gm, pos[0] + c*vmid, pos[1] + s * vmid,
                                            vsz, self->convolve_total_width - self->convolve_radius*2 + .2, heading,self->self_feasibility_lut);
    }

    if (self->failsafe_timer>35) {
        // Unrestrict goal
        // if the goal is covered in restricted but we can otherwise go we should unrestrict the goal region.
        ripl_goal_list_t *goals = message_buffer_get(self->goals_buffer);
        if (goals&&goals->num_goals) {
            ripl_goal_t *goal = &goals->goals[0];
            double s, c;
            bot_fasttrig_sincos(goal->theta, &s, &c);
            gridmap_render_rectangle_min_keep_infeasible(gm,
                                                         goal->pos[0]+c*-self->overall_length/2,
                                                         goal->pos[1]+s*-self->overall_length/2,
                                                         self->overall_length,
                                                         fmax(self->overall_width,goal->size[1]),
                                                         goal->theta,
                                                         self->self_feasibility_lut);
        }
    }


    //-------------------------
    double t = (bot_timestamp_now()-ti)/1.0e6;
    if (t > 0.100)
        printf("render_estop: ti:%lf\n",t);
}

// static void send_velocity_advice(check_gridmap_t *self)
// {
//
//     ripl_navigator_plan_t *nav_plan = message_buffer_get(self->nav_plan_buffer);
//     ripl_navigator_status_t *nav_status = message_buffer_get(self->nav_status_buffer);
//     if (!nav_plan || !nav_status)
//         return;
// }


// 1. fill
// 2. carve zones
// 3. carve lanes (or clines)
// 4. elastic
// 5. erect zone fences
// 6. erect curbs
// 7. carve intersections
// 8. draw parking stops


//add map handler that takes in the wheelchair gridmap
static void on_map_server_gridmap_rect(const lcm_recv_buf_t *rbuf, const char *channel,
                                  const ripl_gridmap_t *msg, void *user)
{

    check_gridmap_t *self = (check_gridmap_t *)user;
    pthread_mutex_lock (&self->global_map_gridmap_mutex);

    if (self->global_map.map != NULL)
        free(self->global_map.map);
    if (self->global_map.complete_map != NULL) {
        free(self->global_map.complete_map);
        self->global_map.complete_map = NULL;
    }
    carmen3d_map_uncompress_lcm_map(&(self->global_map), msg);

    //copy over to the int array - in the manner of the gridmap.h definition
    if(self->complete_global_map != NULL){
        free(self->complete_global_map);
        self->complete_global_map = NULL;
    }

    self->complete_global_map = (uint8_t *)calloc(1, self->global_map.config.y_size * self->global_map.config.x_size * sizeof (uint8_t));
    double cx, cy;

    carmen3d_map3d_map_index_to_global_coordinates(&cx, &cy, self->global_map.midpt,
                                                   self->global_map.map_zero,
                                                   self->global_map.config.resolution,
                                                   self->global_map.config.x_size/2,
                                                   self->global_map.config.y_size/2);

    gridmap_t *new_testmap = gridmap_create_fill(cx,
                                                 cy,
                                                 self->global_map.config.x_size * self->global_map.config.resolution,
                                                 self->global_map.config.y_size * self->global_map.config.resolution,
                                                 self->global_map.config.resolution,
                                                 0);

    int count = 0;

    /*int i = self->global_map.config.x_size/2;
    int j = self->global_map.config.y_size/2;


    double x,y;
    carmen3d_map3d_map_index_to_global_coordinates(&x, &y, self->global_map.midpt,
                                                   self->global_map.map_zero,
                                                   self->global_map.config.resolution,
                                                   i, j);

    gridmap_render_rectangle_restricted(new_testmap, x, y,
                                        self->global_map.config.resolution,
                                        self->global_map.config.resolution,
                                        0,
                                        self->gridmap_lut);*/

    for(int i=0; i < self->global_map.config.x_size; i++){
        for(int j=0; j< self->global_map.config.y_size; j++){
            if(self->global_map.map[i][j] > 0.7){

                double x,y;

                gridmap_to_x_y(new_testmap, i,j, &x,&y);
                /*                carmen3d_map3d_map_index_to_global_coordinates(&x, &y, self->global_map.midpt,
                                                               self->global_map.map_zero,
                                                               self->global_map.config.resolution,
                                                               i, j); */
                gridmap_render_rectangle_restricted(new_testmap, x, y,
                                                    self->global_map.config.resolution,
                                                    self->global_map.config.resolution,
                                                    0,
                                                    //self->obst_luts[1]);
                                                    self->gridmap_lut);
                count++;

            }

            if(self->global_map.map[i][j] == 0.5){
                double x,y;
                /*carmen3d_map3d_map_index_to_global_coordinates(&x, &y, self->global_map.midpt,
                                                               self->global_map.map_zero,
                                                               self->global_map.config.resolution,
                                                               i, j); */
                gridmap_to_x_y(new_testmap, i,j, &x,&y);
                //fprintf(stderr, "Drawing :%d,%d => %f, %f\n", i, j, x, y);
                gridmap_render_rectangle_restricted(new_testmap, x, y,
                                                    self->global_map.config.resolution,
                                                    self->global_map.config.resolution,
                                                    0,
                                                    //self->obst_luts[1]);
                                                    self->gridmap_lut);
                count++;

            }
        }
        }
        memcpy(self->complete_global_map, new_testmap->data, self->global_map.config.y_size * self->global_map.config.x_size * sizeof (uint8_t));

        /*
    for(int i=0; i < self->global_map.config.x_size; i++){
        for(int j=0; j< self->global_map.config.y_size; j++){
            self->complete_global_map[j * self->global_map.config.x_size + i]
	        = (uint8_t) (self->global_map.map[i][j]* 255);
        }
    }
    */




    pthread_mutex_unlock (&self->global_map_gridmap_mutex);
    gridmap_destroy(new_testmap);

    fprintf(stderr,"Acquired gridmap\n");
}

//add map handler that takes in the wheelchair gridmap
static void on_map_server_gridmap(const lcm_recv_buf_t *rbuf, const char *channel,
                                  const ripl_gridmap_t *msg, void *user)
{

    check_gridmap_t *self = (check_gridmap_t *)user;
    pthread_mutex_lock (&self->global_map_gridmap_mutex);

    if (self->global_map.complete_map != NULL) {
        free(self->global_map.complete_map);
        self->global_map.complete_map = NULL;
    }
    if (self->global_map.map != NULL){
        free(self->global_map.map);
        self->global_map.map = NULL;
    }

    carmen3d_map_uncompress_lcm_map(&(self->global_map), msg);


    //copy over to the int array - in the manner of the gridmap.h definition
    if(self->complete_global_map != NULL){
        free(self->complete_global_map);
        self->complete_global_map = NULL;
    }

    self->complete_global_map = (uint8_t *)calloc(1, self->global_map.config.y_size * self->global_map.config.x_size * sizeof (uint8_t));

    for(int i=0; i < self->global_map.config.x_size; i++){
        for(int j=0; j< self->global_map.config.y_size; j++){
            //blur it
            double value = 0;
            if(self->global_map.map[i][j] > 0.6 || self->global_map.map[i][j] == 0.5){
                value = 1.0;
            }

            self->complete_global_map[j * self->global_map.config.x_size + i]
                = (uint8_t)(value * 255); //(self->global_map.map[i][j] * 255);
        }
    }

    pthread_mutex_unlock (&self->global_map_gridmap_mutex);


    fprintf(stderr,"Acquired gridmap\n");
}

int get_global_map(check_gridmap_t *self)
{
    /* subscripe to map, and wait for it to come in... */
    ripl_map_request_msg_t msg;
    msg.utime =  carmen_get_time()*1e6;
    msg.requesting_prog = "CHECK_GRIDMAP";

    if(0){
        //we are not using the rects method for now - seems to introduce a wierd offset
        ripl_gridmap_t_subscribe(self->lcm, "MAP_SERVER", on_map_server_gridmap_rect, self);
    }
    else{
        ripl_gridmap_t_subscribe(self->lcm, "MAP_SERVER", on_map_server_gridmap, self);
    }
    int sent_map_req = 0;

    int64_t c_utime = bot_timestamp_now();

    while (self->global_map.map == NULL) {
        if(!sent_map_req){
            sent_map_req = 1;
            ripl_map_request_msg_t_publish(self->lcm,"MAP_REQUEST_CHANNEL",&msg);
        }
        //sleep();
        fprintf(stdout, ".");
        lcm_handle(self->lcm);

        int64_t e_utime = bot_timestamp_now();

        double time = (e_utime - c_utime) / 1.0e6;

        if(time > GLOBAL_MAP_TIMEOUT){
            fprintf(stderr, "Global map timed out\n");
            return -1;
        }
    }
    fprintf(stderr,"Got Global Map\n");
    return 0;
    //we should convert this because of row major/column major issue
}

static void *render_thread(void *user)
{
    check_gridmap_t *self = (check_gridmap_t *) user;

    while (1) {
        while (self->paused) {
            usleep(1000);
        }
        // wait up to 130 ms
        //render_request_wait(self, 130000);
        render_request_wait(self, 5000);

        // place center of the gridmap in front of the vehicle.
        // we don't need to render 100m behind us.
        double forward_pos[3] = {self->forward_offset, 0.0, 0.0};
        bot_core_pose_t bot_pose;

        if (!get_local_pose (self, &bot_pose))
            continue;

        bot_quat_rotate (bot_pose.orientation, forward_pos);
        forward_pos[0] += bot_pose.pos[0];
        forward_pos[1] += bot_pose.pos[1];
        forward_pos[2] += bot_pose.pos[2];

        if (self->lcmgl_obs_check){
            bot_lcmgl_switch_buffer(self->lcmgl_obs_check);
        }

        // create gridmap for static scene: contains rects.
        // the two gridmaps must be exactly aligned.
        gridmap_t *new_obsmap = gridmap_create_fill(forward_pos[0], forward_pos[1],
                                                    2 * self->range, 2 * self->range,
                                                    self->resolution, 0);

        // Incorporate the global map if we are maintaining a full gridmap (i.e., not sensing_only_small mode)
        if (self->sensing_only_small == FALSE) {
            //now set the values from the global map
            //conver the center to ints in both maps
            //go over the width and length of the filled map
            //and transfer the float prob to int values
            pthread_mutex_lock (&self->global_map_gridmap_mutex);

            if(self->global_map.map !=NULL){

                // convert to center from local to global
                double c_local[] = {new_obsmap->cx, new_obsmap->cy, forward_pos[2]};
                double c_global[3];
                bot_frames_transform_vec (self->frames, "local", "global", c_local, c_global);
                //carmen_point_t c_pt = {new_obsmap->cx, new_obsmap->cy, forward_pos[2]};
                carmen_point_t c_pt = {c_global[0], c_global[1], c_global[2]};

                int cx, cy;
                carmen3d_map3d_global_to_map_index_coordinates(c_pt, self->global_map.midpt, self->global_map.map_zero,
                                                               self->global_map.config.resolution, &cx, &cy);


                int x_min = fmax(0, cx - self->range / self->resolution);
                int x_max = fmin(self->global_map.config.x_size-1, cx + self->range / self->resolution);

                int y_min = fmax(0, cy - self->range / self->resolution);
                int y_max = fmin(self->global_map.config.y_size-1, cy + self->range / self->resolution);

                int ix_min = fmax(0, self->range / self->resolution - cx);
                int iy_min = fmax(0, self->range / self->resolution - cy);

                int ix_span = x_max - x_min;

                int ix = ix_min;
                int iy = iy_min;
                int i = x_min;

                for (int jl=0; jl < new_obsmap->height; jl++) {
                    for (int il=0; il < new_obsmap->width; il++) {
                        double x,y;
                        gridmap_to_x_y (new_obsmap, il, jl, &x, &y);
                        double xyz_local[] = {x, y, 0};
                        double xyz_global[3];
                        bot_frames_transform_vec (self->frames, "local", "global", xyz_local, xyz_global);

                        carmen_point_t xyz_pt = {xyz_global[0], xyz_global[1], xyz_global[2]};
                        int ig, jg;
                        carmen3d_map3d_global_to_map_index_coordinates (xyz_pt, self->global_map.midpt,
                                                                        self->global_map.map_zero,
                                                                        self->global_map.config.resolution, &ig, &jg);


                        if ((ig < 0) || (ig > (self->global_map.config.x_size-1))
                            || (jg < 0) || (jg > self->global_map.config.y_size-1))
                            continue;

                        double max_size = self->global_map.config.x_size * self->global_map.config.y_size;
                        double location = jg*self->global_map.config.x_size + ig;
                        if ((location < 0) || (location > max_size))
                            fprintf (stdout, "ERROR: Index into memory is %.2f!!!\n", location);

                        new_obsmap->data[jl*new_obsmap->width + il] =
                            self->complete_global_map[jg*self->global_map.config.x_size + ig];
                    }
                }
            }

            pthread_mutex_unlock (&self->global_map_gridmap_mutex);

            // Carve out a region of the SLAM occupancy map around the robot.
            /*if (self->sensing_only_local) {
                //fprintf(stderr," -------------------Carving out map \n");
                polygon2d_t *circle_poly = polygon2d_new_circle( (double) bot_pose.pos[0], (double) bot_pose.pos[1],
                                                                CARVE_LOCAL_RADIUS_SIZE, CARVE_LOCAL_RESOLUTION);

                gridmap_polygon_fill (new_obsmap, circle_poly, 0);
                polygon2d_free(circle_poly);
                }*/

             if (self->sensing_only_local) {
                 //fill a polygon - that goes up to the laser points

                 // This should be updated to operate on each LIDAR separately
                 // Carve out a polygon the geometry of which is dictated by
                 // the LIDAR returns (up to a certain range)
                 if(self->front_laser) {// && self->rear_laser){
                     //fprintf(stderr,"Finding clearing\n");
                     polygon2d_t *lidar_poly = free_lidar_poly(self, CARVE_LOCAL_RADIUS_SIZE);
                     gridmap_polygon_fill (new_obsmap, lidar_poly, 0);
                     polygon2d_free(lidar_poly);
                 }

                 else{
                     //fprintf(stdout," -------------------Carving out map \n");
                     polygon2d_t *semicircle_poly = free_semicircle_poly (self, "SKIRT_FRONT", CARVE_LOCAL_RADIUS_SIZE);
                     //polygon2d_t *circle_poly = polygon2d_new_circle( (double) bot_pose.pos[0], (double) bot_pose.pos[1],
                     //                                               CARVE_LOCAL_RADIUS_SIZE, CARVE_LOCAL_RESOLUTION);


                     //gridmap_polygon_fill (new_obsmap, circle_poly, 0);
                     //polygon2d_free(circle_poly);
                                                                      //if (semicircle_poly)
                     gridmap_polygon_fill (new_obsmap, semicircle_poly, 0);
                     polygon2d_free(semicircle_poly);
                 }
            }
        }

        // Populate the obstacle gridmap based upon perceived obstacles
        draw_obs_gridmap(self, new_obsmap);

        pthread_mutex_lock(&self->obs_gridmap_mutex);
        // if there was an un-consumed map, free it.
        if (self->new_obsmap){
            gridmap_destroy(self->new_obsmap);
        }
        self->new_obsmap = new_obsmap;

        pthread_mutex_unlock(&self->obs_gridmap_mutex);

        //// send obstacle map (gridmap)
        if (self->publish_gridmap) {
            int64_t now = bot_timestamp_now();
            double dt = (now - self->last_gridmap_send_utime)/1000000.0;
            if (dt >= 1.0 / GRIDMAP_SEND_HZ) {

                self->last_gridmap_send_utime = now;

                self->tile_generation++;
                gridmap_util_publish(new_obsmap, self->lcm, "OBSTACLE_MAP", &self->tile_generation, now);
            }

        }

        if (0) {
            // compute the feasibility of the current position and
            // print it out for debugging purposes
            double dbg_cost, dbg_restricted, dbg_max;
            double pos[3], heading;
            get_local_pos_heading(self, pos, &heading);

            check_gridmap_check_path_ex(self, new_obsmap, pos[0], pos[1], heading, 0,
                                        self->obsmap_offsets[0], self->obsmap_offsets[1],
                                        -self->convolve_total_width/2 + self->convolve_radius,
                                        2*self->convolve_radius,
                                        self->convolve_line_searches,
                                        &dbg_cost, &dbg_restricted, &dbg_max);

            printf("DBG_RESTRICTED: %15f %15f\n", dbg_restricted, dbg_cost);
        }

    }


    //check_gridmap_update(self);

    return NULL;
}

// use a new set of gridmaps if there are some waiting.  We guarantee
// that queries are consistent (return the same answer) between calls
// to _update.
int check_gridmap_update(check_gridmap_t *self) {

    pthread_mutex_lock(&self->obs_gridmap_mutex);

    if (self->new_obsmap !=NULL) {
        if (self->obsmap)
            gridmap_destroy(self->obsmap);
        self->obsmap=self->new_obsmap;
        self->new_obsmap = NULL;
    }

    pthread_mutex_unlock(&self->obs_gridmap_mutex);

    if (self->lcmgl_debug)
       bot_lcmgl_switch_buffer(self->lcmgl_debug);

    /*if (self->lcmgl_obs_check){
      fprintf(stderr,"Up\n");
      bot_lcmgl_switch_buffer(self->lcmgl_obs_check);
      }*/
    /*bot_lcmgl_t *lcmgl = bot_lcmgl_init(self->lcm, "RRT-CHECKGRID1");
    if (lcmgl)
    bot_lcmgl_switch_buffer(lcmgl);*/
    return 0;
}

// perform a line search between (xa,ya) and (xb,yb) offset by a
// distance w in direction (c, s). Return the maximum cost value along
// the line and set restricted to the maximum restricted value along
// the line (either 1 or 0).
static inline void do_line_search(check_gridmap_t *self, gridmap_t *gm,
                                 double xa, double ya, double xb, double yb,
                                 double s, double c, double w, int *max_cost, int *restricted)
{
    xa += w*c;
    ya += w*s;
    xb += w*c;
    yb += w*s;

    gridmap_test_line_restricted_worst(gm, xa, ya, xb, yb, max_cost, restricted);

    bot_lcmgl_t *lcmgl = self->lcmgl_debug;

    if (lcmgl) {
        if (*max_cost<0)
            lcmglColor4f(1, 0, 0, 0.2);
        else if (*max_cost==0) {
            lcmglColor4f(1,1,0,0.3);
        }
        else {
            //double v = rc/255.0;
            //lcmglColor3fv(color_util_jet(v));
            lcmglColor4f(0,0,1,0.3);
        }

        lcmglVertex3d(xa, ya, 0);
        lcmglVertex3d(xb, yb, 0);

        if (*restricted<=0)
            lcmglColor4f(0, 1, 0, 0.2);
        else {
            //double v = rc/255.0;
            //lcmglColor3fv(color_util_jet(v));
            lcmglColor4f(1,1,1,0.3);
        }
        lcmglVertex3d(xa+0.5, ya+0.5, 0);
        lcmglVertex3d(xb+0.5, yb+0.5, 0);
    }

}

int check_gridmap_check_point(check_gridmap_t *self,
                               double x0, double y0)
{
    // This function isn't really used, but it doesn't make sense to return 0
    // if that indicates cost/no collision
    if(!self->obsmap){
        return 0;
    }

    int ix0, iy0;
    gridmap_to_ix_iy(self->obsmap, x0, y0, &ix0, &iy0);

    // This function isn't really used, but it doesn't make sense to return 0
    // if that indicates cost/no collision
    if (ix0 < 0 || ix0 >= self->obsmap->width)
        return 0;
    if (iy0 < 0 || iy0 >= self->obsmap->height)
        return 0;

    int ix = (int) (ix0+0.5); // compute integer indices again
    int iy = (int) (iy0+0.5);

    uint8_t value = self->obsmap->data[iy*self->obsmap->width + ix];

    // else?????
    if(value > 100){
        return 1;
    }
}

// Perform a number of line queries, returning the worst distance.

// Note: xmax, xmin can be signed: positive values specify positions
// in front of the front axle, negative = behind.

// The motion specification creates a rectangle. This whole figure
// is rotated by theta.
//
//   |                                                                    |     w
//   |                                                                    |     i     theta
//   *|----xmin---|*|-------------- distance ---------------|*|---xmax---|*     d    ------->
//   |          (x0,y0)                                    (x1,y1)        |     t
//   |                                                                    |     h
// (xa,ya)                                                             (xb,yb)

void check_gridmap_check_path_ex(check_gridmap_t *self,
                                 gridmap_t *gm,
                                 double x0, double y0, double theta, double distance,
                                 double xmin, double xmax,
                                 double w0, double wstep_size, int wsteps,
                                 double *cost, double *restricted, double *max)
{

    if (!gm) {
        *cost = -1;
        *restricted = 1;
        *max = -1;
        return;
    }

    bot_lcmgl_t *lcmgl = self->lcmgl_debug;

    if (lcmgl) {
        lcmglColor4f(1, 1, 0, 0.2);
        lcmglEnable(GL_BLEND);
        lcmglBegin(GL_LINES);
    }

    // compute sin/cos for theta pointing along distance direction
    double s, c;
    bot_fasttrig_sincos(theta, &s, &c);

    double x1 = x0 + distance * c;
    double y1 = y0 + distance * s;

    double xa = x0 + (xmin * c);
    double ya = y0 + (xmin * s);

    double xb = x1 + (xmax * c);
    double yb = y1 + (xmax * s);

    // compute sin/cos for theta pointing along the width direction
    // (pointing up in the figure)
    bot_fasttrig_sincos(theta + M_PI/2, &s, &c);



    int worst_cost = 0, worst_restricted = 0;

    for (int step = 0; step < wsteps; step++) {

        double w = w0 + step*wstep_size;

        int this_max_cost = 0, this_restricted = 0;

        do_line_search(self, gm, xa, ya, xb, yb, s, c, w, &this_max_cost, &this_restricted);

        // shouldn't be necessary as the gridmap values are in [0, 255]
        if (this_max_cost < 0)
            this_max_cost = 255;

        worst_cost = bot_max(worst_cost, (this_max_cost&0xfe));

        worst_restricted = bot_max(worst_restricted, this_restricted);
    }

    if (worst_cost >= 254) {
        // infeasible
        *cost = -1;
        *max = -1;
    } else {
        *max = worst_cost;
        *cost = worst_cost * distance;
    }

    lcmgl = self->lcmgl_obs_check;

    if(lcmgl){
        lcmglBegin(GL_LINES);
        if (*cost<0) // infeasible. red
            lcmglColor4f(1, 0, 0, 0.2);
        else if (*cost==0) { // zero cost. yellow
            lcmglColor4f(1,1,0,0.3);
        }
        else {
            double v = *cost/255.0;
            lcmglColor4fv(bot_color_util_jet(v));
            //lcmglColor4f(0,0,1,0.3);
        }
        lcmglVertex3d(x0, y0, 0);
        lcmglVertex3d(x1, y1, 0);
    }

    *restricted = worst_restricted;

    if (lcmgl) {
        lcmglEnd();
    }

}


// compatibility wrapper
// car_front - distance from rear axle to front
// car_behind - distance from rear axle to rear
//
// failsafe - when failsafe >= 1, check reduced footprint (currently width)
void
check_gridmap_check_path (check_gridmap_t *self, int is_forward, int failsafe,
                          double x0, double y0, double theta0,
                          double x1, double y1, double theta1,
                          struct check_path_result *res)
{

    double distance = sqrt(bot_sq(x1-x0) + bot_sq(y1-y0));
    double heading = atan2((y1-y0),(x1-x0));

    //fprintf (stdout, "before NULL check\n");
    //fprintf (stdout, "paused = %d\n", self->paused);

    if (self->obsmap == NULL) {
        res->cost = -1;
        res->obs_cost = -1;
        res->obs_max = -1;
        res->obs_restricted = 1;
        return;
    }

    // require thetas to be equal
    //if (fabs(bot_mod2pi(theta0 - theta1)) > bot_to_radians(7)) {
    //    fprintf(stderr,"check_gridmap_check_path: ERROR: angle too large th0:%f th1:%f dtheta:%f\n",theta0,theta1, bot_to_degrees(bot_mod2pi(theta0 - theta1)));
    //}

    // Don't check the entire footprint.
    double xmin = self->obsmap_offsets[0];
    double xmax = self->obsmap_offsets[1];
    if (!is_forward) {
        xmin = -self->obsmap_offsets[1];
        xmax = -self->obsmap_offsets[0];
    }
    if (self->failsafe>1) {    // TODO: do this only in k-point turn and failsafe
        if (is_forward) {
            xmin += SKIP_CHECK_LENGTH_TAIL;   // If going forward, skip the rear part
        } else {
            xmin += SKIP_CHECK_LENGTH_HEAD;   // If going reverse, skip the front part
        }
    }


    // When the failsafe is greater than 1, check reduced vehicle footprint
    //
    // convolve_total_width = overall_width (vehicle width) + 0.1 (buffer)
    // convolve_line_searches = number of line searches to perform (specified in config file)
    // convolve_radius = half the width of each strip/line segment = (convolve_total_width/convolve_line_searches) / 2
    if (failsafe >= 1) {
        double convolve_total_width = self->convolve_total_width - failsafe * FAILSAFE_WIDTH_FACTOR;
        double convolve_radius = convolve_total_width / ((double) self->convolve_line_searches) / 2.0;
        check_gridmap_check_path_ex(self, self->obsmap, x0, y0, heading, distance,
                                    xmin, xmax,
                                    -convolve_total_width/2 + convolve_radius,
                                    2*convolve_radius,
                                    self->convolve_line_searches,
                                    &res->obs_cost, &res->obs_restricted, &res->obs_max);
    }
    else {
        check_gridmap_check_path_ex(self, self->obsmap, x0, y0, heading, distance,
                                    xmin, xmax,
                                    -self->convolve_total_width/2 + self->convolve_radius,
                                    2*self->convolve_radius,
                                    self->convolve_line_searches,
                                    &res->obs_cost, &res->obs_restricted, &res->obs_max);
    }


    res->restricted = res->obs_restricted;//fmax(res->lane_restricted, res->obs_restricted);
    res->cost = res->obs_cost; //fmax(res->lane_cost, res->obs_cost);
    res->max = res->obs_max;// fmax(res->obs_max, res->lane_max);

    if (res->obs_cost < 0) {
        res->cost = -1;
        res->max = -1;
    }
}


// sorry, quick code to get pose
/*int
check_gridmap_get_pose(check_gridmap_t *self, bot_core_pose_t *pose)
{
    bot_core_pose_t pose_current;
    if (!get_local_pose(self, &pose_current))
    return 0;
    memcpy(pose, &pose_current, sizeof(bot_core_pose_t));
    return 1;
}*/

/*
 * check_gridmap_create
 *
 * sensing_only_local: Specify whether to populate the area of the gridmap around the
 *                     robot based only on detected obstacles and not use the
 *                     SLAM occupancy map.
 *
 * sensing_only_small: Generate spatially compact gridmap that only renders
 *                     locally-perceived obstacles. (Added for door open/closed detection.)
 *
 * publish_gridmap:    Specify whether to publish the OBSTACLE_MAP message.
 *
 * clear_using_laser : Clear the map using the extent of the laser
 *
 * clear_person:       Clears the area around the current person (used for person following)
 *
 * ignore_local:       Ignores the obstacles reported from local sensing - useful for
 *                     matching stuff against the map
 */

check_gridmap_t *check_gridmap_create(const int constraints, gboolean render_traj,
                                      gboolean sensing_only_local, gboolean sensing_only_small,
                                      gboolean publish_gridmap, gboolean clear_person,
                                      gboolean ignore_local,
                                      double width_buffer)
{
    return check_gridmap_create_laser(constraints, render_traj,
                                      sensing_only_local, sensing_only_small,
                                      publish_gridmap, clear_person,
                                      ignore_local, FALSE,
                                      width_buffer);
}


check_gridmap_t *check_gridmap_create_laser(const int constraints, gboolean render_traj,
                                      gboolean sensing_only_local, gboolean sensing_only_small,
                                      gboolean publish_gridmap, gboolean clear_person,
                                      gboolean ignore_local, gboolean clear_using_laser,
                                      double width_buffer)
{
    check_gridmap_t *self = (check_gridmap_t*) calloc(1, sizeof(check_gridmap_t));
    pthread_mutex_init(&self->obs_gridmap_mutex, NULL);
    pthread_mutex_init (&self->global_map_gridmap_mutex, NULL);
    pthread_mutex_init(&self->signal_render_mutex, NULL);

    self->lcm = bot_lcm_get_global(NULL);
    bot_glib_mainloop_attach_lcm (self->lcm);

    memset(&self->global_map, 0, sizeof(ripl_map_t));
    self->global_map.complete_map = NULL;
    self->global_map.map = NULL;
    self->tile_generation = 1;
    //g_thread_init(NULL);
    self->lcmgl = NULL;//bot_lcmgl_init(self->lcm, "RRT-CHECKGRID");
    self->lcmgl_debug = NULL;//bot_lcmgl_init(self->lcm,"RRT-CHECKGRID-DBG");
    if(render_traj){
        self->lcmgl_obs_check = bot_lcmgl_init(self->lcm,"RRT-CHECKGRID-DBG");
      //self->lcmgl_debug = bot_lcmgl_init(self->lcm,"RRT-CHECKGRID-FULL-DBG");
    }
    else{
      self->lcmgl_obs_check = NULL;
      //self->lcmgl_debug = NULL;
    }
    self->publish_gridmap = publish_gridmap;
    if (self->publish_gridmap == FALSE)
        fprintf (stdout, "This check_gridmap instantiation will not pubish the gridmap LCM message!\n");

    self->param = bot_param_new_from_server(self->lcm, 0);
    self->frames = bot_frames_get_global (self->lcm, self->param);
    self->sensing_only_local = sensing_only_local;
    self->sensing_only_small = sensing_only_small;

    // When in sensing_only_small mode, only local sensing will be used (i.e., no carmen map)
    if (self->sensing_only_small)
        fprintf (stdout, "check_gridmap(): Maintaining a spatially compact gridmap based only on locally-perceived obstacles\n");
    if (self->sensing_only_small && !self->sensing_only_local) {
        fprintf (stderr, "check_gridmap(): Ignoring sensing_only_local = FALSE setting\n");
        self->sensing_only_local = TRUE;
    }

    self->obsmap = NULL;
    self->new_obsmap = NULL;
    self->complete_global_map = NULL;

    if(clear_person){
        self->clear_person = TRUE;
    }
    else{
        self->clear_person = FALSE;
    }

    if(ignore_local){
        self->ignore_local = TRUE;
    }
    else{
        self->ignore_local = FALSE;
        //this should turn off local only
    }



    self->goals_buffer = message_buffer_create((message_buffer_free_func_t) ripl_goal_list_t_destroy);
    self->person_buffer = NULL;
    if(self->clear_person){
      self->person_buffer = message_buffer_create((message_buffer_free_func_t) ripl_guide_info_t_destroy);
    }
    //self->nav_status_buffer = message_buffer_create((message_buffer_free_func_t) ripl_navigator_status_t_destroy);
    //self->nav_plan_buffer = message_buffer_create((message_buffer_free_func_t) ripl_navigator_plan_t_destroy);
    self->rects_buffer = message_buffer_create((message_buffer_free_func_t) obs_rect_list_t_destroy);
    self->sim_rects_buffer = message_buffer_create((message_buffer_free_func_t) obs_rect_list_t_destroy);
    self->map_rects_buffer = message_buffer_create((message_buffer_free_func_t) obs_rect_list_t_destroy);
    self->tracks_buffer = message_buffer_create((message_buffer_free_func_t) obs_track_list_t_destroy);
    self->sim_tracks_buffer = message_buffer_create((message_buffer_free_func_t) obs_track_list_t_destroy);

    // condition to flag rerender.
    pthread_cond_init(&self->signal_render, NULL);
    pthread_mutex_init(&self->signal_render_mutex, NULL);

    // Derive footprint parameters from vehicle bounds.
    double front_left[2], front_right[2];
    double rear_left[2], rear_right[2];
    if ((bot_param_get_double_array(self->param, "calibration.vehicle_bounds.front_left", front_left, 2) != 2) ||
        (bot_param_get_double_array(self->param, "calibration.vehicle_bounds.front_right", front_right, 2) != 2) ||
        (bot_param_get_double_array(self->param, "calibration.vehicle_bounds.rear_left", rear_left, 2) != 2) ||
        (bot_param_get_double_array(self->param, "calibration.vehicle_bounds.rear_right", rear_right, 2) != 2))
        abort();

    double range;
    double forward_offset;
    double resolution;
    if (!sensing_only_small) {
	if(!bot_param_get_double (self->param, "motion_planning.gridmap.range", &range))
	    self->range = range;
	else
	    self->range = GRIDMAP_RANGE;

	if(!bot_param_get_double (self->param, "motion_planning.gridmap.forward_offset", &forward_offset))
	    self->forward_offset = forward_offset;
	else
	    self->forward_offset = GRIDMAP_FORWARD_OFFSET;

	if(!bot_param_get_double (self->param, "motion_planning.gridmap.resolution", &resolution))
	    self->resolution = resolution;
	else
	    self->resolution = GRIDMAP_RESOLUTION;
    } else {
	if(!bot_param_get_double (self->param, "motion_planning.gridmap.range_small", &range))
	    self->range = range;
	else
	    self->range = GRIDMAP_RANGE;

	if(!bot_param_get_double (self->param, "motion_planning.gridmap.forward_offset_small", &forward_offset))
	    self->forward_offset = forward_offset;
	else
	    self->forward_offset = GRIDMAP_FORWARD_OFFSET;

	if(!bot_param_get_double (self->param, "motion_planning.gridmap.resolution_small", &resolution))
	    self->resolution = resolution;
	else
	    self->resolution = GRIDMAP_RESOLUTION;
    }


    printf("check_gridmap(): INFO:\n");
    if (sensing_only_local)
	printf("check_gridmap(): INFO: Using only obstacle detections for collision checking around robot (sensing_only_local is True\n");


    self->overall_width = fmax(front_left[1]-front_right[1],rear_left[1]-rear_right[1]);
    self->overall_length = fmax(front_left[0]-rear_left[0],front_right[0]-rear_right[0]);

    printf("check_gridmap(): INFO:\n");
    printf("check_gridmap(): INFO: overall footprint width:  %0.2lf\n",self->overall_width);
    printf("check_gridmap(): INFO: overall footprint length: %0.2lf\n",self->overall_length);


    // create render tables
    // Number of line searches to divide the width of the vehicle into (which in turn defines the convolve radius)
    self->convolve_line_searches = bot_param_get_double_or_fail(self->param, "motion_planner.gridmap.convolve_line_searches");

    //sachi - changed to add more of a buffer - we will play with the failsafe on failure
    self->convolve_total_width = self->overall_width + width_buffer;//0.1;
    self->convolve_radius = self->convolve_total_width / ((double) self->convolve_line_searches) / 2.0;

    // obstacle map line search length (distance to search in front of and behind vehicle origin)
    // center of convolve circle for footprint
    // backward: e.g. distance from axle to rear bumper.
    self->obsmap_offsets[0] =  fmin(rear_left[0],rear_right[0]);
    // forward: e.g. distance from axle to front bumper/tine tip.
    self->obsmap_offsets[1] =  fmax(front_left[0],front_right[0]);


    printf("check_gridmap(): INFO:\n");
    printf("check_gridmap(): INFO: convolve line searches: %d\n",self->convolve_line_searches);
    printf("check_gridmap(): INFO: convolve total width:   %0.2lf\n",self->convolve_total_width);
    printf("check_gridmap(): INFO: convolve radius:        %0.2lf\n",self->convolve_radius);
    printf("check_gridmap(): INFO:\n");
    printf("check_gridmap(): INFO: obsmap_offsets: (%0.2lf, %0.2lf)\n",self->obsmap_offsets[0],self->obsmap_offsets[1]);


    double failsafe_fudge = 0.1; //0;//0.15;
    bot_param_get_double(self->param, "motion_planner.gridmap.failsafe_fudge",&failsafe_fudge);
    printf("check_gridmap(): INFO:\n");
    printf("check_gridmap(): INFO: failsafe fudge: %0.2lf\n",failsafe_fudge);

    printf ("check_gridmap(): INFO:\ncheck_gridmap(): INFO: Copying Gridmap : %d\n", !self->sensing_only_small );


    // Max value out to cliff distance, then linearly decrease to max_dist
    // cliff distance should be <= max distance
    self->gridmap_lut = gridmap_lut_create_cliff_restricted_linear(256,
                                                                   0.5, //0.15, //0.1 //0.3 - works - mostly // max_dist
                                                                   0.3, //0.1, //0.2 - works mostly // cliff
                                                                   0, // restricted distance
                                                                   255, // max value
                                                                   0); // restricted

    for (int i = 0; i < NUM_OBST_LUTS; i++) {
        double extra_radius = i * failsafe_fudge;
        self->obst_luts[i] = gridmap_lut_create_cliff_restricted_linear(256,
                                                                        self->convolve_radius + 0.0 + extra_radius, // max_dist
                                                                        self->convolve_radius + extra_radius, // cliff_dist
                                                                        0, // restricted distance
                                                                        255, 0);
        //self->obst_luts[i] = gridmap_lut_create_cliff_restricted_cos(256,
        //                                                             self->convolve_radius + 3.0 + extra_radius,
        //                                                             self->convolve_radius + extra_radius,
        //                                                             0, // restricted
        //                                                             M_PI/2, 255, 0);
    }

    /*    for(int i=0; i < self->obst_luts[0]->size; i++){
      fprintf(stderr,"\t [%d] Value : %d\n", i, self->obst_luts[0]->table[i]);
      }*/

    // lut used to carve our current position as feasible.
    self->self_feasibility_lut = gridmap_lut_create_cliff_restricted_increasing(256,
                                                                                .1,  // width
                                                                                0,    // zero cost corridor
                                                                                HUGE,    // restricted
                                                                                0,    // decay
                                                                                SELF_FEASIBILITY_COST,  // min
                                                                                SELF_FEASIBILITY_COST,
                                                                                255); // max

    printf("check_gridmap(): INFO:\n");
    printf("check_gridmap(): INFO: gridmap width/height (range): %0.2lf\n",self->range);
    printf("check_gridmap(): INFO: gridmap forward offset:       %0.2lf\n",self->forward_offset);
    printf("check_gridmap(): INFO: gridmap resolution:           %0.2lf\n",self->resolution);

    //ripl_navigator_plan_t_subscribe(self->lcm, "NAVIGATOR_PLAN", on_nav_plan, self);
    //ripl_navigator_status_t_subscribe(self->lcm, "NAVIGATOR_STATUS", on_nav_status, self);
    ripl_goal_list_t_subscribe(self->lcm, "GOALS", on_goals, self);

    // Don't account for sim_rects if we are in sensing_only_small mode
    if (self->sensing_only_small == FALSE) {
        fprintf(stderr, "Subscribing to the Sim rects\n");
        obs_rect_list_t_subscribe(self->lcm, "SIM_RECTS", on_sim_rects, self);
        obs_rect_list_t_subscribe(self->lcm, "MAP_SERVER_RECTS", on_sim_rects, self);
    }

    //we need to have an option of not subscribing to obstacles
    if(!self->ignore_local){
        obs_obstacle_list_t_subscribe (self->lcm, "OBSTACLES", on_obstacles, self);
    }
    ripl_failsafe_t_subscribe(self->lcm, "FAILSAFE", on_failsafe, self);

    if(self->ignore_local){
        clear_using_laser = FALSE;
    }

    if(clear_using_laser){
        bot_core_planar_lidar_t_subscribe(self->lcm, "SKIRT_FRONT" , on_planar_lidar, self);
        bot_core_planar_lidar_t_subscribe(self->lcm, "SKIRT_REAR" , on_planar_lidar, self);
    }

    //if (self->sensing_only_local == FALSE)
    // If we can't get a map, revert to a smaller gridmap so that we don't think
    // too much of the local environment is safe (a cost of 253 is returned for
    // locations outside the gridmap
    if(self->sensing_only_small==FALSE){
        int sucess = get_global_map(self);
        if(sucess < 0){
            fprintf (stdout, "check_gridmap(): Unable to get map from map server. Setting sense_only_small to true\n");
            self->sensing_only_small = TRUE;
        }
    }

    if(self->clear_person)
        ripl_guide_info_t_subscribe(self->lcm, "GUIDE_POS", guide_pos_handler, self);

    // do this last
    pthread_create(&self->render_thread, NULL, render_thread, self);

    return self;
}

void check_gridmap_destroy(check_gridmap_t *self)
{
    if (!self)
        return;

    if (self->lcm)
        bot_glib_mainloop_detach_lcm (self->lcm);

    pthread_cancel (self->render_thread);
    pthread_join (self->render_thread, NULL);

    if (self->front_laser)
        bot_core_planar_lidar_t_destroy(self->front_laser);
    if (self->rear_laser)
        bot_core_planar_lidar_t_destroy(self->rear_laser);
    // Free message buffers
    if (self->goals_buffer)
        message_buffer_destroy (self->goals_buffer);
    //if (self->a_buffer)
    //    message_buffer_destroy (self->nav_status_buffer);
    //if (self->nav_plan_buffer)
    //    message_buffer_destroy (self->nav_plan_buffer);
    if (self->rects_buffer)
        message_buffer_destroy (self->rects_buffer);
    if (self->sim_rects_buffer)
        message_buffer_destroy (self->sim_rects_buffer);
    if (self->map_rects_buffer)
        message_buffer_destroy (self->map_rects_buffer);
    if (self->tracks_buffer)
        message_buffer_destroy (self->tracks_buffer);
    if (self->sim_tracks_buffer)
        message_buffer_destroy (self->sim_tracks_buffer);

    if (self->gridmap_lut) {
        gridmap_lut_destroy (self->gridmap_lut);
        self->gridmap_lut = NULL;
    }

    for (int i=0; i < NUM_OBST_LUTS; i++) {
        if (self->obst_luts[i]) {
            gridmap_lut_destroy (self->obst_luts[i]);
            self->obst_luts[i] = NULL;
        }
    }

    if (self->self_feasibility_lut) {
        gridmap_lut_destroy (self->self_feasibility_lut);
        self->self_feasibility_lut = NULL;
    }


    pthread_mutex_lock (&self->global_map_gridmap_mutex);
    fprintf (stdout, "on destroy before: global_map.map = %p\n", (void*) self->global_map.map);
    fprintf (stdout, "on destroy before: global_map.complete_map = %p\n", (void*) self->global_map.complete_map);
    fprintf (stdout, "on destroy before: complete_global_map = %p\n", (void*) self->complete_global_map);
    if (self->global_map.map) {
        free (self->global_map.map);
        self->global_map.map = NULL;
    }
    if (self->global_map.complete_map) {
        free (self->global_map.complete_map);
        self->global_map.complete_map = NULL;
    }
    if (self->complete_global_map) {
        free (self->complete_global_map);
        self->complete_global_map = NULL;
    }
    fprintf (stdout, "on destroy after: global_map.map = %p\n", (void*) self->global_map.map);
    fprintf (stdout, "on destroy after: global_map.complete_map = %p\n", (void*) self->global_map.complete_map);
    fprintf (stdout, "on destroy after: complete_global_map = %p\n", (void*) self->complete_global_map);
    pthread_mutex_unlock (&self->global_map_gridmap_mutex);

    // free the message buffers

    pthread_cond_destroy(&self->signal_render);
    pthread_mutex_destroy(&self->signal_render_mutex);

    pthread_mutex_destroy (&self->obs_gridmap_mutex);
    pthread_mutex_destroy (&self->global_map_gridmap_mutex);

    free(self);
}

double check_gridmap_convolve_radius(check_gridmap_t *self) {
    return self->convolve_radius;
}


// start stop processing
int check_gridmap_on(check_gridmap_t *cg) {
    cg->paused=FALSE;
    return 0;

}


int check_gridmap_off(check_gridmap_t *cg) {
    cg->paused=TRUE;
    return 0;
}
