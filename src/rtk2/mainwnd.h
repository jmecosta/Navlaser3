/*
 * mainwnd.h
 *
 *  Created on: 19 Jun 2011
 *      Author: jmecosta
 */

#ifndef MAINWND_H_
#define MAINWND_H_

// Non-local headers
#include "rtk.h"

// Local headers
#include "error.h"

/***************************************************************************
 * Default colors
 ***************************************************************************/

#define MAP_UPDATE_TIME 1.0
#define VECTORMAP_UPDATE_TIME 1.0

/***************************************************************************
 * Default colors
 ***************************************************************************/

#define COLOR_GRID_MAJOR         0xC0C0C0
#define COLOR_GRID_MINOR         0xE0E0E0
#define COLOR_AIO                0x000000
#define COLOR_ACTARRAY_DATA      0x00C000
#define COLOR_ACTARRAY_CMD       0x0000C0
#define COLOR_DIO                0x000000
#define COLOR_LASER              0x0000C0
#define COLOR_LASER_EMP          0xD0D0FF
#define COLOR_LASER_OCC          0x0000C0
#define COLOR_LOCALIZE           0xFF00FF
#define COLOR_LOCALIZE_PARTICLES 0x0000FF
#define COLOR_FIDUCIAL           0xF000F0
#define COLOR_POSITION_ROBOT     0xC00000
#define COLOR_POSITION_CONTROL   0xFF0000
#define COLOR_POWER              0x000000
#define COLOR_PTZ_DATA           0x00C000
#define COLOR_PTZ_DATA_TILT      0x0000C0
#define COLOR_PTZ_CMD            0x00C000
#define COLOR_PTZ_CMD_TILT       0x0000C0
#define COLOR_SONAR              0xC0C080
#define COLOR_SONAR_SCAN         0xC0C080
#define COLOR_IR                 0xC0C080
#define COLOR_IR_SCAN            0xC00080
#define COLOR_WIFI               0x000000
#define COLOR_BUMPER             0xC080C0
#define COLOR_BUMPER_ACTIVE      0x00FF00


/***************************************************************************
 * Top-level GUI elements
 ***************************************************************************/

// Main window displaying sensor stuff
class mainwnd
{
public:
	mainwnd ();
	mainwnd *mainwnd_create(rtk_app_t *app, const char *host, int port);
  const char *host;
  int port;

  // The rtk canvas
  rtk_canvas_t *canvas;

  // The grid figure (fixed to cs)
  rtk_fig_t *grid_fig;

  // The base figure for the robot
  // Robot is always at (0, 0, 0) in this cs
  rtk_fig_t *robot_fig;

  // Menu containing file options
  rtk_menu_t *file_menu;
  rtk_menuitem_t *exit_item;

  // The stills menu
  rtk_menu_t *stills_menu;
  rtk_menuitem_t *stills_jpeg_menuitem;
  rtk_menuitem_t *stills_ppm_menuitem;

  // Export stills info
  int stills_series;
  int stills_count;

  // The movie menu
  rtk_menu_t *movie_menu;
  rtk_menuitem_t *movie_x1_menuitem;
  rtk_menuitem_t *movie_x2_menuitem;

  // Export movie info
  int movie_count;

  // Menu containing view settings
  rtk_menu_t *view_menu;
  rtk_menuitem_t *view_item_rotate;
  rtk_menuitem_t *view_item_1m;
  rtk_menuitem_t *view_item_10m;
  rtk_menuitem_t *view_item_2f;
  rtk_menuitem_t *view_item_ego;
  // Menu containing the device list
  rtk_menu_t *device_menu;

  // Destroy the main window
  void mainwnd_destroy(mainwnd *wnd);

  // Update the window
  // Returns 1 if the program should quit.
  int mainwnd_update(mainwnd *wnd);

  void mainwnd_update_export(mainwnd *wnd);

};

#endif /* MAINWND_H_ */
