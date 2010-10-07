/**
 ******************************************************************************
 *
 * @file       opmapgadgetwidget.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup OPMapPlugin OpenPilot Map Plugin
 * @{
 * @brief The OpenPilot Map plugin 
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef OPMAP_GADGETWIDGET_H_
#define OPMAP_GADGETWIDGET_H_

// ******************************************************

#include <QtGui/QWidget>
#include <QtGui/QMenu>
#include <QStringList>
#include <QStandardItemModel>
#include <QList>
#include <QMutex>
#include <QMutexLocker>
#include <QPointF>

#include "opmapcontrol/opmapcontrol.h"

#include "uavobjects/uavobjectmanager.h"
#include "uavobjects/positionactual.h"
#include "uavobjects/homelocation.h"
#include "extensionsystem/pluginmanager.h"

#include "opmap_overlay_widget.h"
#include "opmap_zoom_slider_widget.h"
#include "opmap_statusbar_widget.h"
#include "opmap_waypointeditor_dialog.h"
#include "opmap_edit_waypoint_dialog.h"

#include "utils/coordinateconversions.h"

// ******************************************************

namespace Ui
{
    class OPMap_Widget;
}

using namespace mapcontrol;

// ******************************************************

// local waypoint list item structure
typedef struct t_waypoint
{
    mapcontrol::WayPointItem *map_wp_item;
    internals::PointLatLng coord;
    double altitude;
    QString description;
    bool locked;
    int time_seconds;
    int hold_time_seconds;
} t_waypoint;

// ******************************************************

enum opMapModeType { Normal_MapMode = 0,
                     MagicWaypoint_MapMode = 1};

// ******************************************************

class OPMapGadgetWidget : public QWidget
{
    Q_OBJECT

public:
    OPMapGadgetWidget(QWidget *parent = 0);
   ~OPMapGadgetWidget();

    /**
    * @brief public functions
    *
    * @param
    */
    void goHome();
    void setZoom(int zoom);
    void setPosition(QPointF pos);
    void setMapProvider(QString provider);
    void setUseOpenGL(bool useOpenGL);
    void setShowTileGridLines(bool showTileGridLines);
    void setAccessMode(QString accessMode);
    void setUseMemoryCache(bool useMemoryCache);
    void setCacheLocation(QString cacheLocation);
    void setMapMode(opMapModeType mode);

public slots:

protected:
    void resizeEvent(QResizeEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void contextMenuEvent(QContextMenuEvent *event);
    void keyPressEvent(QKeyEvent* event);

private slots:
    void updatePosition();

    void updateMousePos();

    void zoomIn();
    void zoomOut();

    /**
    * @brief signals received from the various map plug-in widget user controls
    *
    * @param
    */
    void comboBoxFindPlace_returnPressed();
    void on_toolButtonFindPlace_clicked();
    void on_toolButtonZoomM_clicked();
    void on_toolButtonZoomP_clicked();
    void on_toolButtonMapHome_clicked();
    void on_toolButtonMapUAV_clicked();
    void on_toolButtonMapUAVheading_clicked();
    void on_toolButtonShowUAVtrail_clicked();
    void on_horizontalSliderZoom_sliderMoved(int position);
    void on_toolButtonAddWaypoint_clicked();
    void on_treeViewWaypoints_clicked(QModelIndex index);
    void on_toolButtonHome_clicked();
    void on_toolButtonNextWaypoint_clicked();
    void on_toolButtonPrevWaypoint_clicked();
    void on_toolButtonHoldPosition_clicked();
    void on_toolButtonGo_clicked();
    void on_toolButtonMagicWaypointMapMode_clicked();
    void on_toolButtonNormalMapMode_clicked();

    /**
    * @brief signals received from the map object
    *
    * @param
    */
    void zoomChanged(double zoomt,double zoom, double zoomd);
    void OnCurrentPositionChanged(internals::PointLatLng point);
    void OnTileLoadComplete();
    void OnTileLoadStart();
    void OnMapDrag();
    void OnMapZoomChanged();
    void OnMapTypeChanged(MapType::Types type);
    void OnEmptyTileError(int zoom, core::Point pos);
    void OnTilesStillToLoad(int number);
    void WPNumberChanged(int const& oldnumber,int const& newnumber, WayPointItem* waypoint);
    void WPValuesChanged(WayPointItem* waypoint);
    void WPInserted(int const& number, WayPointItem* waypoint);
    void WPDeleted(int const& number);

    /**
    * @brief mouse right click context menu signals
    *
    * @param
    */
    void onReloadAct_triggered();
    void onCopyMouseLatLonToClipAct_triggered();
    void onCopyMouseLatToClipAct_triggered();
    void onCopyMouseLonToClipAct_triggered();
    void onFindPlaceAct_triggered();
    void onShowCompassAct_toggled(bool show);
    void onShowUAVAct_toggled(bool show);
    void onShowHomeAct_toggled(bool show);
    void onGoZoomInAct_triggered();
    void onGoZoomOutAct_triggered();
    void onGoMouseClickAct_triggered();
    void onGoHomeAct_triggered();
    void onGoUAVAct_triggered();
    void onFollowUAVpositionAct_toggled(bool checked);
    void onFollowUAVheadingAct_toggled(bool checked);
    void onShowUAVtrailAct_toggled(bool checked);
    void onClearUAVtrailAct_triggered();
    void onOpenWayPointEditorAct_triggered();
    void onAddWayPointAct_triggered();
    void onEditWayPointAct_triggered();
    void onLockWayPointAct_triggered();
    void onDeleteWayPointAct_triggered();
    void onClearWayPointsAct_triggered();
    void onMapModeActGroup_triggered(QAction *action);
    void onZoomActGroup_triggered(QAction *action);

private:
    int min_zoom;
    int max_zoom;

    double m_heading;	// uav heading

    internals::PointLatLng mouse_lat_lon;

    int prev_tile_number;

    opMapModeType m_map_mode;

    QStringList findPlaceWordList;
    QCompleter *findPlaceCompleter;

    QTimer *m_updateTimer;
    QTimer *m_statusUpdateTimer;

    ExtensionSystem::PluginManager *m_plugin_manager;
    UAVObjectManager *m_objManager;

    Ui::OPMap_Widget *m_widget;

    mapcontrol::OPMapWidget *m_map;

    opmap_waypointeditor_dialog waypoint_editor_dialog;

    opmap_edit_waypoint_dialog waypoint_edit_dialog;

    opmap_overlay_widget *m_overlay_widget;

    QGraphicsScene *m_map_graphics_scene;
    QGraphicsProxyWidget *m_map_scene_proxy;
    opmap_zoom_slider_widget *m_zoom_slider_widget;
    opmap_statusbar_widget *m_statusbar_widget;

    QStandardItemModel wayPoint_treeView_model;

    mapcontrol::WayPointItem *m_mouse_waypoint;

    QList<t_waypoint *> m_waypoint_list;
    QMutex m_waypoint_list_mutex;

    t_waypoint magic_waypoint;

    QMutex m_map_mutex;

    void createActions();

    QAction *closeAct;
    QAction *reloadAct;
    QAction *copyMouseLatLonToClipAct;
    QAction *copyMouseLatToClipAct;
    QAction *copyMouseLonToClipAct;
    QAction *findPlaceAct;
    QAction *showCompassAct;
    QAction *showHomeAct;
    QAction *showUAVAct;
    QAction *zoomInAct;
    QAction *zoomOutAct;
    QAction *goMouseClickAct;
    QAction *goHomeAct;
    QAction *goUAVAct;
    QAction *followUAVpositionAct;
    QAction *followUAVheadingAct;
    QAction *showUAVtrailAct;
    QAction *clearUAVtrailAct;
    QAction *wayPointEditorAct;
    QAction *addWayPointAct;
    QAction *editWayPointAct;
    QAction *lockWayPointAct;
    QAction *deleteWayPointAct;
    QAction *clearWayPointsAct;

    QActionGroup *mapModeActGroup;
    QList<QAction *> mapModeAct;

    QActionGroup *zoomActGroup;
    QList<QAction *> zoomAct;

    void loadComboBoxLines(QComboBox *comboBox, QString filename);
    void saveComboBoxLines(QComboBox *comboBox, QString filename);

    QPointF getLatLon();

    void setMapFollowingMode();
};

#endif /* OPMAP_GADGETWIDGET_H_ */
