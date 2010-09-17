/**
 ******************************************************************************
 *
 * @file       mixercurvewidget.h
 * @author     Edouard Lafargue Copyright (C) 2010.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup ConfigPlugin Configuration Plugin
 * @{
 * @brief A widget which displays a mixer curve
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

#ifndef MIXERCURVEWIDGET_H_
#define MIXERCURVEWIDGET_H_

#include <QGraphicsView>
#include <QtSvg/QSvgRenderer>
#include <QtSvg/QGraphicsSvgItem>


class MixerCurveWidget : public QGraphicsView
{
    Q_OBJECT

public:
    MixerCurveWidget(QWidget *parent = 0);
   ~MixerCurveWidget();
   void itemMoved();

public slots:


private slots:

private:
   QGraphicsSvgItem *plot;

protected:
    void showEvent(QShowEvent *event);
    void resizeEvent(QResizeEvent *event);


};
#endif /* MIXERCURVEWIDGET_H_ */
