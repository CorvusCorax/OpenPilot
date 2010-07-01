/**
 ******************************************************************************
 *
 * @file       pfdgadgetwidget.cpp
 * @author     Edouard Lafargue Copyright (C) 2010.
 * @brief
 * @see        The GNU Public License (GPL) Version 3
 * @defgroup   pfdplugin
 * @{
 *
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

#include "pfdgadgetwidget.h"
#include <iostream>
#include <QDebug>
#include <cmath>

PFDGadgetWidget::PFDGadgetWidget(QWidget *parent) : QGraphicsView(parent)
{
    // TODO: create a proper "needle" object instead of hardcoding all this
    // which is ugly (but easy).

    setMinimumSize(64,64);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    setScene(new QGraphicsScene(this));
    setRenderHints(QPainter::Antialiasing);

    m_renderer = new QSvgRenderer();

    attitudeObj = NULL;
    headingObj = NULL;
    compassBandWidth = 0;
/*
    obj2 = NULL;
    obj3 = NULL;
*/
    // This timer mechanism makes needles rotate smoothly
    connect(&dialTimer, SIGNAL(timeout()), this, SLOT(moveNeedles()));
    dialTimer.start(20);
/*
    connect(&dialTimer2,SIGNAL(timeout()), this, SLOT(moveVerticalScales()));
    dialTimer2.start(20);
    */

}

PFDGadgetWidget::~PFDGadgetWidget()
{
   // Do nothing
}

/*!
  \brief Connects the widget to the relevant UAVObjects

  We want: AttitudeActual, FlightBattery, Location

  */
void PFDGadgetWidget::connectNeedles() {
    if (attitudeObj != NULL)
        disconnect(attitudeObj,SIGNAL(objectUpdated(UAVObject*)),this,SLOT(updateAttitude(UAVObject*)));

    if (headingObj != NULL)
        disconnect(headingObj,SIGNAL(objectUpdated(UAVObject*)),this,SLOT(updateHeading(UAVObject*)));

    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *objManager = pm->getObject<UAVObjectManager>();

   attitudeObj = dynamic_cast<UAVDataObject*>(objManager->getObject("AttitudeActual"));
   if (attitudeObj != NULL ) {
       connect(attitudeObj, SIGNAL(objectUpdated(UAVObject*)), this, SLOT(updateAttitude(UAVObject*)));
   } else {
        std::cout << "Error: Object is unknown (AttitudeActual)." << std::endl;
   }

   headingObj = dynamic_cast<UAVDataObject*>(objManager->getObject("PositionActual"));
   if (headingObj != NULL ) {
       connect(headingObj, SIGNAL(objectUpdated(UAVObject*)), this, SLOT(updateHeading(UAVObject*)));
   } else {
        std::cout << "Error: Object is unknown (PositionActual)." << std::endl;
   }

}

/*!
  \brief Reads the updated attitude and computes the new display position
  */
void PFDGadgetWidget::updateAttitude(UAVObject *object1) {
    // Double check that the field exists:
    QString roll = QString("Roll");
    QString pitch = QString("Pitch");
    UAVObjectField* field = object1->getField(roll);
    UAVObjectField* field2 = object1->getField(pitch);
    if (field && field2) {
        // These factors assume some things about the PFD SVG, namely:
        // - Roll value in degrees
        // - Pitch lines are 300px high for a +20/-20 range, which means
        //   7.5 pixels per pitch degree.
        rollTarget = field->getDouble()*(-1);
        pitchTarget = field2->getDouble()*7.5;
        if (!dialTimer.isActive())
            dialTimer.start(); // Rearm the dial Timer which might be stopped.
    } else {
        std::cout << "UpdateAttitude: Wrong field, maybe an issue with object disconnection ?" << std::endl;
    }
}

/*!
  \brief Updates the compass reading and speed dial.

    Note: the speed dial shows the ground speed at the moment, because
        there is no airspeed by default. Should become configurable in a future
        gadget release (TODO)
  */
void PFDGadgetWidget::updateHeading(UAVObject *object1) {
    // Double check that the field exists:
    QString fieldname = QString("Heading");
    UAVObjectField* field = object1->getField(fieldname);
    if (field) {
        // These factors assume some things about the PFD SVG, namely:
        // - Heading value in degrees
        // - Scale is 540 degrees large
        headingTarget = field->getDouble()*compassBandWidth/(-540);
    } else {
        std::cout << "UpdateHeading: Wrong field, maybe an issue with object disconnection ?" << std::endl;
    }
    fieldname = QString("Groundspeed");
    field = object1->getField(fieldname);
    if (field) {
        // The speed scale represents 30km/h (6 * 5)
        // 3.6 : convert m/s to km/h
        groundspeedTarget = 3.6*field->getDouble()*speedScaleHeight/(30);
    }
    fieldname = QString("Altitude");
    field = object1->getField(fieldname);
    if (field) {
        // The altitude scale represents 30 meters
        altitudeTarget = field->getDouble()*altitudeScaleHeight/(30);
    }

    if (!dialTimer.isActive())
        dialTimer.start(); // Rearm the dial Timer which might be stopped.

}

/*!
  \brief Called by the UAVObject which got updated
  */
void PFDGadgetWidget::updateAirspeed(UAVObject *object3) {
}

/*!
  \brief Called by the UAVObject which got updated
  */
void PFDGadgetWidget::updateAltitude(UAVObject *object3) {
}


/*!
  \brief Called by the UAVObject which got updated
  */
void PFDGadgetWidget::updateBattery(UAVObject *object3) {
}


/*!
  \brief Sets up the PFD from the SVG master file.

  Initializes the display, and does all the one-time calculations.
  */
void PFDGadgetWidget::setDialFile(QString dfn)
{
   if (QFile::exists(dfn))
   {
      m_renderer->load(dfn);
      if(m_renderer->isValid())
      {
/* The PFD element IDs are fixed, not like with the analog dial.
     - Background: background
     - Foreground: foreground (contains all fixed elements, including plane)
     - earth/sky : world
     - Roll scale: rollscale
     - compass frame: compass (part of the foreground)
     - compass band : compass-band
     - Home point: homewaypoint
     - Next point: nextwaypoint
     - Home point bearing: homewaypoint-bearing
     - Next point bearing: nextwaypoint-bearing
     - Speed rectangle (left side): speed-bg
     - Speed scale: speed-scale.
     - Black speed window: speed-window.
     - Altitude rectangle (right site): altitude-bg.
     - Altitude scale: altitude-scale.
     - Black altitude window: altitude-window.
 */
         QGraphicsScene *l_scene = scene();
         l_scene->clear(); // Deletes all items contained in the scene as well.
         m_background = new QGraphicsSvgItem();
         // All other items will be clipped to the shape of the background
         m_background->setFlags(QGraphicsItem::ItemClipsChildrenToShape|
                                QGraphicsItem::ItemClipsToShape);
         m_background->setSharedRenderer(m_renderer);
         m_background->setElementId("background");
         l_scene->addItem(m_background);

         m_world = new QGraphicsSvgItem();
         m_world->setParentItem(m_background);
         m_world->setSharedRenderer(m_renderer);
         m_world->setElementId("world");
         l_scene->addItem(m_world);

         // red Roll scale: rollscale
         m_rollscale = new QGraphicsSvgItem();
         m_rollscale->setSharedRenderer(m_renderer);
         m_rollscale->setElementId("rollscale");
         l_scene->addItem(m_rollscale);

         // Home point:
         m_homewaypoint = new QGraphicsSvgItem();
         // Next point:
         m_nextwaypoint = new QGraphicsSvgItem();
         // Home point bearing:
         m_homepointbearing = new QGraphicsSvgItem();
         // Next point bearing:
         m_nextpointbearing = new QGraphicsSvgItem();

         m_foreground = new QGraphicsSvgItem();
         m_foreground->setParentItem(m_background);
         m_foreground->setSharedRenderer(m_renderer);
         m_foreground->setElementId("foreground");
         l_scene->addItem(m_foreground);

         ////////////////////
         // Compass
         ////////////////////
         // Get the default location of the Compass:
         QMatrix compassMatrix = m_renderer->matrixForElement("compass");
         qreal startX = compassMatrix.mapRect(m_renderer->boundsOnElement("compass")).x();
         qreal startY = compassMatrix.mapRect(m_renderer->boundsOnElement("compass")).y();
         // Then once we have the initial location, we can put it
         // into a QGraphicsSvgItem which we will display at the same
         // place: we do this so that the heading scale can be clipped to
         // the compass dial region.
         m_compass = new QGraphicsSvgItem();
         m_compass->setSharedRenderer(m_renderer);
         m_compass->setElementId("compass");
         m_compass->setFlags(QGraphicsItem::ItemClipsChildrenToShape|
                             QGraphicsItem::ItemClipsToShape);
         l_scene->addItem(m_compass);
         QTransform matrix;
         matrix.translate(startX,startY);
         m_compass->setTransform(matrix,false);

         // Now place the compass scale inside:
         m_compassband = new QGraphicsSvgItem();
         m_compassband->setSharedRenderer(m_renderer);
         m_compassband->setElementId("compass-band");
         m_compassband->setParentItem(m_compass);
         l_scene->addItem(m_compassband);
         matrix.reset();
         // Note: the compass band has to be a path, which means all text elements have to be
         // converted, ortherwise boundsOnElement does not compute the height correctly
         // if the highest element is a text element. This is a Qt Bug as far as I can tell.

         // compass-scale is the while bottom line inside the band: using the band's width
         // includes half the width of the letters, which causes errors:
         compassBandWidth = m_renderer->boundsOnElement("compass-scale").width();

         ////////////////////
         // Speed
         ////////////////////
         // Speedometer on the left hand:
         compassMatrix = m_renderer->matrixForElement("speed-bg");
         startX = compassMatrix.mapRect(m_renderer->boundsOnElement("speed-bg")).x();
         startY = compassMatrix.mapRect(m_renderer->boundsOnElement("speed-bg")).y();
         QGraphicsSvgItem *verticalbg = new QGraphicsSvgItem();
         verticalbg->setSharedRenderer(m_renderer);
         verticalbg->setElementId("speed-bg");
         verticalbg->setFlags(QGraphicsItem::ItemClipsChildrenToShape|
                             QGraphicsItem::ItemClipsToShape);
         l_scene->addItem(verticalbg);
         matrix.reset();
         matrix.translate(startX,startY);
         verticalbg->setTransform(matrix,false);

         // Note: speed-scale should contain exactly 6 major ticks
         // for 30km/h
         m_speedscale = new QGraphicsItemGroup();
         m_speedscale->setParentItem(verticalbg);

         QGraphicsSvgItem *speedscalelines = new QGraphicsSvgItem();
         speedscalelines->setSharedRenderer(m_renderer);
         speedscalelines->setElementId("speed-scale");
         speedScaleHeight = m_renderer->matrixForElement("speed-scale").mapRect(
                        m_renderer->boundsOnElement("speed-scale")).height();
         startX = compassMatrix.mapRect(m_renderer->boundsOnElement("speed-bg")).width();
         startX -= m_renderer->matrixForElement("speed-scale").mapRect(
                        m_renderer->boundsOnElement("speed-scale")).width();
         //speedscalelines->setParentItem(verticalbg);
         matrix.reset();
         matrix.translate(startX,0);
         speedscalelines->setTransform(matrix,false);
         // Quick way to reposition the item before putting it in the group:
         speedscalelines->setParentItem(verticalbg);
         m_speedscale->addToGroup(speedscalelines); // (reparents the item)

         // Add the scale text elements:
         QGraphicsTextItem *speed0 = new QGraphicsTextItem("0");
         speed0->setDefaultTextColor(QColor("White"));
         speed0->setFont(QFont("Arial",(int) speedScaleHeight/30));
         matrix.reset();
         matrix.translate(compassMatrix.mapRect(m_renderer->boundsOnElement("speed-bg")).width()/10,-speedScaleHeight/30);
         speed0->setTransform(matrix,false);
         speed0->setParentItem(verticalbg);
         m_speedscale->addToGroup(speed0);
         for (int i=0; i<6;i++) {
             speed0 = new QGraphicsTextItem("");
             speed0->setDefaultTextColor(QColor("White"));
             speed0->setFont(QFont("Arial",(int) speedScaleHeight/30));
             speed0->setPlainText(QString().setNum(i*5+1));
             matrix.translate(0,speedScaleHeight/6);
             speed0->setTransform(matrix,false);
             speed0->setParentItem(verticalbg);
             m_speedscale->addToGroup(speed0);    
         }
         // Now vertically center the speed scale on the speed background
         QRectF rectB = verticalbg->boundingRect();
         QRectF rectN = speedscalelines->boundingRect();
         m_speedscale->setPos(0,rectB.height()/2-rectN.height()/2-rectN.height()/6);

         ////////////////////
         // Altitude
         ////////////////////
         // Right hand, very similar to speed
         compassMatrix = m_renderer->matrixForElement("altitude-bg");
         startX = compassMatrix.mapRect(m_renderer->boundsOnElement("altitude-bg")).x();
         startY = compassMatrix.mapRect(m_renderer->boundsOnElement("altitude-bg")).y();
         verticalbg = new QGraphicsSvgItem();
         verticalbg->setSharedRenderer(m_renderer);
         verticalbg->setElementId("altitude-bg");
         verticalbg->setFlags(QGraphicsItem::ItemClipsChildrenToShape|
                             QGraphicsItem::ItemClipsToShape);
         l_scene->addItem(verticalbg);
         matrix.reset();
         matrix.translate(startX,startY);
         verticalbg->setTransform(matrix,false);

         // Note: altitude-scale should contain exactly 6 major ticks
         // for 30 meters
         m_altitudescale = new QGraphicsItemGroup();
         m_altitudescale->setParentItem(verticalbg);

         QGraphicsSvgItem *altitudescalelines = new QGraphicsSvgItem();
         altitudescalelines->setSharedRenderer(m_renderer);
         altitudescalelines->setElementId("altitude-scale");
         altitudeScaleHeight = m_renderer->matrixForElement("altitude-scale").mapRect(
                        m_renderer->boundsOnElement("altitude-scale")).height();
         // Quick way to reposition the item before putting it in the group:
         altitudescalelines->setParentItem(verticalbg);
         m_altitudescale->addToGroup(altitudescalelines); // (reparents the item)

         // Add the scale text elements:
         speed0 = new QGraphicsTextItem("XXXX");
         speed0->setDefaultTextColor(QColor("White"));
         speed0->setFont(QFont("Arial",(int) altitudeScaleHeight/30));
         matrix.reset();
         matrix.translate(m_renderer->matrixForElement("altitude-scale").mapRect(m_renderer->boundsOnElement("altitude-scale")).width()
                          + m_renderer->matrixForElement("altitude-bg").mapRect(m_renderer->boundsOnElement("altitude-bg")).width()/10,-altitudeScaleHeight/30);
         speed0->setTransform(matrix,false);
         speed0->setParentItem(verticalbg);
         m_altitudescale->addToGroup(speed0);
         for (int i=0; i<6;i++) {
             speed0 = new QGraphicsTextItem("XXXX");
             speed0->setDefaultTextColor(QColor("White"));
             speed0->setFont(QFont("Arial",(int) altitudeScaleHeight/30));
             speed0->setPlainText(QString().setNum(i*5+1));
             matrix.translate(0,altitudeScaleHeight/6);
             speed0->setTransform(matrix,false);
             speed0->setParentItem(verticalbg);
             m_altitudescale->addToGroup(speed0);
         }
         // Now vertically center the speed scale on the speed background
         rectB = verticalbg->boundingRect();
         rectN = altitudescalelines->boundingRect();
         m_altitudescale->setPos(0,rectB.height()/2-rectN.height()/2-rectN.height()/6);

        l_scene->setSceneRect(m_background->boundingRect());

        // Now Initialize the center for all transforms of the relevant elements to the
        // center of the background:

        // 1) Move the center of the needle to the center of the background.
        rectB = m_background->boundingRect();
        rectN = m_world->boundingRect();
        m_world->setPos(rectB.width()/2-rectN.width()/2,rectB.height()/2-rectN.height()/2);
        // 2) Put the transform origin point of the needle at its center.
        m_world->setTransformOriginPoint(rectN.width()/2,rectN.height()/2);

        rectN = m_rollscale->boundingRect();
        m_rollscale->setPos(rectB.width()/2-rectN.width()/2,rectB.height()/2-rectN.height()/2);
        m_rollscale->setTransformOriginPoint(rectN.width()/2,rectN.height()/2);

        // Also to the same init for the compass:
        rectB = m_compass->boundingRect();
        rectN = m_compassband->boundingRect();
        m_compassband->setPos(rectB.width()/2-rectN.width()/2,rectB.height()/2-rectN.height()/2);
        m_compassband->setTransformOriginPoint(rectN.width()/2,rectN.height()/2);

        // Last: we just loaded the dial file which is by default valid for a "zero" value
        // of the needles, so we have to reset the needles too upon dial file loading, otherwise
        // we would end up with an offset when we change a dial file and the needle value
        // is not zero at that time.
        rollValue = 0;
        pitchValue = 0;
        headingValue = 0;
        groundspeedValue = 0;
        altitudeValue = 0;
        if (!dialTimer.isActive())
            dialTimer.start(); // Rearm the dial Timer which might be stopped.
     }
   }
   else
   { std::cout<<"no file: "<<std::endl; }
}

void PFDGadgetWidget::paint()
{
    update();
}

void PFDGadgetWidget::paintEvent(QPaintEvent *event)
{
    // Skip painting until the dial file is loaded
    if (! m_renderer->isValid()) {
        std::cout<<"Dial file not loaded, not rendering"<<std::endl;
        return;
    }
   QGraphicsView::paintEvent(event);
}

// This event enables the dial to be dynamically resized
// whenever the gadget is resized, taking advantage of the vector
// nature of SVG dials.
void PFDGadgetWidget::resizeEvent(QResizeEvent *event)
{
    fitInView(m_background, Qt::KeepAspectRatio );
}


/*!
  \brief Update method for the vertical scales
  */
void PFDGadgetWidget::moveVerticalScales() {

}

// Take an input value and move the elements accordingly.
// Movement is smooth, starts fast and slows down when
// approaching the target.
//
void PFDGadgetWidget::moveNeedles()
{
    int dialCount = 5; // Gets decreased by one for each element
                       // which has finished moving

    /// TODO: optimize!!!

    //////
    // Roll
    //////
    double rollDiff;
    if ((abs((rollValue-rollTarget)*10) > 5)) {
        rollDiff =(rollTarget - rollValue)/5;
    } else {
        rollDiff = rollTarget - rollValue;
        dialCount--;
    }
    m_world->setRotation(m_world->rotation()+rollDiff);
    m_rollscale->setRotation(m_rollscale->rotation()+rollDiff);
    rollValue += rollDiff;

    //////
    // Pitch
    //////
    double pitchDiff;
    if ((abs((pitchValue-pitchTarget)*10) > 5)) {
        pitchDiff = (pitchTarget - pitchValue)/5;
    } else {
        pitchDiff = pitchTarget - pitchValue;
        dialCount--;
    }
    QPointF opd = QPointF(0,pitchDiff);
    m_world->setTransform(QTransform::fromTranslate(opd.x(),opd.y()), true);
    QPointF oop = m_world->transformOriginPoint();
    m_world->setTransformOriginPoint((oop.x()-opd.x()),(oop.y()-opd.y()));
    pitchValue += pitchDiff;

    //////
    // Heading
    //////
    double headingOffset = 0;
    double headingDiff;
    if ((abs((headingValue-headingTarget)*10) > 5)) {
        headingDiff = (headingTarget - headingValue)/5;
    } else {
        headingDiff = headingTarget-headingValue;
        dialCount--;
    }
    double threshold = -180*compassBandWidth/540;
    // Note: rendering can jump oh so very slightly when crossing the 180 degree
    // boundary, should not impact actual useability of the display.
    if ((headingValue < threshold) && ((headingValue+headingDiff)>=threshold)) {
        // We went over 180�: activate a -360 degree offset
        headingOffset = 2*threshold;
    } else if ((headingValue >= threshold) && ((headingValue+headingDiff)<threshold)) {
        // We went under 180�: remove the -360 degree offset
        headingOffset = -2*threshold;
    }
    opd = QPointF(headingDiff+headingOffset,0);
    m_compassband->setTransform(QTransform::fromTranslate(opd.x(),opd.y()), true);
    headingValue += headingDiff;

    //////
    // Speed
    //////
    if (abs(groundspeedValue-groundspeedTarget) > speedScaleHeight/120) {
        groundspeedValue += (groundspeedTarget-groundspeedValue)/5;
    } else {
        groundspeedValue = groundspeedTarget;
        dialCount--;
    }
    qreal x = m_speedscale->transform().dx();
    //opd = QPointF(x,fmod(groundspeedValue,speedScaleHeight/6));
    // fmod does rounding errors!! the formula below works better:
    opd = QPointF(x,groundspeedValue-floor(groundspeedValue/speedScaleHeight*6)*speedScaleHeight/6);
    m_speedscale->setTransform(QTransform::fromTranslate(opd.x(),opd.y()), false);
    // TODO: optimize this by skipping if not necessary...
    // Now update the text elements inside the scale:
    // (Qt documentation states that the list has the same order
    // as the item add order in the QGraphicsItemGroup)
    QList <QGraphicsItem *> textList = m_speedscale->childItems();
    qreal val = 5*floor(groundspeedValue/speedScaleHeight*6)+20;
    foreach( QGraphicsItem * item, textList) {
        if (QGraphicsTextItem *text = qgraphicsitem_cast<QGraphicsTextItem *>(item)) {
            QString s = (val<0) ? QString() : QString().sprintf("%.0f",val);
            text->setPlainText(s);
            val -= 5;
        }
    }

    //////
    // Altitude
    //////
    if (abs(altitudeValue-altitudeTarget) > altitudeScaleHeight/120) {
        altitudeValue += (altitudeTarget-altitudeValue)/5;
    } else {
        altitudeValue = altitudeTarget;
        dialCount--;
    }
    x = m_altitudescale->transform().dx();
    //opd = QPointF(x,fmod(altitudeValue,altitudeScaleHeight/6));
    // fmod does rounding errors!! the formula below works better:
    opd = QPointF(x,altitudeValue-floor(altitudeValue/altitudeScaleHeight*6)*altitudeScaleHeight/6);
    m_altitudescale->setTransform(QTransform::fromTranslate(opd.x(),opd.y()), false);
    // TODO: optimize this by skipping if not necessary...
    // Now update the text elements inside the scale:
    // (Qt documentation states that the list has the same order
    // as the item add order in the QGraphicsItemGroup)
    textList = m_altitudescale->childItems();
    val = 5*floor(altitudeValue/altitudeScaleHeight*6)+20;
    foreach( QGraphicsItem * item, textList) {
        if (QGraphicsTextItem *text = qgraphicsitem_cast<QGraphicsTextItem *>(item)) {
            QString s = (val<0) ? QString() : QString().sprintf("%.0f",val);
            text->setPlainText(s);
            val -= 5;
        }
    }

   //update();
   if (!dialCount)
       dialTimer.stop();
}
