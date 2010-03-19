/*
 * mapgadget.h
 *
 *  Created on: Mar 11, 2010
 *      Author: peter
 */

#ifndef MAPGADGET_H_
#define MAPGADGET_H_

#include <coreplugin/iuavgadget.h>

class IUAVGadget;
//class QList<int>;
class QWidget;
class QString;
class MapGadgetWidget;

using namespace Core;

class MapGadget : public Core::IUAVGadget
{
    Q_OBJECT
public:
    MapGadget(MapGadgetWidget *widget = 0);
    ~MapGadget();

    QList<int> context() const { return m_context; }
    QWidget *widget() { return m_widget; }
    QString contextHelpId() const { return QString(); }

    QWidget *toolBar() { return m_toolbar; }
private:
        QWidget *m_widget;
        QWidget *m_toolbar;
	QList<int> m_context;
};


#endif // MAPGADGET_H_
