#include "control_panel/RobotWorkspace.h"

#include <QListWidget>

#include <QDial>
#include <QCheckBox>
#include <QMenu>

Q_DECLARE_METATYPE(QList<QUuid>)

RobotWorkspace::RobotWorkspace(QWidget *parent, const QUuid _uuid, RobotTab *_rt) :
    QSplitter(parent),
    uuid(_uuid),
    rt(_rt)
{
    this->setAcceptDrops(true);

    // Set our orientation
    if(rt->settings.contains(uuid.toString() + "/Orientation"))
        setOrientation(Qt::Orientation(rt->settings.value(uuid.toString() + "/Orientation").toInt()));
    else
        rt->settings.setValue(uuid.toString() + "/Orientation", (int)orientation());

    // If there are children, restore them
    if(rt->settings.contains(uuid.toString() + "/children"))
    {
        // Get the UUID list
        QList<QUuid> child_list = rt->settings.value(uuid.toString() + "/children").value<QList<QUuid> >();
        if(child_list.count())
        {
            for(int i = 0; i < child_list.count(); i++)
            {
                // Make sure this isn't a dead reference
                if(!rt->settings.childGroups().contains(child_list[i].toString()))
                    continue;
                // Find out what type it is
                if(rt->settings.contains(child_list[i].toString() + "/WidgetType"))
                {
                    // Good to go! Make the new widget and append it to the our QSplitter
                    control_panel::ControlPanelPlugin *nw = NULL;
                    nw = rt->cp->ros_interface.createNewPluginInstance(rt->settings.value(child_list[i].toString() + "/WidgetType").toString(), rt->settings.value("namespace").toString());
                    if(!nw)
                    {
                        std::cerr << "Failed to load plugin \"" << rt->settings.value(child_list[i].toString() + "/WidgetType").toString().toStdString() << "\" (uuid " << child_list[i].toString().toStdString() << ")" << std::endl;
                        continue;
                    }
                    nw->settings = &rt->settings;
                    nw->uuid = child_list[i];

                    connect(nw, SIGNAL(destroyed()), this, SLOT(checkChildren()));
                    connect(nw, SIGNAL(unload(QString,QString)), rt->cp, SLOT(unloadNodeletPlugin(QString,QString)));
                    connect(nw, SIGNAL(setKeyCB(control_panel::ControlPanelPlugin*,bool)), rt, SLOT(setKeyCB(control_panel::ControlPanelPlugin*,bool)));

                    nw->setup();

                    insertWidget(i, nw);
                    nw->setParent(this);
                }
                else
                {
                    // Just another workspace. It will add its own children.
                    RobotWorkspace *rs = NULL;
                    rs = new RobotWorkspace(0, child_list[i], rt);
                    rs->setMinimumSize(5, 5);
                    if(!rs)
                    {
                        std::cerr << "Failed to load workspace (uuid " << child_list[i].toString().toStdString() << ")" << std::endl;
                        continue;
                    }
                    if(rs->count() < 1)
                    {
                        std::cerr << "WARN: Workspace was empty, removing (uuid " << child_list[i].toString().toStdString() << ")" << std::endl;
                        std::cerr << "-> Likely caused by a previous fault. This can be safely ignored." << std::endl;
                        delete rs;
                        rt->settings.remove(child_list[i].toString());
                        continue;
                    }
                    insertWidget(i, rs);
                    connect(rs, SIGNAL(destroyed()), this, SLOT(checkChildren()));
                    rs->setParent(this);
                }
            }

            // Restore Layout
            if(rt->settings.contains(uuid.toString() + "/layout"))
                restoreState(rt->settings.value(uuid.toString() + "/layout").toByteArray());

            childrenUpdate();
            layoutUpdate();
        }
    }

    this->setChildrenCollapsible(false);
    connect(this, SIGNAL(splitterMoved(int, int)), this, SLOT(layoutUpdate()));
}

RobotWorkspace::~RobotWorkspace()
{
    for(unsigned int i = 0; i < count(); i++)
        widget(i)->deleteLater();
}

void RobotWorkspace::dragEnterEvent(QDragEnterEvent *event)
{
    if(dynamic_cast<QListWidget *>(event->source()))
        event->acceptProposedAction();
}

void RobotWorkspace::dragLeaveEvent(QDragLeaveEvent *event)
{
    if(event)
        event = NULL;
}

unsigned char RobotWorkspace::getQuad(const QPoint &p, const int &w, const int &h)
{
    const float m = (float)h / w;
    if(p.y() > m * p.x())
    {
        if(p.y() > -m * p.x() + h)
            return 1;
        else
            return 2;
    }
    else
    {
        if(p.y() > -m * p.x() + h)
            return 0;
        else
            return 3;
    }
}

void RobotWorkspace::dropEvent(QDropEvent *event)
{
    QListWidget *drag = dynamic_cast<QListWidget *>(event->source());
    if(!drag)
    {
        event->ignore();
        return;
    }
    else
    {
        if(addNewWidget(event->pos(), drag->currentItem()->text()))
            event->accept();
        else
            event->ignore();
    }
}

void RobotWorkspace::checkChildren()
{
    childrenUpdate();

    if(count() == 1)
    {
        if(dynamic_cast<RobotWorkspace *>(parentWidget()))
        {
            // remove our config
            rt->settings.remove(uuid.toString());

            deleteLater();
        }
    }
}

control_panel::ControlPanelPlugin * RobotWorkspace::getNewWidget(const QString &wn)
{
    control_panel::ControlPanelPlugin *nw = rt->cp->ros_interface.createNewPluginInstance(wn, rt->settings.value("namespace").toString());
    if(NULL == nw)
        return NULL;
    nw->settings = &rt->settings;
    nw->uuid = QUuid::createUuid();
    nw->settings->setValue(nw->uuid.toString() + "/WidgetType", wn);
    connect(nw, SIGNAL(setKeyCB(control_panel::ControlPanelPlugin*,bool)), rt, SLOT(setKeyCB(control_panel::ControlPanelPlugin*,bool)));
    nw->setup();
    nw->start();
    return nw;
}

bool RobotWorkspace::addNewWidget(const QPoint &p, const QString &wn)
{
    control_panel::ControlPanelPlugin *nw = getNewWidget(wn);
    connect(nw, SIGNAL(unload(QString,QString)), rt->cp, SLOT(unloadNodeletPlugin(QString,QString)));
    int pos = 0;

    if(this->count() == 1)
    {
        RobotWorkspace *pp = dynamic_cast<RobotWorkspace *>(widget(0));
        if(pp)
            return pp->addNewWidget(widget(0)->mapFrom(this, p), wn);

        const unsigned char quad = getQuad(p, width(), height());
        if( quad == 0 || quad == 2)
            setOrientation(Qt::Horizontal);
        else
            setOrientation(Qt::Vertical);
        rt->settings.setValue(uuid.toString() + "/Orientation", (int)orientation());
        if( quad == 0 || quad == 1)
            pos = count();
        //else
        //    pos = 0;
    }
    else if(this->count() > 1)
    {
        int i;
        QPoint n;

        for(i = 0; i < count(); i++)
        {
            n = widget(i)->mapFrom(this, p);
            if(n.y() >= 0 && n.x() >= 0 && n.y() < widget(i)->height() && n.x() < widget(i)->width())
                break;
        }
        if(i == count())
            return false;

        RobotWorkspace *pp = dynamic_cast<RobotWorkspace *>(widget(i));
        if(pp)
            return pp->addNewWidget(n, wn);

        const unsigned char quad = getQuad(n, widget(i)->width(), widget(i)->height());
        if((orientation() == Qt::Horizontal && (quad == 0 || quad == 2)) || (orientation() == Qt::Vertical && (quad == 1 || quad == 3)))
        {
            if(quad == 0 || quad == 1)
                pos = i + 1;
            else
                pos = i;
        }
        else
        {
            RobotWorkspace *rs = new RobotWorkspace(0, QUuid::createUuid(), rt);
            if(quad == 0 || quad == 2)
                rs->setOrientation(Qt::Horizontal);
            else
                rs->setOrientation(Qt::Vertical);
            rt->settings.setValue(uuid.toString() + "/Orientation", (int)rs->orientation());
            const QByteArray layout = saveState();
            disconnect(widget(i), SIGNAL(destroyed()), this, SLOT(checkChildren()));
            widget(i)->setParent(rs);
            connect(rs->widget(0), SIGNAL(destroyed()), rs, SLOT(checkChildren()));
            if(quad == 0 || quad == 1)
                rs->addWidget(nw);
            else
                rs->insertWidget(0, nw);
            rs->childrenUpdate();
            nw->setParent(rs);
            insertWidget(i, rs);
            rs->setParent(this);
            restoreState(layout);
            connect(nw, SIGNAL(destroyed()), rs, SLOT(checkChildren()));
            connect(rs, SIGNAL(destroyed()), this, SLOT(checkChildren()));
	    nw->setFocus();
            childrenUpdate();
            return true;
        }
    }

    if(nw)
    {
        insertWidget(pos, nw);
        connect(nw, SIGNAL(destroyed()), this, SLOT(checkChildren()));
        nw->setParent(this);
	nw->setFocus();
    }

    childrenUpdate();
    return true;
}

void RobotWorkspace::layoutUpdate()
{
    rt->settings.setValue(uuid.toString() + "/layout", saveState());
}

void RobotWorkspace::childrenUpdate()
{
    QList<QUuid> children_uuids;
    for(int i = 0; i < count(); i++)
    {
        RobotWorkspace *rw = NULL;
        if((rw = dynamic_cast<RobotWorkspace *>(widget(i))))
        {
            children_uuids.append(rw->uuid);
        }
        else
        {
            control_panel::ControlPanelPlugin *rp = dynamic_cast<control_panel::ControlPanelPlugin *>(widget(i));
            if(!rp)
            {
                // We will assume that this is the one that was destroyed
                continue;
            }
            children_uuids.append(rp->uuid);
        }
    }

    rt->settings.setValue(uuid.toString() + "/children", QVariant::fromValue< QList<QUuid> >(children_uuids));
    layoutUpdate();
}
