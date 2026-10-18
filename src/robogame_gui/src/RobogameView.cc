#include "RobogameView.hh"

#include <QMetaObject>
#include <QQuickWindow>
#include <QTimer>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>

namespace robogame_gui
{
static const char kViewTopic[] = "/robogame/view";
static const char kStateTopic[] = "/robogame/view/state";
static const char kAspectTopic[] = "/robogame/view/aspect";

// The item named name under item. QML items hang off their parent item,
// not always as its QObject child, so findChild can miss them.
static QQuickItem *FindItem(QQuickItem *_item, const QString &_name)
{
  if (!_item)
    return nullptr;
  if (_item->objectName() == _name)
    return _item;
  for (auto child : _item->childItems())
  {
    if (auto found = FindItem(child, _name))
      return found;
  }
  return nullptr;
}

RobogameView::RobogameView() = default;

void RobogameView::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Camera view";

  this->publisher =
      this->node.Advertise<gz::msgs::StringMsg>(kViewTopic);
  this->node.Subscribe(kStateTopic, &RobogameView::OnState, this);

  // Once a second rather than on resize only, for an fpv_toggle that
  // starts after the window.
  this->aspectPublisher =
      this->node.Advertise<gz::msgs::Double>(kAspectTopic);
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &RobogameView::SendAspect);
  timer->start(1000);
}

void RobogameView::SendAspect()
{
  if (!this->renderWindow)
  {
    auto window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
    if (!window || !window->QuickWindow())
      return;
    this->renderWindow =
        FindItem(window->QuickWindow()->contentItem(), "rw");
    if (!this->renderWindow)
      return;
  }

  if (this->renderWindow->height() <= 0)
    return;

  gz::msgs::Double msg;
  msg.set_data(this->renderWindow->width() / this->renderWindow->height());
  this->aspectPublisher.Publish(msg);
}

void RobogameView::Choose(const QString &_view, const QString &_robot)
{
  gz::msgs::StringMsg msg;
  msg.set_data((_view + " " + _robot).toStdString());
  this->publisher.Publish(msg);
}

void RobogameView::OnState(const gz::msgs::StringMsg_V &_msg)
{
  if (_msg.data_size() < 2)
    return;

  QString newView = QString::fromStdString(_msg.data(0));
  QString newRobot = QString::fromStdString(_msg.data(1));
  QStringList newRobots;
  for (int i = 2; i < _msg.data_size(); ++i)
    newRobots << QString::fromStdString(_msg.data(i));

  // Properties change on the Qt thread only.
  QMetaObject::invokeMethod(this, [=, this]()
  {
    if (newView == this->view && newRobot == this->robot &&
        newRobots == this->robots)
      return;

    this->view = newView;
    this->robot = newRobot;
    this->robots = newRobots;
    emit this->StateChanged();
  }, Qt::QueuedConnection);
}
}  // namespace robogame_gui

GZ_ADD_PLUGIN(robogame_gui::RobogameView, gz::gui::Plugin)
