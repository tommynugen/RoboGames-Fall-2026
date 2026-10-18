#include "RobogameView.hh"

#include <QMetaObject>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>

namespace robogame_gui
{
static const char kViewTopic[] = "/robogame/view";
static const char kStateTopic[] = "/robogame/view/state";

RobogameView::RobogameView() = default;

void RobogameView::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Camera view";

  this->publisher =
      this->node.Advertise<gz::msgs::StringMsg>(kViewTopic);
  this->node.Subscribe(kStateTopic, &RobogameView::OnState, this);
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
