// Camera view card for the Gazebo window's side panel.
//
// Two drop downs, the view and the robot, that ask robogame_sim's
// fpv_toggle for a view on /robogame/view, "top robot_2" or
// "fpv robot_1". fpv_toggle publishes what it shows on
// /robogame/view/state, the view, the robot, then every robot with an
// FPV camera, so the drop downs follow the 1 to 4 keys too.

#ifndef ROBOGAME_GUI_ROBOGAME_VIEW_HH_
#define ROBOGAME_GUI_ROBOGAME_VIEW_HH_

#include <QString>
#include <QStringList>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/transport/Node.hh>

namespace robogame_gui
{
class RobogameView : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString view READ View NOTIFY StateChanged)
  Q_PROPERTY(QString robot READ Robot NOTIFY StateChanged)
  Q_PROPERTY(QStringList robots READ Robots NOTIFY StateChanged)

 public:
  RobogameView();

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  QString View() const { return view; }
  QString Robot() const { return robot; }
  QStringList Robots() const { return robots; }

  // Asks fpv_toggle for view, looking through robot for "fpv".
  Q_INVOKABLE void Choose(const QString &_view, const QString &_robot);

 signals:
  void StateChanged();

 private:
  // On a gz-transport thread: hands the state to the Qt thread.
  void OnState(const gz::msgs::StringMsg_V &_msg);

  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;

  QString view;
  QString robot;
  QStringList robots;
};
}  // namespace robogame_gui

#endif
