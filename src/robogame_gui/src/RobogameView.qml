import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: robogameView
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 150
  anchors.fill: parent

  // The names fpv_toggle takes, with the key that does the same.
  ListModel {
    id: views
    ListElement { name: "top"; label: "Top down (1)" }
    ListElement { name: "front"; label: "Top front (2)" }
    ListElement { name: "front_right"; label: "Top front right (3)" }
    ListElement { name: "fpv"; label: "Robot FPV (4)" }
  }

  function viewIndex(name) {
    for (var i = 0; i < views.count; ++i) {
      if (views.get(i).name === name)
        return i;
    }
    return -1;
  }

  // Follows fpv_toggle, so the keys show here too. Set here rather than
  // bound: a combo box drops its binding once the user picks from it.
  function follow() {
    viewBox.currentIndex = viewIndex(_RobogameView.view)
    robotBox.currentIndex = _RobogameView.robots.indexOf(_RobogameView.robot)
  }

  Connections {
    target: _RobogameView
    function onStateChanged() { follow() }
  }

  Component.onCompleted: follow()

  GridLayout {
    anchors.fill: parent
    anchors.margins: 10
    columns: 2

    Label {
      text: "View"
    }
    ComboBox {
      id: viewBox
      Layout.fillWidth: true
      model: views
      textRole: "label"
      // onActivated is the user's choice only, not follow().
      onActivated: function(index) {
        _RobogameView.Choose(views.get(index).name, robotBox.currentText)
      }
      // No FPV without the robots' cameras.
      delegate: ItemDelegate {
        width: viewBox.width
        text: model.label
        enabled: model.name !== "fpv" || _RobogameView.robots.length > 0
        highlighted: viewBox.highlightedIndex === index
      }
    }

    Label {
      text: "Robot"
    }
    ComboBox {
      id: robotBox
      Layout.fillWidth: true
      model: _RobogameView.robots
      enabled: _RobogameView.robots.length > 0
      // Picking a robot looks through it, as F1 to F9 do.
      onActivated: function(index) {
        _RobogameView.Choose("fpv", _RobogameView.robots[index])
      }
    }

    Label {
      Layout.columnSpan: 2
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      color: "gray"
      text: _RobogameView.robots.length > 0 ?
          "Keys in the 3D view: 1 2 3 views, 4 FPV, F1 to F9 robot" :
          "Keys in the 3D view: 1 2 3 views. No FPV, fpv:=false"
    }
  }
}
