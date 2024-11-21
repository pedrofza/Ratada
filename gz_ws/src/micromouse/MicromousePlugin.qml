/*
 * Copyright 2024 Pedro Fontoura Zawadniak
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

import QtQuick 2.9
import QtQuick.Controls 2.15
import QtQuick.Dialogs 1.3
import QtQuick.Layouts 1.15
import "qrc:/qml"

GridLayout {
    columns: 8
    columnSpacing: 10
    Layout.minimumWidth: 350
    Layout.minimumHeight: 550
    anchors.fill: parent
    anchors.leftMargin: 10
    anchors.rightMargin: 10

    TextField {
        Layout.columnSpan: 7
        Layout.fillWidth: true
        id: textField
    }

    FileDialog {
        id: mazeFileDialog
        title: "Please choose a file"
        folder: shortcuts.home
        onAccepted: {
            var url = mazeFileDialog.fileUrl;
            var urlStr = MicromousePlugin.QurlToQString(url);
            MicromousePlugin.SetMazefile(urlStr);
            textField.text = urlStr;
        }
    }

    Button {
        Layout.columnSpan: 1
        text: qsTr("Select maze")
        onClicked: { mazeFileDialog.open(); }
    }

    TextField {
        Layout.columnSpan: 7
        Layout.fillWidth: true
        id: robotTextField
    }

    FileDialog {
        id: robotFileDialog
        title: "Please choose a file"
        folder: shortcuts.home
        onAccepted: {
            var url = robotFileDialog.fileUrl;
            var urlStr = MicromousePlugin.QurlToQString(url);
            MicromousePlugin.SetRobotfile(urlStr);
            robotTextField.text = urlStr;
        }
    }

    Button {
        Layout.columnSpan: 1
        text: qsTr("Select Robot")
        onClicked: { robotFileDialog.open(); }
    }

    Button {
        Layout.columnSpan: 8
        Layout.fillWidth: true
        text: qsTr("Load")
        highlighted: true
        onClicked: { MicromousePlugin.OnLoadButton(); }
    }

    Button {
        Layout.columnSpan: 8
        Layout.fillWidth: true
        text: qsTr("Reset")
        highlighted: true
        onClicked: { MicromousePlugin.OnResetButton(); }
    }

    Button {
        Layout.columnSpan: 8
        Layout.fillWidth: true
        text: qsTr("Start!");
        onClicked: {MicromousePlugin.StartMicromouse(); }
    }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: positionXText
    color: "black"
    text: qsTr("X (mm)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: positionX
    editable: true
    onValueChanged: MicromousePlugin.position.x = positionX.value / 1000

    from: -10000
    value: 0
    to: 10000
    stepSize: 1
  }
  
  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: positionYText
    color: "black"
    text: qsTr("Y (mm)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: positionY
    editable: true
    onValueChanged: MicromousePlugin.position.y = positionY.value / 1000

    from: -10000
    value: 0
    to: 10000
    stepSize: 1
  }
  
  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: positionZText
    color: "black"
    text: qsTr("Z (mm)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: positionZ
    editable: true
    onValueChanged: MicromousePlugin.position.z = positionZ.value / 1000

    from: -10000
    value: 0
    to: 10000
    stepSize: 1
  }
  





  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: orientationXText
    color: "black"
    text: qsTr("Roll (deg)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: orientationX
    editable: true
    onValueChanged: MicromousePlugin.orientation.x = orientationX.value

    from: -360
    value: 0
    to: 360
    stepSize: 1
  }
  
  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: orientationYText
    color: "black"
    text: qsTr("Pitch (deg)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: orientationY
    editable: true
    onValueChanged: MicromousePlugin.orientation.y = orientationY.value

    from: -360
    value: 0
    to: 360
    stepSize: 1
  }
  
  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: orientationZText
    color: "black"
    text: qsTr("Yaw (deg)")
  }

  SpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: orientationZ
    editable: true
    onValueChanged: MicromousePlugin.orientation.z = orientationZ.value

    from: -360
    value: 0
    to: 360
    stepSize: 1
  }
  
}
