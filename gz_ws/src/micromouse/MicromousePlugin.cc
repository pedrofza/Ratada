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

#include "MicromousePlugin.hh"

#include <gz/msgs/details/empty.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity_factory_v.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <gz/gui/Helpers.hh>

#include <QQuaternion>
#include <qquaternion.h>

using namespace custom;
using namespace gz;

class custom::MicromousePluginPrivate
{
public:
    void AskForMaze();
    void AskForRobotPose();
    void AskForRobot();
    transport::Node node;
    std::string worldname;
    QString mazefile;
    QString robotfile;
    QVector3D position;
    QVector3D orientation;
};

MicromousePlugin::MicromousePlugin()
    : dataPtr(std::make_unique<MicromousePluginPrivate>())
{
}

MicromousePlugin::~MicromousePlugin()
{
}

void MicromousePlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
    if (this->title.empty())
    {
        this->title = "Micromouse";
    }

    this->dataPtr->worldname = gz::gui::worldNames()[0].toStdString();

    if (!_pluginElem)
        return;

    // Take maze filename during runtime
    auto messageElem = _pluginElem->FirstChildElement("mazefile");
    if (nullptr != messageElem && nullptr != messageElem->GetText())
    {
        QString url(messageElem->GetText());
        this->SetMazefile(url);
        gzmsg << "Micromouse loaded maze" << url.toStdString() << std::endl;
    }

    messageElem = _pluginElem->FirstChildElement("robotfile");
    if (nullptr != messageElem && nullptr != messageElem->GetText())
    {
        QString url(messageElem->GetText());
        this->SetRobotfile(url);
        gzmsg << "Robot set to " << url.toStdString() << std::endl;
    }
}

void MicromousePlugin::Update(const sim::UpdateInfo &_info,
                              sim::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
}

void MicromousePluginPrivate::AskForMaze()
{
    QFile file{mazefile};
    bool open_result = file.open(QIODevice::ReadOnly | QIODevice::Text);
    if (!open_result) {
        gzerr << "Failed to open file [" << mazefile.toStdString() << "]" << std::endl;
        return;
    }

    QTextStream in(&file);
    QString content = in.readAll();
    file.close();

    std::string topic = "/micromouse/set_maze";
    msgs::StringMsg req;
    req.set_data(content.toStdString());
    int timeout = 3500;
    msgs::Boolean rep;
    bool result;
    bool executed = this->node.Request(topic, req, timeout, rep, result);
    if (!executed) {
        gzerr << "Failed to request service [" << topic << "]" << std::endl;
        gzerr << req.data() << std::endl;
        return;
    }
    if (result) {
        if (!rep.data()) {
            gzerr << "Invalid maze" << std::endl;
        }
    } else {
        gzerr << "Failed service call [" << topic << "]" << std::endl;
    }
}


void MicromousePluginPrivate::AskForRobot()
{
    QFile file{robotfile};
    bool open_result = file.open(QIODevice::ReadOnly | QIODevice::Text);
    if (!open_result) {
        gzerr << "Failed to open file [" << robotfile.toStdString() << "]" << std::endl;
        return;
    }

    QTextStream in(&file);
    QString content = in.readAll();
    file.close();

    std::string topic = "/micromouse/set_robot";
    msgs::StringMsg req;
    req.set_data(content.toStdString());
    int timeout = 3500;
    msgs::Boolean rep;
    bool result;
    bool executed = this->node.Request(topic, req, timeout, rep, result);
    if (!executed) {
        gzerr << "Failed to request service [" << topic << "]" << std::endl;
        return;
    }
    if (result) {
        if (!rep.data()) {
            gzerr << "Invalid maze" << std::endl;
        }
    } else {
        gzerr << "Failed service call [" << topic << "]" << std::endl;
    }
}

void MicromousePluginPrivate::AskForRobotPose()
{
    std::string topic = "/micromouse/set_robot_pose";
    msgs::Pose req;
    req.mutable_position()->set_x(this->position.x());
    req.mutable_position()->set_y(this->position.y());
    req.mutable_position()->set_z(this->position.z());
    auto q = QQuaternion::fromEulerAngles(this->orientation.x(), this->orientation.y(), this->orientation.z());
    req.mutable_orientation()->set_w(q.scalar());
    req.mutable_orientation()->set_x(q.x());
    req.mutable_orientation()->set_y(q.y());
    req.mutable_orientation()->set_z(q.z());
    int timeout = 3500;
    msgs::Boolean rep;
    bool result;
    bool executed = this->node.Request(topic, req, timeout, rep, result);
    if (!executed) {
        gzerr << "Failed to request service [" << topic << "]" << std::endl;
        return;
    }
    if (result) {
        if (!rep.data()) {
            gzerr << "Invalid maze" << std::endl;
        }
    } else {
        gzerr << "Failed service call [" << topic << "]" << std::endl;
    }
}

void MicromousePlugin::OnLoadButton()
{
    this->dataPtr->AskForMaze();
    this->dataPtr->AskForRobotPose();
    this->dataPtr->AskForRobot();
}

void MicromousePlugin::OnResetButton()
{
    this->dataPtr->AskForRobotPose();
    std::string reset = "micromouse/reset";;
    msgs::StringMsg req;
    int timeout = 3500;
    msgs::StringMsg rep;
    bool result;
    this->dataPtr->node.Request(reset, req, timeout, rep, result);
}

void MicromousePlugin::StartMicromouse()
{
    std::string topic = "micromouse/start_comp";
    msgs::Empty req;
    int timeout = 3500;
    msgs::Boolean rep;
    bool result;
    bool executed = this->dataPtr->node.Request(topic, req, timeout, rep, result);
    if (!executed) {
        gzerr << "Failed to request service [" << topic << "]" << std::endl;
        return;
    }
    if (result) {
        if (!rep.data()) {
            gzerr << "Could not start competition" << std::endl;
        }
    } else {
        gzerr << "Failed service call [" << topic << "]" << std::endl;
    }
}

QString MicromousePlugin::QurlToQString(QUrl url) const
{
    return url.toLocalFile();
}

QString MicromousePlugin::Mazefile() const
{
    return this->dataPtr->mazefile;
}

void MicromousePlugin::SetMazefile(const QString &_mazefile)
{
    this->dataPtr->mazefile = _mazefile;
    this->MazefileChanged();
}

QString MicromousePlugin::Robotfile() const
{
    return this->dataPtr->robotfile;
}

void MicromousePlugin::SetRobotfile(const QString &_robotfile)
{
    this->dataPtr->robotfile = _robotfile;
    this->RobotfileChanged();
}

void MicromousePlugin::SetOrientation(const QVector3D &_orientation)
{
    this->dataPtr->orientation = _orientation;
    this->OrientationChanged();
}

void MicromousePlugin::SetPosition(const QVector3D &_position)
{
    this->dataPtr->position = _position;
    this->PositionChanged();
}

QVector3D MicromousePlugin::Position() const
{
    return this->dataPtr->position;
}

QVector3D MicromousePlugin::Orientation() const
{
    return this->dataPtr->orientation;
}

GZ_ADD_PLUGIN(MicromousePlugin, sim::GuiSystem, gui::Plugin);
