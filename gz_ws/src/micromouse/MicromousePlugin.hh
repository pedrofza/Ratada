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

#ifndef MICROMOUSEPLUGIN_HH_
#define MICROMOUSEPLUGIN_HH_

#include <gz/sim/gui/GuiSystem.hh>
#include <memory>

#include <QVector3D>
#include <QString>

namespace custom
{

class MicromousePluginPrivate;

class MicromousePlugin : public gz::sim::GuiSystem
{

    Q_OBJECT

    Q_PROPERTY(
        QString mazefile
        READ Mazefile
        WRITE SetMazefile
        NOTIFY MazefileChanged
    )

    Q_PROPERTY(
        QString robotfile
        READ Robotfile
        WRITE SetRobotfile
        NOTIFY RobotfileChanged
    )

    Q_PROPERTY(
        QVector3D position
        READ Position
        WRITE SetPosition
        NOTIFY PositionChanged
    )

    Q_PROPERTY(
        QVector3D orientation
        READ Orientation
        WRITE SetOrientation
        NOTIFY OrientationChanged
    )

public:
    MicromousePlugin();
    virtual ~MicromousePlugin();
    virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
    void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
    Q_INVOKABLE QString QurlToQString(QUrl url) const;

protected slots:
    void OnLoadButton();
    void OnResetButton();
    void StartMicromouse();

public:
    Q_INVOKABLE QString Mazefile() const;
    Q_INVOKABLE QString Robotfile() const;
    Q_INVOKABLE QVector3D Position() const;
    Q_INVOKABLE QVector3D Orientation() const;
    Q_INVOKABLE void SetMazefile(const QString &_mazefile);
    Q_INVOKABLE void SetRobotfile(const QString &_robotfile);
    Q_INVOKABLE void SetPosition(const QVector3D &_position);
    Q_INVOKABLE void SetOrientation(const QVector3D &_orientation);

signals:
    void MazefileChanged();
    void RobotfileChanged();
    void PositionChanged();
    void OrientationChanged();

private:
    std::unique_ptr<MicromousePluginPrivate> dataPtr;
};

}

#endif // MICROMOUSEPLUGIN_HH_
