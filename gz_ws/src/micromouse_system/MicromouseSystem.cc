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

#include "MicromouseSystem.hh"
#include <chrono>
#include <gz/math/Pose3.hh>
#include <gz/msgs/details/time.pb.h>
#include <gz/sim/Types.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <sdf/Root.hh>
#include <mutex>
#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components.hh>
#include <cmath>

using namespace gz;
using namespace custom;
using maze_coordinate = int;

enum class CompetitionState
{
    configuring,
    running,
    over
};

enum class MazeThing
{
    pole,
    hwall,
    vwall,
    hspace,
    vspace,
    cspace
};

struct MazeCoordinate
{
    MazeCoordinate(maze_coordinate x_, maze_coordinate y_);
    maze_coordinate x;
    maze_coordinate y;
};

MazeCoordinate::MazeCoordinate(maze_coordinate x_, maze_coordinate y_)
    : x(x_), y(y_)
{
}

class PhysicalDims
{
public:
    double maze_height;
    double thickness;
    double wall_length;
    bool validate() const;
};

bool PhysicalDims::validate() const
{
    return maze_height > 0 && thickness > 0 && wall_length > 0;
}

class Maze
{
public:
    static Maze fromMicromouseOnline(std::string maze_string);
    bool validate() const;
    std::string toSdf() const;

    std::vector<std::vector<MazeThing>> things;

    static const std::string goal_link_name;
    static const std::string start_link_name;
    std::vector<MazeCoordinate> goals;
    std::vector<MazeCoordinate> starts;
    PhysicalDims descr;
};

const std::string Maze::goal_link_name = "goal_link";
const std::string Maze::start_link_name = "start_link";

std::string reverse_string(std::string in)
{
    std::vector<std::string> lines;
    std::stringstream iss(in);
    for (std::string line; std::getline(iss, line);) {
        lines.push_back(line);
    }
    std::reverse(lines.begin(), lines.end());
    std::ostringstream os;
    for (const auto& line : lines) {
        os << line << "\n";
    }
    return os.str();
}

std::string area_obj(std::string name, int id, double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z);

std::string start_obj(int id, double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z)
{
    std::ostringstream ss;
    ss << area_obj("start", id, pos_x, pos_y, pos_z, size_x, size_y, size_z);
    ss << R"(<sensor xmlns:gz="https://gazebosim.org/api/sensors/8/custom_sensors.html" name="start" type="custom" gz:type="competition">)" << "\n";
    ss << R"(<gz:competition><role>1</role></gz:competition>)" << "\n";
    ss << "</sensor>" << "\n";
    return ss.str();
}

std::string goal_obj(int id, double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z)
{
    std::ostringstream ss;
    ss << area_obj("goal", id, pos_x, pos_y, pos_z, size_x, size_y, size_z);
    ss << R"(<sensor xmlns:gz="https://gazebosim.org/api/sensors/8/custom_sensors.html" name="goal" type="custom" gz:type="competition">)" << "\n";
    ss << R"(<gz:competition><role>2</role></gz:competition>)" << "\n";
    ss << "</sensor>" << "\n";
    return ss.str();
}

std::string area_obj(std::string name, int id, double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z) {
    std::ostringstream ss;
    // Area visual
    ss << R"(<visual name="visual_)" << name << "_" << id << R"(">)" << "\n";
    ss << "<visibility_flags>0</visibility_flags>" << "\n";
    ss << "<pose>" << pos_x << " " << pos_y << " " << pos_z << " 0 0 0</pose>" << "\n";
    ss << "<geometry><box><size>" << size_x << " " << size_y << " " << size_z << "</size></box></geometry>" << "\n";
    ss << "<material><ambient>1 1 1 0.15</ambient><diffuse>1 1 1 0.15</diffuse><specular>1 1 1 0.15</specular></material>" << "\n";
    ss << "</visual>" << "\n";

    // Area collision
    ss << R"(<collision name="collision_)" << name << "_" << id << R"(">)" << "\n";
    ss << "<surface><contact>" << "\n";
    ss << "<collide_bitmask>0</collide_bitmask>" << "\n";  // Area doesn't collide with anything, but still generates contacts (I assume)
    ss << "</contact></surface>" << "\n";
    ss << "<pose>" << pos_x << " " << pos_y << " " << pos_z << " 0 0 0</pose>" << "\n";
    ss << "<geometry><box><size>" << size_x << " " << size_y << " " << size_z << "</size></box></geometry>" << "\n";
    ss << "</collision>" << "\n";

    return ss.str();
}

std::string maze_obj(double id, double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z) {
static double constexpr MAZE_RED_TAPE_VISUAL_OFFSET = 0.00005;
    std::ostringstream ss;
    // Wall visual
    ss << R"(<visual name="visual_)" << id << R"(">)" << "\n";
    ss << "<pose>" << pos_x << " " << pos_y << " " << pos_z << " 0 0 0</pose>" << "\n";
    ss << "<geometry><box><size>" << size_x << " " << size_y << " " << size_z << "</size></box></geometry>" << "\n";
    ss << "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>1 1 1 1</specular></material>" << "\n";
    ss << "</visual>" << "\n";

    // Red wall top visual
    ss << R"(<visual name="tape_visual_)" << id << R"(">)" << "\n";
    ss << "<pose>" << pos_x << " " << pos_y << " " << (pos_z + size_z / 2) + MAZE_RED_TAPE_VISUAL_OFFSET << " 0 0 0</pose>" << "\n";
    ss << "<geometry><plane><normal>0 0 1</normal><size>" << size_x << " " << size_y << " " << size_z << "</size></plane></geometry>" << "\n";
    ss << "<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse><specular>1 0 0 1</specular></material>" << "\n";
    ss << "</visual>" << "\n";

    // Wall collision
    ss << R"(<collision name="collision_)" << id << R"(">)" << "\n";
    ss << "<pose>" << pos_x << " " << pos_y << " " << pos_z << " 0 0 0</pose>" << "\n";
    ss << "<geometry><box><size>" << size_x << " " << size_y << " " << size_z << "</size></box></geometry>" << "\n";
    ss << "</collision>" << "\n";
    return ss.str();
}

Maze Maze::fromMicromouseOnline(std::string maze_string)
{
    Maze maze;
    maze.descr.maze_height = 0.05;
    maze.descr.thickness = 0.012;
    maze.descr.wall_length = 0.168;
    std::istringstream iss(reverse_string(maze_string));
    size_t line_idx = 0;
    maze_coordinate pos_x = 0;
    maze_coordinate pos_y = 0;
    for (std::string line; std::getline(iss, line);) {
        std::vector<MazeThing> things;
        pos_x = 0;
        if (line_idx % 2 == 0) {  // Horizontal wall line
            size_t idx_in_line = 0;
            while (idx_in_line < line.length()) {
                const char* a = line.c_str() + idx_in_line;
                if (strncmp(a, "o", 1) == 0) {
                    idx_in_line += 1;
                    things.push_back(MazeThing::pole);
                } else if (strncmp(a, "   ", 3) == 0) {
                    idx_in_line += 3;
                    things.push_back(MazeThing::hspace);
                } else if (strncmp(a, "---", 3) == 0) {
                    idx_in_line += 3;
                    things.push_back(MazeThing::hwall);
                }
                ++pos_x;
            }
        } else {  // Vertical wall line
            size_t idx_in_line = 0;
            while (idx_in_line < line.length()) {
                const char* a = line.c_str() + idx_in_line;
                if (idx_in_line % 4 == 0) {
                    size_t char_sz = 1;
                    if (strncmp(a, " ", char_sz) == 0) {
                        idx_in_line += char_sz;
                        things.push_back(MazeThing::vspace);
                    } else if (strncmp(a, "|", char_sz) == 0) {
                        idx_in_line += char_sz;
                        things.push_back(MazeThing::vwall);
                    }
                } else {
                    size_t char_sz = 3;
                    if (strncmp(a, "   ", char_sz) == 0) {
                        idx_in_line += char_sz;
                        things.push_back(MazeThing::cspace);
                    } else if (strncmp(a, " G ", char_sz) == 0) {
                        maze.goals.push_back({pos_x, pos_y});
                        idx_in_line += char_sz;
                        things.push_back(MazeThing::cspace);
                    } else if (strncmp(a, " S ", char_sz) == 0) {
                        maze.starts.push_back({pos_x, pos_y});
                        idx_in_line += char_sz;
                        things.push_back(MazeThing::cspace);
                    }
                    ++pos_x;
                }
            }
            ++pos_y;
        }
        ++line_idx;
        maze.things.push_back(things);
    }
    return maze;
}

bool Maze::validate() const
{
    return goals.size() >= 1 && starts.size() == 1 && descr.validate();
}

std::string Maze::toSdf() const
{
    // Object to build the sdf string on
    std::stringstream ss;

    // Add xml tag
    std::string xml_version = "1.0";
    ss << R"(<?xml version=")" << xml_version << R"("?>)" << "\n";

    // Open sdf tag
    std::string sdf_version = "1.11";
    ss << R"(<sdf version=")" << sdf_version << R"(">)" << "\n";

    // Open model tag
    std::string model_name = "micromouse_maze";
    ss << R"(<model name=")" << model_name << R"(">)" << "\n";
    
    ss << "<static>true</static>" << "\n";

    // Add maze wall link (Every wall as single link has better performance)
    std::string link_name = "micromouse_maze_link";
    ss << R"(<link name=")" << link_name << R"(">)" << "\n";
    ss << "<pose>0 0 0 0 0 0</pose>" << "\n";

    const double thickness = descr.thickness;
    const double half_thickness = thickness / 2;
    const double wall_length = descr.wall_length;
    const double half_wall_length = wall_length / 2;
    const double pole_lenth = descr.thickness;
    const double half_pole_length = pole_lenth / 2;
    const double wall_height = descr.maze_height;
    const double half_wall_height = wall_height / 2;
    const double cell_length = descr.wall_length + descr.thickness;
    const double half_cell_length = cell_length / 2;

    double x_pos = half_thickness;
    double y_pos = -half_thickness;
    const double z_pos = half_wall_height;

    size_t thing_id = 0;
    size_t max_i = 0;
    for (auto& vec : things) {
        if (vec.size() > max_i) max_i = vec.size();
    }

    for(size_t i = 0; i < max_i; ++i) {
        double x_pos = 0;
        bool is_there_x = false;
        double start_x = x_pos;
        double end_x = x_pos;
        for (size_t j = 0; j < this->things.size(); ++j){
            double my_x;
            if (i >= this->things[j].size()) {
                continue;
            }
            auto thing = this->things[j][i];
            switch(thing) {
                case MazeThing::vspace:
                    x_pos += wall_length;
                    if (is_there_x) {
                        double l = end_x - start_x;
                        my_x = (start_x + end_x) / 2;
                        ss << maze_obj(thing_id, my_x, y_pos, z_pos, l, thickness, wall_height);
                        thing_id++;
                        is_there_x = false;
                    }
                    start_x = x_pos;
                    break;
                case MazeThing::vwall:
                    x_pos += wall_length;
                    is_there_x = true;
                    end_x = x_pos;
                    break;
                case MazeThing::pole:
                    x_pos += pole_lenth;
                    end_x = x_pos;
                    break;
                case MazeThing::hwall: // fallthrough
                case MazeThing::hspace: // fallthrough
                case MazeThing::cspace:
                    // Do nothing
                    break;
            }
        }
        if (is_there_x) {
            double l = end_x - start_x;
            double my_x = (start_x + end_x) / 2;
            ss << maze_obj(thing_id, my_x, y_pos, z_pos, l, thickness, wall_height);
            thing_id++;
            is_there_x = false;
        }
        y_pos -= half_wall_length + half_pole_length;
    }
    x_pos = half_thickness;
    for (auto& line : this->things) {
        double y_pos = 0;
        bool is_there_y = false;
        double start_y = y_pos;
        double end_y = y_pos;
        for (auto thing : line) {
            double my_y;
            switch(thing) {
                case MazeThing::hspace:
                    y_pos -= wall_length;
                    if (is_there_y) {
                        double l = -(end_y - start_y);
                        my_y = (start_y + end_y) / 2;
                        ss << maze_obj(thing_id, x_pos, my_y, z_pos, thickness, l, wall_height);
                        thing_id++;
                        is_there_y = false;
                    }
                    start_y = y_pos;
                    break;
                case MazeThing::hwall:
                    y_pos -= wall_length;
                    is_there_y = true;
                    end_y = y_pos;
                    break;
                case MazeThing::pole:
                    y_pos -= pole_lenth;
                    end_y = y_pos;
                    break;
                case MazeThing::vwall: // fallthrough
                case MazeThing::vspace: // fallthrough
                case MazeThing::cspace:
                    // Do nothing
                    break;
            }
        }
        if (is_there_y) {
            double l = -(end_y - start_y);
            double my_y = (start_y + end_y) / 2;
            ss << maze_obj(thing_id, x_pos, my_y, z_pos, thickness, l, wall_height);
            thing_id++;
            is_there_y = false;
        }
        x_pos += half_wall_length + half_pole_length;
    }

    // Find lone poles
    x_pos = half_thickness;
    y_pos = 0;
    for (size_t i = 0; i < things.size(); ++i) {
        y_pos = 0;
        for (size_t j = 0; j < things[i].size(); ++j) {
            MazeThing thing = things[i][j];
            bool was_put = false;
            if (thing == MazeThing::pole) {
                if (i != 0) {
                    MazeThing thingthing = this->things[i - 1][j];
                    if (thingthing == MazeThing::hwall || thingthing == MazeThing::vwall) {
                        was_put = true;
                    }
                }
                if (i != things.size() - 1) {
                    MazeThing thingthing = this->things[i + 1][j];
                    if (thingthing == MazeThing::hwall || thingthing == MazeThing::vwall) {
                        was_put = true;
                    }
                }
                if (j != 0) {
                    MazeThing thingthing = this->things[i][j - 1];
                    if (thingthing == MazeThing::hwall || thingthing == MazeThing::vwall) {
                        was_put = true;
                    }
                }
                if (j != things.size() - 1) {
                    MazeThing thingthing = this->things[i][j + 1];
                    if (thingthing == MazeThing::hwall || thingthing == MazeThing::vwall) {
                        was_put = true;
                    }
                }
                if (!was_put) {
                    double my_y = y_pos -= half_thickness;
                    ss << maze_obj(thing_id, x_pos, my_y, z_pos, thickness, thickness, wall_height);
                    thing_id++;
                }
            }
            switch (thing) {
                    case MazeThing::cspace:
                    case MazeThing::hspace:
                    case MazeThing::hwall:
                        y_pos -= wall_length;
                    break;
                    case MazeThing::vspace:
                    case MazeThing::vwall:
                    case MazeThing::pole:
                        y_pos -= thickness;
                    break;
                }
        }
        x_pos += half_wall_length + half_pole_length;
    }
    ss << "</link>" << "\n";

    ss << R"(<link name=")" << this->start_link_name << R"(">)" << "\n";
    int start_id = 0;
    for (auto start : this->starts) {
        double x = half_pole_length + half_cell_length + cell_length * start.y;
        double y = -(half_pole_length + half_cell_length + cell_length * start.x);
        ss << area_obj("start", start_id++, x, y, descr.wall_length / 2, cell_length, cell_length, cell_length);
    }
    ss << "</link>" << "\n";

    ss << R"(<link name=")" << this->goal_link_name << R"(">)" << "\n";
    int goal_id = 0;
    for (auto goal : this->goals) {
        double x = half_pole_length + half_cell_length + cell_length * goal.y;
        double y = -(half_pole_length + half_cell_length + cell_length * goal.x);
        ss << area_obj("goal", goal_id++, x, y, descr.wall_length / 2, cell_length, cell_length, cell_length);
    }

    ss << "</link>" << "\n";
    ss << "</model>" << "\n";
    ss << "</sdf>" << "\n";
    return ss.str();
}

class custom::MicromouseSystemPrivate
{
public:
    transport::Node node;
    transport::Node::Publisher pub;
    transport::Node::Publisher time_pub;
    sim::Entity world_entity {sim::kNullEntity};
    bool SetMazeService(const msgs::StringMsg &_req, msgs::Boolean &_res);
    bool SetRobotService(const msgs::StringMsg &_req, msgs::Boolean &_res);
    bool SetRobotPoseService(const msgs::Pose &_req, msgs::Boolean &_res);
    bool StartCompService(const msgs::Empty &_req, msgs::Boolean &_res);
    bool ResetService(const msgs::StringMsg &_req, msgs::StringMsg &_res);
    bool ClearService(const msgs::Empty &_req, msgs::Empty &_res);
    bool RepositionService(const msgs::Empty &_req, msgs::Empty &_res);

    public: std::unique_ptr<sim::SdfEntityCreator> creator{nullptr};

    std::vector<sim::Entity> goal_col_entities;
    std::vector<sim::Entity> start_col_entities;

    CompetitionState state = CompetitionState::configuring;

    void ProcessRobotRequest(sim::EntityComponentManager &_ecm);
    void HandleResetRequest(sim::EntityComponentManager &_ecm);
    std::mutex resetMutex;
    bool resetPending = false;
    
    void SetRobotPose(sim::EntityComponentManager &_ecm);
    std::mutex robotRequestMutex;
    std::string lastRobotRequest;
    gz::math::Pose3d lastRobotPoseRequest;

    bool isRobotRequestProcessed = false;
    sim::Entity robot_entity {sim::kNullEntity};

    void HandleMazeRequests(sim::EntityComponentManager &_ecm);
    std::mutex mazeRequestMutex;
    std::unique_ptr<Maze> lastMazeRequest;
    bool isMazeRequestProcessed = false;
    sim::Entity currentMazeEntity {sim::kNullEntity};

    bool timerRunning = false;
    bool prevStartContains = false;
    bool prevGoalContains = false;
    std::chrono::steady_clock::duration best_time = std::chrono::steady_clock::duration::max();
    std::chrono::steady_clock::duration start_time = std::chrono::steady_clock::duration::zero();
};

bool MicromouseSystemPrivate::SetMazeService(const msgs::StringMsg &_req, msgs::Boolean &_res)
{
    std::lock_guard<std::mutex> lock(this->mazeRequestMutex);
    this->lastMazeRequest = std::make_unique<Maze>(Maze::fromMicromouseOnline(_req.data()));
    if (nullptr == this->lastMazeRequest) {
        _res.set_data(false);
        return false;
    }
    _res.set_data(this->lastMazeRequest->validate());
    this->isMazeRequestProcessed = false;
    return true;
}

bool MicromouseSystemPrivate::SetRobotService(const msgs::StringMsg &_req, msgs::Boolean &_res)
{
    std::lock_guard<std::mutex> lock(this->robotRequestMutex);
    this->lastRobotRequest = _req.data();
    if (this->lastRobotRequest.empty()) {
        _res.set_data(false);
        return true;
    }
    this->isRobotRequestProcessed = false;
    _res.set_data(true);
    return true;
}

bool MicromouseSystemPrivate::SetRobotPoseService(const msgs::Pose &_req, msgs::Boolean &_res)
{
    std::lock_guard<std::mutex> lock(this->robotRequestMutex);
    this->lastRobotPoseRequest.Pos().Set(_req.position().x(), _req.position().y(), _req.position().z());
    this->lastRobotPoseRequest.Rot().Set(_req.orientation().w(), _req.orientation().x(), _req.orientation().y(),  _req.orientation().z());
    _res.set_data(true);
    return true;
}

bool MicromouseSystemPrivate::ResetService(const msgs::StringMsg &_req, msgs::StringMsg &_res)
{
    (void)_req;
    (void)_res;
    std::lock_guard<std::mutex> lock(this->resetMutex);
    this->resetPending = true;
    return true;
}

bool MicromouseSystemPrivate::ClearService(const msgs::Empty &_req, msgs::Empty &_res)
{
    (void)_req;
    (void)_res;
    if (this->state != CompetitionState::configuring) {
        return false;
    }
    return true;
}

bool MicromouseSystemPrivate::RepositionService(const msgs::Empty &_req, msgs::Empty &_res)
{
    (void)_req;
    (void)_res;
    if (this->state != CompetitionState::running) {
        return false;
    }
    return true;
}

bool MicromouseSystemPrivate::StartCompService(const msgs::Empty &_req, msgs::Boolean &_res)
{
    (void)_req;
    std::lock_guard<std::mutex> lock(this->mazeRequestMutex);
    if (nullptr == this->lastMazeRequest) {
        _res.set_data(false);
        return true;
    }

    gz::msgs::Boolean msg;
    msg.set_data(true);
    bool pub_result = this->pub.Publish(msg);
    if (!pub_result) {
        gzerr << "Error publishing. Result: " << pub_result << std::endl;
        _res.set_data(false);
    } else {
        gzdbg << "Publishing. Result: " << pub_result << std::endl;
        this->state = CompetitionState::running;
        _res.set_data(true);
    }

    return true;
}

MicromouseSystem::MicromouseSystem()
    : dataPtr(std::make_unique<MicromouseSystemPrivate>())
{
    // Advertise services
    std::string setMaze = "micromouse/set_maze";
    this->dataPtr->node.Advertise(setMaze, &MicromouseSystemPrivate::SetMazeService, this->dataPtr.get());

    std::string setRobot = "micromouse/set_robot";
    this->dataPtr->node.Advertise(setRobot, &MicromouseSystemPrivate::SetRobotService, this->dataPtr.get());

    std::string setRobotPose = "micromouse/set_robot_pose";
    this->dataPtr->node.Advertise(setRobotPose, &MicromouseSystemPrivate::SetRobotPoseService, this->dataPtr.get());

    std::string startComp = "micromouse/start_comp";
    this->dataPtr->node.Advertise(startComp, &MicromouseSystemPrivate::StartCompService, this->dataPtr.get());

    std::string reset = "micromouse/reset";
    this->dataPtr->node.Advertise(reset, &MicromouseSystemPrivate::ResetService, this->dataPtr.get());

    std::string reposition = "micromouse/reposition";
    this->dataPtr->node.Advertise(reposition, &MicromouseSystemPrivate::RepositionService, this->dataPtr.get());

    std::string clear = "micromouse/clear";
    this->dataPtr->node.Advertise(reposition, &MicromouseSystemPrivate::ClearService, this->dataPtr.get());

    std::string timetopic = "/micromouse/lap";
    this->dataPtr->time_pub = this->dataPtr->node.Advertise<msgs::Time>(timetopic);

    std::string statustopic = "/micromouse/status";
    this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::Boolean>(statustopic);
    if (!this->dataPtr->pub) {
        gzerr << "Error advertising topic [" << statustopic << "]" << std::endl;
    } else {
        gzdbg << "Advertising topic [" << statustopic << "]" << std::endl;
    }
}

MicromouseSystem::~MicromouseSystem()
{
}

void MicromouseSystem::Configure(
    const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &_eventMgr)
{
    (void)_sdf;

    this->dataPtr->world_entity = _entity;
    this->dataPtr->creator = std::make_unique<sim::SdfEntityCreator>(_ecm, _eventMgr);
}


void MicromouseSystemPrivate::HandleMazeRequests(sim::EntityComponentManager &_ecm)
{
    {
        std::lock_guard<std::mutex> lock(this->mazeRequestMutex);
        if (this->isMazeRequestProcessed || !this->lastMazeRequest) {
            return;
        }
    }

    // Process the request
    if (this->currentMazeEntity != sim::kNullEntity) {
        this->creator->RequestRemoveEntity(this->currentMazeEntity);
        this->currentMazeEntity = sim::kNullEntity;
        return;
    }
    
    std::lock_guard<std::mutex> lock(this->mazeRequestMutex);
    std::string maze_sdf = this->lastMazeRequest->toSdf();
    sdf::Root root;
    auto errors = root.LoadSdfString(maze_sdf);
    if (!errors.empty()) {
        gzerr << "Maze creation failed: failed to parse maze string" << std::endl;
    }
    auto model = root.Model();
    this->currentMazeEntity = creator->CreateEntities(model);
    if (this->currentMazeEntity == sim::kNullEntity) {
        gzerr << "Maze creation failed: falied to to create entity" << std::endl;
    } else {
        this->creator->SetParent(this->currentMazeEntity, this->world_entity);
    }

    _ecm.Each<sim::components::Name>(
        [&](const sim::Entity &entity, sim::components::Name *name) -> bool
        {
            if (name->Data() == Maze::goal_link_name) {
                this->goal_col_entities = _ecm.ChildrenByComponents(entity, sim::components::Collision());
            } else if (name->Data() == Maze::start_link_name) {
                this->start_col_entities = _ecm.ChildrenByComponents(entity, sim::components::Collision());
            }
            return true;
        });
    
    this->isMazeRequestProcessed = true;
}


void MicromouseSystemPrivate::SetRobotPose(gz::sim::EntityComponentManager &_ecm)
{
    if (this->robot_entity == sim::kNullEntity) {
        gzerr << "There is no robot to be reset [robot = kNullEntity]" << std::endl;
        return;
    }

    double y = this->lastMazeRequest->descr.thickness + this->lastMazeRequest->descr.wall_length/2;
    auto state = _ecm.SetComponentData<sim::components::WorldPoseCmd>(
        this->robot_entity,
        math::Pose3d(y+this->lastRobotPoseRequest.X(), -y + this->lastRobotPoseRequest.Y(), this->lastRobotPoseRequest.Z(),
        this->lastRobotPoseRequest.Roll(), this->lastRobotPoseRequest.Pitch(), this->lastRobotPoseRequest.Yaw())) ? sim::ComponentState::OneTimeChange : sim::ComponentState::NoChange;
        _ecm.SetChanged(this->robot_entity, sim::components::WorldPoseCmd::typeId, state);
}

void MicromouseSystemPrivate::HandleResetRequest(gz::sim::EntityComponentManager &_ecm)
{
    std::lock_guard<std::mutex> lock(this->resetMutex);
    if (this->resetPending) {
        this->SetRobotPose(_ecm);
        this->resetPending = false;
    }
}
void MicromouseSystemPrivate::ProcessRobotRequest(gz::sim::EntityComponentManager &_ecm)
{
    {
        std::lock_guard<std::mutex> lock(this->robotRequestMutex);
        if (this->isRobotRequestProcessed || this->lastRobotRequest.empty()) {
            return;
        }
    }
    std::lock_guard<std::mutex> lock1(this->robotRequestMutex);
    std::lock_guard<std::mutex> lock2(this->mazeRequestMutex);
    if (this->robot_entity != sim::kNullEntity) {
        _ecm.RequestRemoveEntity(this->robot_entity);
        this->robot_entity = sim::kNullEntity;
        return;
    }
    sdf::Root root;
    auto errors = root.LoadSdfString(this->lastRobotRequest);
    auto model = root.Model();
    this->robot_entity = this->creator->CreateEntities(model);
    this->SetRobotPose(_ecm);
    _ecm.CreateComponent(this->robot_entity, sim::components::AxisAlignedBox());
    this->creator->SetParent(this->robot_entity, this->world_entity);
    this->isRobotRequestProcessed = true;
}

void MicromouseSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("MicromouseSystem::PreUpdate");
    (void)_info;
    this->dataPtr->HandleMazeRequests(_ecm);
    this->dataPtr->ProcessRobotRequest(_ecm);
    this->dataPtr->HandleResetRequest(_ecm);
}

void MicromouseSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    (void)_info;
    GZ_PROFILE("MicromouseSystem::PostUpdate");

    if (!this->dataPtr->isMazeRequestProcessed) {
        return;
    }
    
    if (this->dataPtr->lastMazeRequest == nullptr) {   
        gzerr << "Uai" << std::endl;
        return;
    }

    auto aabb = _ecm.ComponentData<sim::components::AxisAlignedBox>(this->dataPtr->robot_entity);
    bool goal_contains = false;
    for (sim::Entity goal : this->dataPtr->goal_col_entities) {
        // Check if robot position is within limits, disregarding robot's Z position
        auto pose = _ecm.ComponentData<sim::components::Pose>(goal);
        auto pos = pose->Pos();
        pos.Z(aabb->Center().Z());
        if (aabb->Contains(pos)) {
            goal_contains = true;
        }
    }

    bool start_contains = false;
    for (const auto& start : this->dataPtr->start_col_entities) {
        // Check if robot position is within limits, disregarding robot's Z position
        auto pose = _ecm.ComponentData<sim::components::Pose>(start);
        auto pos = pose->Pos();
        pos.Z(aabb->Center().Z());
        if (aabb->Contains(pos)) {
            start_contains = true;
        }
    }
    if (!start_contains && this->dataPtr->prevStartContains) {
        // start timer
        this->dataPtr->start_time = _info.simTime;
        this->dataPtr->timerRunning = true;
        gzmsg << "Starting run at " << this->dataPtr->start_time.count() << std::endl;
    }

    if (goal_contains && !this->dataPtr->prevGoalContains && this->dataPtr->timerRunning) {
        auto finish_time = _info.simTime;
        auto diff = finish_time - this->dataPtr->start_time;

        msgs::Time time_msg;
        std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(diff);
        std::chrono::nanoseconds total_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);
        std::chrono::nanoseconds nsec = total_nanoseconds - sec;
        time_msg.set_sec(sec.count());
        time_msg.set_nsec(nsec.count());
        this->dataPtr->time_pub.Publish(time_msg);

        if (diff.count() < this->dataPtr->best_time.count()) {
            this->dataPtr->best_time = diff;
        }
        this->dataPtr->timerRunning = false;
    }
    this->dataPtr->prevStartContains = start_contains;
    this->dataPtr->prevGoalContains = goal_contains;
}

GZ_ADD_PLUGIN(
    custom::MicromouseSystem,
    sim::System,
    sim::ISystemConfigure,
    sim::ISystemPreUpdate,
    sim::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(custom::MicromouseSystem, "MicromouseSystem")
