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

#include "micromouse/micromouse.hpp"

using namespace std::chrono_literals;

#include "micromouse/robot_specs.hpp"
#include "micromouse/robot.hpp"
#include "micromouse/maze.hpp"
#include "micromouse/maze_utils.hpp"
#include "micromouse/solver.hpp"
#include "micromouse/solver_utils.hpp"
#include "micromouse/angle_utils.hpp"
#include <cmath>
#include <cinttypes>

static double constexpr MAZE_XSIZE = 0.18;
static double constexpr MAZE_YSIZE = 0.18;
static double constexpr WALL_THICKNESS = 0.012;

static double constexpr ROBOT_YZISE = 0.096;
static double constexpr SENSOR_DISTANCE (ROBOT_YZISE / 2);

//assumes sensors at 45deg
static double constexpr BETWEEN_WALLS_IDEAL_MEASUREMENT = ((((MAZE_YSIZE - WALL_THICKNESS) / 2) - SENSOR_DISTANCE) * M_SQRT2);
static double constexpr MAX_THEO_ERROR = (((MAZE_YSIZE - 2*WALL_THICKNESS) - ROBOT_YZISE) * M_SQRT2);

static double constexpr LIN_BASE = 0.15;
static double constexpr ANG_BASE = 1.0;

static double constexpr TURN_EXTRA_HEADROOM = 0;

static RobotConstructionSpecification specs;
static SimpleRobotPoseManager* poseManager;

static LogicMaze maze {classicMicromouseMaze()};
static PathManager pm(maze);

static Pose2 startingPose;

static bool waitingForNextMovement = true;
static StateTransition movement;

static double k = 50;

static bool didRead = false;

static double leftMeasurement;
static double rightMeasurement;
static double frontMeasurement;

static Pose2 currentPose;

static State currentMouseState;
static Pose2 currentIdealPose;

static State goalMouseState;
static Pose2 goalIdealPose;

static bool wasMovingForward = false;
static bool shouldFixPosition = false;

int main(int argc, char **argv)
{
    specs.leftWheelPPR = 29.86 * 12;
	specs.rightWheelPPR = 29.86 * 12;
	specs.leftWheelRadius = 0.016;
	specs.rightWheelRadius = 0.016;
	specs.wheelBaseDistance = ROBOT_YZISE;

    poseManager = new SimpleRobotPoseManager(specs);
    poseManager->setPose({0, 0, M_PI_2});
    startingPose = poseManager->getPose();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MicromouseNode>());
    rclcpp::shutdown();
}

MicromouseNode::MicromouseNode()
    : Node("micromouse_node"),
    cmdVel(*this, "cmd_vel"),
    motor_left(*this, "left_motor"),
    motor_right(*this, "right_motor"),
    range_center(*this,"lidar_center"),
    range_right(*this,"lidar_right"),
    range_left(*this,"lidar_left"),
    imu(*this, "imu"),
    encoder_left(*this, "left_wheel_encoder"),
    encoder_right(*this, "right_wheel_encoder")
{
    auto cb_micromouse = [this](std_msgs::msg::Bool::UniquePtr msg) -> void
    {
        bool data = msg.get()->data;
        micromouseStarted = data;
    };
    micromoue_sub = this->create_subscription<std_msgs::msg::Bool>("micromouse", 10, cb_micromouse);
    update_timer_ = this->create_timer(10ms, std::bind(&MicromouseNode::update_callback, this));
    RCLCPP_INFO(this->get_logger(), "Micromouse simulation node has been initialised");
}

MicromouseNode::~MicromouseNode()
{
    RCLCPP_INFO(this->get_logger(), "Micromouse simulation node has been terminated");
}
#ifdef RCLCPP_DEBUG
#undef RCLCPP_DEBUG
#define RCLCPP_DEBUG RCLCPP_INFO
#endif


void MicromouseNode::update_callback()
{
    // Do not do anything until the start signal has been received
    if (!micromouseStarted) {
        return;
    }

    int64_t leftWheelPulses = this->encoder_left.TicksDelta();
    int64_t rightWheelPulses = this->encoder_right.TicksDelta();
    poseManager->updatePose(leftWheelPulses, rightWheelPulses);
    
    if (waitingForNextMovement) {
        // Current mouse position
        currentMouseState = pm.getRobotState();
        currentIdealPose = stateToPose(currentMouseState, MAZE_XSIZE, MAZE_YSIZE);
        if (movement == StateTransition::forward) {
            wasMovingForward = true;
            shouldFixPosition = true;
        }
        movement = pm.nextMovement();
        switch (movement) {
            case StateTransition::forward:
                RCLCPP_INFO(this->get_logger(), "Next Movement: StateTransition::forward");
                break;
            case StateTransition::turnRight:
                RCLCPP_INFO(this->get_logger(), "Next Movement: StateTransition::turnRight:");
                break;
            case StateTransition::turnLeft:
                RCLCPP_INFO(this->get_logger(), "Next Movement: StateTransition::turnLeft");
                break;
            case StateTransition::invalid:
                RCLCPP_INFO(this->get_logger(), "Next Movement: StateTransition::invalid");
                break;
        }

        goalMouseState = pm.getRobotState();
        goalIdealPose = stateToPose(goalMouseState, MAZE_XSIZE, MAZE_YSIZE);
        
        waitingForNextMovement = false;
        didRead = false;
        //Correct one axis of pose
        if (shouldFixPosition && wasMovingForward)
        {
            shouldFixPosition = false;
            currentPose = poseManager->getPose();
            switch (currentMouseState.orientation)
            {
                case Direction::up:
                case Direction::down:
                    currentPose.position.x = currentIdealPose.position.x;
                break;
                case Direction::right:
                case Direction::left:
                    currentPose.position.y = currentIdealPose.position.y;
                break;
            }
            currentPose.angle = currentIdealPose.angle;
            poseManager->setPose(currentPose);
        }
    }

    currentPose = poseManager->getPose();
    Pose2 deltaPoseEnd = goalIdealPose - currentPose;
    double dap = signedAngleDifference(currentIdealPose.normalizedAngle(), currentPose.normalizedAngle());
    
    leftMeasurement = this->range_left.Read();
    rightMeasurement = this->range_right.Read();
    frontMeasurement = this->range_center.Read();

    double error = 0;
    double measu_limit = BETWEEN_WALLS_IDEAL_MEASUREMENT + 0.01;

    // Has two parallel walls on either side
    if(leftMeasurement < measu_limit && rightMeasurement < measu_limit) {
        error = leftMeasurement - rightMeasurement;
    } else {
        if (leftMeasurement < measu_limit) {
            error = leftMeasurement - BETWEEN_WALLS_IDEAL_MEASUREMENT;
        } else if (rightMeasurement < measu_limit) {
            error = BETWEEN_WALLS_IDEAL_MEASUREMENT - rightMeasurement;
        }
    }

    double dist = 0;

    switch (movement)
    {
        case StateTransition::forward:
            switch(currentMouseState.orientation)
            {
                case Direction::up:
                case Direction::down:
                    dist = abs(deltaPoseEnd.position.y);
                break;
                case Direction::right:
                case Direction::left:
                    dist = abs(deltaPoseEnd.position.x);
                break;
            }
            {
                double ang = k * error;
                double ang_limit = 1.0;
                if (ang > ang_limit) ang = ang_limit;
                if (ang < -ang_limit) ang = -ang_limit;

                this->update_cmd_vel(LIN_BASE, ang);
            }

            if(dist <= 0.01) {
                waitingForNextMovement = true;
            }

            if (frontMeasurement <= 0.036) {
                waitingForNextMovement = true;
            }

            //Detect walls when entering the cell
            if (!didRead && dist >= 0.10 && dist <= .12) {
                didRead = true;
                if(leftMeasurement < .1) {
                    maze.placeWall(goalMouseState.position, localToGlobalDirection(goalMouseState.orientation, Direction::left));
                    RCLCPP_INFO(this->get_logger(), "+Wall %d %d left", goalMouseState.position.x, goalMouseState.position.y);
                }
                
                if (rightMeasurement < .1) {
                    maze.placeWall(goalMouseState.position, localToGlobalDirection(goalMouseState.orientation, Direction::right));

                    RCLCPP_INFO(this->get_logger(), "+Wall %d %d right", goalMouseState.position.x, goalMouseState.position.y);
                }

                if(frontMeasurement < .25) {
                    maze.placeWall(goalMouseState.position, localToGlobalDirection(goalMouseState.orientation, Direction::up));
                    RCLCPP_INFO(this->get_logger(), "+Wall %d %d up", goalMouseState.position.x, goalMouseState.position.y);
                }
            }
        break;
        
        case StateTransition::turnLeft:
            this->update_cmd_vel(0.0, ANG_BASE);
            if (dap >= M_PI_2 - TURN_EXTRA_HEADROOM) {
                waitingForNextMovement = true;
            }
        break;
        
        case StateTransition::turnRight:
            this->update_cmd_vel(0.0, -ANG_BASE);
            if (dap <= - M_PI_2 + TURN_EXTRA_HEADROOM) {
                waitingForNextMovement = true;
            }
        break;
        
        default:
            this->update_cmd_vel(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "defaulting");
        break;
    }
}

void MicromouseNode::update_cmd_vel(double linear, double angular)
{
    if (micromouseStarted)
    {
        this->cmdVel.SetVel(linear, angular);
    }
}
