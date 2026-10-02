// ============================================================================
// vio_benchmark
//
// Flies the two trajectories a powerline inspection actually consists of, so a
// visual-inertial estimator (OpenVINS) can be scored on each of them:
//
//   profile:=helix    climb round tower_1 on a helix, nose always on the tower,
//                     with each turn's camera view overlapping the one below it
//                     by `overlap` (0.3 = 30 %).
//
//   profile:=transit  fly straight down the line above the conductors, from
//                     20 m before tower_1 to 20 m past tower_5 (840 m).
//
// This program only FLIES. It never looks at the estimator's answer except to
// check that one is being produced before the measured part starts. Scoring is
// done afterwards, from a bag, by scripts/vio_eval.py; scripts/vio_trial.sh
// runs the whole thing (OpenVINS, recorder, this flight, the evaluation)
// through walkerd. Picked from walker's mission list it flies the default
// profile (helix): walker passes no ROS parameters.
//
// THE SEQUENCE
//   WAIT_FOR_DATA  telemetry, a healthy position_fused, the attitude, /clock
//   PREPARE        take control authority, switch the stereo pair on, then sit
//                  still for a few seconds: OpenVINS initialises on the jolt
//                  of the takeoff and needs a stationary window before it
//   TAKEOFF        the automatic takeoff (stops at ~1.2 m on this setup)
//   CLIMB          straight up to the approach height
//   CHECK_YAW      a two-second test turn, to learn which way the bridge
//                  reads the yaw-rate axis (see YAW RATE SIGN below)
//   GOTO_START     fly to where the segment starts, facing the way it starts
//   SETTLE         hover there; wait until the estimator is publishing
//   SEGMENT        THE MEASURED PART -- helix or transit
//   HOLD_END       hover a moment so the end of the segment is clean
//   RETURN         back above the spawn point at return_height
//   DESCEND        down to 4 m
//   LAND           automatic landing, then release authority
//
// Every step publishes a JSON line on ~/phase (/vio_benchmark/phase) carrying
// the SIMULATION time, the step, and the segment geometry. The evaluator cuts
// the SEGMENT out of the bag by those times.
//
// HOW WE STEER
// Same topic as demo_orbit_mission,
//     /wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate
//     axes = [v east, v north, v up (m/s), yaw rate (deg/s)]
// but closed loop: every tick the command is the reference velocity plus a
// correction towards the reference position, read off position_fused. An
// open-loop helix would not come back round to the same place, and "the views
// overlap by 30 %" would then only be true on paper.
//
// YAW RATE SIGN. The bridge reads that yaw rate the DJI way by default --
// positive is CLOCKWISE seen from above (setpoint_yaw_frame=ned in
// gui/simty/converters.py). Everything in here is computed the ENU way
// (counter-clockwise positive) and flipped once, in publishVelocity(). If the
// bridge has been switched to setpoint_yaw_frame=enu, that is backwards and
// the heading loop would run away -- so CHECK_YAW asks for a slow left turn
// once in the air, watches which way the nose actually goes, and flips the
// sign itself if needed (and says so). -p yaw_rate_clockwise:=false starts
// from the other assumption.
//
// WHERE THINGS ARE
// position_fused is PX4's local NED frame, whose origin is where the aircraft
// took off -- the world origin, the spawn point. So world ENU = local ENU plus
// (spawn_east, spawn_north), both 0 by default. The towers are at x = 60, 260,
// 460, 660, 860 m east, on y = 0, 36 m tall (worlds/README.md).
//
// TAKEOFF: the bridge's takeoff arms and climbs to MIS_TAKEOFF_ALT (1.8 m,
// models/m4e/walker.toml); CLIMB takes over from there.
//
// HEIGHT comes from position_fused (-z), NOT from height_above_ground: that
// one is the downward rangefinder, and over a cross-arm or a conductor it
// reads the distance to the steel, not to the ground.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <psdk_interfaces/msg/display_mode.hpp>
#include <psdk_interfaces/msg/flight_status.hpp>
#include <psdk_interfaces/msg/position_fused.hpp>
#include <psdk_interfaces/srv/perception_stereo_vision_setup.hpp>

using namespace std::chrono_literals;
using psdk_interfaces::msg::DisplayMode;
using psdk_interfaces::msg::FlightStatus;
using psdk_interfaces::msg::PositionFused;
using PerceptionSetup = psdk_interfaces::srv::PerceptionStereoVisionSetup;
using std_srvs::srv::Trigger;

const double TICK_HZ = 20.0;
const double DEG = M_PI / 180.0;

// Tallest steel on the line: the earth-wire peaks, 36 m (worlds/README.md).
const double LINE_TOP_M = 36.0;
// Longest cross-arm, measured from the tower's centre line.
const double LONGEST_ARM_M = 6.4;

enum Step
{
  WAIT_FOR_DATA,
  PREPARE,
  TAKEOFF,
  CLIMB,
  CHECK_YAW,
  GOTO_START,
  SETTLE,
  SEGMENT,
  HOLD_END,
  RETURN,
  DESCEND,
  LAND,
  DONE
};

// A point in the WORLD's ENU frame, and the heading to hold there (radians,
// ENU: 0 = facing east, counter-clockwise positive).
struct Target
{
  double e = 0.0, n = 0.0, u = 0.0, yaw = 0.0;
};

double wrapAngle(double a)
{
  while (a >  M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

double clampAbs(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

class VioBenchmark : public rclcpp::Node
{
public:
  VioBenchmark()
  : Node("vio_benchmark",
         // Simulation time, so the segment's reference trajectory runs at the
         // speed the physics runs at, and so the ~/phase stamps are in the same
         // clock as the images, the IMU and the ground truth. The timer below
         // stays a wall timer; only now() follows /clock.
         rclcpp::NodeOptions().append_parameter_override("use_sim_time", true))
  {
    profile_ = declare_parameter<std::string>("profile", "helix");

    // ---- the helix ----
    tower_east_         = declare_parameter<double>("helix.tower_east", 60.0);
    tower_north_        = declare_parameter<double>("helix.tower_north", 0.0);
    radius_             = declare_parameter<double>("helix.radius", 12.0);
    overlap_            = declare_parameter<double>("helix.overlap", 0.3);
    vfov_deg_           = declare_parameter<double>("helix.vertical_fov_deg", 90.0);
    tower_half_width_   = declare_parameter<double>("helix.tower_half_width", 2.5);
    helix_bottom_       = declare_parameter<double>("helix.bottom", 5.0);
    helix_top_          = declare_parameter<double>("helix.top", 36.0);
    helix_speed_        = declare_parameter<double>("helix.speed", 2.0);
    helix_start_deg_    = declare_parameter<double>("helix.start_angle_deg", 180.0);
    helix_ccw_          = declare_parameter<bool>("helix.counter_clockwise", true);

    // ---- the transit ----
    transit_start_east_ = declare_parameter<double>("transit.start_east", 40.0);
    transit_end_east_   = declare_parameter<double>("transit.end_east", 880.0);
    transit_north_      = declare_parameter<double>("transit.north", 0.0);
    transit_height_     = declare_parameter<double>("transit.height", 42.0);
    transit_speed_      = declare_parameter<double>("transit.speed", 5.0);

    // ---- flying ----
    spawn_east_         = declare_parameter<double>("spawn_east", 0.0);
    spawn_north_        = declare_parameter<double>("spawn_north", 0.0);
    return_height_      = declare_parameter<double>("return_height", 45.0);
    approach_speed_     = declare_parameter<double>("approach_speed", 4.0);
    return_speed_       = declare_parameter<double>("return_speed", 8.0);
    max_climb_speed_    = declare_parameter<double>("max_climb_speed", 2.0);
    position_gain_      = declare_parameter<double>("position_gain", 0.8);
    yaw_gain_           = declare_parameter<double>("yaw_gain", 1.5);
    max_yaw_rate_deg_   = declare_parameter<double>("max_yaw_rate_deg", 30.0);
    yaw_rate_clockwise_ = declare_parameter<bool>("yaw_rate_clockwise", true);
    settle_seconds_     = declare_parameter<double>("settle_seconds", 5.0);
    hold_end_seconds_   = declare_parameter<double>("hold_end_seconds", 3.0);
    pre_takeoff_wait_   = declare_parameter<double>("pre_takeoff_wait", 5.0);
    start_perception_   = declare_parameter<bool>("start_perception", true);
    require_estimator_  = declare_parameter<bool>("require_estimator", true);
    estimator_topic_    = declare_parameter<std::string>("estimator_topic", "/ov_msckf/odomimu");
    estimator_wait_     = declare_parameter<double>("estimator_wait", 30.0);
    allow_unsafe_       = declare_parameter<bool>("allow_unsafe", false);
    takeoff_timeout_    = declare_parameter<double>("takeoff_timeout", 60.0);
    offboard_timeout_   = declare_parameter<double>("offboard_timeout", 15.0);
    goto_timeout_       = declare_parameter<double>("goto_timeout", 300.0);

    // ---- listen to the drone ----
    rclcpp::QoS qos(10);
    qos.best_effort();

    flight_status_sub_ = create_subscription<FlightStatus>(
      "/wrapper/psdk_ros2/flight_status", qos,
      [this](const FlightStatus::SharedPtr msg) {
        flight_status_ = msg->flight_status;
        got_status_ = true;
      });

    display_mode_sub_ = create_subscription<DisplayMode>(
      "/wrapper/psdk_ros2/display_mode", qos,
      [this](const DisplayMode::SharedPtr msg) { display_mode_ = msg->display_mode; });

    // NED metres in PX4's local frame -> ENU in the world frame.
    position_sub_ = create_subscription<PositionFused>(
      "/wrapper/psdk_ros2/position_fused", qos,
      [this](const PositionFused::SharedPtr msg) {
        e_ = msg->position.y + spawn_east_;
        n_ = msg->position.x + spawn_north_;
        u_ = -msg->position.z;
        position_healthy_ = msg->x_health && msg->y_health && msg->z_health;
        got_position_ = true;
      });

    // The wrapper's attitude is ENU -> FLU (REP-103), so this yaw is ENU too.
    attitude_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/wrapper/psdk_ros2/attitude", qos,
      [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
        const auto & q = msg->quaternion;
        yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        got_attitude_ = true;
      });

    // Only to know the estimator is alive -- its numbers are never used here.
    estimator_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      estimator_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr) { last_estimate_ = now(); got_estimate_ = true; });

    // ---- how we steer ----
    rclcpp::QoS command_qos(10);
    command_qos.reliable();
    velocity_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      "/wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate", command_qos);
    phase_pub_ = create_publisher<std_msgs::msg::String>("~/phase", command_qos);

    authority_client_  = create_client<Trigger>("/wrapper/psdk_ros2/obtain_ctrl_authority");
    release_client_    = create_client<Trigger>("/wrapper/psdk_ros2/release_ctrl_authority");
    takeoff_client_    = create_client<Trigger>("/wrapper/psdk_ros2/takeoff");
    land_client_       = create_client<Trigger>("/wrapper/psdk_ros2/land");
    perception_client_ = create_client<PerceptionSetup>("/wrapper/psdk_ros2/start_perception");

    configured_ = configure();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / TICK_HZ), std::bind(&VioBenchmark::tick, this));
  }

private:
  // ------------------------------------------------------------------
  // the segment geometry, worked out once
  // ------------------------------------------------------------------

  bool configure()
  {
    if (profile_ == "helix") {
      // How much tower one turn sees. The camera is level and body-fixed, so
      // its view is centred on the drone's own height and spans
      //     footprint = 2 * d * tan(vfov / 2)
      // vertically at the distance d it is looking. d is taken to the tower's
      // FACE, not its centre line: the face is nearer, so the footprint there
      // is the smaller one, and an overlap that holds there holds everywhere.
      const double d = radius_ - tower_half_width_;
      const double footprint = 2.0 * d * std::tan(0.5 * vfov_deg_ * DEG);
      pitch_ = footprint * (1.0 - overlap_);
      const double height = helix_top_ - helix_bottom_;
      turns_ = height / pitch_;

      // Constant speed along the helix. One turn is a circle of 2*pi*r
      // horizontally while rising by pitch, so the angular rate is
      //     omega = speed / sqrt(r^2 + (pitch / 2 pi)^2)
      omega_ = helix_speed_ / std::hypot(radius_, pitch_ / (2.0 * M_PI));
      climb_rate_ = pitch_ * omega_ / (2.0 * M_PI);
      duration_ = height / climb_rate_;
      theta0_ = helix_start_deg_ * DEG;
      direction_ = helix_ccw_ ? 1.0 : -1.0;

      RCLCPP_INFO(get_logger(),
        "HELIX round (%.0f, %.0f): radius %.1f m (%.1f m to the face), view %.1f m tall there, "
        "overlap %.0f %% -> climb %.2f m per turn; %.0f -> %.0f m is %.2f turns, "
        "%.1f deg/s, %.2f m/s up, %.0f s",
        tower_east_, tower_north_, radius_, d, footprint, overlap_ * 100.0, pitch_,
        helix_bottom_, helix_top_, turns_, omega_ / DEG, climb_rate_, duration_);

      if (overlap_ < 0.0 || overlap_ >= 0.95) {
        RCLCPP_ERROR(get_logger(), "helix.overlap must be in [0, 0.95), got %.2f", overlap_);
        return false;
      }
      if (d <= 0.0 || helix_top_ <= helix_bottom_ || helix_speed_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "helix geometry does not make sense (radius, top/bottom, speed)");
        return false;
      }
      if (!allow_unsafe_ && radius_ < LONGEST_ARM_M + 2.5) {
        RCLCPP_ERROR(get_logger(),
          "helix.radius %.1f m passes within 2.5 m of the %.1f m cross-arms. "
          "Use >= %.1f, or -p allow_unsafe:=true if you mean it.",
          radius_, LONGEST_ARM_M, LONGEST_ARM_M + 2.5);
        return false;
      }
      if (!allow_unsafe_ && helix_bottom_ < 3.0) {
        RCLCPP_ERROR(get_logger(), "helix.bottom %.1f m is below 3 m", helix_bottom_);
        return false;
      }
      return true;
    }

    if (profile_ == "transit") {
      const double length = std::fabs(transit_end_east_ - transit_start_east_);
      duration_ = length / transit_speed_;
      RCLCPP_INFO(get_logger(),
        "TRANSIT along y=%.0f from x=%.0f to x=%.0f (%.0f m) at %.1f m, %.1f m/s, %.0f s",
        transit_north_, transit_start_east_, transit_end_east_, length,
        transit_height_, transit_speed_, duration_);
      if (length <= 0.0 || transit_speed_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "transit needs start_east != end_east and a positive speed");
        return false;
      }
      if (!allow_unsafe_ && transit_height_ < LINE_TOP_M + 2.0) {
        RCLCPP_ERROR(get_logger(),
          "transit.height %.1f m is within 2 m of the %.0f m tower tops and earth wires. "
          "Use >= %.0f, or -p allow_unsafe:=true if you mean it.",
          transit_height_, LINE_TOP_M, LINE_TOP_M + 2.0);
        return false;
      }
      return true;
    }

    RCLCPP_ERROR(get_logger(), "profile must be 'helix' or 'transit', got '%s'", profile_.c_str());
    return false;
  }

  // Where the segment wants the drone t seconds after it started: position,
  // velocity (feed-forward) and heading, plus how far along it is.
  void reference(double t, Target & p, double & ve, double & vn, double & vu,
                 double & yaw_rate, double & progress)
  {
    if (profile_ == "helix") {
      t = std::min(t, duration_);
      const double theta = theta0_ + direction_ * omega_ * t;
      p.e = tower_east_  + radius_ * std::cos(theta);
      p.n = tower_north_ + radius_ * std::sin(theta);
      p.u = helix_bottom_ + climb_rate_ * t;
      p.yaw = wrapAngle(theta + M_PI);                 // nose on the tower's centre line
      ve = -direction_ * radius_ * omega_ * std::sin(theta);
      vn =  direction_ * radius_ * omega_ * std::cos(theta);
      vu = climb_rate_;
      yaw_rate = direction_ * omega_;
      progress = omega_ * t / (2.0 * M_PI);            // turns completed
      return;
    }
    const double sign = transit_end_east_ >= transit_start_east_ ? 1.0 : -1.0;
    const double along = std::min(t * transit_speed_,
                                  std::fabs(transit_end_east_ - transit_start_east_));
    p.e = transit_start_east_ + sign * along;
    p.n = transit_north_;
    p.u = transit_height_;
    p.yaw = sign > 0.0 ? 0.0 : M_PI;
    ve = sign * transit_speed_;
    vn = 0.0;
    vu = 0.0;
    yaw_rate = 0.0;
    progress = along;                                  // metres flown
  }

  Target segmentStart()
  {
    Target p;
    double ve, vn, vu, yr, progress;
    reference(0.0, p, ve, vn, vu, yr, progress);
    return p;
  }

  // ------------------------------------------------------------------
  // small helpers
  // ------------------------------------------------------------------

  bool isArmed()  { return flight_status_ != FlightStatus::FLIGHT_STATUS_STOPED; }
  bool isFlying() { return flight_status_ == FlightStatus::FLIGHT_STATUS_ON_AIR; }
  bool weAreSteering() { return display_mode_ == DisplayMode::DISPLAY_MODE_NAVI_SDK_CTRL; }
  bool isComingDown()
  {
    return display_mode_ == DisplayMode::DISPLAY_MODE_AUTO_LANDING ||
           display_mode_ == DisplayMode::DISPLAY_MODE_FORCE_AUTO_LANDING ||
           display_mode_ == DisplayMode::DISPLAY_MODE_NAVI_GO_HOME;
  }
  bool estimatorAlive()
  {
    return got_estimate_ && (now() - last_estimate_).seconds() < 2.0;
  }

  double secondsInStep() { return ticks_in_step_ / TICK_HZ; }

  static const char * nameOf(Step s)
  {
    switch (s) {
      case WAIT_FOR_DATA: return "WAIT_FOR_DATA";
      case PREPARE:       return "PREPARE";
      case TAKEOFF:       return "TAKEOFF";
      case CLIMB:         return "CLIMB";
      case CHECK_YAW:     return "CHECK_YAW";
      case GOTO_START:    return "GOTO_START";
      case SETTLE:        return "SETTLE";
      case SEGMENT:       return "SEGMENT";
      case HOLD_END:      return "HOLD_END";
      case RETURN:        return "RETURN";
      case DESCEND:       return "DESCEND";
      case LAND:          return "LAND";
      case DONE:          return "DONE";
    }
    return "?";
  }
  const char * stepName() { return nameOf(step_); }

  void goToStep(Step next)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s at (%.1f, %.1f, %.1f)",
                stepName(), nameOf(next), e_, n_, u_);
    step_ = next;
    ticks_in_step_ = 0;
    settled_ticks_ = 0;
  }

  // Everything here is ENU: yaw_rate is counter-clockwise positive, radians.
  // The one place it is turned into what the bridge reads -- degrees, and
  // clockwise positive unless the bridge was told otherwise.
  void publishVelocity(double east, double north, double up, double yaw_rate)
  {
    const double yaw_rate_deg = clampAbs(yaw_rate / DEG, max_yaw_rate_deg_);
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.axes = {static_cast<float>(east), static_cast<float>(north), static_cast<float>(up),
                static_cast<float>(yaw_rate_clockwise_ ? -yaw_rate_deg : yaw_rate_deg)};
    velocity_pub_->publish(joy);
  }

  // Fly towards a reference: feed-forward velocity plus a pull towards the
  // reference position, capped. Returns the remaining distance.
  double track(const Target & ref, double ve, double vn, double vu, double yaw_rate_ff,
               double max_speed)
  {
    double ce = ve + position_gain_ * (ref.e - e_);
    double cn = vn + position_gain_ * (ref.n - n_);
    const double horizontal = std::hypot(ce, cn);
    if (horizontal > max_speed) {
      ce *= max_speed / horizontal;
      cn *= max_speed / horizontal;
    }
    const double cu = clampAbs(vu + position_gain_ * (ref.u - u_), max_climb_speed_);
    const double yaw_rate = yaw_rate_ff + yaw_gain_ * wrapAngle(ref.yaw - yaw_);
    publishVelocity(ce, cn, cu, yaw_rate);
    return std::sqrt((ref.e - e_) * (ref.e - e_) + (ref.n - n_) * (ref.n - n_) +
                     (ref.u - u_) * (ref.u - u_));
  }

  // Fly to a fixed point and turn to its heading. True once it has been
  // within 0.5 m and 5 degrees for two seconds.
  bool goTo(const Target & target, double max_speed)
  {
    const double distance = track(target, 0.0, 0.0, 0.0, 0.0, max_speed);
    const bool there = distance < 0.5 && std::fabs(wrapAngle(target.yaw - yaw_)) < 5.0 * DEG;
    settled_ticks_ = there ? settled_ticks_ + 1 : 0;
    return settled_ticks_ >= 2 * TICK_HZ;
  }

  // Heading from here towards a point; keeps the stereo pair looking where
  // the drone is going on the long legs.
  double yawTowards(double e, double n)
  {
    if (std::hypot(e - e_, n - n_) < 2.0) { return yaw_; }
    return std::atan2(n - n_, e - e_);
  }

  void sendCommand(rclcpp::Client<Trigger>::SharedPtr client, std::string name)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s is not available -- is the PSDK bridge running?", name.c_str());
      return;
    }
    client->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, name](rclcpp::Client<Trigger>::SharedFuture future) {
        auto reply = future.get();
        if (reply->success) {
          RCLCPP_INFO(get_logger(), "%s: ok", name.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "%s: refused (%s)", name.c_str(), reply->message.c_str());
        }
      });
  }

  void startPerception()
  {
    if (!perception_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "start_perception is not available -- is the PSDK bridge running?");
      return;
    }
    auto request = std::make_shared<PerceptionSetup::Request>();
    request->stereo_cameras_direction = "FRONT";
    request->start_stop = true;
    perception_client_->async_send_request(
      request, [this](rclcpp::Client<PerceptionSetup>::SharedFuture future) {
        auto reply = future.get();
        if (reply->success) {
          RCLCPP_INFO(get_logger(), "start_perception: ok (%s)", reply->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "start_perception refused: %s", reply->message.c_str());
        }
      });
  }

  // One JSON line per tick on ~/phase. Hand-written: the fields are fixed and
  // nothing here is worth a JSON library.
  void publishPhase(double progress, const Target * ref)
  {
    char buffer[640];
    int used = std::snprintf(buffer, sizeof(buffer),
      "{\"t\": %.4f, \"phase\": \"%s\", \"profile\": \"%s\", \"progress\": %.4f, "
      "\"pos\": [%.3f, %.3f, %.3f], \"yaw\": %.4f",
      now().seconds(), stepName(), profile_.c_str(), progress, e_, n_, u_, yaw_);
    if (ref != nullptr && used > 0 && used < static_cast<int>(sizeof(buffer))) {
      used += std::snprintf(buffer + used, sizeof(buffer) - used,
        ", \"ref\": [%.3f, %.3f, %.3f]", ref->e, ref->n, ref->u);
    }
    if (used > 0 && used < static_cast<int>(sizeof(buffer))) {
      if (profile_ == "helix") {
        std::snprintf(buffer + used, sizeof(buffer) - used,
          ", \"centre\": [%.3f, %.3f], \"radius\": %.3f, \"pitch\": %.4f, \"overlap\": %.3f, "
          "\"bottom\": %.3f, \"top\": %.3f, \"speed\": %.3f}",
          tower_east_, tower_north_, radius_, pitch_, overlap_, helix_bottom_, helix_top_,
          helix_speed_);
      } else {
        std::snprintf(buffer + used, sizeof(buffer) - used,
          ", \"start\": [%.3f, %.3f, %.3f], \"end\": [%.3f, %.3f, %.3f], \"speed\": %.3f}",
          transit_start_east_, transit_north_, transit_height_,
          transit_end_east_, transit_north_, transit_height_, transit_speed_);
      }
    }
    std_msgs::msg::String msg;
    msg.data = buffer;
    phase_pub_->publish(msg);
  }

  // ------------------------------------------------------------------
  // the flight sequence -- 20 times per second
  // ------------------------------------------------------------------
  void tick()
  {
    ticks_in_step_++;

    if (ticks_in_step_ % (int)(2 * TICK_HZ) == 0 && step_ != DONE) {
      RCLCPP_INFO(get_logger(), "[%s %.0fs] pos (%.1f, %.1f, %.1f) yaw %.0f deg %s%s",
                  stepName(), secondsInStep(), e_, n_, u_, yaw_ / DEG,
                  weAreSteering() ? "SDK_CTRL" : "not-offboard",
                  estimatorAlive() ? " estimator:up" : " estimator:none");
    }

    double progress = 0.0;
    Target ref;
    bool have_ref = false;

    switch (step_) {

      case WAIT_FOR_DATA:
        if (!configured_) {
          RCLCPP_ERROR(get_logger(), "not flying: fix the parameters above and start again");
          goToStep(DONE);
        } else if (got_status_ && got_position_ && position_healthy_ && got_attitude_ &&
                   now().nanoseconds() > 0)
        {
          RCLCPP_INFO(get_logger(), "telemetry, position and sim clock are flowing");
          goToStep(PREPARE);
        } else if (ticks_in_step_ % 100 == 0) {
          RCLCPP_WARN(get_logger(),
            "waiting: flight_status %s, position_fused %s, attitude %s, /clock %s",
            got_status_ ? "ok" : "-", got_position_ ? (position_healthy_ ? "ok" : "unhealthy") : "-",
            got_attitude_ ? "ok" : "-", now().nanoseconds() > 0 ? "ok" : "-");
        }
        break;

      case PREPARE:
        if (ticks_in_step_ == 1) {
          sendCommand(authority_client_, "obtain_ctrl_authority");
          if (start_perception_) { startPerception(); }
          RCLCPP_INFO(get_logger(),
            "holding still for %.0f s so the estimator gets a stationary window before takeoff",
            pre_takeoff_wait_);
        }
        if (secondsInStep() >= std::max(2.0, pre_takeoff_wait_)) {
          goToStep(TAKEOFF);
          sendCommand(takeoff_client_, "takeoff");
        }
        break;

      // No velocity commands: they would switch PX4 to OFFBOARD mid-takeoff.
      case TAKEOFF:
        if (isFlying() && u_ >= 0.5 && display_mode_ != DisplayMode::DISPLAY_MODE_AUTO_TAKEOFF &&
            secondsInStep() > 5.0)
        {
          goToStep(CLIMB);
        } else if (secondsInStep() >= takeoff_timeout_) {
          RCLCPP_ERROR(get_logger(), "only %.2f m after %.0f s, landing", u_, takeoff_timeout_);
          goToStep(LAND);
        } else if (!isArmed() && ticks_in_step_ % 100 == 0) {
          sendCommand(takeoff_client_, "takeoff");
        }
        break;

      // Straight up where we are, to the height the segment starts at.
      case CLIMB: {
        if (ticks_in_step_ == 1) {
          hold_ = Target{e_, n_, segmentStart().u, yaw_};
        }
        const bool there = goTo(hold_, approach_speed_);
        if (weAreSteering() && there) {
          goToStep(CHECK_YAW);
        } else if (!weAreSteering() && secondsInStep() >= offboard_timeout_) {
          RCLCPP_ERROR(get_logger(),
            "PX4 never switched to SDK_CTRL after %.0f s. Turn on project testing mode on the "
            "bridge console (key P). Landing.", offboard_timeout_);
          goToStep(LAND);
        }
        break;
      }

      // Ask for a slow LEFT turn (counter-clockwise, ENU positive) and see
      // which way the nose goes. Position is held throughout.
      case CHECK_YAW: {
        if (ticks_in_step_ == 1) { yaw_check_start_ = yaw_; }
        if (secondsInStep() < 2.5) {
          Target still = hold_;
          still.yaw = yaw_;                    // no heading correction, only the test rate
          track(still, 0.0, 0.0, 0.0, 15.0 * DEG, approach_speed_);
        } else {
          const double turned = wrapAngle(yaw_ - yaw_check_start_) / DEG;
          if (turned < -5.0) {
            yaw_rate_clockwise_ = !yaw_rate_clockwise_;
            RCLCPP_WARN(get_logger(),
              "asked to turn left, turned RIGHT by %.0f deg: the bridge reads the yaw-rate axis "
              "the other way. Flipped it (yaw_rate_clockwise is now %s); pass "
              "-p yaw_rate_clockwise:=%s next time to skip this.",
              -turned, yaw_rate_clockwise_ ? "true" : "false",
              yaw_rate_clockwise_ ? "true" : "false");
          } else if (turned < 5.0) {
            RCLCPP_WARN(get_logger(),
              "asked to turn left, turned only %.1f deg -- is the yaw axis getting through? "
              "Carrying on with yaw_rate_clockwise=%s", turned,
              yaw_rate_clockwise_ ? "true" : "false");
          } else {
            RCLCPP_INFO(get_logger(), "yaw-rate sign confirmed (turned left %.0f deg)", turned);
          }
          goToStep(GOTO_START);
        }
        break;
      }

      // Across to the start at that height. Nose towards the start on the way,
      // then round to the start heading once there.
      case GOTO_START: {
        Target start = segmentStart();
        const double remaining = std::hypot(start.e - e_, start.n - n_);
        Target leg = start;
        if (remaining > 3.0) { leg.yaw = yawTowards(start.e, start.n); }
        if (goTo(leg, approach_speed_) && remaining <= 3.0) {
          goToStep(SETTLE);
        } else if (secondsInStep() >= goto_timeout_) {
          RCLCPP_ERROR(get_logger(), "could not reach the start in %.0f s, returning", goto_timeout_);
          goToStep(RETURN);
        }
        break;
      }

      case SETTLE: {
        ref = segmentStart();
        have_ref = true;
        track(ref, 0.0, 0.0, 0.0, 0.0, approach_speed_);
        const bool waited = secondsInStep() >= settle_seconds_;
        if (waited && (estimatorAlive() || !require_estimator_)) {
          if (!estimatorAlive()) {
            RCLCPP_WARN(get_logger(), "no estimate on %s -- flying the segment anyway",
                        estimator_topic_.c_str());
          }
          goToStep(SEGMENT);
          segment_t0_ = now();
        } else if (waited && secondsInStep() >= settle_seconds_ + estimator_wait_) {
          RCLCPP_ERROR(get_logger(),
            "nothing on %s after %.0f s: OpenVINS is not running, or never initialised (it "
            "starts on the takeoff jolt, and needs the stereo pair). Not flying the segment. "
            "Run with -p require_estimator:=false to fly it regardless.",
            estimator_topic_.c_str(), estimator_wait_);
          goToStep(RETURN);
        }
        break;
      }

      // THE MEASURED PART.
      case SEGMENT: {
        const double t = (now() - segment_t0_).seconds();
        double ve, vn, vu, yaw_rate;
        reference(t, ref, ve, vn, vu, yaw_rate, progress);
        have_ref = true;
        const double speed_cap = (profile_ == "helix" ? helix_speed_ : transit_speed_) + 2.0;
        const double off = track(ref, ve, vn, vu, yaw_rate, speed_cap);
        if (ticks_in_step_ % (int)(2 * TICK_HZ) == 0) {
          RCLCPP_INFO(get_logger(), "  %s %.2f %s, %.0f/%.0f s, %.2f m off the reference",
                      profile_.c_str(), progress, profile_ == "helix" ? "turns" : "m",
                      t, duration_, off);
        }
        if (t >= duration_) {
          RCLCPP_INFO(get_logger(), "segment finished after %.0f s", t);
          goToStep(HOLD_END);
        }
        break;
      }

      case HOLD_END: {
        double ve, vn, vu, yaw_rate;
        reference(duration_, ref, ve, vn, vu, yaw_rate, progress);
        have_ref = true;
        track(ref, 0.0, 0.0, 0.0, 0.0, approach_speed_);
        if (secondsInStep() >= hold_end_seconds_) { goToStep(RETURN); }
        break;
      }

      // Up to return_height FIRST, where we are -- clear of every tower and
      // wire -- and only then across to the spawn point.
      case RETURN: {
        if (ticks_in_step_ == 1) { return_climbed_ = u_ >= return_height_ - 0.5; }
        if (!return_climbed_) {
          Target up{e_, n_, return_height_, yaw_};
          if (ticks_in_step_ == 1) { hold_ = up; }
          if (goTo(hold_, approach_speed_)) { return_climbed_ = true; settled_ticks_ = 0; }
        } else {
          Target home{spawn_east_, spawn_north_, return_height_, yawTowards(spawn_east_, spawn_north_)};
          const double remaining = std::hypot(home.e - e_, home.n - n_);
          if (remaining <= 3.0) { home.yaw = yaw_; }
          if (goTo(home, return_speed_)) { goToStep(DESCEND); }
        }
        if (secondsInStep() >= goto_timeout_) { goToStep(LAND); }
        break;
      }

      case DESCEND: {
        if (ticks_in_step_ == 1) { hold_ = Target{spawn_east_, spawn_north_, 4.0, yaw_}; }
        if (goTo(hold_, approach_speed_) || secondsInStep() >= goto_timeout_) { goToStep(LAND); }
        break;
      }

      // As demo_orbit_mission: stop, then stop commanding and land.
      case LAND:
        if (secondsInStep() < 2.0) {
          publishVelocity(0.0, 0.0, 0.0, 0.0);
        } else if (!isArmed() && secondsInStep() > 4.0) {
          RCLCPP_INFO(get_logger(), "landed and disarmed");
          sendCommand(release_client_, "release_ctrl_authority");
          goToStep(DONE);
        } else if (!isComingDown() && ticks_in_step_ % 100 == 1) {
          sendCommand(land_client_, "land");
        }
        break;

      case DONE:
        if (ticks_in_step_ == 1) {
          RCLCPP_INFO(get_logger(), "flight finished");
        }
        if (ticks_in_step_ == (int)TICK_HZ) {
          rclcpp::shutdown();   // lets scripts/vio_trial.sh move on to scoring
          return;
        }
        break;
    }

    if (now().nanoseconds() > 0) { publishPhase(progress, have_ref ? &ref : nullptr); }
  }

  // ---- settings ----
  std::string profile_;
  double tower_east_, tower_north_, radius_, overlap_, vfov_deg_, tower_half_width_;
  double helix_bottom_, helix_top_, helix_speed_, helix_start_deg_;
  bool helix_ccw_;
  double transit_start_east_, transit_end_east_, transit_north_, transit_height_, transit_speed_;
  double spawn_east_, spawn_north_, return_height_, approach_speed_, return_speed_;
  double max_climb_speed_, position_gain_, yaw_gain_, max_yaw_rate_deg_;
  bool yaw_rate_clockwise_;
  double settle_seconds_, hold_end_seconds_, pre_takeoff_wait_;
  bool start_perception_, require_estimator_;
  std::string estimator_topic_;
  double estimator_wait_;
  bool allow_unsafe_;
  double takeoff_timeout_, offboard_timeout_, goto_timeout_;

  // ---- the segment, worked out in configure() ----
  bool configured_ = false;
  double pitch_ = 0.0, turns_ = 0.0, omega_ = 0.0, climb_rate_ = 0.0, duration_ = 0.0;
  double theta0_ = 0.0, direction_ = 1.0;

  // ---- ROS stuff ----
  rclcpp::Subscription<FlightStatus>::SharedPtr flight_status_sub_;
  rclcpp::Subscription<DisplayMode>::SharedPtr display_mode_sub_;
  rclcpp::Subscription<PositionFused>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimator_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Client<Trigger>::SharedPtr authority_client_, release_client_, takeoff_client_, land_client_;
  rclcpp::Client<PerceptionSetup>::SharedPtr perception_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- what the drone is doing ----
  uint8_t flight_status_ = FlightStatus::FLIGHT_STATUS_STOPED;
  uint8_t display_mode_  = DisplayMode::DISPLAY_MODE_MANUAL_CTRL;
  double e_ = 0.0, n_ = 0.0, u_ = 0.0, yaw_ = 0.0;
  bool got_status_ = false, got_position_ = false, position_healthy_ = false, got_attitude_ = false;
  bool got_estimate_ = false;
  rclcpp::Time last_estimate_{0, 0, RCL_ROS_TIME};

  // ---- where we are in the sequence ----
  Step step_ = WAIT_FOR_DATA;
  int ticks_in_step_ = 0;
  int settled_ticks_ = 0;
  rclcpp::Time segment_t0_{0, 0, RCL_ROS_TIME};
  Target hold_;
  bool return_climbed_ = false;
  double yaw_check_start_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioBenchmark>());
  if (rclcpp::ok()) { rclcpp::shutdown(); }
  return 0;
}
