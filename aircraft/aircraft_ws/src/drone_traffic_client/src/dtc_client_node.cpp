
#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"
#include "autopilot_interface_msgs/action/orbit.hpp"
#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/srv/set_speed.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;


// ============================================================
// 1. DTC Client : 드론별 명령 수신 및 전달
// ============================================================
// 지상 제어기에서 생성한 비행 명령을 각 드론에 전달하는 역할
//
// 전체 명령 전달 구조:
//
// dtc_controller_node.py (지상 편대 제어기)
//          ↓
//    /dtc_commands
//          ↓
//      DTCClient
//          ↓
// autopilot_interface (비행 제어 기능)
//          ↓
//         PX4
//
// 각 드론은 자신에게 할당된 DRONE_ID의 명령만 처리
// ============================================================

class DTCClient : public rclcpp::Node
{
public:

    // ========================================================
    // 2. ROS2 노드 초기화 및 통신 연결
    // ========================================================
    // - 드론 ID 확인
    // - 지상 제어기 명령 수신 설정
    // - 비행 제어를 위한 Action / Service 연결
    // - 명령 처리용 반복 타이머 생성
    // ========================================================

    DTCClient()
        : Node("dtc_client"),
          action_accepted_(true),
          request_being_sent_(false),
          target_action_("")
    {
        // ----------------------------------------------------
        // 2-1. 현재 드론 ID 설정
        // ----------------------------------------------------
        // 환경변수 DRONE_ID를 읽어 자신의 기체 번호 결정
        // 환경변수가 없으면 기본값 1 사용

        // NOLINTNEXTLINE(concurrency-mt-unsafe) - "DRONE_ID" read from getenv once at construction, before threads start
        const char* env_drone_id = std::getenv("DRONE_ID");
        drone_id_ = env_drone_id ? std::string(env_drone_id) : "1";

        RCLCPP_INFO(
            this->get_logger(),
            "Starting DTC Client for Drone %s",
            drone_id_.c_str()
        );

        // ----------------------------------------------------
        // 2-2. 지상 제어기의 비행 명령 수신
        // ----------------------------------------------------
        // /dtc_commands 토픽을 구독
        // 새로운 명령이 도착하면 cmd_cb() 실행

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dtc_commands",
            10,
            std::bind(
                &DTCClient::cmd_cb,
                this,
                std::placeholders::_1
            )
        );

        // ----------------------------------------------------
        // 2-3. 비행 제어 Action / Service 연결
        // ----------------------------------------------------
        // Action:
        //   takeoff  : 이륙
        //   land     : 착륙
        //   orbit    : 원형 선회 비행
        //   offboard : 외부 제어기 기반 비행
        //
        // Service:
        //   reposition : 지정된 목표 위치로 이동
        //   set_speed  : 비행 속도 설정
        //
        // 각 드론은 /Drone1, /Drone2 등의 고유 경로 사용

        tkf_cli_ = rclcpp_action::create_client<
            autopilot_interface_msgs::action::Takeoff
        >(
            this,
            "/Drone" + drone_id_ + "/takeoff_action"
        );

        land_cli_ = rclcpp_action::create_client<
            autopilot_interface_msgs::action::Land
        >(
            this,
            "/Drone" + drone_id_ + "/land_action"
        );

        orbit_cli_ = rclcpp_action::create_client<
            autopilot_interface_msgs::action::Orbit
        >(
            this,
            "/Drone" + drone_id_ + "/orbit_action"
        );

        offboard_cli_ = rclcpp_action::create_client<
            autopilot_interface_msgs::action::Offboard
        >(
            this,
            "/Drone" + drone_id_ + "/offboard_action"
        );

        repo_cli_ = this->create_client<
            autopilot_interface_msgs::srv::SetReposition
        >(
            "/Drone" + drone_id_ + "/set_reposition"
        );

        speed_cli_ = this->create_client<
            autopilot_interface_msgs::srv::SetSpeed
        >(
            "/Drone" + drone_id_ + "/set_speed"
        );

        // ----------------------------------------------------
        // 2-4. 명령 실행 및 재시도 타이머
        // ----------------------------------------------------
        // 1초마다 enforcement_loop() 실행
        // 아직 수락되지 않은 명령의 전송을 시도
        // 이는 드론의 실제 비행 제어 주기와는 다름

        enforcer_timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            1s,
            std::bind(
                &DTCClient::enforcement_loop,
                this
            )
        );
    }

private:

    // ========================================================
    // 3. 지상 제어기 명령 수신
    // ========================================================
    // /dtc_commands에서 JSON 형식의 명령 수신
    //
    // 처리 순서:
    // ① JSON 문자열 해석
    // ② 명령 대상 드론 ID 확인
    // ③ 비행 동작과 목표 매개변수 저장
    // ④ 새로운 명령을 실행 대기 상태로 변경
    //
    // 여기서는 실제 비행 명령을 실행하지 않고 저장만 수행
    // ========================================================

    void cmd_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        try {

            // ------------------------------------------------
            // 3-1. JSON 명령 해석 및 드론 ID 확인
            // ------------------------------------------------

            auto cmd = json::parse(msg->data);

            // drone_id가 숫자 또는 문자열로 전달되는 경우 모두 처리
            std::string rx_id;

            if (cmd.contains("drone_id")) {

                if (cmd["drone_id"].is_number()) {
                    rx_id = std::to_string(
                        cmd["drone_id"].get<int>()
                    );

                } else {
                    rx_id = cmd["drone_id"].get<std::string>();
                }
            }

            // 다른 드론에 대한 명령이면 무시
            if (rx_id != drone_id_) return;

            // ------------------------------------------------
            // 3-2. 비행 명령 및 목표 매개변수 저장
            // ------------------------------------------------
            // action          : 수행할 비행 동작
            // alt             : 목표 고도
            // east / north    : 목표 위치
            // radius          : 선회 반지름
            // controller_name : Offboard 제어기 이름
            // duration        : Offboard 제어 지속 시간
            // speed           : 목표 속도
            //
            // JSON에 값이 없으면 지정된 기본값 사용

            target_action_ = cmd.value("action", "");

            target_alt_ = cmd.value("alt", 40.0f);
            target_east_ = cmd.value("east", 0.0f);
            target_north_ = cmd.value("north", 0.0f);
            target_radius_ = cmd.value("radius", 50.0f);

            target_vtol_heading_ = cmd.value(
                "vtol_heading", 90.0f
            );

            target_vtol_loiter_n_ = cmd.value(
                "vtol_loiter_n", 150.0f
            );

            target_vtol_loiter_e_ = cmd.value(
                "vtol_loiter_e", 0.0f
            );

            target_vtol_loiter_alt_ = cmd.value(
                "vtol_loiter_alt", 100.0f
            );

            target_controller_name_ = cmd.value(
                "controller_name", ""
            );

            target_duration_ = cmd.value(
                "duration", 3.0f
            );

            target_speed_ = cmd.value(
                "speed", 5.0f
            );

            // ------------------------------------------------
            // 3-3. 새로운 명령을 실행 대기 상태로 변경
            // ----------------------------------------------------
            // action_accepted_ = false:
            //   아직 비행 제어 시스템이 명령을 수락하지 않음
            //
            // request_being_sent_ = false:
            //   새로운 명령의 전송을 시도할 수 있음

            action_accepted_ = false;
            request_being_sent_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "New Command Queued: %s",
                target_action_.c_str()
            );

        } catch (const json::exception& e) {

            // JSON 형식이나 데이터 처리에 문제가 발생한 경우
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to process command: %s",
                e.what()
            );
        }
    }

    // ========================================================
    // 4. 비행 명령 실행 및 재시도 관리
    // ========================================================
    // 1초마다 실행되는 명령 전달 관리 함수
    //
    // ① 실행할 명령이 있는지 확인
    // ② 해당 Action / Service 준비 상태 확인
    // ③ 저장된 목표 매개변수로 비행 명령 전송
    // ④ 명령 수락 여부 확인
    // ⑤ 거부된 명령은 다음 실행 시 재시도
    //
    // 명령 수락은 실제 비행 완료를 의미하지 않음
    // ========================================================

    void enforcement_loop()
    {
        // ----------------------------------------------------
        // 4-1. 명령 실행 가능 여부 확인
        // ----------------------------------------------------
        // 이미 수락되었거나, 명령이 없거나,
        // 전송 중인 요청이 있으면 실행하지 않음

        if (
            action_accepted_
            || target_action_.empty()
            || request_being_sent_
        ) return;

        RCLCPP_INFO(
            this->get_logger(),
            "Attempting to enforce %s...",
            target_action_.c_str()
        );

        // 중복 전송 방지를 위해 요청 진행 상태 설정
        request_being_sent_ = true;

        // ----------------------------------------------------
        // 4-2. Action 명령 수락 여부 확인 함수
        // ----------------------------------------------------
        // Action 서버의 목표 요청 응답을 처리
        //
        // 거부되면 다시 전송할 수 있도록 상태 변경
        // 수락되면 해당 명령의 재전송 중단

        auto action_cb = [this](auto gh) {

            request_being_sent_ = false;

            if (!gh) {

                RCLCPP_WARN(
                    this->get_logger(),
                    "Autopilot REJECTED %s. Retrying...",
                    target_action_.c_str()
                );

            } else {

                RCLCPP_INFO(
                    this->get_logger(),
                    "Autopilot ACCEPTED %s!",
                    target_action_.c_str()
                );

                action_accepted_ = true;
            }
        };

        // ====================================================
        // 5. 비행 명령 종류별 처리
        // ====================================================
        // 수신한 action 값에 따라 필요한 제어 기능 호출
        //
        // takeoff    → 이륙 Action
        // orbit      → 선회 Action
        // land       → 착륙 Action
        // offboard   → 외부 제어 Action
        // reposition → 위치 이동 Service
        // set_speed  → 속도 설정 Service
        // ====================================================

        // ----------------------------------------------------
        // 5-1. 이륙 명령
        // ----------------------------------------------------
        // 목표 이륙 고도와 VTOL 전환 관련 매개변수 전달

        if (target_action_ == "takeoff") {

            if (!tkf_cli_->action_server_is_ready()) return;

            auto goal =
                autopilot_interface_msgs::action::Takeoff::Goal();

            goal.takeoff_altitude = target_alt_;
            goal.vtol_transition_heading = target_vtol_heading_;
            goal.vtol_loiter_north = target_vtol_loiter_n_;
            goal.vtol_loiter_east = target_vtol_loiter_e_;
            goal.vtol_loiter_alt = target_vtol_loiter_alt_;

            auto opts = rclcpp_action::Client<
                autopilot_interface_msgs::action::Takeoff
            >::SendGoalOptions();

            opts.goal_response_callback = action_cb;

            tkf_cli_->async_send_goal(goal, opts);

        // ----------------------------------------------------
        // 5-2. 원형 선회 비행 명령
        // ----------------------------------------------------
        // 선회 중심의 상대 위치, 고도, 반지름 전달

        } else if (target_action_ == "orbit") {

            if (!orbit_cli_->action_server_is_ready()) return;

            auto goal =
                autopilot_interface_msgs::action::Orbit::Goal();

            goal.east = target_east_;
            goal.north = target_north_;
            goal.altitude = target_alt_;
            goal.radius = target_radius_;

            auto opts = rclcpp_action::Client<
                autopilot_interface_msgs::action::Orbit
            >::SendGoalOptions();

            opts.goal_response_callback = action_cb;

            orbit_cli_->async_send_goal(goal, opts);

        // ----------------------------------------------------
        // 5-3. 착륙 명령
        // ----------------------------------------------------
        // 착륙 Action 서버에 목표 고도 및 방향 전달

        } else if (target_action_ == "land") {

            if (!land_cli_->action_server_is_ready()) return;

            auto goal =
                autopilot_interface_msgs::action::Land::Goal();

            goal.landing_altitude = target_alt_;
            goal.vtol_transition_heading = target_vtol_heading_;

            auto opts = rclcpp_action::Client<
                autopilot_interface_msgs::action::Land
            >::SendGoalOptions();

            opts.goal_response_callback = action_cb;

            land_cli_->async_send_goal(goal, opts);

        // ----------------------------------------------------
        // 5-4. Offboard 외부 제어 명령
        // ----------------------------------------------------
        // 외부 제어기 이름과 최대 실행 시간을 전달
        //
        // 향후 충돌 회피 제어기를 Offboard 방식으로
        // 구현한다면 관련 연결 구조를 확인할 부분

        } else if (target_action_ == "offboard") {

            if (!offboard_cli_->action_server_is_ready()) return;

            auto goal =
                autopilot_interface_msgs::action::Offboard::Goal();

            goal.controller_name = target_controller_name_;
            goal.max_duration_sec = target_duration_;

            auto opts = rclcpp_action::Client<
                autopilot_interface_msgs::action::Offboard
            >::SendGoalOptions();

            opts.goal_response_callback = action_cb;

            offboard_cli_->async_send_goal(goal, opts);

        // ----------------------------------------------------
        // 5-5. 목표 위치 이동 명령
        // ----------------------------------------------------
        // 지상 편대 제어기에서 계산한 위치를 전달
        //
        // 앞서 분석한 command_quad_polygon()에서
        // 전송한 reposition 명령이 여기서 처리됨
        //
        // 현재는 목표 위치를 전달할 뿐
        // 이동 경로의 충돌 위험을 검사하지 않음

        } else if (target_action_ == "reposition") {

            if (!repo_cli_->service_is_ready()) return;

            auto req = std::make_shared<
                autopilot_interface_msgs::srv::SetReposition::Request
            >();

            req->east = target_east_;
            req->north = target_north_;
            req->altitude = target_alt_;

            // 서비스 응답을 비동기로 수신
            repo_cli_->async_send_request(
                req,
                [this](
                    rclcpp::Client<
                        autopilot_interface_msgs::srv::SetReposition
                    >::SharedFuture f
                ) {
                    request_being_sent_ = false;

                    auto res = f.get();

                    if (res->success) {

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Autopilot ACCEPTED Reposition!"
                        );

                        action_accepted_ = true;

                    } else {

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Autopilot REJECTED Reposition: %s. Retrying...",
                            res->message.c_str()
                        );
                    }
                }
            );

        // ----------------------------------------------------
        // 5-6. 비행 속도 설정 명령
        // ----------------------------------------------------
        // 목표 속도를 자동비행장치에 전달
        //
        // 주의:
        // set_speed는 목표 비행 속도 설정 기능으로,
        // 각 방향의 속도 벡터를 직접 제어하는 기능과 다름

        } else if (target_action_ == "set_speed") {

            if (!speed_cli_->service_is_ready()) return;

            auto req = std::make_shared<
                autopilot_interface_msgs::srv::SetSpeed::Request
            >();

            req->speed = target_speed_;

            speed_cli_->async_send_request(
                req,
                [this](
                    rclcpp::Client<
                        autopilot_interface_msgs::srv::SetSpeed
                    >::SharedFuture f
                ) {
                    request_being_sent_ = false;

                    auto res = f.get();

                    if (res->success) {

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Autopilot ACCEPTED SetSpeed!"
                        );

                        action_accepted_ = true;

                    } else {

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Autopilot REJECTED SetSpeed: %s. Retrying...",
                            res->message.c_str()
                        );
                    }
                }
            );
        }
    }

    // ========================================================
    // 6. 드론별 명령 및 통신 상태 저장
    // ========================================================
    // DTCClient가 처리하는 명령과 매개변수를 저장
    //
    // 명령 관련:
    //   drone_id_       : 현재 드론 ID
    //   target_action_  : 실행할 비행 명령
    //
    // 위치 및 비행 관련:
    //   target_alt_     : 목표 고도
    //   target_east_    : 목표 동쪽 위치
    //   target_north_   : 목표 북쪽 위치
    //   target_radius_  : 선회 반지름
    //   target_speed_   : 목표 속도
    //
    // 상태 관련:
    //   action_accepted_   : 명령 수락 여부
    //   request_being_sent_: 명령 전송 진행 여부
    // ========================================================

    std::string drone_id_;
    std::string target_action_;

    float target_alt_, target_east_, target_north_;
    float target_radius_, target_speed_, target_duration_;

    float target_vtol_heading_, target_vtol_loiter_n_;
    float target_vtol_loiter_e_, target_vtol_loiter_alt_;

    std::string target_controller_name_;

    std::atomic<bool> action_accepted_;
    std::atomic<bool> request_being_sent_; // 동일 명령 중복 전송 방지

    // --------------------------------------------------------
    // 6-1. ROS2 통신 객체
    // --------------------------------------------------------

    // 지상 제어기 명령 구독자
    rclcpp::Subscription<
        std_msgs::msg::String
    >::SharedPtr cmd_sub_;

    // 기체 비행 제어 Action 클라이언트
    rclcpp_action::Client<
        autopilot_interface_msgs::action::Takeoff
    >::SharedPtr tkf_cli_;

    rclcpp_action::Client<
        autopilot_interface_msgs::action::Land
    >::SharedPtr land_cli_;

    rclcpp_action::Client<
        autopilot_interface_msgs::action::Orbit
    >::SharedPtr orbit_cli_;

    rclcpp_action::Client<
        autopilot_interface_msgs::action::Offboard
    >::SharedPtr offboard_cli_;

    // 위치 이동 및 속도 설정 Service 클라이언트
    rclcpp::Client<
        autopilot_interface_msgs::srv::SetReposition
    >::SharedPtr repo_cli_;

    rclcpp::Client<
        autopilot_interface_msgs::srv::SetSpeed
    >::SharedPtr speed_cli_;

    // 명령 실행 및 재시도를 위한 반복 타이머
    rclcpp::TimerBase::SharedPtr enforcer_timer_;
};


// ============================================================
// 7. ROS2 노드 실행
// ============================================================
// ROS2 초기화 → DTCClient 생성 → 명령 수신 및 처리 반복
// 프로그램 종료 시 ROS2 통신 종료
// ============================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DTCClient>());
    rclcpp::shutdown();
    return 0;
}
