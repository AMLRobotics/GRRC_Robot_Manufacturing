#include <stdlib.h>
#include <stdio.h>
#include <ur_kinematics/ur_kin.h>
#include <math.h>
#include "ros/ros.h"
#include "ur_ros_cartesian_control/cartesian2joint.h"
#include "ur_ros_cartesian_control/joint2cartesian.h"
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
 
// 각도를 -pi ~ pi 사이로 정규화하는 헬퍼 함수
double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// 8개의 IK 해 중에서 최적의 해를 반환하는 함수
std::vector<double> select_best_ik_solution(std::vector<std::vector<double>>& solutions, std::vector<double> q) {
    std::vector<double> best_sol;
    double min_dist = 1e10;
    std::vector<double> current_q = q;

    for (int i = 0; i < solutions.size(); i++) {
        // ========================================================
        // [추가된 핵심 코드] 
        // 3번째 관절(팔꿈치, index 2)이 0.0 이상으로 꺾이는 자세는 무조건 버립니다.
        // ========================================================
        if (solutions[i][2] >= 0.0 || solutions[i][0] >= 0.0 || solutions[i][3] >= 0.0) {
            continue; 
        }

        double current_dist = 0;
        for (int j = 0; j < 6; j++) {
            // [핵심 추가] 각도 언래핑 (Angle Unwrapping)
            // 후보 해답(solutions[i][j])을 last_q[j]와 가장 가까운 바퀴로 옮깁니다.
            double diff = solutions[i][j] - current_q[j];
            while (diff > M_PI) { solutions[i][j] -= 2.0 * M_PI; diff -= 2.0 * M_PI; }
            while (diff < -M_PI) { solutions[i][j] += 2.0 * M_PI; diff += 2.0 * M_PI; }

            current_dist += pow(diff, 2);
        }

        if (current_dist < min_dist) {
            min_dist = current_dist;
            best_sol = solutions[i];
        }
    }
    return best_sol;
}

//Calculate inverse kinematics of UR10 to prevent from crashing between Lidar and UR.
//If results of base joint(q1) belong between Min and Max danger zone, then program send danger signal.
bool get_joint_position(ur_ros_cartesian_control::cartesian2joint::Request  &req,
                        ur_ros_cartesian_control::cartesian2joint::Response &res)
{
    // 1. RPY (Degree)를 Radian으로 변환
    double roll = req.roll * M_PI / 180.0;
    double pitch = req.pitch * M_PI / 180.0;
    double yaw = req.yaw * M_PI / 180.0;

    // 2. [핵심] tf2를 이용해 안전하고 완벽한 동차 변환 행렬 생성
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // 내부적으로 완벽한 오일러 -> 쿼터니언 변환 수행

    tf2::Transform transform;
    transform.setRotation(q);
    
    // 주의: req.x, req.y, req.z는 반드시 "미터(m)" 단위여야 합니다!
    transform.setOrigin(tf2::Vector3(req.x, req.y, req.z));

    // 3. ur_kinematics가 요구하는 1D Array (Row-Major) 형태로 변환
    double T[16];
    for (int i = 0; i < 3; i++) {
        T[i * 4 + 0] = transform.getBasis()[i].x(); // 회전행렬 1열
        T[i * 4 + 1] = transform.getBasis()[i].y(); // 회전행렬 2열
        T[i * 4 + 2] = transform.getBasis()[i].z(); // 회전행렬 3열
        T[i * 4 + 3] = transform.getOrigin()[i];    // 위치 (Translation)
    }
    // 마지막 4번째 행 (0, 0, 0, 1)
    T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;

    // 4. 역기구학 풀이
    double solution[8 * 6]; 
    int sol_num = ur_kinematics::inverse(T, solution); // 리턴 타입은 int입니다.
    
    // printf("발견된 IK 해의 개수: %d\n", sol_num);

    if (sol_num == 0) {
        ROS_WARN("유효한 역기구학 해를 찾지 못했습니다. 목표 위치가 작업 반경 밖일 수 있습니다.");
        return false; 
    }

    // 5. 8개의 해 중에서 최적의 자세 필터링 (기존 코드 유지)
    std::vector<std::vector<double>> solution_vector;
    solution_vector.resize(sol_num, std::vector<double>(6, 0.0));

    for(int i = 0; i < sol_num; i++)
        {
            for(int j = 0; j < 6; j++) 
            {
                double raw_angle = solution[6 * i + j];
                
                // [추가된 정규화 로직] 
                // 각도가 파이(180도)보다 크면 2파이(360도)를 빼서 음수 각도로 변환
                if (raw_angle > M_PI) {
                    raw_angle -= 2.0 * M_PI;
                }
                
                solution_vector[i][j] = raw_angle;
            }
        }

    // 파이썬에서 작성했던 'select_best_ik_solution' 알고리즘 호출
    std::vector<double> selected_solution = select_best_ik_solution(solution_vector, req.last_joint_state);
    
    res.jointDegs = selected_solution;

    return true;
}

bool get_cartesian_position(ur_ros_cartesian_control::joint2cartesian::Request  &req,
         ur_ros_cartesian_control::joint2cartesian::Response &res)
{
    double T[16] = {0.0}, q[6] = {0.0};

    for(int i = 0; i < 6; i++)
    {
        q[i] = req.jointDegs[i];
    }
    
    ur_kinematics::forward(q, T);

    res.x = T[3];
    res.y = T[7];
    res.z = T[11];
    res.pitch = -asin(T[8]);
    res.roll = atan2(-T[9], T[10]);
    res.yaw = atan2(T[0], T[4]);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "UR10_IK_Solver");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::ServiceServer service_ik = n.advertiseService("/IK_solve", get_joint_position);
    ros::ServiceServer service_fk = n.advertiseService("/FK_solve", get_cartesian_position);

    ros::spin();
    return 0;
}