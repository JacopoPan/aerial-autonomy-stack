
#include "collision_avoidance.hpp"

// ============================================================
// 1. 충돌 회피 제어기 초기화
// ============================================================

CollisionAvoidance::CollisionAvoidance()
    : safe_distance_(5.0)
{
}

// ============================================================
// 2. 최소 안전거리 설정
// ============================================================

void CollisionAvoidance::set_safe_distance(double distance)
{
    safe_distance_ = distance;
}

// ============================================================
// 3. 안전 속도 계산
// ============================================================
// 현재는 충돌 회피를 수행하지 않는 기본 구조
//
// 향후 구현할 기능:
// ① 자신의 위치와 속도 확인
// ② 주변 드론과의 상대 위치 및 상대 속도 계산
// ③ CBF 안전 제약조건 생성
// ④ 안전 제약을 만족하는 속도 계산
// ⑤ 최종 안전 속도 반환
// ============================================================

Velocity3D CollisionAvoidance::compute_safe_velocity(
    const DroneState& own_state,
    const std::vector<DroneState>& neighbors,
    const Velocity3D& desired_velocity
)
{
    // 추후 CBF 알고리즘 구현
    (void)own_state;
    (void)neighbors;

    // 현재는 기본 편대 속도를 수정하지 않음
    return desired_velocity;
}
