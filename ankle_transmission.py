"""ankle_transmission_block.hpp의 forward 변환(모터각도 -> roll/pitch)을 파이썬으로 포팅.

RS06 앵클 모터 2개(ANKLE_A=qm5, ANKLE_B=qm6)가 링키지로 결합돼 있어서 로그에 남는
CAN 프레임은 항상 "모터 좌표계" 값이다. 상위제어기(tellus_sim2real)가 실제로 보고
쓰는 값은 roll/pitch 좌표계라서, CAN 로그만 보고는 상위제어기가 뭘 하는지 바로 알 수
없다 — 이 모듈로 모터 좌표계 -> roll/pitch 좌표계 변환을 걸어줘야 비교가 된다.

C++ 원본(inc가 아니라 저장소 루트의 ankle_transmission_block.hpp, LEFT/RIGHT
부호 처리 포함)을 1:1로 옮긴 것이다. 이미 실기에서 검증된 로직이라 여기서 부호를
임의로 고치지 않았다 — 원본이 바뀌면 이 파일도 같이 바뀌어야 한다.
"""

import math

Q1_C, Q1_S = 13.0, 55.0
Q2_C, Q2_S = -11.5, 54.5

EXPS = [
    (0, 0),
    (1, 0), (0, 1),
    (2, 0), (1, 1), (0, 2),
    (3, 0), (2, 1), (1, 2), (0, 3),
    (4, 0), (3, 1), (2, 2), (1, 3), (0, 4),
    (5, 0), (4, 1), (3, 2), (2, 3), (1, 4), (0, 5),
]

COEF_ROLL = [
    -0.64525189897, -24.0988414341, -24.4839660682, 8.05650691565,
    0.306780925171, -6.85956401710, 2.20736469770, -0.325221112300,
    -0.355183027713, 1.51917542947, 0.992104191788, 0.493826642472,
    -0.150008279940, -0.888453996626, -1.02116303025, -0.581964750131,
    -0.798741265770, -0.785986296883, -0.661584728371, -0.828985743533,
    -0.383055713498,
]

COEF_PITCH = [
    7.83282321421, 15.7142451114, -16.2128003792, -8.02224509563,
    -1.59211129335, -7.43935255886, 0.0962497692418, 0.528728716188,
    -0.0757622187754, -0.183851264614, 0.0760475243749, 0.385177269264,
    -0.287387336763, 0.405169790617, 0.143087927359, -0.0206135343435,
    -0.0134291943747, 0.953701170156, -0.330671622416, 0.0297788502222,
    0.111160432338,
]


def _eval_poly2d_with_derivatives(m, n, coef, q1_s, q2_s):
    m_pows = [1.0] * 6
    n_pows = [1.0] * 6
    for i in range(1, 6):
        m_pows[i] = m_pows[i - 1] * m
        n_pows[i] = n_pows[i - 1] * n

    val = 0.0
    dval_dm = 0.0
    dval_dn = 0.0

    for c, (px, py) in zip(coef, EXPS):
        mp, np_ = m_pows[px], n_pows[py]
        val += c * mp * np_
        if px > 0:
            dval_dm += c * px * m_pows[px - 1] * np_
        if py > 0:
            dval_dn += c * mp * py * n_pows[py - 1]

    return val, dval_dm / q1_s, dval_dn / q2_s


def _motor_fb_to_roll_pitch(qm5_deg, qm6_deg, qdotm5_deg_s, qdotm6_deg_s):
    """inc가 아니라 루트의 ankle_transmission_block.hpp AnkleTransmissionBase::motorFBToRollPitch 그대로."""
    qm5_deg, qm6_deg = -qm5_deg, -qm6_deg
    qdotm5_deg_s, qdotm6_deg_s = -qdotm5_deg_s, -qdotm6_deg_s

    m = (qm5_deg - Q1_C) / Q1_S
    n = (qm6_deg - Q2_C) / Q2_S

    q5_deg, dq5_dqm5, dq5_dqm6 = _eval_poly2d_with_derivatives(m, n, COEF_ROLL, Q1_S, Q2_S)
    q6_deg, dq6_dqm5, dq6_dqm6 = _eval_poly2d_with_derivatives(m, n, COEF_PITCH, Q1_S, Q2_S)

    qdot5_deg_s = dq5_dqm5 * qdotm5_deg_s + dq5_dqm6 * qdotm6_deg_s
    qdot6_deg_s = dq6_dqm5 * qdotm5_deg_s + dq6_dqm6 * qdotm6_deg_s

    return q5_deg, q6_deg, qdot5_deg_s, qdot6_deg_s


def motor_to_roll_pitch(direction, qm5_deg, qm6_deg, qdotm5_deg_s=0.0, qdotm6_deg_s=0.0):
    """AnkleTransmissionBlock<D>::computeVelocityEstimate 그대로.

    direction: "LEFT" 또는 "RIGHT"
    qm5_deg=ANKLE_A 모터각[deg], qm6_deg=ANKLE_B 모터각[deg] (커맨드든 피드백이든 동일하게 적용)
    반환: (roll_deg, pitch_deg, rollrate_deg_s, pitchrate_deg_s)
    """
    qm5_in, qm6_in = qm5_deg, qm6_deg
    if direction == "RIGHT":
        qm5_in, qm6_in = -qm5_deg, -qm6_deg

    q5_deg, q6_deg, qdot5_deg_s, qdot6_deg_s = _motor_fb_to_roll_pitch(
        qm5_in, qm6_in, qdotm5_deg_s, qdotm6_deg_s
    )

    if direction == "RIGHT":
        q5_deg = -q5_deg
        qdot6_deg_s = -qdot6_deg_s

    return q5_deg, q6_deg, qdot5_deg_s, qdot6_deg_s


def rad2deg(x):
    return x * 180.0 / math.pi
