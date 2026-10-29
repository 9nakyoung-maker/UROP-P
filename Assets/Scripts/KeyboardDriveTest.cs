using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// ROS 연동 전, 키보드로 터틀봇을 움직여서 Collider(충돌) 테스트용으로 쓰는 임시 스크립트입니다.
/// (새 Input System 패키지 기준으로 작성됨)
///
/// 사용법:
///   1. 터틀봇의 "루트" 오브젝트 (turtlebot3_burger, DiffDriveController가 붙어있는 곳)에 이 스크립트를 추가합니다.
///   2. 기존 DiffDriveController 컴포넌트는 체크박스를 눌러서 잠시 꺼둡니다. (지우지 말고 비활성화만)
///   3. Play 모드에서 W/S(전진/후진), A/D(좌우 회전) 키로 조종합니다.
///   4. 테스트가 끝나면 이 스크립트를 비활성화하고, DiffDriveController를 다시 켜서 ROS 제어로 돌아갑니다.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class KeyboardDriveTest : MonoBehaviour
{
    [Header("속도 설정 (터틀봇3 버거 기준 최대값)")]
    public float maxLinearSpeed = 0.22f;   // m/s
    public float maxAngularSpeed = 2.84f;  // rad/s

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody가 없습니다. 이 오브젝트(또는 부모)에 Rigidbody를 추가해주세요.");
        }
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        // W/S 또는 위/아래 화살표: 전진/후진
        float vertical = 0f;
        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) vertical += 1f;
        if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) vertical -= 1f;

        // A/D 또는 좌/우 화살표: 회전
        float horizontal = 0f;
        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) horizontal += 1f;
        if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) horizontal -= 1f;

        float linear = vertical * maxLinearSpeed;
        float angular = -horizontal * maxAngularSpeed;

        // 현재 바라보는 방향으로 이동
        Vector3 moveDirection = transform.forward * linear;
        rb.MovePosition(rb.position + moveDirection * Time.fixedDeltaTime);

        // 회전 적용 (Y축 기준)
        Quaternion deltaRotation = Quaternion.Euler(0f, angular * Mathf.Rad2Deg * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * deltaRotation);
    }
}
