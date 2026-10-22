using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;

// Subscribes to /cmd_vel (geometry_msgs/Twist) and drives the kinematic robot by
// integrating the base pose. ROS angular.z is CCW; Unity yaw is CW, so it's negated.
// Defaults are TurtleBot3 Burger velocity limits.
//
// [수정됨] 벽/가구 충돌 방지 + 키보드 테스트 모드 + 디버그 로그
// [수정됨 2] 키보드 좌우 회전 방향 반전 버그 수정 (A 누르면 왼쪽으로 돌도록)
public class DiffDriveController : MonoBehaviour
{
    public string CmdVelTopic = "cmd_vel";
    public float MaxLinear = 0.22f;
    public float MaxAngular = 2.84f;
    public float CmdTimeout = 0.5f;

    [Header("충돌 검사 설정 (벽/가구를 못 뚫게 막는 부분)")]
    public float collisionCheckRadius = 0.1f;
    public LayerMask obstacleLayers = ~0;

    [Header("키보드 테스트 모드")]
    public bool allowKeyboardWhenDisconnected = true;

    [Header("디버그")]
    [Tooltip("켜두면 키 입력/이동이 Console에 로그로 찍힙니다. 테스트 끝나면 꺼주세요.")]
    public bool debugLog = false;

    private RosConnector connector;
    private volatile float cmdLinear;
    private volatile float cmdAngular;
    private volatile bool gotCmd;
    private float lastCmdTime;
    private bool subscribed;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        if (debugLog) Debug.Log("[DiffDriveController] Start() 호출됨. connector=" + (connector != null));
    }

    private void FixedUpdate()
    {
        bool rosConnected = connector != null && connector.RosSocket != null;

        if (rosConnected && !subscribed)
        {
            connector.RosSocket.Subscribe<Geometry.Twist>(CmdVelTopic, OnCmdVel);
            subscribed = true;
        }
        if (gotCmd) { lastCmdTime = Time.time; gotCmd = false; }

        float lin = cmdLinear, ang = cmdAngular;

        bool rosCmdFresh = Time.time - lastCmdTime <= CmdTimeout;
        if (!rosCmdFresh)
        {
            lin = 0f;
            ang = 0f;

            if (allowKeyboardWhenDisconnected)
            {
                var keyboard = Keyboard.current;
                if (keyboard == null)
                {
                    if (debugLog && Time.frameCount % 60 == 0)
                        Debug.LogWarning("[DiffDriveController] Keyboard.current가 null입니다.");
                }
                else
                {
                    float vertical = 0f;
                    if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) vertical += 1f;
                    if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) vertical -= 1f;

                    float horizontal = 0f;
                    if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) horizontal += 1f;
                    if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) horizontal -= 1f;

                    if (debugLog && (vertical != 0f || horizontal != 0f))
                        Debug.Log("[DiffDriveController] 키 입력 감지됨: vertical=" + vertical + " horizontal=" + horizontal);

                    lin = vertical * MaxLinear;
                    // [수정] 부호 반전: A(왼쪽)를 누르면 왼쪽(반시계)으로, D(오른쪽)를 누르면 오른쪽(시계)으로 돌도록
                    ang = -horizontal * MaxAngular;
                }
            }
        }

        lin = Mathf.Clamp(lin, -MaxLinear, MaxLinear);
        ang = Mathf.Clamp(ang, -MaxAngular, MaxAngular);

        float dt = Time.fixedDeltaTime;

        transform.Rotate(0f, -ang * Mathf.Rad2Deg * dt, 0f, Space.World);

        Vector3 moveDelta = transform.forward * (lin * dt);
        float moveDistance = moveDelta.magnitude;

        if (moveDistance > 0f)
        {
            Vector3 moveDir = moveDelta.normalized;
            Vector3 castOrigin = transform.position + Vector3.up * collisionCheckRadius;

            bool blocked = Physics.SphereCast(
                castOrigin,
                collisionCheckRadius,
                moveDir,
                out RaycastHit hit,
                moveDistance,
                obstacleLayers,
                QueryTriggerInteraction.Ignore);

            if (blocked)
            {
                float safeDistance = Mathf.Max(0f, hit.distance - 0.01f);
                if (debugLog) Debug.Log("[DiffDriveController] 충돌 감지! 대상: " + hit.collider.name);
                transform.position += moveDir * safeDistance;
            }
            else
            {
                transform.position += moveDelta;
            }
        }
    }

    private void OnCmdVel(Geometry.Twist msg)
    {
        cmdLinear = (float)msg.linear.x;
        cmdAngular = (float)msg.angular.z;
        gotCmd = true;
    }
}
