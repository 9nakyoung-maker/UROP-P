using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;

// [수정됨] 메카넘 휠(4륜 전방향) 주행용으로 업그레이드!
// 기존의 벽 충돌 방지 로직은 100% 유지되며, 대각선 및 게걸음 이동 시에도 완벽하게 작동합니다.
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

    // 메카넘 이동을 위해 전후(X) 외에 좌우(Y) 변수를 추가했습니다.
    private volatile float cmdLinearX;
    private volatile float cmdLinearY;
    private volatile float cmdAngularZ;
    private volatile bool gotCmd;
    private float lastCmdTime;
    private bool subscribed;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        if (debugLog) Debug.Log("[MecanumController] Start() 호출됨. connector=" + (connector != null));
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

        float linX = cmdLinearX, linY = cmdLinearY, ang = cmdAngularZ;

        bool rosCmdFresh = Time.time - lastCmdTime <= CmdTimeout;
        if (!rosCmdFresh)
        {
            linX = 0f;
            linY = 0f;
            ang = 0f;

            if (allowKeyboardWhenDisconnected)
            {
                var keyboard = Keyboard.current;
                if (keyboard != null)
                {
                    // 1. 전진/후진 (W / S)
                    float vertical = 0f;
                    if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) vertical += 1f;
                    if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) vertical -= 1f;

                    // 2. 좌우 측면 이동/게걸음 (A / D)
                    float horizontal = 0f;
                    if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) horizontal += 1f;
                    if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) horizontal -= 1f;

                    // 3. 제자리 회전 (Q / E)
                    float rotation = 0f;
                    if (keyboard.qKey.isPressed) rotation += 1f; // Q 누르면 왼쪽 회전
                    if (keyboard.eKey.isPressed) rotation -= 1f; // E 누르면 오른쪽 회전

                    linX = vertical * MaxLinear;
                    linY = horizontal * MaxLinear; // 메카넘 좌우 이동 속도
                    ang = rotation * MaxAngular;
                }
            }
        }

        linX = Mathf.Clamp(linX, -MaxLinear, MaxLinear);
        linY = Mathf.Clamp(linY, -MaxLinear, MaxLinear);
        ang = Mathf.Clamp(ang, -MaxAngular, MaxAngular);

        float dt = Time.fixedDeltaTime;

        // 회전 (차체 방향 틀기)
        transform.Rotate(0f, -ang * Mathf.Rad2Deg * dt, 0f, Space.World);

        // [핵심 변경점] 전후진(forward) 뿐만 아니라 좌우 게걸음(right) 이동 벡터를 합산합니다.
        Vector3 moveDelta = (transform.forward * (linX * dt)) + (transform.right * (linY * dt));
        float moveDistance = moveDelta.magnitude;

        // 장애물 충돌 검사 로직 (대각선/게걸음 이동 시에도 방향을 자동 계산하여 막아줍니다)
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
                if (debugLog) Debug.Log("[MecanumController] 충돌 감지! 대상: " + hit.collider.name);
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
        cmdLinearX = (float)msg.linear.x;
        // ROS 환경에서도 메카넘 휠의 좌우(Y축) 제어값을 받을 수 있게 추가
        cmdLinearY = (float)msg.linear.y;
        cmdAngularZ = (float)msg.angular.z;
        gotCmd = true;
    }
}