using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using Twist = RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist;

// Unity 키보드(W/S/A/D/Q/E) 입력을 실제 로봇의 cmd_vel 로 보내는 스크립트
// 붙이는 곳: RosConnector가 있는 오브젝트 (rosmentor)
//
// [수정] 팀원 원본(김도훈, 2026-11-01)에서 바뀐 점
//  - ROS#의 UnityPublisher는 Start()에서 바로 토픽을 등록(advertise)하는데, RosConnector는 별도 스레드에서
//    접속하기 때문에 와이파이로 로봇에 붙을 때는 Start 시점에 아직 연결이 안 돼 있을 수 있음
//    → NullReferenceException 후 아무것도 안 보내짐. 연결될 때까지 기다렸다가 등록하도록 변경.
//  - 연결 전·실패 시 Publish 하지 않음 (에러 도배 방지)
//  - 화면 왼쪽 위에 연결 상태 표시
public class CmdVelKeyboardPublisher : UnityPublisher<Twist>
{
    [Header("속도 설정")]
    public float maxLinear = 0.2f;    // 앞뒤/옆 최고속도 (m/s)
    public float maxAngular = 1.0f;   // 회전 최고속도 (rad/s)
    public float publishRate = 10f;   // 1초에 몇 번 보낼지

    [Header("디버그")]
    public bool debugLog = true;
    public bool showStatus = true;

    private Twist message;
    private float timer;
    private bool wasMoving;
    private bool ready;               // 토픽 등록까지 끝났는지
    private string status = "로봇 연결 기다리는 중...";

    protected override void Start()
    {
        message = new Twist();
        if (string.IsNullOrEmpty(Topic)) Topic = "/controller/cmd_vel";
        StartCoroutine(AdvertiseWhenConnected());
    }

    IEnumerator AdvertiseWhenConnected()
    {
        var connector = GetComponent<RosConnector>();
        float waited = 0f;
        // RosConnector가 접속을 끝낼 때까지 대기
        while (connector.RosSocket == null || connector.IsConnected == null || !connector.IsConnected.WaitOne(0))
        {
            waited += Time.unscaledDeltaTime;
            if (waited > connector.SecondsTimeout + 1f)
            {
                status = "연결 실패 ✖ — 와이파이(HW-...)와 Ros Bridge Server Url 확인";
                Debug.LogError("[CmdVelPub] " + status + " : " + connector.RosBridgeServerUrl);
                yield break;
            }
            yield return null;
        }
        base.Start();   // 이제 토픽 등록(advertise)
        ready = true;
        status = $"연결됨 ✓ {Topic}";
        Debug.Log($"[CmdVelPub] 연결 완료, {Topic} 등록함");
    }

    void Update()
    {
        if (!ready) return;
        var kb = Keyboard.current;
        if (kb == null) return;

        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        float v = (kb.wKey.isPressed ? 1 : 0) - (kb.sKey.isPressed ? 1 : 0);
        float w = (kb.aKey.isPressed ? 1 : 0) - (kb.dKey.isPressed ? 1 : 0);    // ROS: +면 왼쪽 회전
        float side = (kb.qKey.isPressed ? 1 : 0) - (kb.eKey.isPressed ? 1 : 0); // ROS: +면 왼쪽 이동
        bool moving = v != 0 || w != 0 || side != 0;

        // 누르는 동안 계속 보내고, 떼는 순간 정지 명령 한 번
        if (moving || wasMoving)
        {
            message.linear.x = v * maxLinear;
            message.linear.y = side * maxLinear;
            message.angular.z = w * maxAngular;
            Publish(message);
            if (debugLog)
                Debug.Log($"[CmdVelPub] 보냄 x={message.linear.x:F2} y={message.linear.y:F2} z={message.angular.z:F2}");
        }
        wasMoving = moving;
    }

    // Play 끌 때 로봇 멈추기
    void OnDisable()
    {
        if (!ready || message == null) return;
        message.linear.x = 0; message.linear.y = 0; message.angular.z = 0;
        try { Publish(message); } catch { }
    }

    void OnGUI()
    {
        if (!showStatus) return;
        GUI.Box(new Rect(Screen.width - 430, Screen.height - 36, 420, 26), "실제 로봇: " + status);   // 오른쪽 아래
    }
}
