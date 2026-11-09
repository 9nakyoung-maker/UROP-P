using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;
using RosSharp.RosBridgeClient;
using Twist = RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist;
using LaserScan = RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan;

// =====================================================================================
//  CmdVelKeyboardPublisher — Unity → 실제 로봇 cmd_vel
//  붙이는 곳: RosConnector가 있는 오브젝트 (rosmentor)
//
//  1) 키보드 W/S(앞뒤) A/D(회전) Q/E(옆) → 실제 로봇
//  2) NavMeshAutoDriver 자율 주행(Z/X/C) 움직임도 → 실제 로봇
//  3) 충돌 방지: 실제 로봇 라이다(/scan)로 가는 방향에 벽·물체가 가까우면 그 방향 이동을 막음
//     (Unity 속 벽은 실제 로봇 위치와 어긋날 수 있어서, 실제 센서로 막는 게 안전)
//
//  팀원 원본(김도훈)에서 바뀐 점: 연결될 때까지 기다렸다가 등록, 자율 주행 연동, 라이다 충돌 방지
// =====================================================================================
public class CmdVelKeyboardPublisher : UnityPublisher<Twist>
{
    [Header("속도 설정")]
    public float maxLinear = 0.2f;    // 앞뒤/옆 최고속도 (m/s)
    public float maxAngular = 1.0f;   // 회전 최고속도 (rad/s)
    public float publishRate = 10f;   // 1초에 몇 번 보낼지

    [Header("자율 주행(NavMeshAutoDriver) 명령도 실제 로봇으로")]
    public bool sendAutoDrive = true;
    [Tooltip("비워 두면 같은 오브젝트에서 자동으로 찾음")]
    public NavMeshAutoDriver autoDriver;

    [Header("충돌 방지 (실제 로봇 라이다)")]
    public bool useLidarSafety = true;
    public string scanTopic = "/scan";
    [Tooltip("라이다에서 이 거리(m) 안에 물체가 있으면 그 방향으로는 안 감 (로봇 몸 길이 절반 + 여유)")]
    public float stopDistance = 0.30f;
    [Tooltip("이 거리(m)부터 천천히")]
    public float slowDistance = 0.60f;
    [Tooltip("옆(Q/E)으로 갈 때 멈추는 거리(m)")]
    public float sideStopDistance = 0.20f;
    [Tooltip("앞/뒤/왼/오 방향마다 좌우로 몇 도까지 볼지")]
    public float sectorHalfAngle = 30f;
    [Tooltip("라이다 0°가 로봇 앞이 아니면 여기서 보정 (예: 180)")]
    public float scanAngleOffset = 0f;
    [Tooltip("이보다 가까운 값은 로봇 자기 몸(팔·기둥)으로 보고 무시 (m)")]
    public float ignoreCloserThan = 0.08f;

    [Header("디버그")]
    public bool debugLog = false;
    public bool showStatus = true;

    private Twist message;
    private float timer, realBlockedTime;
    private bool wasMoving;
    private bool ready;               // 토픽 등록까지 끝났는지
    private string status = "로봇 연결 기다리는 중...";
    private string safety = "";
    private string source = "";
    private RosConnector connector;
    private string scanSubId;

    // 라이다 결과 (다른 스레드에서 들어옴)
    private readonly object scanLock = new object();
    private float front = float.PositiveInfinity, back = float.PositiveInfinity, left = float.PositiveInfinity, right = float.PositiveInfinity;
    private readonly System.Diagnostics.Stopwatch clock = System.Diagnostics.Stopwatch.StartNew();
    private double lastScanAt = -1;

    protected override void Start()
    {
        message = new Twist();
        if (string.IsNullOrEmpty(Topic)) Topic = "/controller/cmd_vel";
        if (autoDriver == null) autoDriver = GetComponent<NavMeshAutoDriver>();
        StartCoroutine(AdvertiseWhenConnected());
    }

    IEnumerator AdvertiseWhenConnected()
    {
        connector = GetComponent<RosConnector>();
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

        if (useLidarSafety)
        {
            try
            {
                scanSubId = connector.RosSocket.Subscribe<LaserScan>(scanTopic, OnScan);
                Debug.Log($"[CmdVelPub] 충돌 방지용 라이다 {scanTopic} 구독");
            }
            catch (System.Exception e) { Debug.LogWarning($"[CmdVelPub] 라이다 구독 실패 ({scanTopic}): {e.Message}"); }
        }
    }

    // 라이다 한 바퀴 → 앞/뒤/왼/오 가장 가까운 거리
    void OnScan(LaserScan s)
    {
        if (s == null || s.ranges == null) return;
        float f = float.PositiveInfinity, b = f, l = f, r = f;
        float half = sectorHalfAngle * Mathf.Deg2Rad, off = scanAngleOffset * Mathf.Deg2Rad;
        for (int i = 0; i < s.ranges.Length; i++)
        {
            float d = s.ranges[i];
            if (float.IsNaN(d) || float.IsInfinity(d) || d < Mathf.Max(s.range_min, ignoreCloserThan) || d > s.range_max) continue;
            float a = Mathf.DeltaAngle(0f, (s.angle_min + i * s.angle_increment + off) * Mathf.Rad2Deg) * Mathf.Deg2Rad;   // -π ~ π, 0 = 로봇 앞, + = 왼쪽
            if (Mathf.Abs(a) < half) f = Mathf.Min(f, d);
            if (Mathf.PI - Mathf.Abs(a) < half) b = Mathf.Min(b, d);
            if (Mathf.Abs(a - Mathf.PI / 2f) < half) l = Mathf.Min(l, d);
            if (Mathf.Abs(a + Mathf.PI / 2f) < half) r = Mathf.Min(r, d);
        }
        lock (scanLock) { front = f; back = b; left = l; right = r; lastScanAt = clock.Elapsed.TotalSeconds; }
    }

    void Update()
    {
        if (!ready) return;

        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        // 1) 키보드
        var kb = Keyboard.current;
        float v = 0f, w = 0f, side = 0f;
        if (kb != null)
        {
            v = (kb.wKey.isPressed ? 1 : 0) - (kb.sKey.isPressed ? 1 : 0);
            w = (kb.aKey.isPressed ? 1 : 0) - (kb.dKey.isPressed ? 1 : 0);    // ROS: +면 왼쪽 회전
            side = (kb.qKey.isPressed ? 1 : 0) - (kb.eKey.isPressed ? 1 : 0); // ROS: +면 왼쪽 이동
        }
        bool keyMoving = v != 0 || w != 0 || side != 0;
        bool autoActive = !keyMoving && sendAutoDrive && autoDriver != null && autoDriver.isActiveAndEnabled
                          && autoDriver.DriveRealRobot && autoDriver.IsDriving;

        float x = 0f, y = 0f, z = 0f;
        if (keyMoving) { x = v * maxLinear; y = side * maxLinear; z = w * maxAngular; source = "키보드"; }
        else if (autoActive)
        {
            Vector3 c = autoDriver.Cmd;   // 키보드를 누르면 키보드가 우선
            x = Mathf.Clamp(c.x, -maxLinear, maxLinear);
            y = Mathf.Clamp(c.y, -maxLinear, maxLinear);
            z = Mathf.Clamp(c.z, -maxAngular, maxAngular);
            source = "자율 주행";
        }
        else source = "";

        // 2) 충돌 방지: 가는 방향에 실제 장애물이 가까우면 그 방향 이동을 0으로 (회전은 허용)
        bool blocked = ApplySafety(ref x, ref y);
        if (blocked && autoActive)
        {
            realBlockedTime += 1f / publishRate;
            if (realBlockedTime > 1f) { autoDriver.StopByRealObstacle(safety); realBlockedTime = 0f; }
        }
        else realBlockedTime = 0f;

        // 3) 보내기: 움직이는 동안 계속, 멈추는 순간 정지 명령 한 번
        bool moving = keyMoving || autoActive;
        if (moving || wasMoving)
        {
            message.linear.x = x;
            message.linear.y = y;
            message.angular.z = z;
            Publish(message);
            if (debugLog)
                Debug.Log($"[CmdVelPub] {source} 보냄 x={x:F2} y={y:F2} z={z:F2} {safety}");
        }
        wasMoving = moving;
    }

    bool ApplySafety(ref float x, ref float y)
    {
        safety = "";
        if (!useLidarSafety || scanSubId == null) return false;
        float f, b, l, r; double at;
        lock (scanLock) { f = front; b = back; l = left; r = right; at = lastScanAt; }
        if (at < 0 || clock.Elapsed.TotalSeconds - at > 1.0)
        {
            safety = $"⚠ 라이다({scanTopic}) 데이터 없음 — 충돌 방지 꺼짐";
            return false;
        }
        bool blocked = false;
        if (x > 0f) x = Limit(x, f, stopDistance, "앞", ref blocked);
        else if (x < 0f) x = -Limit(-x, b, stopDistance, "뒤", ref blocked);
        if (y > 0f) y = Limit(y, l, sideStopDistance, "왼쪽", ref blocked);
        else if (y < 0f) y = -Limit(-y, r, sideStopDistance, "오른쪽", ref blocked);
        return blocked;
    }

    float Limit(float speed, float dist, float stop, string where, ref bool blocked)
    {
        if (dist <= stop) { blocked = true; safety = $"⛔ {where} 장애물 {dist:0.00}m — 그쪽으로 안 감"; return 0f; }
        if (dist < slowDistance)
        {
            float k = (dist - stop) / Mathf.Max(slowDistance - stop, 0.01f);
            return speed * Mathf.Clamp(k, 0.3f, 1f);
        }
        return speed;
    }

    // Play 끌 때 로봇 멈추기
    void OnDisable()
    {
        if (ready && message != null)
        {
            message.linear.x = 0; message.linear.y = 0; message.angular.z = 0;
            try { Publish(message); } catch { }
        }
        if (scanSubId != null && connector != null && connector.RosSocket != null)
        {
            try { connector.RosSocket.Unsubscribe(scanSubId); } catch { }
            scanSubId = null;
        }
    }

    void OnGUI()
    {
        if (!showStatus) return;
        string dist = "";
        if (useLidarSafety && scanSubId != null)
        {
            float f, b, l, r; lock (scanLock) { f = front; b = back; l = left; r = right; }
            dist = $"\n라이다  앞 {Fmt(f)}  뒤 {Fmt(b)}  왼 {Fmt(l)}  오 {Fmt(r)}";
        }
        string src = source != "" ? $"  [{source}]" : "";
        string text = "실제 로봇: " + status + src + dist + (safety != "" ? "\n" + safety : "");
        GUI.Box(new Rect(Screen.width - 470, Screen.height - 80, 460, 70), text);   // 오른쪽 아래
    }

    static string Fmt(float d) => float.IsInfinity(d) ? "-" : d.ToString("0.00") + "m";
}
