// LanderPiNav2Bridge.cs  (ROS# / rosbridge 용)
//
// 하는 일
//   1) 실제 로봇이 라이다 지도(map) 위 어디에 있는지(/tf) 받아서 Unity 로봇을 그 자리로 옮김
//   2) Z / X / C 키(또는 화면 버튼)로 에어컨 앞 · 스위치 앞 · 홈 을 실제 로봇의 자율주행(Nav2)에 보냄 (/goal_pose)
//   3) V 키 = 그 자리에 멈추기,  I 키 = "로봇이 지금 홈(Point_1)에 있어" 알려 주기 (/initialpose)
//
// 준비
//   - 씬에 RosConnector(ws://192.168.149.1:9090) 가 이미 있어야 해요 (rosmentor 에 붙어 있는 것)
//   - MapOrigin = 로봇이 지도 만들기를 시작한 자리 (SavedMap 이 이 안에 있어야 해요)
//   - 로봇 쪽: navigation.launch.py map:=416MAP  +  rosbridge 가 켜져 있어야 해요
//
// 좌표 바꾸기:  ROS(x 앞, y 왼쪽)  →  Unity(z 앞, -x 왼쪽)

using System;
using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Geo = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;

public class LanderPiNav2Bridge : MonoBehaviour
{
    [Header("연결 (비워 두면 씬에서 자동으로 찾음)")]
    public RosConnector rosConnector;

    [Header("지도 원점 = 로봇이 지도 만들기를 시작한 자리")]
    public Transform mapOrigin;

    [Header("실제 위치를 따라 움직일 Unity 로봇 (처음엔 캡슐로 시험)")]
    public Transform followRobot;
    public float robotHeight = 0f;
    [Tooltip("모델 앞쪽이 Unity 파란 화살표(Z) 방향이 아니면 여기서 돌려 맞추기 (도)")]
    public float modelYawOffset = 0f;

    [Header("목적지 (Waypoints 안의 점들을 끌어다 넣기)")]
    public Transform airconPoint;   // Point_3 (에어컨)
    public Transform switchPoint;   // Point_2 (스위치)
    public Transform homePoint;     // Point_1 (홈)

    [Header("키")]
    public KeyCode airconKey = KeyCode.Z;
    public KeyCode switchKey = KeyCode.X;
    public KeyCode homeKey = KeyCode.C;
    public KeyCode stopKey = KeyCode.V;
    public KeyCode initialPoseKey = KeyCode.I;

    [Header("기타")]
    public float arriveDistance = 0.3f;
    public string goalTopic = "/goal_pose";
    public string initialPoseTopic = "/initialpose";
    public string tfTopic = "/tf";
    public string mapFrame = "map";
    public string odomFrame = "odom";
    public string baseFrame = "base_footprint";
    public bool showButtons = true;

    // 내부 상태
    private string goalPubId, initPubId;
    private bool ready = false;
    private readonly object lk = new object();
    private bool hasMO = false, hasOB = false;
    private float moX, moY, moYaw;   // map -> odom
    private float obX, obY, obYaw;   // odom -> base_footprint
    private bool poseAnnounced = false;
    private bool keysWork = true;
    private Transform currentGoal = null;
    private bool arrived = false;
    private string status = "로봇과 연결하는 중...";

    // ---------------- 연결 ----------------
    IEnumerator Start()
    {
        if (rosConnector == null) rosConnector = FindFirstObjectByType<RosConnector>();
        if (rosConnector == null) { status = "RosConnector 를 못 찾았어요"; Debug.LogError("[Nav2] " + status); yield break; }
        if (mapOrigin == null) { status = "Map Origin 칸이 비어 있어요"; Debug.LogError("[Nav2] " + status); yield break; }

        // RosConnector 가 연결될 때까지 기다리기
        while (rosConnector.RosSocket == null || rosConnector.IsConnected == null || !rosConnector.IsConnected.WaitOne(0))
            yield return new WaitForSeconds(0.5f);

        goalPubId = rosConnector.RosSocket.Advertise<Geo.PoseStamped>(goalTopic);
        initPubId = rosConnector.RosSocket.Advertise<Geo.PoseWithCovarianceStamped>(initialPoseTopic);
        rosConnector.RosSocket.Subscribe<Tf2.TFMessage>(tfTopic, OnTf, 0, 10);

        ready = true;
        status = "연결됨. 로봇이 홈에 있으면 [I] 또는 [출발 자리 알려주기]";
        Debug.Log("[Nav2] " + status);
    }

    // rosbridge 쓰레드에서 불림 → 숫자만 저장하고, 실제 움직임은 Update 에서
    private void OnTf(Tf2.TFMessage msg)
    {
        if (msg == null || msg.transforms == null) return;
        foreach (var t in msg.transforms)
        {
            string parent = t.header.frame_id.TrimStart('/');
            string child = t.child_frame_id.TrimStart('/');
            var q = t.transform.rotation;
            float yaw = (float)Math.Atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            float x = (float)t.transform.translation.x;
            float y = (float)t.transform.translation.y;

            lock (lk)
            {
                if (parent == mapFrame && child == odomFrame) { moX = x; moY = y; moYaw = yaw; hasMO = true; }
                else if (parent == odomFrame && child == baseFrame) { obX = x; obY = y; obYaw = yaw; hasOB = true; }
            }
        }
    }

    // ---------------- 매 프레임 ----------------
    void Update()
    {
        ReadKeys();
        MoveFollowRobot();
        CheckArrival();
    }

    void ReadKeys()
    {
        if (!keysWork) return;
        try
        {
            if (Input.GetKeyDown(airconKey)) GoTo(airconPoint, "에어컨");
            if (Input.GetKeyDown(switchKey)) GoTo(switchPoint, "스위치");
            if (Input.GetKeyDown(homeKey)) GoTo(homePoint, "홈");
            if (Input.GetKeyDown(stopKey)) StopHere();
            if (Input.GetKeyDown(initialPoseKey)) SendInitialPoseAtHome();
        }
        catch (InvalidOperationException)
        {
            keysWork = false;   // 새 Input System 만 켜진 프로젝트 → 화면 버튼을 쓰세요
            Debug.LogWarning("[Nav2] 이 프로젝트에서는 키 입력을 못 읽어요. 화면 왼쪽 위 버튼을 쓰세요.");
        }
    }

    bool TryGetRobotPose(out float rx, out float ry, out float ryaw)
    {
        lock (lk)
        {
            rx = ry = ryaw = 0f;
            if (!hasMO || !hasOB) return false;
            float c = Mathf.Cos(moYaw), s = Mathf.Sin(moYaw);
            rx = moX + c * obX - s * obY;
            ry = moY + s * obX + c * obY;
            ryaw = moYaw + obYaw;
            return true;
        }
    }

    void MoveFollowRobot()
    {
        if (followRobot == null || mapOrigin == null) return;
        if (!TryGetRobotPose(out float rx, out float ry, out float ryaw)) return;

        Vector3 local = new Vector3(-ry, robotHeight, rx);
        followRobot.position = mapOrigin.TransformPoint(local);
        followRobot.rotation = mapOrigin.rotation * Quaternion.Euler(0f, -ryaw * Mathf.Rad2Deg + modelYawOffset, 0f);

        if (!poseAnnounced)
        {
            poseAnnounced = true;
            Debug.Log($"[Nav2] 실제 로봇 위치 받기 시작! 지도 위 x={rx:F2} m, y={ry:F2} m");
        }
    }

    // ---------------- 좌표 바꾸기 ----------------
    void UnityToRos(Transform t, out double x, out double y, out double yaw)
    {
        Vector3 p = mapOrigin.InverseTransformPoint(t.position);
        float localYawDeg = (Quaternion.Inverse(mapOrigin.rotation) * t.rotation).eulerAngles.y;
        x = p.z;
        y = -p.x;
        yaw = -localYawDeg * Mathf.Deg2Rad;
    }

    static Geo.Pose MakePose(double x, double y, double yaw)
    {
        var pose = new Geo.Pose();
        pose.position = new Geo.Point { x = x, y = y, z = 0.0 };
        pose.orientation = new Geo.Quaternion { x = 0.0, y = 0.0, z = Math.Sin(yaw / 2.0), w = Math.Cos(yaw / 2.0) };
        return pose;
    }

    static Std.Header MapHeader(string frame)
    {
        var h = new Std.Header();
        h.frame_id = frame;
        return h;
    }

    // ---------------- 보내기 ----------------
    void GoTo(Transform target, string label)
    {
        if (!ready) { status = "아직 로봇과 연결 안 됨"; return; }
        if (target == null) { status = label + " 점이 비어 있어요 (Inspector 확인)"; return; }

        UnityToRos(target, out double x, out double y, out double yaw);
        var msg = new Geo.PoseStamped();
        msg.header = MapHeader(mapFrame);
        msg.pose = MakePose(x, y, yaw);
        rosConnector.RosSocket.Publish(goalPubId, msg);

        currentGoal = target;
        arrived = false;
        status = $"{label} 으로 가는 중 (x={x:F2}, y={y:F2})";
        Debug.Log("[Nav2] " + status);
    }

    void StopHere()
    {
        if (!ready) return;
        currentGoal = null;
        if (!TryGetRobotPose(out float rx, out float ry, out float ryaw))
        {
            status = "로봇 위치를 아직 몰라서 멈추기를 못 보냄. 위험하면 로봇 쪽 창에서 Ctrl+C";
            return;
        }
        var msg = new Geo.PoseStamped();
        msg.header = MapHeader(mapFrame);
        msg.pose = MakePose(rx, ry, ryaw);
        rosConnector.RosSocket.Publish(goalPubId, msg);
        status = "멈추기: 지금 자리를 목적지로 보냄";
        Debug.Log("[Nav2] " + status);
    }

    void SendInitialPoseAtHome()
    {
        if (!ready) { status = "아직 로봇과 연결 안 됨"; return; }
        if (homePoint == null) { status = "Home Point 가 비어 있어요"; return; }

        UnityToRos(homePoint, out double x, out double y, out double yaw);
        var msg = new Geo.PoseWithCovarianceStamped();
        msg.header = MapHeader(mapFrame);
        msg.pose = new Geo.PoseWithCovariance();
        msg.pose.pose = MakePose(x, y, yaw);
        double[] cov = new double[36];
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685;
        msg.pose.covariance = cov;
        rosConnector.RosSocket.Publish(initPubId, msg);

        status = $"출발 자리(홈) 보냄 x={x:F2}, y={y:F2}. 몇 초 뒤 Unity 로봇이 홈에 나타나면 OK";
        Debug.Log("[Nav2] " + status);
    }

    // ---------------- 도착 확인 ----------------
    void CheckArrival()
    {
        if (currentGoal == null || arrived) return;
        if (!TryGetRobotPose(out float rx, out float ry, out float ryaw)) return;

        Vector3 g = mapOrigin.InverseTransformPoint(currentGoal.position);
        float gx = g.z, gy = -g.x;
        float d = Mathf.Sqrt((rx - gx) * (rx - gx) + (ry - gy) * (ry - gy));
        if (d < arriveDistance)
        {
            arrived = true;
            status = "도착! " + currentGoal.name;
            Debug.Log("[Nav2] " + status);
        }
    }

    // ---------------- 화면 버튼 ----------------
    void OnGUI()
    {
        if (!showButtons) return;
        GUILayout.BeginArea(new Rect(10, 10, 320, 300), GUI.skin.box);
        GUILayout.Label("실제 로봇 자율주행 (Nav2)");
        if (GUILayout.Button($"출발 자리 알려주기 (홈)  [{initialPoseKey}]", GUILayout.Height(28))) SendInitialPoseAtHome();
        if (GUILayout.Button($"에어컨 앞으로  [{airconKey}]", GUILayout.Height(28))) GoTo(airconPoint, "에어컨");
        if (GUILayout.Button($"스위치 앞으로  [{switchKey}]", GUILayout.Height(28))) GoTo(switchPoint, "스위치");
        if (GUILayout.Button($"홈으로  [{homeKey}]", GUILayout.Height(28))) GoTo(homePoint, "홈");
        if (GUILayout.Button($"멈추기  [{stopKey}]", GUILayout.Height(28))) StopHere();
        GUILayout.Label(status);
        GUILayout.EndArea();
    }
}
