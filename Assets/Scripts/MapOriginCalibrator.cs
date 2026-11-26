using System;
using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;
using Geo = RosSharp.RosBridgeClient.MessageTypes.Geometry;

// =====================================================================================
//  MapOriginCalibrator — 버튼 한 번으로 Unity 방(MapOrigin)을 로봇 지도(416MAP)에 맞추기
//  붙이는 곳: rosmentor (RosConnector가 있는 오브젝트)
//
//  사용법
//   1) 실제 로봇을 홈 자리(Point_1)에 두고, 로봇 앞쪽을 Point_1 파란 화살표 방향으로 맞춤
//   2) RViz 2D Pose Estimate로 빨간 라이다 점이 검은 벽에 딱 겹치게 맞춤
//   3) Unity ▶ Play → 왼쪽 아래 [로봇이 홈에 있음 → Unity 맞추기] 클릭
//   4) MapOrigin이 자동으로 옮겨지고, 값이 저장됨 (다음 Play부터 자동 적용)
//   5) 씬에도 영구 저장하려면: ▶ 끄고 → 이 컴포넌트 ⋮ 메뉴 → "저장된 보정값을 씬에 적용" → Ctrl+S
//
//  계산 원리: 로봇이 말해 준 자기 위치(x, y, 방향)가 Unity의 Point_1 자리·방향과 같아지도록
//            MapOrigin의 위치(X,Z)와 회전(Y)을 거꾸로 계산 (높이 Y는 그대로 둠)
// =====================================================================================
public class MapOriginCalibrator : MonoBehaviour
{
    [Header("필수 연결")]
    [Tooltip("비워두면 같은 오브젝트 → 씬에서 자동으로 찾음")]
    public RosConnector rosConnector;
    [Tooltip("RobotPoseSync / LanderPiNav2Bridge 에 넣은 것과 같은 MapOrigin")]
    public Transform mapOrigin;
    [Tooltip("실제 로봇을 둘 홈 자리 (Waypoints/Point_1)")]
    public Transform homePoint;

    [Header("저장된 보정값 자동 적용")]
    [Tooltip("Play를 누를 때 마지막으로 맞춘 MapOrigin 값을 자동으로 다시 적용")]
    public bool applySavedOnStart = true;

    [Header("ROS (수정 불필요)")]
    public string amclPoseTopic = "/amcl_pose";
    public string tfTopic = "/tf";
    public string mapFrame = "map";
    public string odomFrame = "odom";
    public string baseFrame = "base_footprint";

    public bool showButton = true;

    const string PrefKey = "MapOriginCalib_";

    readonly object lockObj = new object();
    bool hasAmcl, hasMo, hasOb;
    float amX, amY, amYaw;
    float moX, moY, moYaw, obX, obY, obYaw;
    readonly System.Diagnostics.Stopwatch clock = System.Diagnostics.Stopwatch.StartNew();
    double amclAt = -1;

    string amclSub, tfSub;
    bool ready;
    string status = "로봇 연결 기다리는 중...";

    void Awake()
    {
        if (applySavedOnStart) ApplySaved(false);
    }

    IEnumerator Start()
    {
        if (rosConnector == null) rosConnector = GetComponent<RosConnector>();
        if (rosConnector == null) rosConnector = FindFirstObjectByType<RosConnector>();
        if (rosConnector == null || mapOrigin == null || homePoint == null)
        {
            status = "Inspector 칸 비어 있음 (Ros Connector / Map Origin / Home Point)";
            Debug.LogError("[MapCalib] " + status);
            yield break;
        }
        if (homePoint.IsChildOf(mapOrigin))
        {
            status = "Home Point가 MapOrigin 안에 들어 있어요. 밖으로 빼야 맞출 수 있어요";
            Debug.LogError("[MapCalib] " + status);
            yield break;
        }

        while (rosConnector.RosSocket == null || rosConnector.IsConnected == null || !rosConnector.IsConnected.WaitOne(0))
            yield return new WaitForSeconds(0.5f);

        amclSub = rosConnector.RosSocket.Subscribe<Geo.PoseWithCovarianceStamped>(amclPoseTopic, OnAmcl, 0, 1);
        tfSub = rosConnector.RosSocket.Subscribe<Tf2.TFMessage>(tfTopic, OnTf, 0, 10);
        ready = true;
        status = "준비됨. 로봇을 홈에 두고 빨간 점을 벽에 맞춘 뒤 버튼을 누르세요";
    }

    // ---------------- 로봇 위치 받기 ----------------
    static float Yaw(double x, double y, double z, double w)
        => (float)Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    void OnAmcl(Geo.PoseWithCovarianceStamped m)
    {
        if (m == null || m.pose == null || m.pose.pose == null) return;
        var p = m.pose.pose;
        lock (lockObj)
        {
            amX = (float)p.position.x; amY = (float)p.position.y;
            amYaw = Yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
            hasAmcl = true; amclAt = clock.Elapsed.TotalSeconds;
        }
    }

    void OnTf(Tf2.TFMessage msg)
    {
        if (msg == null || msg.transforms == null) return;
        foreach (var t in msg.transforms)
        {
            string parent = t.header.frame_id.TrimStart('/');
            string child = t.child_frame_id.TrimStart('/');
            var q = t.transform.rotation;
            float yaw = Yaw(q.x, q.y, q.z, q.w);
            float x = (float)t.transform.translation.x, y = (float)t.transform.translation.y;
            lock (lockObj)
            {
                if (parent == mapFrame && child == odomFrame) { moX = x; moY = y; moYaw = yaw; hasMo = true; }
                else if (parent == odomFrame && child == baseFrame) { obX = x; obY = y; obYaw = yaw; hasOb = true; }
            }
        }
    }

    // TF(map→odom→base) 우선: 로봇이 지금 실제로 쓰는 위치. 없으면 /amcl_pose
    bool TryGetRobotPose(out float rx, out float ry, out float ryaw, out string src)
    {
        lock (lockObj)
        {
            if (hasMo && hasOb)
            {
                float c = Mathf.Cos(moYaw), s = Mathf.Sin(moYaw);
                rx = moX + c * obX - s * obY;
                ry = moY + s * obX + c * obY;
                ryaw = moYaw + obYaw;
                src = "tf";
                return true;
            }
            if (hasAmcl)
            {
                rx = amX; ry = amY; ryaw = amYaw; src = "amcl_pose";
                return true;
            }
        }
        rx = ry = ryaw = 0; src = "";
        return false;
    }

    // ---------------- 맞추기 ----------------
    public void Calibrate()
    {
        if (!ready) { Debug.LogWarning("[MapCalib] " + status); return; }
        if (!TryGetRobotPose(out float rx, out float ry, out float ryaw, out string src))
        {
            status = "로봇 위치를 아직 못 받았어요. 내비게이션이 켜져 있는지, 2D Pose Estimate를 했는지 확인";
            Debug.LogWarning("[MapCalib] " + status);
            return;
        }

        Vector3 s = mapOrigin.lossyScale;
        if (Mathf.Abs(s.x - 1f) > 0.01f || Mathf.Abs(s.z - 1f) > 0.01f)
            Debug.LogWarning($"[MapCalib] MapOrigin 크기(Scale)가 1이 아니에요 ({s}). 결과가 어긋날 수 있어요. Scale을 1,1,1로 두는 걸 권장.");

        Vector3 before = mapOrigin.position;
        float beforeYaw = mapOrigin.eulerAngles.y;

        // 1) 회전: Point_1 방향 = MapOrigin 방향 + (로봇 방향을 Unity 각도로 바꾼 것)
        //    RobotPoseSync: 로봇 회전 = MapOrigin 회전 * (0, -ryaw°, 0)
        float homeYaw = homePoint.eulerAngles.y;
        float originYaw = homeYaw + ryaw * Mathf.Rad2Deg;
        Quaternion rot = Quaternion.Euler(0f, originYaw, 0f);

        // 2) 위치: Point_1 = MapOrigin + 회전 * (-ry, 0, rx)   (높이는 그대로)
        Vector3 local = new Vector3(-ry * s.x, 0f, rx * s.z);
        Vector3 offset = rot * local;
        Vector3 hp = homePoint.position;
        Vector3 pos = new Vector3(hp.x - offset.x, before.y, hp.z - offset.z);

        mapOrigin.SetPositionAndRotation(pos, rot);
        Save(pos, originYaw);

        float moved = Vector3.Distance(new Vector3(before.x, 0, before.z), new Vector3(pos.x, 0, pos.z));
        status = $"맞추기 완료 ✓ (로봇 x={rx:F2}, y={ry:F2}, 방향 {ryaw * Mathf.Rad2Deg:F0}°)  MapOrigin {moved:F2}m 이동, 회전 {Mathf.DeltaAngle(beforeYaw, originYaw):F0}°";
        Debug.Log($"[MapCalib] {status}\n  [{src}] 새 MapOrigin Position=({pos.x:F3}, {pos.y:F3}, {pos.z:F3})  Rotation Y={originYaw:F2}\n" +
                  "  씬에 영구 저장: ▶ 끄고 → MapOriginCalibrator ⋮ → '저장된 보정값을 씬에 적용' → Ctrl+S");
    }

    // ---------------- 저장 / 불러오기 ----------------
    void Save(Vector3 pos, float yaw)
    {
        PlayerPrefs.SetFloat(PrefKey + "x", pos.x);
        PlayerPrefs.SetFloat(PrefKey + "y", pos.y);
        PlayerPrefs.SetFloat(PrefKey + "z", pos.z);
        PlayerPrefs.SetFloat(PrefKey + "yaw", yaw);
        PlayerPrefs.SetInt(PrefKey + "has", 1);
        PlayerPrefs.Save();
    }

    bool ApplySaved(bool log)
    {
        if (mapOrigin == null || PlayerPrefs.GetInt(PrefKey + "has", 0) != 1) return false;
        var pos = new Vector3(PlayerPrefs.GetFloat(PrefKey + "x"), PlayerPrefs.GetFloat(PrefKey + "y"), PlayerPrefs.GetFloat(PrefKey + "z"));
        float yaw = PlayerPrefs.GetFloat(PrefKey + "yaw");
#if UNITY_EDITOR
        if (!Application.isPlaying) UnityEditor.Undo.RecordObject(mapOrigin, "MapOrigin 보정값 적용");
#endif
        mapOrigin.SetPositionAndRotation(pos, Quaternion.Euler(0f, yaw, 0f));
#if UNITY_EDITOR
        if (!Application.isPlaying) UnityEditor.EditorUtility.SetDirty(mapOrigin);
#endif
        if (log) Debug.Log($"[MapCalib] 저장된 보정값 적용: Position=({pos.x:F3}, {pos.y:F3}, {pos.z:F3}) Rotation Y={yaw:F2}");
        return true;
    }

    [ContextMenu("저장된 보정값을 씬에 적용 (▶ 끈 상태에서)")]
    void ApplySavedToScene()
    {
        if (!ApplySaved(true)) Debug.LogWarning("[MapCalib] 저장된 보정값이 없어요. Play 중에 [Unity 맞추기]를 먼저 누르세요.");
        else Debug.Log("[MapCalib] 이제 Ctrl+S로 씬을 저장하세요.");
    }

    [ContextMenu("저장된 보정값 지우기")]
    void ClearSaved()
    {
        PlayerPrefs.DeleteKey(PrefKey + "has");
        PlayerPrefs.Save();
        Debug.Log("[MapCalib] 저장된 보정값을 지웠어요. 다음 Play부터는 씬에 있는 MapOrigin 값을 그대로 씁니다.");
    }

    void OnDisable()
    {
        if (rosConnector != null && rosConnector.RosSocket != null)
        {
            try { if (amclSub != null) rosConnector.RosSocket.Unsubscribe(amclSub); } catch { }
            try { if (tfSub != null) rosConnector.RosSocket.Unsubscribe(tfSub); } catch { }
        }
        amclSub = tfSub = null;
    }

    // ---------------- 화면 버튼 (왼쪽 아래) ----------------
    void OnGUI()
    {
        if (!showButton) return;
        Rect box = new Rect(10, Screen.height - 120, 470, 110);
        GUI.Box(box, "지도 맞추기 (MapOrigin)");

        string pose = "로봇 위치: 아직 못 받음";
        if (TryGetRobotPose(out float rx, out float ry, out float ryaw, out string src))
            pose = $"로봇 위치: x={rx:F2}, y={ry:F2}, 방향 {ryaw * Mathf.Rad2Deg:F0}°  [{src}]";
        GUI.Label(new Rect(20, Screen.height - 98, 450, 20), pose);

        GUI.enabled = ready;
        if (GUI.Button(new Rect(20, Screen.height - 76, 450, 30), "로봇이 홈(Point_1)에 있음 → Unity 맞추기"))
            Calibrate();
        GUI.enabled = true;

        GUI.Label(new Rect(20, Screen.height - 42, 450, 34), status);
    }
}
