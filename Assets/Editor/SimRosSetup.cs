#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// 메뉴 한 번 클릭으로 "Unity 시뮬레이션 ⇄ ROS" 연동 설정을 맞춰주는 에디터 도구.
/// 넣는 곳: Assets/Editor/SimRosSetup.cs   (Editor 폴더 안이어야 함)
///
/// Tools → ROS 연동 → LanderPi(rosmentor) 시뮬레이션 모드   ← LanderPi로 SLAM/Nav2
/// Tools → ROS 연동 → 터틀봇 시뮬레이션 모드                ← 기존 터틀봇으로
/// </summary>
public static class SimRosSetup
{
    const string SimUrl = "ws://localhost:9090";
    const int IgnoreRaycastLayer = 2;

    // =====================================================================
    // LanderPi (rosmentor)
    // =====================================================================
    [MenuItem("Tools/ROS 연동/LanderPi(rosmentor) 시뮬레이션 모드")]
    public static void SetupLanderPi()
    {
        var log = new List<string>();
        var all = AllObjectsInScene();

        GameObject robot = all.Find(g => g.name == "rosmentor");
        if (robot == null) { Fail("씬에서 'rosmentor'를 못 찾았어요."); return; }
        GameObject tb = all.Find(g => g.name == "turtlebot3_burger");

        // 필요한 스크립트 종류 찾기 (팀 프로젝트에 있는 것들)
        Type tConn = FindType("RosConnector"), tDrive = FindType("DiffDriveController"),
             tLidar = FindType("LidarSensor"), tClock = FindType("ClockPublisher"),
             tTf = FindType("LanderPiSimTf"), tTeleop = FindType("LanderPiTeleop");
        var missing = new List<string>();
        if (tConn == null) missing.Add("RosConnector(ros-sharp)");
        if (tDrive == null) missing.Add("DiffDriveController.cs");
        if (tLidar == null) missing.Add("LidarSensor.cs");
        if (tClock == null) missing.Add("ClockPublisher.cs");
        if (tTf == null) missing.Add("LanderPiSimTf.cs");
        if (tTeleop == null) missing.Add("LanderPiTeleop.cs");
        if (missing.Count > 0) { Fail("이 스크립트가 프로젝트에 없어요:\n• " + string.Join("\n• ", missing) + "\n\nAssets/Scripts 에 넣고 다시 실행하세요."); return; }

        // 1) 로봇 켜기 / 끄기
        SetActive(robot, true, log);
        if (tb != null) SetActive(tb, false, log);
        foreach (var g in all.FindAll(x => x.name == "RealRobotLink")) SetActive(g, false, log);

        // 2) rosmentor 루트에 RosConnector / DiffDriveController / ClockPublisher / LanderPiSimTf
        var conn = CopyOrAdd(tConn, tb, robot, log);
        SetString(conn, "RosBridgeServerUrl", SimUrl, log);
        var drive = CopyOrAdd(tDrive, tb, robot, log);
        SetMask(drive, "obstacleLayers", ~(1 << IgnoreRaycastLayer), log);
        CopyOrAdd(tClock, tb, robot, log);
        CopyOrAdd(tTf, null, robot, log);

        // 3) LiDAR: lidar_frame 위치에
        Transform lidarT = FindLink(robot.transform, "lidar_frame");
        if (lidarT == null) { lidarT = robot.transform; log.Add("⚠ lidar_frame이 없어 로봇 중심에 LiDAR를 붙임"); }
        var lidar = CopyOrAdd(tLidar, tb, lidarT.gameObject, log);
        SetString(lidar, "FrameId", lidarT == robot.transform ? "base_footprint" : "lidar_frame", log);
        SetMask(lidar, "Mask", ~(1 << IgnoreRaycastLayer), log);

        // 4) 로봇 자신의 충돌체는 LiDAR·충돌검사에서 제외 (자기 몸을 벽으로 착각하지 않게)
        int changed = 0;
        foreach (var c in robot.GetComponentsInChildren<Collider>(true))
            if (c.gameObject.layer != IgnoreRaycastLayer) { Undo.RecordObject(c.gameObject, "Sim setup"); c.gameObject.layer = IgnoreRaycastLayer; changed++; }
        if (changed > 0) log.Add($"로봇 충돌체 {changed}개 → Ignore Raycast 레이어");

        // 5) LanderPiTeleop: 팔·카메라만 담당, 차체는 ROS(/cmd_vel)가 움직이게
        Component teleop = robot.GetComponent(tTeleop);
        if (teleop == null) { teleop = Undo.AddComponent(robot, tTeleop); log.Add("LanderPiTeleop 추가"); }
        SetBool(teleop, "driveBase", false, log);
        Enable(teleop, log);

        FinishScene(log, "LanderPi(rosmentor)");
    }

    // =====================================================================
    // 터틀봇
    // =====================================================================
    [MenuItem("Tools/ROS 연동/터틀봇 시뮬레이션 모드")]
    public static void SetupTurtlebot()
    {
        var log = new List<string>();
        var all = AllObjectsInScene();
        GameObject tb = all.Find(g => g.name == "turtlebot3_burger");
        if (tb == null) { Fail("씬에서 'turtlebot3_burger'를 못 찾았어요."); return; }

        SetActive(tb, true, log);
        foreach (var n in new[] { "rosmentor", "RealRobotLink" })
            foreach (var g in all.FindAll(x => x.name == n)) SetActive(g, false, log);

        string[] on = { "RosConnector", "DiffDriveController", "LidarSensor", "TfPublisher", "ClockPublisher" };
        string[] off = { "WaypointFollower", "WaypointNavigator", "KeyboardDriveTest" };
        foreach (var mb in tb.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (mb == null) continue;
            string t = mb.GetType().Name;
            if (on.Contains(t)) Enable(mb, log);
            if (off.Contains(t) && mb.enabled) { Undo.RecordObject(mb, "Sim setup"); mb.enabled = false; log.Add(t + " 끔"); }
            if (t == "RosConnector") SetString(mb, "RosBridgeServerUrl", SimUrl, log);
        }
        FinishScene(log, "터틀봇");
    }

    // =====================================================================
    // 공통 도우미
    // =====================================================================
    static void FinishScene(List<string> log, string mode)
    {
        bool inputChanged = SetInputHandlingBoth();
        if (inputChanged) log.Add("Active Input Handling → Both (Unity 재시작 필요)");

        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        EditorSceneManager.SaveOpenScenes();

        string msg = log.Count == 0 ? "이미 전부 맞게 설정돼 있어요 ✓" : "바꾼 것:\n• " + string.Join("\n• ", log);
        msg += "\n\n다음 순서:\n1) Ubuntu 22.04 창에서 rosbridge 실행\n2) Unity Play ▶";
        if (inputChanged) msg += "\n\n⚠ 입력 설정이 바뀌어서 Unity를 한 번 껐다 켜야 해요.";
        Debug.Log($"[SimRosSetup] {mode} 모드: " + msg.Replace("\n", " "));
        EditorUtility.DisplayDialog($"{mode} 시뮬레이션 설정 완료", msg, "확인");
    }

    static Type FindType(string name) =>
        TypeCache.GetTypesDerivedFrom<MonoBehaviour>().FirstOrDefault(t => t.Name == name);

    // 터틀봇에 같은 컴포넌트가 있으면 설정값을 그대로 복사해서 붙이고, 없으면 기본값으로 붙임
    static Component CopyOrAdd(Type type, GameObject sourceRobot, GameObject target, List<string> log)
    {
        Component dst = target.GetComponent(type);
        bool added = false;
        if (dst == null) { dst = Undo.AddComponent(target, type); added = true; }
        Component src = sourceRobot != null ? sourceRobot.GetComponentInChildren(type, true) : null;
        if (added && src != null) { EditorUtility.CopySerialized(src, dst); log.Add($"{type.Name} → {target.name} (터틀봇 설정 복사)"); }
        else if (added) log.Add($"{type.Name} → {target.name} 추가");
        Enable(dst, log);
        return dst;
    }

    static void Enable(Component c, List<string> log)
    {
        if (c is Behaviour b && !b.enabled) { Undo.RecordObject(b, "Sim setup"); b.enabled = true; log.Add(c.GetType().Name + " 켬"); }
    }

    static void SetActive(GameObject g, bool on, List<string> log)
    {
        if (g.activeSelf == on) return;
        Undo.RecordObject(g, "Sim setup"); g.SetActive(on); log.Add(g.name + (on ? " 켬" : " 끔"));
    }

    static void SetString(Component c, string prop, string value, List<string> log)
    {
        var so = new SerializedObject(c); var p = so.FindProperty(prop);
        if (p == null || p.stringValue == value) return;
        log.Add($"{c.GetType().Name}.{prop}: {p.stringValue} → {value}");
        p.stringValue = value; so.ApplyModifiedProperties();
    }

    static void SetBool(Component c, string prop, bool value, List<string> log)
    {
        var so = new SerializedObject(c); var p = so.FindProperty(prop);
        if (p == null || p.boolValue == value) return;
        p.boolValue = value; so.ApplyModifiedProperties();
        log.Add($"{c.GetType().Name}.{prop} = {value}");
    }

    static void SetMask(Component c, string prop, int mask, List<string> log)
    {
        var so = new SerializedObject(c); var p = so.FindProperty(prop);
        if (p == null || p.intValue == mask) return;
        p.intValue = mask; so.ApplyModifiedProperties();
        log.Add($"{c.GetType().Name}.{prop}: 로봇 자기 몸 제외");
    }

    static Transform FindLink(Transform root, string n)
    {
        Transform fallback = null;
        foreach (var t in root.GetComponentsInChildren<Transform>(true))
        {
            if (t.name != n) continue;
            if (t.Find("Visuals") != null) return t;
            if (fallback == null) fallback = t;
        }
        return fallback;
    }

    static bool SetInputHandlingBoth()
    {
        var assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/ProjectSettings.asset");
        if (assets == null || assets.Length == 0) return false;
        var so = new SerializedObject(assets[0]);
        var p = so.FindProperty("activeInputHandler");   // 0 = 옛날 방식, 1 = 새 Input System, 2 = Both
        if (p == null || p.intValue == 2) return false;
        p.intValue = 2; so.ApplyModifiedProperties(); AssetDatabase.SaveAssets();
        return true;
    }

    static List<GameObject> AllObjectsInScene()
    {
        var list = new List<GameObject>();
        foreach (var root in SceneManager.GetActiveScene().GetRootGameObjects())
            foreach (var t in root.GetComponentsInChildren<Transform>(true)) list.Add(t.gameObject);
        return list;
    }

    static void Fail(string m) { Debug.LogError("[SimRosSetup] " + m); EditorUtility.DisplayDialog("설정 실패", m, "확인"); }
}
#endif
