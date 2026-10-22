using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// URDF로 가져온 LanderPi(차체 + 6DOF 팔)를 키보드로 조종하는 스크립트. (새 Input System 기준)
///
/// 붙이는 곳: URDF로 가져온 로봇의 맨 위 오브젝트 (예: rosmentor)
/// ROS# 가져오기(Rigidbody + Joint 구조)와 Unity URDF Importer(Articulation Body 구조) 둘 다 지원합니다.
///
/// [차체]   W/S 전진·후진   A/D 좌우 회전   Q/E 옆으로 이동(메카넘 전용)
/// [팔]     1~5 관절 선택   ↑/↓ 선택한 관절 돌리기
/// [그리퍼] Space 열기/닫기
/// [자세]   0 기본 자세로 복귀   P 앞쪽 위로 뻗기(스위치 누르기 연습용)
/// </summary>
public class LanderPiTeleop : MonoBehaviour
{
    [Header("차체 이동")]
    public float maxLinear = 0.3f;        // m/s
    public float maxAngular = 90f;        // deg/s
    public bool mecanum = true;           // 메카넘 차체면 Q/E로 옆 이동 가능
    [Tooltip("W를 눌렀는데 뒤로 가면 체크")]
    public bool invertForward = false;

    [Header("벽/가구 충돌 검사")]
    public float collisionRadius = 0.12f;
    public LayerMask obstacleLayers = ~0;

    [Header("팔")]
    public float jointSpeed = 60f;        // deg/s
    public float jointLimit = 120f;       // URDF 한계 ±2.09rad ≈ ±120°
    [Tooltip("켜두면 LanderPi URDF에서 계산한 회전축을 사용 (ROS#의 Joint 축이 틀어져 있을 때 필요)")]
    public bool useUrdfAxes = true;
    [Tooltip("joint1~5 회전축 (링크 기준, Unity 좌표). URDF axis를 (-y, z, x)로 변환한 값")]
    public Vector3[] armAxes =
    {
        new Vector3(0.0087f, 1.0000f, 0.0000f),   // joint1: 수직축 → 제자리 회전
        new Vector3(0.9999f, 0.0089f, 0.0086f),   // joint2: 어깨 (좌우축)
        new Vector3(0.9999f, 0.0089f, 0.0086f),   // joint3: 팔꿈치
        new Vector3(0.9999f, 0.0087f, 0.0087f),   // joint4: 손목 꺾기
        new Vector3(0.0000f, -1.0000f, -0.0002f), // joint5: 손목 회전
    };
    public Vector3 jawAxis = new Vector3(1f, 0f, 0f);
    [Tooltip("켜두면 관절 기준점이 아니라 '눈에 보이는 부품 모양'의 원점을 회전 중심으로 사용 (ROS#에서 기준점과 모양이 어긋나 팔이 원을 그리며 날아갈 때)")]
    public bool pivotFromVisual = true;

    [Header("카메라 (로봇 따라가기)")]
    [Tooltip("켜두면 화면용 카메라 하나를 골라 로봇 뒤에서 따라가게 하고, 나머지 화면용 카메라는 끕니다. (미니맵처럼 Output Texture가 있는 카메라는 건드리지 않음)")]
    public bool followCamera = true;
    public float camDistance = 0.9f;
    public float camHeight = 0.6f;
    public float camLookHeight = 0.15f;
    public float camSmooth = 8f;

    [Header("화면 도움말")]
    public bool showHelp = true;

    Camera cam;

    static readonly string[] ArmLinkNames = { "link1", "link2", "link3", "link4", "link5" };
    const string LeftJawName = "left_jaw_link";
    const string RightJawName = "right_jaw_link";
    const float JawTravel = 0.019f;       // m

    // ---- 관절 하나를 표현 (두 가져오기 방식 공통) ----
    class Joint
    {
        public Transform t;
        public ArticulationBody ab;       // Articulation 방식일 때
        public Vector3 axis;              // Rigidbody 방식일 때 (링크 로컬 축)
        public Quaternion initRot;
        public Vector3 initPos;
        public Vector3 pivot;             // 회전 중심 (링크 로컬). 기본은 링크 원점
        public bool prismatic;
        public float value;               // deg 또는 m
        public float min, max;
    }

    bool articulationMode;
    ArticulationBody rootAB;
    Transform movingRoot;                 // 실제로 움직일 Transform
    readonly List<Joint> arm = new List<Joint>();
    Joint leftJaw, rightJaw;
    int selected = 0;
    bool gripperOpen = true;
    string status = "";

    void Start()
    {
        foreach (var ab in GetComponentsInChildren<ArticulationBody>(true))
            if (ab.isRoot) { rootAB = ab; break; }
        articulationMode = rootAB != null;

        if (articulationMode)
        {
            rootAB.immovable = true;
            movingRoot = rootAB.transform;
        }
        else
        {
            // ROS# 방식: 물리 관절(Joint)과 Rigidbody를 제거해서
            // 로봇 전체가 하나의 Transform 계층으로만 움직이게 함 (팔이 차체에서 떨어지는 문제 방지)
            if (!useUrdfAxes) CacheHingeAxes();
            int removed = 0;
            // ROS#의 UrdfJoint*/UrdfInertial 스크립트가 Joint·Rigidbody를 필수로 요구하므로 먼저 제거
            foreach (var mb in GetComponentsInChildren<MonoBehaviour>(true))
            {
                string n = mb.GetType().Name;
                if (n.StartsWith("UrdfJoint") || n == "UrdfInertial") { DestroyImmediate(mb); removed++; }
            }
            foreach (var jnt in GetComponentsInChildren<UnityEngine.Joint>(true)) { DestroyImmediate(jnt); removed++; }
            // Rigidbody는 지우지 않고 키네마틱으로 고정 (다른 스크립트가 다시 켜도 FixedUpdate에서 매번 되돌림)
            bodies = GetComponentsInChildren<Rigidbody>(true);
            FreezeBodies();
            Debug.Log("[LanderPiTeleop] 물리 관절 " + removed + "개 제거, Rigidbody " + bodies.Length + "개 고정 (Play 중에만 적용)");
            movingRoot = transform;
        }

        var all = GetComponentsInChildren<Transform>(true);
        for (int i = 0; i < ArmLinkNames.Length; i++)
        {
            string name = ArmLinkNames[i];
            var j = MakeJoint(FindLink(all, name), false,
                              armAxes != null && i < armAxes.Length ? armAxes[i] : Vector3.up);
            if (j != null) arm.Add(j);
            else Debug.LogWarning("[LanderPiTeleop] 관절을 못 찾음: " + name);
        }
        leftJaw = MakeJoint(FindLink(all, LeftJawName), true, jawAxis);
        rightJaw = MakeJoint(FindLink(all, RightJawName), true, jawAxis);

        // 가져오기 도구가 붙인 예전 키보드 Controller가 있으면 끄기 (새 Input System과 충돌)
        foreach (var mb in GetComponentsInChildren<MonoBehaviour>(true))
            if (mb != this && mb.GetType().Name == "Controller") mb.enabled = false;

        if (leftJaw != null) { leftJaw.min = 0; leftJaw.max = JawTravel; }
        if (rightJaw != null) { rightJaw.min = -JawTravel; rightJaw.max = 0; }
        SetGripper(true);

        if (followCamera) SetupCamera();

        status = (articulationMode ? "Articulation" : "ROS#(Rigidbody)") + " 모드 · 관절 " + arm.Count + "/5";
        Debug.Log("[LanderPiTeleop] " + status);
    }

    // 같은 이름의 메시 오브젝트(Visuals/unnamed/link1 등)가 있으므로,
    // 바로 아래에 "Visuals" 자식이 있는 진짜 링크 오브젝트를 우선 선택
    static Transform FindLink(Transform[] all, string linkName)
    {
        Transform fallback = null;
        foreach (var x in all)
        {
            if (x.name != linkName) continue;
            if (x.Find("Visuals") != null || x.GetComponent<ArticulationBody>() != null) return x;
            if (fallback == null) fallback = x;
        }
        return fallback;
    }

    Rigidbody[] bodies;

    void FreezeBodies()
    {
        if (bodies == null) return;
        foreach (var rb in bodies)
        {
            if (rb == null) continue;
            if (!rb.isKinematic) rb.isKinematic = true;
            if (rb.useGravity) rb.useGravity = false;
        }
    }

    // 링크의 Visuals 아래에서 STL 모양 오브젝트(링크와 같은 이름 또는 첫 MeshFilter)를 찾음
    static Transform FindVisualOrigin(Transform link)
    {
        var vis = link.Find("Visuals");
        if (vis == null) return null;
        Transform firstMesh = null;
        foreach (var x in vis.GetComponentsInChildren<Transform>(true))
        {
            if (x == vis) continue;
            if (x.name == link.name) return x;
            if (firstMesh == null && x.GetComponent<MeshFilter>() != null) firstMesh = x;
        }
        return firstMesh;
    }

    readonly Dictionary<Transform, Vector3> hingeAxes = new Dictionary<Transform, Vector3>();

    void CacheHingeAxes()
    {
        foreach (var h in GetComponentsInChildren<HingeJoint>(true)) hingeAxes[h.transform] = h.axis;
        foreach (var c in GetComponentsInChildren<ConfigurableJoint>(true)) hingeAxes[c.transform] = c.axis;
    }

    Joint MakeJoint(Transform t, bool prismatic, Vector3 urdfAxis)
    {
        if (t == null) return null;
        var j = new Joint { t = t, prismatic = prismatic, initRot = t.localRotation, initPos = t.localPosition,
                            min = -jointLimit, max = jointLimit };
        if (articulationMode)
        {
            j.ab = t.GetComponent<ArticulationBody>();
            if (j.ab == null) return null;
            var d = j.ab.xDrive;
            d.stiffness = 10000f; d.damping = 100f; d.forceLimit = float.MaxValue;
            j.ab.xDrive = d;
            if (!prismatic) { j.min = Mathf.Max(d.lowerLimit, -jointLimit); j.max = Mathf.Min(d.upperLimit, jointLimit); }
        }
        else
        {
            // ROS#는 관절 정보를 자식 링크의 HingeJoint / ConfigurableJoint에 넣어 둠
            if (useUrdfAxes) j.axis = urdfAxis;
            else if (hingeAxes.TryGetValue(t, out var cached)) j.axis = cached;
            else
            {
                j.axis = prismatic ? Vector3.right : Vector3.up;
                Debug.LogWarning("[LanderPiTeleop] " + t.name + " 에 Joint 컴포넌트가 없어 기본 축을 사용합니다.");
            }
            if (j.axis.sqrMagnitude < 1e-6f) j.axis = prismatic ? Vector3.right : Vector3.up;
            j.axis.Normalize();

            if (pivotFromVisual)
            {
                var vis = FindVisualOrigin(t);
                if (vis != null)
                {
                    j.pivot = t.InverseTransformPoint(vis.position);
                    if (j.pivot.magnitude > 0.005f)
                        Debug.Log($"[LanderPiTeleop] {t.name}: 회전 중심을 모양 원점으로 보정 (어긋난 거리 {j.pivot.magnitude * 100f:0.0}cm)");
                }
            }
        }
        return j;
    }

    void Apply(Joint j)
    {
        j.value = Mathf.Clamp(j.value, j.min, j.max);
        if (articulationMode)
        {
            var d = j.ab.xDrive; d.target = j.value; j.ab.xDrive = d;
        }
        else if (j.prismatic)
        {
            j.t.localPosition = j.initPos + j.initRot * j.axis * j.value;
        }
        else
        {
            Quaternion r = Quaternion.AngleAxis(j.value, j.axis);
            j.t.localRotation = j.initRot * r;
            // pivot 점을 중심으로 돌도록 위치 보정 (pivot이 0이면 기존과 같음)
            Vector3 s = j.t.localScale;
            Vector3 offset = j.pivot - r * j.pivot;
            j.t.localPosition = j.initPos + j.initRot * Vector3.Scale(s, offset);
        }
    }

    void FixedUpdate()
    {
        if (!articulationMode)
        {
            // 다른 스크립트가 물리를 다시 켜도 매 프레임 되돌리고, 관절 자세를 다시 고정
            FreezeBodies();
            foreach (var aj in arm) Apply(aj);
            if (leftJaw != null) Apply(leftJaw);
            if (rightJaw != null) Apply(rightJaw);
        }

        var kb = Keyboard.current;
        if (kb == null || movingRoot == null) return;
        float dt = Time.fixedDeltaTime;

        // ---------- 차체 ----------
        float fwd = (kb.wKey.isPressed ? 1 : 0) - (kb.sKey.isPressed ? 1 : 0);
        float turn = (kb.dKey.isPressed ? 1 : 0) - (kb.aKey.isPressed ? 1 : 0);
        float side = mecanum ? (kb.eKey.isPressed ? 1 : 0) - (kb.qKey.isPressed ? 1 : 0) : 0;

        if (fwd != 0 || turn != 0 || side != 0)
        {
            Transform t = movingRoot;
            Quaternion rot = Quaternion.Euler(0f, turn * maxAngular * dt, 0f) * t.rotation;
            float sign = invertForward ? -1f : 1f;
            Vector3 f = Vector3.ProjectOnPlane(t.forward, Vector3.up).normalized * sign;
            Vector3 r = Vector3.ProjectOnPlane(t.right, Vector3.up).normalized * sign;
            Vector3 delta = (f * fwd + r * side).normalized * maxLinear * dt;
            Vector3 pos = t.position;

            if (delta.sqrMagnitude > 0f)
            {
                Vector3 dir = delta.normalized;
                Vector3 origin = pos + Vector3.up * (collisionRadius + 0.02f);
                float allowed = delta.magnitude;
                foreach (var hit in Physics.SphereCastAll(origin, collisionRadius, dir, allowed,
                                                          obstacleLayers, QueryTriggerInteraction.Ignore))
                {
                    if (hit.transform.IsChildOf(transform)) continue;   // 자기 자신의 콜라이더는 무시
                    if (hit.distance <= 0f) continue;                    // 시작부터 겹친 바닥 등은 무시
                    allowed = Mathf.Min(allowed, Mathf.Max(0f, hit.distance - 0.01f));
                }
                pos += dir * allowed;
            }

            if (articulationMode) rootAB.TeleportRoot(pos, rot);
            else t.SetPositionAndRotation(pos, rot);
        }

        // ---------- 팔 ----------
        if (arm.Count > 0)
        {
            float move = (kb.upArrowKey.isPressed ? 1 : 0) - (kb.downArrowKey.isPressed ? 1 : 0);
            if (move != 0)
            {
                var j = arm[Mathf.Clamp(selected, 0, arm.Count - 1)];
                j.value += move * jointSpeed * dt;
                Apply(j);
            }
        }
    }

    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        Key[] numKeys = { Key.Digit1, Key.Digit2, Key.Digit3, Key.Digit4, Key.Digit5 };
        for (int i = 0; i < numKeys.Length && i < arm.Count; i++)
            if (kb[numKeys[i]].wasPressedThisFrame) selected = i;

        if (kb.spaceKey.wasPressedThisFrame) SetGripper(!gripperOpen);
        if (kb.digit0Key.wasPressedThisFrame) SetPose(0, 0, 0, 0, 0);
        if (kb.pKey.wasPressedThisFrame) SetPose(0, -30, -60, -30, 0); // 팔이 아래로 꺾이면 부호를 반대로
    }

    void SetupCamera()
    {
        // 화면에 직접 그리는 카메라들 (미니맵처럼 RenderTexture로 그리는 카메라는 제외)
        var screenCams = new List<Camera>();
        foreach (var c in FindObjectsByType<Camera>(FindObjectsSortMode.None))
            if (c.isActiveAndEnabled && c.targetTexture == null) screenCams.Add(c);

        cam = Camera.main != null && Camera.main.targetTexture == null ? Camera.main
            : (screenCams.Count > 0 ? screenCams[0] : null);
        if (cam == null)
        {
            var go = new GameObject("LanderPi Camera");
            cam = go.AddComponent<Camera>();
            go.tag = "MainCamera";
        }

        foreach (var c in screenCams)
            if (c != cam) { c.enabled = false; Debug.Log("[LanderPiTeleop] 다른 카메라 끔: " + c.name); }

        // 같은 카메라에 붙은 예전 따라가기 스크립트는 끄기 (서로 위치를 덮어쓰지 않게)
        foreach (var mb in cam.GetComponents<MonoBehaviour>())
            if (mb.GetType().Name == "CameraFollow") mb.enabled = false;

        if (cam.transform.IsChildOf(transform)) cam.transform.SetParent(null, true);
        SnapCamera(1f);
        Debug.Log("[LanderPiTeleop] 따라갈 카메라: " + cam.name);
    }

    void SnapCamera(float t)
    {
        if (cam == null || movingRoot == null) return;
        Transform r = movingRoot;
        Vector3 back = -Vector3.ProjectOnPlane(r.forward, Vector3.up).normalized * (invertForward ? -1f : 1f);
        Vector3 want = r.position + back * camDistance + Vector3.up * camHeight;
        cam.transform.position = Vector3.Lerp(cam.transform.position, want, t);
        cam.transform.LookAt(r.position + Vector3.up * camLookHeight);
    }

    void LateUpdate()
    {
        if (followCamera && cam != null) SnapCamera(1f - Mathf.Exp(-camSmooth * Time.deltaTime));
    }

    public void SetPose(params float[] degrees)
    {
        for (int i = 0; i < arm.Count && i < degrees.Length; i++) { arm[i].value = degrees[i]; Apply(arm[i]); }
    }

    public void SetGripper(bool open)
    {
        gripperOpen = open;
        if (leftJaw != null) { leftJaw.value = open ? JawTravel : 0f; Apply(leftJaw); }
        if (rightJaw != null) { rightJaw.value = open ? -JawTravel : 0f; Apply(rightJaw); }
    }

    void OnGUI()
    {
        if (!showHelp) return;
        GUILayout.BeginArea(new Rect(10, 10, 310, 250), GUI.skin.box);
        GUILayout.Label("<b>LanderPi 조종</b>  (" + status + ")", new GUIStyle(GUI.skin.label) { richText = true });
        GUILayout.Label("W/S 전진·후진   A/D 회전" + (mecanum ? "   Q/E 옆이동" : ""));
        GUILayout.Label("1~5 관절 선택   ↑/↓ 회전");
        GUILayout.Label("Space 그리퍼   0 기본자세   P 위로 뻗기");
        for (int i = 0; i < arm.Count; i++)
            GUILayout.Label((i == selected ? "▶ " : "   ") + $"joint{i + 1}: {arm[i].value,6:0.0}°");
        GUILayout.Label("그리퍼: " + (gripperOpen ? "열림" : "닫힘"));
        GUILayout.EndArea();
    }
}
