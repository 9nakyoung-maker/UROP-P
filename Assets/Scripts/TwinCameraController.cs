using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

// =====================================================================================
//  TwinCameraController — 디지털 트윈 화면 시점 바꾸기
//
//  F1 로봇 따라가기 (뒤에서 3인칭)
//  F2 로봇 눈높이 (1인칭)
//  F3 CCTV — 방 모서리 천장 근처에서 방 전체 보기 (F3을 다시 누르면 다른 모서리)
//  F4 위에서 내려다보기 — 평면도처럼 (천장·조명·덕트는 잘라서 안 보이게)
//  F5 자유롭게 돌려 보기 — 로봇 주위를 마우스로 회전
//
//  마우스: 오른쪽 버튼 누른 채 끌기 = 돌리기(F5) / 화면 이동(F4),  휠 = 확대·축소
//  화면 오른쪽 위 버튼으로도 바꿀 수 있어요.
//
//  붙이는 곳: 아무 오브젝트 (예: 빈 오브젝트 "CameraRig" 를 만들어서)
//  Robot 칸 = rosmentor,  Room 칸 = Revit 방 맨 위 오브젝트 (비우면 자동으로 찾음)
// =====================================================================================
public class TwinCameraController : MonoBehaviour
{
    public enum Mode { Follow, FirstPerson, CCTV, TopDown, Orbit }

    [Header("연결")]
    [Tooltip("움직일 카메라. 비우면 Main Camera")]
    public Camera cam;
    [Tooltip("로봇 맨 위 오브젝트 (rosmentor). 비우면 자동으로 찾음")]
    public Transform robot;
    [Tooltip("Revit 방 맨 위 오브젝트. 비우면 이름에 '프로젝트'가 들어간 오브젝트를 찾음")]
    public Transform room;

    [Header("시작 시점")]
    public Mode mode = Mode.CCTV;

    [Header("F1 로봇 따라가기")]
    public float followDistance = 1.2f;
    public float followHeight = 0.7f;
    public float followSmooth = 6f;
    [Tooltip("로봇 앞쪽이 Unity 파란 화살표(Z)와 다르면 90 / -90 / 180")]
    public float robotForwardOffset = 0f;

    [Header("F2 로봇 눈높이")]
    public float eyeHeight = 0.25f;
    public float eyeForward = 0.12f;

    [Header("F3 CCTV")]
    [Tooltip("직접 정한 CCTV 자리 (빈 오브젝트를 원하는 곳·방향에 두고 넣기). 비우면 방 모서리 자동")]
    public Transform cctvPoint;
    [Range(0, 3)] public int cctvCorner = 0;
    [Tooltip("벽·천장에서 안쪽으로 들어올 거리 (m). 벽 속에 묻히면 키우기")]
    public float cctvInset = 0.5f;
    [Tooltip("켜면 CCTV가 로봇을 계속 따라 바라봄")]
    public bool cctvLookAtRobot = false;

    [Header("F4 위에서 내려다보기")]
    [Tooltip("바닥에서 이 높이(m)보다 위(천장·조명·덕트)는 잘라서 안 보이게")]
    public float topDownCutHeight = 1.8f;
    public float topDownMargin = 0.5f;
    [Tooltip("화면에서 방을 돌려 보기 (도)")]
    public float topDownYaw = 0f;

    [Header("F5 자유롭게 돌려 보기")]
    public float orbitDistance = 1.5f;
    public float orbitPitch = 35f;
    public float orbitYaw = 0f;

    [Header("기타")]
    public bool showButtons = true;
    public float mouseRotateSpeed = 0.25f;

    Bounds bounds;
    bool hasBounds;
    float defFov, defNear, defFar, cctvFov, topZoom = 1f;
    Vector3 topPan;
    bool started;
    Mode applied;

    void Start()
    {
        if (cam == null) cam = Camera.main;
        if (cam == null)
        {
            Debug.LogError("[TwinCamera] 카메라를 못 찾았어요. Cam 칸에 카메라를 넣으세요.");
            enabled = false;
            return;
        }
        if (robot == null)
        {
            var r = GameObject.Find("rosmentor");
            if (r != null) robot = r.transform;
        }
        if (room == null) room = FindRoom();

        // 로봇에 붙어 있던 카메라면 떼어 내서 자유롭게 움직이게
        if (cam.transform.parent != null) cam.transform.SetParent(null, true);

        // 카메라를 따로 움직이는 다른 스크립트가 있으면 알려 주기
        foreach (var mb in cam.GetComponents<MonoBehaviour>())
        {
            if (mb == null || mb == this) continue;
            string ns = mb.GetType().Namespace ?? "";
            if (!ns.StartsWith("UnityEngine"))
                Debug.LogWarning($"[TwinCamera] 카메라에 '{mb.GetType().Name}' 스크립트가 있어요. 카메라를 따로 움직이는 스크립트면 꺼 주세요.", mb);
        }

        defFov = cam.fieldOfView;
        defNear = cam.nearClipPlane;
        defFar = cam.farClipPlane;
        cctvFov = defFov;
        CalcBounds();
        started = true;
        applied = mode;
        EnterMode();
    }

    Transform FindRoom()
    {
        foreach (var go in SceneManager.GetActiveScene().GetRootGameObjects())
            if (go.activeInHierarchy && (go.name.Contains("프로젝트") || go.name.Contains("3D뷰")))
                return go.transform;
        return null;
    }

    void CalcBounds()
    {
        hasBounds = false;
        Renderer[] rs = room != null
            ? room.GetComponentsInChildren<Renderer>()
            : FindObjectsByType<Renderer>(FindObjectsSortMode.None);
        foreach (var r in rs)
        {
            if (r == null || !r.enabled || !r.gameObject.activeInHierarchy) continue;
            if (robot != null && r.transform.IsChildOf(robot)) continue;
            if (r is ParticleSystemRenderer || r is LineRenderer || r is TrailRenderer) continue;
            if (!hasBounds) { bounds = r.bounds; hasBounds = true; }
            else bounds.Encapsulate(r.bounds);
        }
        if (!hasBounds)
        {
            bounds = new Bounds(robot != null ? robot.position : Vector3.zero, new Vector3(5f, 3f, 8f));
            hasBounds = true;
        }
        Debug.Log($"[TwinCamera] 방 크기 {bounds.size.x:F1} x {bounds.size.z:F1} m, 높이 {bounds.size.y:F1} m"
                  + (room != null ? $" (기준: {room.name})" : " (기준: 장면 전체 — Room 칸을 채우면 더 정확해요)"));
    }

    // ------------------------------------------------------------------ 입력
    void Update()
    {
        if (!started) return;

        var kb = Keyboard.current;
        if (kb != null)
        {
            if (kb.f1Key.wasPressedThisFrame) mode = Mode.Follow;
            if (kb.f2Key.wasPressedThisFrame) mode = Mode.FirstPerson;
            if (kb.f3Key.wasPressedThisFrame) PressCCTV();
            if (kb.f4Key.wasPressedThisFrame) mode = Mode.TopDown;
            if (kb.f5Key.wasPressedThisFrame) mode = Mode.Orbit;
        }

        var ms = Mouse.current;
        if (ms == null) return;
        Vector2 d = ms.delta.ReadValue();
        float wheel = ms.scroll.ReadValue().y;
        float w = Mathf.Abs(wheel) > 0.01f ? Mathf.Sign(wheel) : 0f;   // 위로 굴리면 +1 = 확대
        bool drag = ms.rightButton.isPressed;

        switch (mode)
        {
            case Mode.Follow:
                if (w != 0f) followDistance = Mathf.Clamp(followDistance * (w > 0 ? 0.9f : 1.1f), 0.3f, 5f);
                break;
            case Mode.FirstPerson:
                if (w != 0f) cam.fieldOfView = Mathf.Clamp(cam.fieldOfView + (w > 0 ? -3f : 3f), 20f, 100f);
                break;
            case Mode.CCTV:
                if (w != 0f) cctvFov = Mathf.Clamp(cctvFov + (w > 0 ? -3f : 3f), 15f, 100f);
                break;
            case Mode.TopDown:
                if (w != 0f) topZoom = Mathf.Clamp(topZoom * (w > 0 ? 0.9f : 1.1f), 0.2f, 3f);
                if (drag)
                {
                    float unitsPerPixel = 2f * cam.orthographicSize / Mathf.Max(Screen.height, 1);
                    topPan -= Quaternion.Euler(0f, topDownYaw, 0f) * new Vector3(d.x, 0f, d.y) * unitsPerPixel;
                }
                break;
            case Mode.Orbit:
                if (drag)
                {
                    orbitYaw += d.x * mouseRotateSpeed;
                    orbitPitch = Mathf.Clamp(orbitPitch - d.y * mouseRotateSpeed, 5f, 85f);
                }
                if (w != 0f) orbitDistance = Mathf.Clamp(orbitDistance * (w > 0 ? 0.9f : 1.1f), 0.3f, 10f);
                break;
        }
    }

    void PressCCTV()
    {
        if (mode == Mode.CCTV && cctvPoint == null) cctvCorner = (cctvCorner + 1) % 4;   // 다시 누르면 다른 모서리
        mode = Mode.CCTV;
    }

    // ------------------------------------------------------------------ 카메라 움직이기 (로봇이 움직인 뒤)
    void LateUpdate()
    {
        if (!started) return;
        if (applied != mode) { applied = mode; EnterMode(); }

        switch (mode)
        {
            case Mode.Follow: DoFollow(false); break;
            case Mode.FirstPerson: DoFirstPerson(); break;
            case Mode.CCTV: DoCCTV(); break;
            case Mode.TopDown: DoTopDown(); break;
            case Mode.Orbit: DoOrbit(); break;
        }
    }

    void EnterMode()
    {
        cam.orthographic = false;
        cam.fieldOfView = defFov;
        cam.nearClipPlane = defNear;
        cam.farClipPlane = defFar;
        if (mode == Mode.Follow) DoFollow(true);
        if (mode == Mode.FirstPerson) cam.nearClipPlane = Mathf.Min(defNear, 0.05f);
    }

    Vector3 Target() => robot != null ? robot.position : bounds.center;
    float FloorY() => robot != null ? robot.position.y : bounds.min.y;

    Vector3 RobotForward()
    {
        if (robot == null) return Vector3.forward;
        return Quaternion.Euler(0f, robot.eulerAngles.y + robotForwardOffset, 0f) * Vector3.forward;
    }

    void DoFollow(bool snap)
    {
        Vector3 t = Target();
        Vector3 want = t - RobotForward() * followDistance + Vector3.up * followHeight;
        cam.transform.position = snap
            ? want
            : Vector3.Lerp(cam.transform.position, want, 1f - Mathf.Exp(-followSmooth * Time.deltaTime));
        cam.transform.LookAt(t + Vector3.up * 0.15f);
    }

    void DoFirstPerson()
    {
        Vector3 f = RobotForward();
        cam.transform.position = Target() + Vector3.up * eyeHeight + f * eyeForward;
        cam.transform.rotation = Quaternion.LookRotation(f, Vector3.up);
    }

    void DoCCTV()
    {
        cam.fieldOfView = cctvFov;
        if (cctvPoint != null)
        {
            cam.transform.position = cctvPoint.position;
            if (cctvLookAtRobot && robot != null) cam.transform.LookAt(robot.position);
            else cam.transform.rotation = cctvPoint.rotation;
            return;
        }
        Vector3 mn = bounds.min, mx = bounds.max;
        // 0: 왼쪽 앞, 1: 오른쪽 앞, 2: 오른쪽 뒤, 3: 왼쪽 뒤 (월드 X·Z 기준)
        float x = (cctvCorner == 0 || cctvCorner == 3) ? mn.x + cctvInset : mx.x - cctvInset;
        float z = (cctvCorner < 2) ? mn.z + cctvInset : mx.z - cctvInset;
        float y = mx.y - cctvInset;
        cam.transform.position = new Vector3(x, y, z);
        Vector3 look = (cctvLookAtRobot && robot != null)
            ? robot.position
            : new Vector3(bounds.center.x, FloorY(), bounds.center.z);
        cam.transform.LookAt(look);
    }

    void DoTopDown()
    {
        float floor = FloorY();
        float camY = bounds.max.y + 2f;
        cam.orthographic = true;
        cam.transform.rotation = Quaternion.Euler(90f, topDownYaw, 0f);
        cam.transform.position = new Vector3(bounds.center.x, camY, bounds.center.z) + topPan;
        // 천장·덕트·조명이 가리지 않게, 바닥에서 topDownCutHeight 보다 위는 잘라 냄
        cam.nearClipPlane = Mathf.Max(0.01f, camY - (floor + topDownCutHeight));
        cam.farClipPlane = Mathf.Max(defFar, camY - bounds.min.y + 5f);

        float a = topDownYaw * Mathf.Deg2Rad;
        Vector3 e = bounds.extents;
        float ex = Mathf.Abs(Mathf.Cos(a)) * e.x + Mathf.Abs(Mathf.Sin(a)) * e.z;   // 화면 가로 방향 반폭
        float ez = Mathf.Abs(Mathf.Sin(a)) * e.x + Mathf.Abs(Mathf.Cos(a)) * e.z;   // 화면 세로 방향 반폭
        float size = Mathf.Max(ez, ex / Mathf.Max(cam.aspect, 0.1f)) + topDownMargin;
        cam.orthographicSize = size * topZoom;
    }

    void DoOrbit()
    {
        Vector3 t = Target() + Vector3.up * 0.15f;
        Quaternion q = Quaternion.Euler(orbitPitch, orbitYaw, 0f);
        cam.transform.position = t - q * Vector3.forward * orbitDistance;
        cam.transform.LookAt(t);
    }

    // ------------------------------------------------------------------ 화면 버튼 (오른쪽 위)
    void OnGUI()
    {
        if (!showButtons || !started) return;
        const float bw = 190f, bh = 24f;
        float x = Screen.width - bw - 10f, y = 10f;
        GUI.Box(new Rect(x - 5f, y - 5f, bw + 10f, bh * 5f + 8f * 2f + 44f), "");
        if (GUI.Button(new Rect(x, y, bw, bh), Mark(Mode.Follow) + "로봇 따라가기 [F1]")) mode = Mode.Follow; y += bh + 2f;
        if (GUI.Button(new Rect(x, y, bw, bh), Mark(Mode.FirstPerson) + "로봇 눈높이 [F2]")) mode = Mode.FirstPerson; y += bh + 2f;
        if (GUI.Button(new Rect(x, y, bw, bh), Mark(Mode.CCTV) + "CCTV (다시: 모서리) [F3]")) PressCCTV(); y += bh + 2f;
        if (GUI.Button(new Rect(x, y, bw, bh), Mark(Mode.TopDown) + "위에서 보기 [F4]")) mode = Mode.TopDown; y += bh + 2f;
        if (GUI.Button(new Rect(x, y, bw, bh), Mark(Mode.Orbit) + "자유롭게 돌려 보기 [F5]")) mode = Mode.Orbit; y += bh + 4f;
        GUI.Label(new Rect(x, y, bw, 44f), Hint());
    }

    string Mark(Mode m) => mode == m ? "▶ " : "";

    string Hint()
    {
        switch (mode)
        {
            case Mode.Follow: return "휠: 가까이·멀리";
            case Mode.FirstPerson: return "휠: 확대·축소";
            case Mode.CCTV: return "휠: 확대·축소\nF3 다시: 다른 모서리";
            case Mode.TopDown: return "휠: 확대·축소\n오른쪽 버튼 끌기: 이동";
            default: return "오른쪽 버튼 끌기: 돌리기\n휠: 가까이·멀리";
        }
    }
}
