using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
#if UNITY_EDITOR
using UnityEditor;
#endif

// =====================================================================================
//  MyLineFollower (팀원 버전을 다듬은 것)
//  - 바닥에 놓은 경로 점(Point_0, Point_1 ...) 사이를 다익스트라 최단 경로로 자동 주행
//  - Z: 에어컨   X: 조명 스위치   C: 홈(Point_0)   V: 정지      (키는 Inspector에서 변경 가능)
//
//  팀원 원본에서 바뀐 점
//   1) 이웃 연결 자동: 두 점 사이에 벽·가구가 없으면 알아서 연결 (Auto Connect). 수동 연결도 가능
//   2) 실제 LanderPi 속도(0.3 m/s), 제자리 회전 후 전진 → 실물 로봇처럼 움직임
//   3) 키 충돌 해결: A키는 차체 회전(LanderPiTeleop)과 겹쳐서 Z/X/C로 변경, 새 Input System 사용
//   4) 경로 선이 분홍색·1m 두께로 나오던 문제 → 빨간색 3cm 선
//   5) 막히면 다른 길로 다시 계산하거나 옆으로 비켜서 우회 (안 되면 멈추고 Console에 알림)
//   6) 목적지를 바꾸면 홈으로 되돌아가지 않고 지금 위치에서 바로 최단 경로로 출발
//
//  붙이는 곳: 로봇 맨 위 오브젝트 (rosmentor)
// =====================================================================================

[System.Serializable]
public class NodeData
{
    public string Name;
    public Transform WayPoint;
    [Tooltip("Auto Connect를 끈 경우에만 사용: 이 점과 바로 이어지는 점들")]
    public List<Transform> Neighbors = new List<Transform>();

    [HideInInspector] public float gScore = float.MaxValue;
    [System.NonSerialized] public NodeData parent;
    [System.NonSerialized] public List<NodeData> links = new List<NodeData>();
}

public class MyLineFollower : MonoBehaviour
{
    [Header("경로 점들 (0번 = Point_0 = Home)")]
    [Tooltip("경로 점들을 모아둔 부모 오브젝트. Nodes가 비어 있으면 이 자식들로 자동 채움")]
    public Transform WaypointsParent;
    public List<NodeData> Nodes = new List<NodeData>();

    [Header("점끼리 연결")]
    [Tooltip("켜두면 두 점 사이에 장애물이 없을 때 자동 연결 (Neighbors 입력 불필요)")]
    public bool AutoConnect = true;
    [Tooltip("로봇 반지름(m). 이만큼 여유가 있어야 '지나갈 수 있다'고 판단")]
    public float RobotRadius = 0.15f;

    [Header("목적지 (Nodes 번호)")]
    public int AirconNodeIndex = 3;        // Point_3 = 에어컨
    public int LightSwitchNodeIndex = 2;   // Point_2 = 조명 스위치
    public int HomeNodeIndex = 1;          // Point_1 = 원점(홈)
    public Key AirconKey = Key.Z;
    public Key LightSwitchKey = Key.X;
    public Key HomeKey = Key.C;
    public Key StopKey = Key.V;

    [Header("이동 (실물 LanderPi 기준)")]
    public float MoveSpeed = 0.3f;          // m/s
    public float TurnSpeed = 90f;           // 도/초
    [Tooltip("방향이 이 각도 넘게 틀어져 있으면 제자리에서 먼저 회전")]
    public float TurnInPlaceAngle = 30f;
    public float ArrivalDistance = 0.1f;
    [Tooltip("로봇이 옆이나 뒤를 보고 달리면 90 / -90 / 180 으로")]
    public float ForwardAngleOffset = 0f;
    public bool StartAtHome = true;

    [Header("막혔을 때 우회")]
    [Tooltip("가다가 물체에 막히면 그 길을 '막힘'으로 기억하고, 남은 목적지까지 다른 길로 다시 계산")]
    public bool AutoReroute = true;
    [Tooltip("앞으로 이 거리(m) 안에 물체가 보이면 부딪히기 전에 미리 우회 (0이면 끔, 막혀서 멈췄을 때만 우회)")]
    public float LookAheadDistance = 0.4f;
    [Tooltip("이 시간(초) 동안 거의 못 움직이면 '막힘'으로 보고 우회 시작")]
    public float StuckSeconds = 1.5f;
    [Tooltip("막힌 길을 몇 초 동안 피할지 (지나면 다시 시도)")]
    public float BlockedMemorySeconds = 30f;
    [Tooltip("다른 길이 없을 때, 옆으로 비켜 가는 임시 점을 만들어 지나가기")]
    public bool TryDetourPoint = true;
    [Tooltip("임시 점을 옆으로 최대 몇 m까지 찾을지")]
    public float DetourMaxOffset = 1.2f;
    [Tooltip("한 번 이동에서 우회를 최대 몇 번까지 시도할지")]
    public int MaxReplans = 5;

    [Header("경로 선")]
    public Color LineColor = Color.red;
    public float LineWidth = 0.03f;
    public float LineOffsetY = 0.01f;
    public bool ShowHelp = true;

    readonly List<Transform> currentPath = new List<Transform>();
    int currentPathIndex;
    bool isMoving;
    LineRenderer lineRenderer;
    string status = "대기 중";
    Vector3 progressPos; float progressTime;
    Transform legFrom;                  // 방금 출발한 (진짜) 경로 점
    int finalGoalIndex;                 // 최종 목적지 번호
    int replanCount;                    // 이번 이동에서 우회한 횟수
    readonly Dictionary<int, float> blockedUntil = new Dictionary<int, float>();   // 막힌 길(점-점) → 언제까지 피할지
    readonly List<GameObject> tempPoints = new List<GameObject>();                 // 임시 우회 점

    void Start()
    {
        FillNodesFromParent();
        BuildLinks();
        SetupLine();

        if (StartAtHome && Valid(HomeNodeIndex))
        {
            Vector3 p = Nodes[HomeNodeIndex].WayPoint.position;
            transform.position = new Vector3(p.x, transform.position.y, p.z);
        }
    }

    // ------------------------------------------------------------------ 입력
    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null) return;
        if (kb[AirconKey].wasPressedThisFrame) SetDestination(AirconNodeIndex);
        else if (kb[LightSwitchKey].wasPressedThisFrame) SetDestination(LightSwitchNodeIndex);
        else if (kb[HomeKey].wasPressedThisFrame) SetDestination(HomeNodeIndex);
        else if (kb[StopKey].wasPressedThisFrame) Stop("정지");
    }

    // ------------------------------------------------------------------ 이동 (물리 주기에 맞춰서, RobotBodyBlocker가 그 다음에 검사)
    void FixedUpdate()
    {
        if (!isMoving) return;
        if (currentPathIndex >= currentPath.Count) { isMoving = false; status = "도착 ✓"; ClearTempPoints(); Debug.Log("[MyLineFollower] 목적지에 도착했습니다!"); return; }

        float dt = Time.fixedDeltaTime;
        Vector3 target = currentPath[currentPathIndex].position;
        Vector3 to = target - transform.position; to.y = 0f;

        if (to.magnitude < ArrivalDistance)
        {
            if (!IsTemp(currentPath[currentPathIndex])) legFrom = currentPath[currentPathIndex];
            currentPathIndex++;
            return;
        }

        // 회전
        float desiredYaw = Quaternion.LookRotation(to).eulerAngles.y + ForwardAngleOffset;
        float yawErr = Mathf.DeltaAngle(transform.eulerAngles.y, desiredYaw);
        float step = Mathf.Clamp(yawErr, -TurnSpeed * dt, TurnSpeed * dt);
        transform.Rotate(0f, step, 0f, Space.World);

        // 방향이 거의 맞으면 전진 (앞에 물체가 보이면 부딪히기 전에 먼저 우회)
        if (Mathf.Abs(yawErr) < TurnInPlaceAngle)
        {
            if (AutoReroute && ObstacleAhead(to)) { OnBlocked("앞에 물체가 있어요"); return; }
            transform.position += to.normalized * Mathf.Min(MoveSpeed * dt, to.magnitude);
        }

        // 막힘 감지: 3초 동안 2cm도 못 갔고 회전도 거의 끝났으면 멈춤
        if ((transform.position - progressPos).sqrMagnitude > 0.0004f || Mathf.Abs(yawErr) > TurnInPlaceAngle)
        { progressPos = transform.position; progressTime = Time.time; }
        else if (Time.time - progressTime > StuckSeconds)
            OnBlocked($"'{currentPath[currentPathIndex].name}'로 가다가 막힘");
    }

    void Stop(string why)
    {
        isMoving = false; status = why;
        ClearTempPoints();
        Debug.LogWarning("[MyLineFollower] " + why);
    }

    // ------------------------------------------------------------------ 경로 계산
    public void SetDestination(int targetIndex)
    {
        if (!Valid(targetIndex)) { Debug.LogError($"[MyLineFollower] {targetIndex}번 점이 없어요. Nodes 목록을 확인하세요."); return; }

        ClearTempPoints(); blockedUntil.Clear(); replanCount = 0; finalGoalIndex = targetIndex;
        BuildLinks();   // 그 사이 가구를 옮겼을 수도 있으니 매번 다시 연결
        legFrom = null;   // 지금 위치에서 바로 출발 (가까운 점을 일부러 거치지 않음)
        NodeData goal = Nodes[targetIndex];
        var path = PlanFromHere(goal);

        if (path == null) { Stop($"지금 위치 → {goal.WayPoint.name} 이어진 길이 없어요 (점 사이에 장애물이 있는지 확인)"); return; }

        currentPath.Clear(); currentPath.AddRange(path);
        currentPathIndex = 0; isMoving = true;
        progressPos = transform.position; progressTime = Time.time;
        status = "이동 중 → " + goal.WayPoint.name;
        DrawFullPathLine();

        var names = new List<string>(); foreach (var t in currentPath) names.Add(t.name);
        Debug.Log("[MyLineFollower] 경로: " + string.Join(" ➔ ", names));
    }

    // 지금 위치에서 목적지까지 최단 경로.
    // "로봇 → 바로 갈 수 있는 점" + "그 점 → 목적지" 거리 합이 가장 짧은 걸 고름.
    // → 가다가 목적지를 바꿔도 홈 같은 가까운 점으로 되돌아가지 않고, 목적지가 바로 보이면 곧장 감.
    List<Transform> PlanFromHere(NodeData goal, NodeData avoidFirst = null)
    {
        if (goal == null || goal.WayPoint == null) return null;

        // 목적지에서 거꾸로 다익스트라 → 모든 점의 '목적지까지 남은 거리'
        foreach (var n in Nodes) { n.gScore = float.MaxValue; n.parent = null; }
        goal.gScore = 0f;
        var open = new List<NodeData> { goal };
        while (open.Count > 0)
        {
            open.Sort((a, b) => a.gScore.CompareTo(b.gScore));
            NodeData cur = open[0]; open.RemoveAt(0);
            foreach (var nb in cur.links)
            {
                if (IsBlocked(cur, nb)) continue;
                float s = cur.gScore + Vector3.Distance(cur.WayPoint.position, nb.WayPoint.position);
                if (s < nb.gScore) { nb.gScore = s; nb.parent = cur; if (!open.Contains(nb)) open.Add(nb); }
            }
        }

        // 로봇에서 장애물 없이 바로 갈 수 있는 점 중 총거리가 가장 짧은 점을 첫 점으로
        Vector3 pos = transform.position;
        NodeData best = null; float bestCost = float.MaxValue;
        foreach (var n in Nodes)
        {
            if (n == null || n.WayPoint == null || n == avoidFirst || n.gScore == float.MaxValue) continue;
            Vector3 d = n.WayPoint.position - pos; d.y = 0f;
            float cost = d.magnitude + n.gScore;
            if (cost >= bestCost) continue;
            if (d.magnitude > ArrivalDistance && !Clear(pos, n.WayPoint.position)) continue;
            best = n; bestCost = cost;
        }
        if (best == null) return null;

        var path = new List<Transform>();
        for (var c = best; c != null; c = c.parent) path.Add(c.WayPoint);   // parent는 목적지 쪽을 가리킴
        return path;
    }

    List<Transform> CalculateDijkstraPath(NodeData start, NodeData target)
    {
        if (start == null || target == null) return null;
        foreach (var n in Nodes) { n.gScore = float.MaxValue; n.parent = null; }
        var open = new List<NodeData> { start };
        start.gScore = 0f;

        while (open.Count > 0)
        {
            open.Sort((a, b) => a.gScore.CompareTo(b.gScore));
            NodeData cur = open[0]; open.RemoveAt(0);
            if (cur == target)
            {
                var path = new List<Transform>();
                for (var c = target; c != null; c = c.parent) path.Add(c.WayPoint);
                path.Reverse();
                return path;
            }
            foreach (var nb in cur.links)
            {
                if (IsBlocked(cur, nb)) continue;   // 막혔던 길은 한동안 피함
                float s = cur.gScore + Vector3.Distance(cur.WayPoint.position, nb.WayPoint.position);
                if (s < nb.gScore) { nb.gScore = s; nb.parent = cur; if (!open.Contains(nb)) open.Add(nb); }
            }
        }
        return null;
    }

    NodeData GetClosestNode(Vector3 pos)
    {
        NodeData best = null; float bestD = float.MaxValue;
        foreach (var n in Nodes)
        {
            if (n.WayPoint == null) continue;
            Vector3 d = n.WayPoint.position - pos; d.y = 0f;
            if (d.sqrMagnitude < bestD) { bestD = d.sqrMagnitude; best = n; }
        }
        return best;
    }

    // ------------------------------------------------------------------ 막혔을 때 우회
    // ① 막힌 길을 기억 → ② 최종 목적지까지 다른 길 다시 계산 → ③ 길이 없으면 옆으로 비켜 가는 임시 점
    void OnBlocked(string why)
    {
        if (!AutoReroute || replanCount >= MaxReplans || !Valid(finalGoalIndex))
        {
            Stop($"{why} — 길을 막은 물체를 치우거나, 점을 벽·가구에서 더 떨어뜨리세요");
            return;
        }
        replanCount++;

        Transform next = NextRealTarget();
        NodeData from = legFrom != null ? NodeOf(legFrom) : null;
        NodeData to = next != null ? NodeOf(next) : null;
        if (from != null && to != null && from != to) BlockEdge(from, to);
        Debug.LogWarning($"[MyLineFollower] {why} → 우회 시도 {replanCount}/{MaxReplans}");

        // ① 다른 길 찾기 (지금 위치에서, 막힌 점으로 바로 가는 건 빼고)
        BuildLinks();
        var path = PlanFromHere(Nodes[finalGoalIndex], to);
        if (path != null)
        {
            ClearTempPoints();
            currentPath.Clear(); currentPath.AddRange(path);
            currentPathIndex = 0;
            AfterReroute("다른 길로");
            return;
        }

        // ② 다른 길이 없으면 옆으로 비켜 가는 임시 점
        if (TryDetourPoint && next != null && TryDetour(next))
        {
            AfterReroute("옆으로 비켜서");
            return;
        }

        Stop($"{why} — 돌아갈 길이 없어요. 길을 막은 물체를 치우거나 점을 추가하세요");
    }

    void AfterReroute(string how)
    {
        progressPos = transform.position; progressTime = Time.time;
        status = $"우회 중({how}) → {Nodes[finalGoalIndex].WayPoint.name} ({replanCount}/{MaxReplans})";
        DrawFullPathLine();
        var names = new List<string>(); foreach (var t in currentPath) names.Add(t.name);
        Debug.Log($"[MyLineFollower] {how} 가는 새 경로: " + string.Join(" ➔ ", names));
    }

    // 지금 위치에서 옆으로 조금 비킨 점을 찾아, 거기를 거쳐 원래 가려던 점으로 가도록 경로를 바꿈
    bool TryDetour(Transform next)
    {
        Vector3 pos = transform.position;
        Vector3 dir = next.position - pos; dir.y = 0f;
        if (dir.sqrMagnitude < 1e-4f) return false;
        dir.Normalize();
        Vector3 side = Vector3.Cross(Vector3.up, dir);

        var rest = new List<Transform>();   // 남은 진짜 경로 점들
        for (int i = currentPathIndex; i < currentPath.Count; i++)
            if (!IsTemp(currentPath[i])) rest.Add(currentPath[i]);

        // 옆으로 비킨 점(p1) → 바로 목적 점이 보이면 끝, 안 보이면 앞으로 더 간 점(p2)까지 거쳐서 물체를 빙 돌아감
        for (float off = 0.3f; off <= DetourMaxOffset + 0.001f; off += 0.15f)
            foreach (float sign in new[] { 1f, -1f })
            {
                Vector3 p1 = pos + side * (sign * off);
                if (!Free(p1) || !Clear(pos, p1)) continue;

                if (Clear(p1, next.position)) { SetDetour(rest, next.position.y, p1); return true; }

                for (float fwd = 0.3f; fwd <= 2.0f; fwd += 0.3f)
                {
                    Vector3 p2 = p1 + dir * fwd;
                    if (!Free(p2) || !Clear(p1, p2)) break;          // 앞이 막혔으면 이 쪽은 포기
                    if (Clear(p2, next.position)) { SetDetour(rest, next.position.y, p1, p2); return true; }
                }
            }
        return false;
    }

    void SetDetour(List<Transform> rest, float y, params Vector3[] points)
    {
        ClearTempPoints();
        currentPath.Clear();
        foreach (var p in points)
        {
            var go = new GameObject("DetourPoint (임시)");
            go.transform.position = new Vector3(p.x, y, p.z);
            tempPoints.Add(go);
            currentPath.Add(go.transform);
        }
        currentPath.AddRange(rest);
        currentPathIndex = 0;
    }

    // 그 자리에 로봇이 설 수 있는지 (로봇 크기만큼 비어 있는지)
    bool Free(Vector3 p)
    {
        float h = Mathf.Max(RobotRadius + 0.03f, 0.15f);
        foreach (var c in Physics.OverlapSphere(p + Vector3.up * h, RobotRadius * 0.9f, ~0, QueryTriggerInteraction.Ignore))
        {
            if (c.transform.IsChildOf(transform)) continue;
            if (c.bounds.max.y < p.y + 0.05f) continue;   // 바닥
            return false;
        }
        return true;
    }

    // 가는 방향 바로 앞에 물체가 있는지 (바닥·로봇 자신은 무시)
    bool ObstacleAhead(Vector3 toTarget)
    {
        if (LookAheadDistance <= 0f) return false;
        float dist = Mathf.Min(LookAheadDistance, toTarget.magnitude);
        if (dist < 0.05f) return false;
        float h = Mathf.Max(RobotRadius + 0.03f, 0.15f);
        foreach (var hit in Physics.SphereCastAll(transform.position + Vector3.up * h, RobotRadius * 0.9f, toTarget.normalized, dist, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.distance <= 0f) continue;
            if (hit.transform.IsChildOf(transform)) continue;
            if (hit.normal.y > 0.7f) continue;
            return true;
        }
        return false;
    }

    Transform NextRealTarget()
    {
        for (int i = currentPathIndex; i < currentPath.Count; i++)
            if (!IsTemp(currentPath[i])) return currentPath[i];
        return null;
    }

    NodeData NodeOf(Transform t)
    {
        foreach (var n in Nodes) if (n.WayPoint == t) return n;
        return null;
    }

    bool IsTemp(Transform t) => t != null && tempPoints.Exists(g => g != null && g.transform == t);

    void ClearTempPoints()
    {
        foreach (var g in tempPoints) if (g != null) Destroy(g);
        tempPoints.Clear();
    }

    int PairKey(NodeData a, NodeData b)
    {
        int i = Nodes.IndexOf(a), j = Nodes.IndexOf(b);
        return Mathf.Min(i, j) * 10000 + Mathf.Max(i, j);
    }

    void BlockEdge(NodeData a, NodeData b) => blockedUntil[PairKey(a, b)] = Time.time + BlockedMemorySeconds;

    bool IsBlocked(NodeData a, NodeData b)
    {
        if (blockedUntil.Count == 0) return false;
        return blockedUntil.TryGetValue(PairKey(a, b), out float until) && Time.time < until;
    }

    // ------------------------------------------------------------------ 점 연결
    void FillNodesFromParent()
    {
        Nodes.RemoveAll(n => n == null || n.WayPoint == null);
        if (Nodes.Count > 0 || WaypointsParent == null) return;
        foreach (Transform c in WaypointsParent) Nodes.Add(new NodeData { Name = c.name, WayPoint = c });
        Debug.Log($"[MyLineFollower] {WaypointsParent.name}의 자식 {Nodes.Count}개로 경로 점 목록을 채웠어요.");
    }

    void BuildLinks()
    {
        foreach (var n in Nodes) n.links = new List<NodeData>();
        for (int i = 0; i < Nodes.Count; i++)
            for (int j = i + 1; j < Nodes.Count; j++)
            {
                NodeData a = Nodes[i], b = Nodes[j];
                if (a.WayPoint == null || b.WayPoint == null) continue;
                bool linked = AutoConnect
                    ? Clear(a.WayPoint.position, b.WayPoint.position)
                    : a.Neighbors.Contains(b.WayPoint) || b.Neighbors.Contains(a.WayPoint);
                if (linked) { a.links.Add(b); b.links.Add(a); }
            }
    }

    // 로봇 크기의 공을 굴려서 두 점 사이에 장애물이 없는지 검사 (로봇 자신은 무시)
    bool Clear(Vector3 a, Vector3 b)
    {
        float h = Mathf.Max(RobotRadius + 0.03f, 0.15f);
        Vector3 from = a + Vector3.up * h, dir = b - a; dir.y = 0f;
        float dist = dir.magnitude;
        if (dist < 1e-3f) return true;
        foreach (var hit in Physics.SphereCastAll(from, RobotRadius, dir / dist, dist, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.distance <= 0f) continue;                       // 시작부터 겹친 바닥 등
            if (hit.transform.IsChildOf(transform)) continue;      // 로봇 자신
            if (hit.normal.y > 0.7f) continue;                     // 바닥면
            return false;
        }
        return true;
    }

    bool Valid(int i) => Nodes != null && i >= 0 && i < Nodes.Count && Nodes[i] != null && Nodes[i].WayPoint != null;

    // ------------------------------------------------------------------ 경로 선
    void SetupLine()
    {
        var go = new GameObject("PathLine (MyLineFollower)");
        lineRenderer = go.AddComponent<LineRenderer>();
        lineRenderer.useWorldSpace = true;
        lineRenderer.widthMultiplier = LineWidth;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = lineRenderer.endColor = LineColor;
        lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lineRenderer.positionCount = 0;
    }

    void DrawFullPathLine()
    {
        if (lineRenderer == null || currentPath.Count == 0) return;
        float y = currentPath[0].position.y + LineOffsetY;
        lineRenderer.positionCount = currentPath.Count + 1;
        lineRenderer.SetPosition(0, new Vector3(transform.position.x, y, transform.position.z));
        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector3 p = currentPath[i].position;
            lineRenderer.SetPosition(i + 1, new Vector3(p.x, y, p.z));
        }
    }

    void OnGUI()
    {
        if (!ShowHelp) return;
        GUILayout.BeginArea(new Rect(10, Screen.height - 110, 330, 100), GUI.skin.box);
        GUILayout.Label("자동 주행 (MyLineFollower)");
        GUILayout.Label($"{AirconKey}: 에어컨   {LightSwitchKey}: 조명 스위치   {HomeKey}: 홈   {StopKey}: 정지");
        GUILayout.Label(status);
        GUILayout.EndArea();
    }

    // ------------------------------------------------------------------ Scene 화면 표시
    void OnDrawGizmos()
    {
        if (Nodes == null) return;
        if (!Application.isPlaying) BuildLinks();
        for (int i = 0; i < Nodes.Count; i++)
        {
            var n = Nodes[i];
            if (n == null || n.WayPoint == null) continue;
            Vector3 p = n.WayPoint.position;
            Gizmos.color = i == HomeNodeIndex ? Color.cyan : i == AirconNodeIndex ? Color.blue : i == LightSwitchNodeIndex ? Color.yellow : Color.white;
            Gizmos.DrawSphere(p, 0.05f);
            Gizmos.color = new Color(1f, 0.3f, 0.3f);
            if (n.links != null) foreach (var nb in n.links) Gizmos.DrawLine(p, nb.WayPoint.position);
#if UNITY_EDITOR
            string tag = i == HomeNodeIndex ? " (홈)" : i == AirconNodeIndex ? " (에어컨)" : i == LightSwitchNodeIndex ? " (스위치)" : "";
            Handles.Label(p + Vector3.up * 0.1f, i + ": " + n.WayPoint.name + tag);
#endif
        }
    }

#if UNITY_EDITOR
    // Inspector 버튼에서 사용: Scene 화면 가운데(바닥)에 새 경로 점 추가
    public void AddPointAtSceneView()
    {
        if (WaypointsParent == null)
        {
            WaypointsParent = new GameObject("Waypoints").transform;
            Undo.RegisterCreatedObjectUndo(WaypointsParent.gameObject, "Add waypoint");
        }
        Vector3 pos = transform.position;
        var sv = SceneView.lastActiveSceneView;
        if (sv != null && Physics.Raycast(sv.camera.transform.position, sv.camera.transform.forward, out var hit, 100f)) pos = hit.point;
        var p = new GameObject("Point_" + WaypointsParent.childCount).transform;
        p.position = pos; p.SetParent(WaypointsParent, true);
        Undo.RegisterCreatedObjectUndo(p.gameObject, "Add waypoint");
        Undo.RecordObject(this, "Add waypoint");
        Nodes.Add(new NodeData { Name = p.name, WayPoint = p });
        Selection.activeTransform = p;
    }

    public void RefillFromParent()
    {
        Undo.RecordObject(this, "Refill waypoints");
        Nodes.Clear();
        FillNodesFromParent();
    }
#endif
}

#if UNITY_EDITOR
[CustomEditor(typeof(MyLineFollower))]
public class MyLineFollowerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var f = (MyLineFollower)target;
        EditorGUILayout.HelpBox("1) [점 추가]로 경로 점을 만들고 Scene에서 원하는 곳(바닥)으로 옮기기\n2) 에어컨·스위치·홈 번호 입력\n3) Play → Z / X / C", MessageType.Info);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("➕ 점 추가 (Scene 화면 가운데)", GUILayout.Height(26))) f.AddPointAtSceneView();
        if (GUILayout.Button("🔄 목록 다시 채우기", GUILayout.Height(26))) f.RefillFromParent();
        GUILayout.EndHorizontal();
        DrawDefaultInspector();
    }
}
#endif
