using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.InputSystem;

// =====================================================================================
//  NavMeshAutoDriver — 방 전체를 길로 쓰는 자율 주행
//  - 방 바닥 전체(NavMesh)에서 고정 목적지 3개(에어컨·스위치·홈)까지 최단 경로를 스스로 계산
//  - 앞 감지 센서(카메라·라이다 역할)로 길을 막은 물체를 보면
//      ① 그 물체를 지도에 '못 가는 곳'으로 표시  ② 살짝 뒤로 물러나서  ③ 다른 길로 다시 계산
//  - 실물 LanderPi처럼 제자리 회전 → 직진, 0.3 m/s
//  - Z: 에어컨   X: 조명 스위치   C: 홈   V: 정지
//
//  필요: Package Manager → AI Navigation 설치, 바닥에 NavMesh Surface → Bake
//  붙이는 곳: 로봇 맨 위 오브젝트 (rosmentor). 같은 키를 쓰는 MyLineFollower는 자동으로 꺼짐
// =====================================================================================
public class NavMeshAutoDriver : MonoBehaviour
{
    [Header("고정 목적지 3개 (Point_3 / Point_2 / Point_1 을 끌어다 놓기)")]
    public Transform AirconPoint;
    public Transform LightSwitchPoint;
    public Transform HomePoint;
    public Key AirconKey = Key.Z;
    public Key LightSwitchKey = Key.X;
    public Key HomeKey = Key.C;
    public Key StopKey = Key.V;

    [Header("이동 (실물 LanderPi 기준)")]
    public float MoveSpeed = 0.3f;          // m/s
    public float TurnSpeed = 90f;           // 도/초
    [Tooltip("방향이 이 각도 넘게 틀어져 있으면 제자리에서 먼저 회전")]
    public float TurnInPlaceAngle = 30f;
    public float ArrivalDistance = 0.15f;
    [Tooltip("로봇이 옆이나 뒤를 보고 달리면 90 / -90 / 180 으로")]
    public float ForwardAngleOffset = 0f;
    public bool StartAtHome = true;
    [Tooltip("목적지 점이 벽에 붙어 있어도, 이 높이 차 안의 가장 가까운 바닥으로 갑니다")]
    public float FloorTolerance = 0.4f;

    [Header("앞 감지 센서 (카메라·라이다 역할)")]
    [Tooltip("가는 방향으로 이 거리(m) 안에 물체가 보이면 멈추고 피해 감")]
    public float SensorRange = 0.5f;
    [Tooltip("센서 굵기(m). NavMesh Agent Radius보다 조금 작게")]
    public float SensorRadius = 0.12f;
    public float SensorHeight = 0.15f;
    [Tooltip("이 시간(초) 동안 거의 못 움직이면 막힌 걸로 봄")]
    public float StuckSeconds = 1.5f;
    [Tooltip("막히면 이만큼(m) 뒤로 물러난 뒤 다시 계산")]
    public float BackUpDistance = 0.15f;
    [Tooltip("한 번 이동에서 피하기를 최대 몇 번까지")]
    public int MaxReplans = 8;
    [Header("빠져나오기 (회전이 안 되거나 끼었을 때)")]
    [Tooltip("목표가 뒤쪽에 있고 가까우면, 돌지 않고 후진으로 감")]
    public bool AllowReverse = true;
    [Tooltip("이 거리(m) 이내의 점까지만 후진 주행 (회전이 막혔을 땐 거리 상관없이 후진)")]
    public float ReverseMaxDistance = 1.0f;
    [Tooltip("메카넘 휠: 옆으로 비켜서 빠져나오기")]
    public bool AllowStrafe = true;
    [Tooltip("옆으로 비키는 거리(m)")]
    public float StrafeDistance = 0.2f;
    [Tooltip("막은 물체를 지도에 '못 가는 곳'으로 표시 (NavMesh에 구멍)")]
    public bool MarkObstacles = true;
    [Tooltip("이보다 큰 물체(벽·바닥 같은 방 구조)는 표시하지 않음 (m)")]
    public float MaxObstacleSize = 2.5f;

    [Header("실제 로봇 연동")]
    [Tooltip("자율 주행 움직임을 실제 로봇(cmd_vel)으로도 보냄. 같은 오브젝트의 CmdVelKeyboardPublisher가 대신 보내 줌")]
    public bool DriveRealRobot = true;

    /// <summary>자율 주행 중인지 (실제 로봇으로 보낼지 판단용)</summary>
    public bool IsDriving => isMoving;
    /// <summary>지금 내리는 속도 명령 (ROS 기준): x = 앞(m/s), y = 왼쪽(m/s), z = 왼쪽 회전(rad/s)</summary>
    public Vector3 Cmd { get; private set; }

    [Header("경로 선 / 화면 표시")]
    public Color LineColor = Color.red;
    public float LineWidth = 0.03f;
    public bool ShowHelp = true;

    readonly List<Vector3> corners = new List<Vector3>();
    readonly List<NavMeshObstacle> marked = new List<NavMeshObstacle>();
    int cornerIndex, replanCount;
    bool isMoving, busy;
    Transform goal;
    string goalName = "";
    Vector3 progressPos; float progressTime, progressYaw;
    Vector3 lastTravelDir = Vector3.forward;   // 마지막으로 달리던 방향 (후진할 때 반대로)
    bool preferReverse;                        // 회전이 막혀서 다음 점까지는 후진으로
    float lastMoved, lastTurned;               // 빠져나오기 동작에서 실제로 움직인 거리·각도
    NavMeshPath navPath;
    LineRenderer line;
    string status = "대기 중";

    void Start()
    {
        navPath = new NavMeshPath();
        SetupLine();
        lastTravelDir = TravelForward();

        var old = GetComponent("MyLineFollower") as Behaviour;   // 같은 Z/X/C 키를 쓰므로 끔
        if (old != null && old.enabled) { old.enabled = false; Debug.Log("[NavMeshAutoDriver] 키가 겹쳐서 MyLineFollower를 껐어요."); }

        if (StartAtHome && HomePoint != null)
        {
            Vector3 p = HomePoint.position;
            transform.position = new Vector3(p.x, transform.position.y, p.z);
        }
        if (!NavMesh.SamplePosition(transform.position, out _, 2f, NavMesh.AllAreas))
        {
            status = "NavMesh가 없어요 — 바닥에 NavMesh Surface를 붙이고 Bake 하세요";
            Debug.LogError("[NavMeshAutoDriver] " + status);
        }
    }

    // ------------------------------------------------------------------ 입력
    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null) return;
        if (kb[AirconKey].wasPressedThisFrame) GoTo(AirconPoint, "에어컨");
        else if (kb[LightSwitchKey].wasPressedThisFrame) GoTo(LightSwitchPoint, "조명 스위치");
        else if (kb[HomeKey].wasPressedThisFrame) GoTo(HomePoint, "홈");
        else if (kb[StopKey].wasPressedThisFrame) Stop("정지");
    }

    public void GoTo(Transform target, string name)
    {
        if (target == null) { Debug.LogError($"[NavMeshAutoDriver] '{name}' 지점이 비어 있어요. Inspector에 점을 끌어다 놓으세요."); return; }
        StopAllCoroutines(); busy = false;
        goal = target; goalName = name; replanCount = 0; preferReverse = false;
        if (Plan()) { isMoving = true; status = "이동 중 → " + name; }
        else Stop($"{name}까지 갈 수 있는 길이 없어요 (NavMesh Bake, 점 위치 확인)");
    }

    // 지금 위치 → 목적지 최단 경로 (NavMesh가 벽·가구를 알아서 피해 줌)
    bool Plan()
    {
        // 로봇 기준점이 바닥보다 높거나 낮아도 되도록, 로봇 근처의 가장 가까운 NavMesh를 먼저 찾음
        if (!NavMesh.SamplePosition(transform.position, out NavMeshHit sh, 3f, NavMesh.AllAreas))
        {
            Debug.LogError("[NavMeshAutoDriver] 로봇 주변 3m 안에 NavMesh(파란 바닥)가 없어요. NavMesh Surface를 Bake 했는지 확인하세요.");
            return false;
        }
        Vector3 start = sh.position;
        float floorY = start.y;                     // 실제 바닥 높이
        Vector3 g = goal.position; g.y = floorY;    // 벽에 붙은 스위치·에어컨 → 그 아래 바닥 기준
        if (!Sample(g, floorY, out Vector3 end))
        {
            Debug.LogError($"[NavMeshAutoDriver] '{goal.name}' 근처 바닥에 NavMesh가 없어요. 점을 파란 바닥 위(또는 가까이)로 옮기세요.", goal);
            return false;
        }

        if (!NavMesh.CalculatePath(start, end, NavMesh.AllAreas, navPath) || navPath.status == NavMeshPathStatus.PathInvalid)
        {
            Debug.LogError($"[NavMeshAutoDriver] 로봇 위치와 '{goal.name}'이(가) 파란 바닥으로 이어져 있지 않아요. Agent Radius를 줄이고 다시 Bake 해 보세요.", goal);
            return false;
        }
        if (navPath.status == NavMeshPathStatus.PathPartial)
            Debug.LogWarning($"[NavMeshAutoDriver] {goalName}까지 완전히 이어진 길이 없어서, 갈 수 있는 데까지만 가요.");

        corners.Clear(); corners.AddRange(navPath.corners);
        // [0]은 로봇에서 가장 가까운 파란 바닥 점.
        // 로봇이 파란 바닥 안이면 [0]을 건너뛰고, 밖(장애물 근처 색 없는 곳)이면 먼저 [0]으로 돌아간 뒤 출발
        Vector3 off = start - transform.position; off.y = 0f;
        bool offMesh = off.magnitude > 0.05f;
        cornerIndex = (corners.Count > 1 && !offMesh) ? 1 : 0;
        if (offMesh)
        {
            preferReverse = true;   // 보통 장애물을 마주 본 채 갇히므로, 돌지 않고 후진으로 빠져나옴
            Debug.Log($"[NavMeshAutoDriver] 파란 바닥 밖({off.magnitude * 100f:0}cm 떨어짐)이라 가장 가까운 파란 바닥으로 먼저 돌아갈게요.");
        }
        progressPos = transform.position; progressTime = Time.time; progressYaw = transform.eulerAngles.y;
        DrawLine();
        return true;
    }

    // 점 근처의 '바닥' NavMesh 위치 찾기 (책상 위 같은 다른 높이는 제외)
    bool Sample(Vector3 p, float floorY, out Vector3 result)
    {
        foreach (float r in new[] { 0.3f, 0.6f, 1.0f, 1.5f, 2.5f })
            if (NavMesh.SamplePosition(p, out NavMeshHit hit, r, NavMesh.AllAreas) && Mathf.Abs(hit.position.y - floorY) < FloorTolerance)
            { result = hit.position; return true; }
        result = p;
        return false;
    }

    // ------------------------------------------------------------------ 이동
    void FixedUpdate()
    {
        if (!busy) Cmd = Vector3.zero;   // 이번 주기에 움직이지 않으면 정지 명령
        if (!isMoving || busy) return;
        if (cornerIndex >= corners.Count) { Arrive(); return; }

        float dt = Time.fixedDeltaTime;
        Vector3 to = corners[cornerIndex] - transform.position; to.y = 0f;
        float dist = to.magnitude;
        bool last = cornerIndex == corners.Count - 1;
        if (dist < (last ? ArrivalDistance : Mathf.Max(ArrivalDistance * 0.5f, 0.05f))) { cornerIndex++; preferReverse = false; return; }
        Vector3 dir = to / dist;

        // 앞으로 갈지 후진으로 갈지: 목표가 뒤쪽이고 가까우면(또는 회전이 막혔으면) 돌지 않고 후진
        float travelYaw = Quaternion.LookRotation(dir).eulerAngles.y;
        float errF = Mathf.DeltaAngle(transform.eulerAngles.y, travelYaw + ForwardAngleOffset);
        float errR = Mathf.DeltaAngle(transform.eulerAngles.y, travelYaw + ForwardAngleOffset + 180f);
        bool reverse = AllowReverse && Mathf.Abs(errR) + 20f < Mathf.Abs(errF) && (preferReverse || dist <= ReverseMaxDistance);
        float yawErr = reverse ? errR : errF;

        // 회전량
        float yawStep = Mathf.Clamp(yawErr, -TurnSpeed * dt, TurnSpeed * dt);

        // 방향이 맞으면 가는 쪽을 보고 → 비어 있으면 이동, 막혀 있으면 피하기
        Vector3 delta = Vector3.zero;
        if (Mathf.Abs(yawErr) < TurnInPlaceAngle)
        {
            if (Sense(dir, Mathf.Min(SensorRange, dist), out RaycastHit hit))
            {
                StartCoroutine(Recover(hit.collider, $"앞에 '{hit.collider.name}'"));
                return;
            }
            float speed = reverse ? MoveSpeed * 0.6f : MoveSpeed;   // 후진은 천천히
            delta = dir * Mathf.Min(speed * dt, dist);
            lastTravelDir = dir;
        }
        Move(delta, yawStep, dt);
        status = (reverse ? "후진으로 이동 중 → " : "이동 중 → ") + goalName;

        // 막힘 감지: 위치도 각도도 거의 안 변하면 (회전이 벽에 걸린 경우 포함)
        bool progressed = (transform.position - progressPos).sqrMagnitude > 0.0004f
                       || Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, progressYaw)) > 3f;
        if (progressed) { progressPos = transform.position; progressYaw = transform.eulerAngles.y; progressTime = Time.time; }
        else if (Time.time - progressTime > StuckSeconds)
            StartCoroutine(Recover(FindBlocker(), Mathf.Abs(yawErr) >= TurnInPlaceAngle ? "회전이 막힘" : "움직이지 못함"));
    }

    /// <summary>실제 로봇 라이다가 장애물을 봐서 멈췄을 때 (CmdVelKeyboardPublisher가 부름)</summary>
    public void StopByRealObstacle(string why)
    {
        if (isMoving) Stop($"실제 로봇: {why} → 자율 주행 멈춤");
    }

    // Unity 모델을 움직이고, 같은 움직임을 실제 로봇용 속도 명령(ROS 기준)으로 바꿔 둠
    void Move(Vector3 worldDelta, float yawDeg, float dt)
    {
        Vector3 fwd = TravelForward();
        Vector3 right = Vector3.Cross(Vector3.up, fwd);
        if (dt > 0f)
        {
            Vector3 v = worldDelta / dt;
            // ROS: x 앞 +, y 왼쪽 +, z 반시계(왼쪽) 회전 +   /   Unity: Y축 + 회전 = 오른쪽(시계)
            Cmd = new Vector3(Vector3.Dot(v, fwd), -Vector3.Dot(v, right), -yawDeg / dt * Mathf.Deg2Rad);
        }
        if (yawDeg != 0f) transform.Rotate(0f, yawDeg, 0f, Space.World);
        if (worldDelta != Vector3.zero) transform.position += worldDelta;
    }

    void Arrive()
    {
        Cmd = Vector3.zero;
        isMoving = false;
        status = $"도착 ✓ {goalName}";
        Debug.Log($"[NavMeshAutoDriver] {goalName}에 도착했어요!");
    }

    void Stop(string why)
    {
        StopAllCoroutines();
        Cmd = Vector3.zero;
        isMoving = false; busy = false; status = why;
        if (line != null) line.positionCount = 0;
        Debug.LogWarning("[NavMeshAutoDriver] " + why);
    }

    // ------------------------------------------------------------------ 막혔을 때: 표시 → 물러나기 → 다시 계산
    IEnumerator Recover(Collider seen, string why)
    {
        busy = true;
        replanCount++;
        if (replanCount > MaxReplans) { Stop($"{why} — {MaxReplans}번 피해 봤지만 길이 없어요. 물체를 치우거나 점 위치를 확인하세요"); yield break; }

        status = $"막힘: {why} → 다시 계산 ({replanCount}/{MaxReplans})";
        Debug.LogWarning("[NavMeshAutoDriver] " + status, seen);

        // ① 막은 물체를 지도에서 '못 가는 곳'으로 (NavMesh에 구멍을 뚫음)
        if (MarkObstacles) MarkAsObstacle(seen);

        // ② 빠져나오기: 후진 → 제자리 회전 → (안 되면) 후진하며 돌기 → 옆으로 비키기 → 그래도 안 되면 후진 주행
        yield return Escape();

        // ③ 구멍이 지도에 반영될 시간을 준 뒤 다시 계산
        Cmd = Vector3.zero;
        yield return new WaitForSeconds(0.25f);
        busy = false;
        if (Plan()) status = $"다른 길로 이동 중 → {goalName} ({replanCount}/{MaxReplans})";
        else Stop($"{why} — 돌아갈 길이 없어요");
    }

    // 끼었을 때 단계별로 빠져나오기. 각 동작은 실제로 움직였는지 재 보고, 막히면 바로 다음 방법으로 넘어감
    IEnumerator Escape()
    {
        Vector3 back = -lastTravelDir;

        // 1) 후진. 뒤도 막혀 있으면 옆으로 비키기
        status = "빠져나오는 중: 후진";
        yield return Drive(back, BackUpDistance, 0f, 0f, 2f);
        if (lastMoved < 0.03f && AllowStrafe)
        {
            status = "빠져나오는 중: 뒤도 막혀서 옆으로";
            yield return Drive(FreerSide(), StrafeDistance, 0f, 0f, 2f);
        }

        // 2) 가려는 방향으로 제자리 회전
        if (!DesiredYawErr(out float err) || Mathf.Abs(err) < TurnInPlaceAngle) yield break;
        status = "빠져나오는 중: 제자리 회전";
        yield return Drive(Vector3.zero, 0f, Mathf.Sign(err) * TurnSpeed, Mathf.Abs(err), 3f);
        if (DesiredYawErr(out err) && Mathf.Abs(err) < TurnInPlaceAngle) yield break;

        // 3) 제자리에서 못 돌면 → 후진하면서 돌기 (3점 턴) 후 다시 회전
        Debug.Log($"[NavMeshAutoDriver] 제자리에서 {lastTurned:0}°밖에 못 돌았어요 → 후진하면서 돌기");
        status = "빠져나오는 중: 후진하며 돌기";
        yield return Drive(back, BackUpDistance * 1.5f, Mathf.Sign(err) * TurnSpeed * 0.7f, 0f, 2.5f);
        if (DesiredYawErr(out err) && Mathf.Abs(err) >= TurnInPlaceAngle)
            yield return Drive(Vector3.zero, 0f, Mathf.Sign(err) * TurnSpeed, Mathf.Abs(err), 2f);
        if (DesiredYawErr(out err) && Mathf.Abs(err) < TurnInPlaceAngle) yield break;

        // 4) 옆으로 비킨 뒤 다시 회전 (메카넘)
        if (AllowStrafe)
        {
            status = "빠져나오는 중: 옆으로 비켜서 회전";
            yield return Drive(FreerSide(), StrafeDistance, 0f, 0f, 2f);
            if (DesiredYawErr(out err) && Mathf.Abs(err) >= TurnInPlaceAngle)
                yield return Drive(Vector3.zero, 0f, Mathf.Sign(err) * TurnSpeed, Mathf.Abs(err), 2f);
            if (DesiredYawErr(out err) && Mathf.Abs(err) < TurnInPlaceAngle) yield break;
        }

        // 5) 그래도 못 돌면 다음 점까지는 후진으로 따라가기
        if (AllowReverse)
        {
            preferReverse = true;
            Debug.Log("[NavMeshAutoDriver] 회전할 공간이 없어서 다음 점까지 후진으로 갈게요.");
        }
    }

    // 정해진 방향으로 distance(m)만큼 이동하면서 yawRate(도/초)로 회전. turnLimit(도)만큼 돌면 멈춤.
    // 0.3초 동안 위치·각도가 안 바뀌면(벽에 막혀 되돌려지면) 포기. 결과는 lastMoved, lastTurned.
    IEnumerator Drive(Vector3 dir, float distance, float yawRate, float turnLimit, float maxTime)
    {
        Vector3 startPos = transform.position; float startYaw = transform.eulerAngles.y;
        float t = 0f, stuckT = 0f;
        dir.y = 0f; if (dir.sqrMagnitude > 1e-6f) dir.Normalize();
        float speed = MoveSpeed * 0.5f;

        while (t < maxTime)
        {
            Vector3 flat = transform.position - startPos; flat.y = 0f;
            float turned = Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, startYaw));
            if (distance > 0f && flat.magnitude >= distance) break;
            if (distance <= 0f && turnLimit > 0f && turned >= turnLimit) break;

            float dt = Time.fixedDeltaTime;
            Vector3 p0 = transform.position; float y0 = transform.eulerAngles.y;
            float step = yawRate * dt;
            if (yawRate != 0f && turnLimit > 0f) step = Mathf.Clamp(step, -(turnLimit - turned), turnLimit - turned);
            Move(distance > 0f ? dir * speed * dt : Vector3.zero, step, dt);
            yield return new WaitForFixedUpdate();
            t += dt;

            bool moved = (transform.position - p0).sqrMagnitude > 1e-7f || Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, y0)) > 0.05f;
            stuckT = moved ? 0f : stuckT + dt;
            if (stuckT > 0.3f) break;
        }
        Cmd = Vector3.zero;
        Vector3 d = transform.position - startPos; d.y = 0f;
        lastMoved = d.magnitude;
        lastTurned = Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, startYaw));
    }

    // 지금 가야 할 점 방향까지 남은 회전 각도
    bool DesiredYawErr(out float err)
    {
        err = 0f;
        Vector3 target;
        if (cornerIndex < corners.Count) target = corners[cornerIndex];
        else if (goal != null) target = goal.position;
        else return false;
        Vector3 to = target - transform.position; to.y = 0f;
        if (to.sqrMagnitude < 1e-4f) return false;
        err = Mathf.DeltaAngle(transform.eulerAngles.y, Quaternion.LookRotation(to).eulerAngles.y + ForwardAngleOffset);
        return true;
    }

    // 왼쪽·오른쪽 중 더 비어 있는 쪽
    Vector3 FreerSide()
    {
        Vector3 right = Vector3.Cross(Vector3.up, lastTravelDir).normalized;
        return FreeDistance(right) >= FreeDistance(-right) ? right : -right;
    }

    float FreeDistance(Vector3 dir)
    {
        const float range = 0.8f;
        float best = range;
        Vector3 from = transform.position + Vector3.up * SensorHeight;
        foreach (var hit in Physics.SphereCastAll(from, SensorRadius, dir, range, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.transform.IsChildOf(transform) || hit.normal.y > 0.7f) continue;
            if (hit.distance <= 0f) return 0f;
            best = Mathf.Min(best, hit.distance);
        }
        return best;
    }

    // 로봇이 실제로 달리는 방향
    Vector3 TravelForward() => Quaternion.Euler(0f, transform.eulerAngles.y - ForwardAngleOffset, 0f) * Vector3.forward;

    // 앞 감지 센서: 로봇 크기 정도의 공을 앞으로 굴려서 부딪히는 물체 찾기 (바닥·로봇 자신 제외)
    bool Sense(Vector3 dir, float range, out RaycastHit nearest)
    {
        nearest = default;
        if (range < 0.03f) return false;
        bool found = false; float best = float.MaxValue;
        Vector3 from = transform.position + Vector3.up * SensorHeight;
        foreach (var hit in Physics.SphereCastAll(from, SensorRadius, dir, range, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.distance <= 0f) continue;
            if (hit.transform.IsChildOf(transform)) continue;
            if (hit.normal.y > 0.7f) continue;
            if (hit.distance < best) { best = hit.distance; nearest = hit; found = true; }
        }
        return found;
    }

    // 몸이 걸렸을 때 바로 앞에 붙어 있는 물체 찾기
    Collider FindBlocker()
    {
        Vector3 c = transform.position + lastTravelDir * (SensorRadius + 0.1f) + Vector3.up * SensorHeight;
        Collider best = null; float bestD = float.MaxValue;
        foreach (var col in Physics.OverlapSphere(c, SensorRadius * 1.5f, ~0, QueryTriggerInteraction.Ignore))
        {
            if (col.transform.IsChildOf(transform)) continue;
            if (col.bounds.max.y < transform.position.y + 0.05f) continue;   // 바닥
            float d = (col.ClosestPoint(c) - c).sqrMagnitude;
            if (d < bestD) { bestD = d; best = col; }
        }
        return best;
    }

    void MarkAsObstacle(Collider c)
    {
        if (c == null) return;
        GameObject go = c.attachedRigidbody != null ? c.attachedRigidbody.gameObject : c.gameObject;
        if (go.transform.IsChildOf(transform)) return;
        Bounds b = c.bounds;
        if (b.size.x > MaxObstacleSize || b.size.z > MaxObstacleSize)
        {
            Debug.Log($"[NavMeshAutoDriver] '{go.name}'은(는) 커서 방 구조(벽·바닥)로 보고 표시하지 않았어요.", go);
            return;
        }
        if (go.GetComponent<NavMeshObstacle>() != null) return;

        var ob = go.AddComponent<NavMeshObstacle>();
        ob.shape = NavMeshObstacleShape.Box;
        Vector3 s = go.transform.lossyScale;
        ob.center = go.transform.InverseTransformPoint(b.center);
        ob.size = new Vector3(b.size.x / Mathf.Max(Mathf.Abs(s.x), 1e-4f),
                              b.size.y / Mathf.Max(Mathf.Abs(s.y), 1e-4f),
                              b.size.z / Mathf.Max(Mathf.Abs(s.z), 1e-4f));
        ob.carving = true;
        ob.carveOnlyStationary = false;
        ob.carvingMoveThreshold = 0.05f;
        ob.carvingTimeToStationary = 0f;
        marked.Add(ob);
        Debug.Log($"[NavMeshAutoDriver] '{go.name}'을(를) 장애물로 지도에 표시했어요.", go);
    }

    [ContextMenu("표시한 장애물 모두 지우기")]
    public void ClearMarkedObstacles()
    {
        foreach (var ob in marked) if (ob != null) Destroy(ob);
        marked.Clear();
        Debug.Log("[NavMeshAutoDriver] 표시한 장애물을 지웠어요.");
    }

    // ------------------------------------------------------------------ 경로 선 / 화면
    void SetupLine()
    {
        var go = new GameObject("PathLine (NavMeshAutoDriver)");
        line = go.AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.widthMultiplier = LineWidth;
        line.material = new Material(Shader.Find("Sprites/Default"));
        line.startColor = line.endColor = LineColor;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.positionCount = 0;
    }

    void DrawLine()
    {
        if (line == null) return;
        line.positionCount = corners.Count;
        for (int i = 0; i < corners.Count; i++) line.SetPosition(i, corners[i] + Vector3.up * 0.02f);
    }

    void OnGUI()
    {
        if (!ShowHelp) return;
        GUILayout.BeginArea(new Rect(10, Screen.height - 110, 380, 100), GUI.skin.box);
        GUILayout.Label("자율 주행 (NavMeshAutoDriver)");
        GUILayout.Label($"{AirconKey}: 에어컨   {LightSwitchKey}: 조명 스위치   {HomeKey}: 홈   {StopKey}: 정지");
        GUILayout.Label(status);
        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        DrawPoint(AirconPoint, Color.blue);
        DrawPoint(LightSwitchPoint, Color.yellow);
        DrawPoint(HomePoint, Color.cyan);
        // 센서 범위
        Gizmos.color = new Color(0f, 1f, 0.4f, 0.6f);
        Vector3 from = transform.position + Vector3.up * SensorHeight;
        Vector3 fwd = Quaternion.Euler(0f, transform.eulerAngles.y - ForwardAngleOffset, 0f) * Vector3.forward;
        Gizmos.DrawWireSphere(from + fwd * SensorRange, SensorRadius);
        Gizmos.DrawLine(from, from + fwd * SensorRange);
    }

    static void DrawPoint(Transform t, Color c)
    {
        if (t == null) return;
        Gizmos.color = c;
        Gizmos.DrawSphere(t.position, 0.08f);
    }
}
