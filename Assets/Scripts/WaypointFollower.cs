using UnityEngine;

public class WaypointFollower : MonoBehaviour
{
    // 유니티 화면에서 목적지를 선택할 수 있게 해주는 목록(목차)입니다.
    public enum TargetDestination
    {
        Aircon,         // 에어컨
        LightSwitch     // 조명 스위치
    }

    [Header("▼ 어디로 갈지 여기서 선택하세요 ▼")]
    public TargetDestination currentTarget;

    [Header("1. 에어컨 경로 점들")]
    public Transform[] airconWaypoints;

    [Header("2. 조명 스위치 경로 점들")]
    public Transform[] lightSwitchWaypoints;

    [Header("로봇 속도")]
    public float moveSpeed = 0.5f; // 전진 속도
    public float turnSpeed = 2.0f; // 회전 속도

    // 로봇이 실제로 따라갈 경로를 담아둘 임시 가방
    private Transform[] activeWaypoints;
    private int currentIndex = 0;

    void Start()
    {
        // 플레이(▶) 버튼을 눌렀을 때, 선택한 목적지에 맞춰 경로를 꺼내옵니다.
        if (currentTarget == TargetDestination.Aircon)
        {
            activeWaypoints = airconWaypoints;
        }
        else if (currentTarget == TargetDestination.LightSwitch)
        {
            activeWaypoints = lightSwitchWaypoints;
        }
    }

    void Update()
    {
        // 경로가 비어있거나 다 돌았으면 멈춤!
        if (activeWaypoints == null || currentIndex >= activeWaypoints.Length)
        {
            return;
        }

        // 현재 향해야 할 목표점
        Transform target = activeWaypoints[currentIndex];

        // 방향 계산 (y축 잠금)
        Vector3 direction = target.position - transform.position;
        direction.y = 0;

        // 목표점에 거의 도착했으면 다음 점으로 타겟 변경
        if (direction.magnitude < 0.2f)
        {
            currentIndex++;
            return;
        }

        // 회전하기
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);

        // 전진하기
        transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime, Space.Self);
    }
}