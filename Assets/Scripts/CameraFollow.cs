using UnityEngine;

/// <summary>
/// 터틀봇을 따라다니는 카메라.
/// topDown = true 이면, 로봇 위 높은 곳에서 수직으로 내려다보는 방식이라
/// 좁은 방에서도 벽에 끼이지 않아 조작하기 훨씬 안정적입니다.
/// </summary>
public class CameraFollow : MonoBehaviour
{
    [Tooltip("따라다닐 대상 (turtlebot3_burger를 여기에 드래그하세요)")]
    public Transform target;

    [Header("모드 선택")]
    [Tooltip("체크하면 위에서 내려다보는 탑뷰, 체크 해제하면 뒤에서 따라가는 3인칭")]
    public bool topDown = true;

    [Header("탑뷰 설정")]
    [Tooltip("로봇 위로 얼마나 높이 띄울지 (미터). 방 전체가 보일 만큼 넉넉하게.")]
    public float topDownHeight = 5f;

    [Header("3인칭 설정 (topDown이 꺼졌을 때만 사용)")]
    public float distanceBehind = 0.3f;
    public float heightAbove = 0.3f;
    public float smoothTime = 0.15f;

    private Vector3 velocity = Vector3.zero;

    private void LateUpdate()
    {
        if (target == null) return;

        if (topDown)
        {
            // 로봇 바로 위, 수직으로 내려다보기
            Vector3 desiredPosition = new Vector3(target.position.x,
                                                   target.position.y + topDownHeight,
                                                   target.position.z);
            transform.position = desiredPosition;
            transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        }
        else
        {
            Vector3 desiredPosition = target.position
                                       - target.forward * distanceBehind
                                       + Vector3.up * heightAbove;

            transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref velocity, smoothTime);
            transform.LookAt(target.position + Vector3.up * 0.1f);
        }
    }
}
