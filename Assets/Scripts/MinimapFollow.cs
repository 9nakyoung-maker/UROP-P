using UnityEngine;

/// <summary>
/// 미니맵용 카메라를 로봇 위에서 수직으로 따라다니게 하는 스크립트.
/// 메인 카메라(CameraFollow)와는 별개로, Target Texture를 가진
/// 별도의 "MinimapCamera" 오브젝트에 붙입니다.
/// </summary>
public class MinimapFollow : MonoBehaviour
{
    [Tooltip("따라다닐 대상 (turtlebot3_burger를 드래그하세요)")]
    public Transform target;

    [Tooltip("로봇 위로 얼마나 높이 띄울지 (미터). 방 전체가 보이도록 넉넉하게.")]
    public float height = 8f;

    [Tooltip("켜두면 로봇이 회전할 때 미니맵도 같이 회전해서,\n항상 '위쪽 = 로봇이 보는 방향'이 됩니다.\n꺼두면 미니맵은 고정된 방향(북쪽 고정)을 유지합니다.")]
    public bool rotateWithRobot = false;

    private void LateUpdate()
    {
        if (target == null) return;

        transform.position = new Vector3(target.position.x, target.position.y + height, target.position.z);

        if (rotateWithRobot)
        {
            transform.rotation = Quaternion.Euler(90f, target.eulerAngles.y, 0f);
        }
        else
        {
            transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        }
    }
}
