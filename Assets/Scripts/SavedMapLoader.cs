// SavedMapLoader.cs
// 로봇이 SLAM으로 만들어 저장한 지도 파일(.pgm + .yaml)을
// Unity ▶ Play 를 누를 때마다 바닥에 바로 그려 주는 스크립트입니다.
// (로봇이 켜져 있지 않아도 지도는 나타나요)
//
// 색깔 뜻
//   검정  = 벽/장애물 (라이다가 막혀 있다고 본 곳)
//   빨강  = 갈 수 없는 곳 (빈 바닥이지만 벽에 너무 가까워서 로봇 몸이 못 들어감)
//   흰색  = 갈 수 있는 곳
//   회색  = 라이다가 아직 못 본 곳 (모름)
//
// 사용법
//   1) Assets 안에 "StreamingAssets" 폴더를 만들고 지도 파일 2개(.pgm, .yaml)를 넣기
//   2) MapOrigin(빈 오브젝트) 안에 Quad를 만들고 이 스크립트 붙이기
//   3) Map Name 칸에 파일 이름(확장자 빼고) 적기
//   4) ▶ Play

using System.Globalization;
using System.IO;
using UnityEngine;

public class SavedMapLoader : MonoBehaviour
{
    [Tooltip("StreamingAssets 폴더에 넣은 지도 파일 이름 (확장자 빼고)")]
    public string mapName = "lidar_test_01";

    [Tooltip("로봇 몸 반지름 (미터). 로봇 가장 넓은 폭의 절반 정도")]
    public float robotRadius = 0.15f;

    [Tooltip("바닥보다 살짝 위에 띄울 높이 (미터)")]
    public float height = 0.02f;

    void Start()
    {
        LoadMap();
    }

    void LoadMap()
    {
        string folder = Application.streamingAssetsPath;
        string yamlPath = Path.Combine(folder, mapName + ".yaml");
        string pgmPath = Path.Combine(folder, mapName + ".pgm");

        if (!File.Exists(yamlPath) || !File.Exists(pgmPath))
        {
            Debug.LogError("[SavedMap] 지도 파일을 못 찾았어요. 여기에 있어야 해요: " + yamlPath + " , " + pgmPath);
            return;
        }

        // ---------- 1. yaml 읽기 (한 칸 크기, 지도 시작점) ----------
        float res = 0.05f;
        float originX = 0f, originY = 0f;
        bool negate = false;
        foreach (string raw in File.ReadAllLines(yamlPath))
        {
            string line = raw.Trim();
            if (line.StartsWith("resolution:"))
            {
                res = ParseFloat(line.Substring("resolution:".Length));
            }
            else if (line.StartsWith("origin:"))
            {
                string inside = line.Substring("origin:".Length).Trim().Trim('[', ']');
                string[] parts = inside.Split(',');
                originX = ParseFloat(parts[0]);
                originY = ParseFloat(parts[1]);
            }
            else if (line.StartsWith("negate:"))
            {
                negate = line.Substring("negate:".Length).Trim() == "1";
            }
        }

        // ---------- 2. pgm 읽기 (지도 그림) ----------
        byte[] bytes = File.ReadAllBytes(pgmPath);
        int pos = 0;
        string magic = NextToken(bytes, ref pos);
        int w = int.Parse(NextToken(bytes, ref pos));
        int h = int.Parse(NextToken(bytes, ref pos));
        int maxVal = int.Parse(NextToken(bytes, ref pos));

        int[] values = new int[w * h];   // 파일 순서: 첫 줄 = 그림 맨 위
        if (magic == "P5")
        {
            pos++; // 숫자 뒤 공백 한 칸 건너뛰기
            for (int i = 0; i < w * h; i++) values[i] = bytes[pos + i];
        }
        else if (magic == "P2")
        {
            for (int i = 0; i < w * h; i++) values[i] = int.Parse(NextToken(bytes, ref pos));
        }
        else
        {
            Debug.LogError("[SavedMap] 이 pgm 형식(" + magic + ")은 못 읽어요.");
            return;
        }

        // ---------- 3. 칸마다 벽/빈곳/모름 나누기 ----------
        // 0 = 빈 곳, 1 = 모름, 2 = 벽   (텍스처 기준: y=0 이 맨 아래)
        int[] cell = new int[w * h];
        for (int row = 0; row < h; row++)
        {
            for (int col = 0; col < w; col++)
            {
                int v = values[row * w + col];
                if (maxVal != 255) v = v * 255 / maxVal;
                if (negate) v = 255 - v;

                int type;
                if (v <= 100) type = 2;        // 어두움 = 벽
                else if (v >= 250) type = 0;   // 거의 흰색 = 빈 곳
                else type = 1;                 // 중간 회색 = 모름

                int ty = h - 1 - row;          // 그림 위아래 뒤집기
                cell[ty * w + col] = type;
            }
        }

        // ---------- 4. 벽 주변 "로봇 몸이 못 들어가는 곳" 표시 ----------
        bool[] blocked = new bool[w * h];
        int r = Mathf.CeilToInt(robotRadius / res);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (cell[y * w + x] != 2) continue;
                for (int dy = -r; dy <= r; dy++)
                {
                    for (int dx = -r; dx <= r; dx++)
                    {
                        if (dx * dx + dy * dy > r * r) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                        if (cell[ny * w + nx] == 0) blocked[ny * w + nx] = true;
                    }
                }
            }
        }

        // ---------- 5. 색칠해서 그림 만들기 ----------
        Color32 wallColor = new Color32(0, 0, 0, 255);
        Color32 blockedColor = new Color32(240, 120, 120, 255);
        Color32 freeColor = new Color32(255, 255, 255, 255);
        Color32 unknownColor = new Color32(128, 128, 128, 255);

        Color32[] pixels = new Color32[w * h];
        for (int i = 0; i < pixels.Length; i++)
        {
            if (cell[i] == 2) pixels[i] = wallColor;
            else if (cell[i] == 1) pixels[i] = unknownColor;
            else if (blocked[i]) pixels[i] = blockedColor;
            else pixels[i] = freeColor;
        }

        Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
        tex.filterMode = FilterMode.Point;
        tex.wrapMode = TextureWrapMode.Clamp;
        tex.SetPixels32(pixels);
        tex.Apply();
        GetComponent<Renderer>().material.mainTexture = tex;

        // ---------- 6. 실제 크기와 위치 맞추기 ----------
        // ROS 좌표(x = 앞, y = 왼쪽)  →  Unity 좌표(z = 앞, -x = 왼쪽)
        // 지도 (0,0) = 로봇이 지도 만들기를 시작한 자리 = 부모(MapOrigin) 위치
        float sizeX = w * res;
        float sizeY = h * res;
        float centerRosX = originX + sizeX / 2f;
        float centerRosY = originY + sizeY / 2f;

        transform.localRotation = Quaternion.Euler(90f, -90f, 0f);
        transform.localScale = new Vector3(sizeX, sizeY, 1f);
        transform.localPosition = new Vector3(-centerRosY, height, centerRosX);

        Debug.Log($"[SavedMap] 지도 불러옴: {mapName} / {w} x {h} 칸 / 한 칸 {res} m / 실제 크기 {sizeX:F1} m x {sizeY:F1} m");
    }

    static float ParseFloat(string s)
    {
        return float.Parse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    // pgm 머리말에서 단어 하나 읽기 (# 으로 시작하는 설명 줄은 건너뜀)
    static string NextToken(byte[] b, ref int pos)
    {
        while (pos < b.Length)
        {
            char c = (char)b[pos];
            if (c == '#')
            {
                while (pos < b.Length && b[pos] != '\n') pos++;
            }
            else if (char.IsWhiteSpace(c))
            {
                pos++;
            }
            else break;
        }
        int start = pos;
        while (pos < b.Length && !char.IsWhiteSpace((char)b[pos])) pos++;
        return System.Text.Encoding.ASCII.GetString(b, start, pos - start);
    }
}
