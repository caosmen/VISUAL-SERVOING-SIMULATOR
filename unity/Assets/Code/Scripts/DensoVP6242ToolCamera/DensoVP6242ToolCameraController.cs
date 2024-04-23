using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DensoVP6242ToolCameraController : MonoBehaviour
{
    private Camera _cameraRGB;
    private Camera _cameraDepth;

    [Header("Behaviour Settings")]
    public float cameraToDepthTextureTick = 0.1f;

    [Header("Render Texture Settings")]
    public RenderTexture renderTextureRGB;
    public RenderTexture renderTextureDepth;

    [Header("Render Texture Settings")]
    public int renderTextureWidth = 1080;
    public int renderTextureHeight = 1080;

    [Header("Raycast Settings")]
    public int raycastPerFrame = 100;
    public float maxDistance = 5f;

    private float[,] _hitPoints;

    void Start()
    {
        _cameraRGB = transform.Find("CameraRGB").GetComponent<Camera>();
        _cameraDepth = transform.Find("CameraDepth").GetComponent<Camera>();

        renderTextureRGB.Release();
        renderTextureDepth.Release();
        
        renderTextureRGB.width = renderTextureWidth;
        renderTextureRGB.height = renderTextureHeight;

        renderTextureDepth.width = renderTextureWidth;
        renderTextureDepth.height = renderTextureHeight;

        renderTextureRGB.Create();
        renderTextureDepth.Create();

        _cameraRGB.targetTexture = renderTextureRGB;
        _cameraDepth.targetTexture = renderTextureDepth;

        _hitPoints = new float[renderTextureWidth, renderTextureHeight];

        InvokeRepeating(nameof(CameraToDepthTexture), 0, cameraToDepthTextureTick);
    }

    private void CameraToDepthTexture()
    {
        GetCameraDepthHitPoints();
    }

    private void GetCameraDepthHitPoints()
    {
        int totalPixels = renderTextureWidth * renderTextureHeight;
        int totalRaycasts = Mathf.Min(raycastPerFrame, totalPixels);
        
        int stepSize = Mathf.Max(1, (int) Mathf.Sqrt(totalPixels / totalRaycasts));
        
        System.Array.Clear(_hitPoints, 0, _hitPoints.Length);

        for (int x = 0; x < renderTextureWidth; x += stepSize)
        {
            for (int y = 0; y < renderTextureHeight; y += stepSize)
            {
                Ray ray = _cameraDepth.ScreenPointToRay(new Vector3(x, y, 0));
                if (Physics.Raycast(ray, out RaycastHit hit, maxDistance))
                {
                    _hitPoints[x, y] = hit.distance;
                }
                else
                {
                    _hitPoints[x, y] = maxDistance;
                }
                
                Debug.DrawRay(ray.origin, ray.direction * _hitPoints[x, y], Color.red);
            }
        }
    }
}
