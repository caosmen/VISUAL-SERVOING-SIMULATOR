using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class PublisherVP6242ToolCameraController : MonoBehaviour
{
    private ROSConnection rosConnection;

    [Header("Camera")]
    public Camera cameraRGB;
    public Camera cameraDepth;

    [Header("ROS Topics Names")]
    public string topicCameraImageName = "/vp6242/camera/image";
    public string topicCameraDepthName = "/vp6242/camera/depth";

    [Header("ROS Publish Frequency")]
    public float publishMessageFrequency = 0.5f;

    private float timeElapsed;

    private Texture2D textureRGB;
    private Texture2D textureDepth;

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();

        rosConnection.RegisterPublisher<ImageMsg>(topicCameraImageName);
        rosConnection.RegisterPublisher<ImageMsg>(topicCameraDepthName);

        InitializeTextures();
    }

    private void InitializeTextures()
    {
        if (cameraRGB && cameraRGB.targetTexture)
        {
            textureRGB = new Texture2D(cameraRGB.targetTexture.width, cameraRGB.targetTexture.height);
        }
        if (cameraDepth && cameraDepth.targetTexture)
        {
            textureDepth = new Texture2D(cameraDepth.targetTexture.width, cameraDepth.targetTexture.height);
        }
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            if (textureRGB && textureDepth)
            {
                PublishRGBImage();
                PublishDepthImage();

                timeElapsed = 0;
            }
        }
    }

    private HeaderMsg InitializeHeaderMsg()
    {
        int currentSeconds = (int) Time.time;
        uint currentNanoseconds = (uint) ((Time.time - currentSeconds) * 1e9);

        return new HeaderMsg
        {
            stamp = new TimeMsg
            {
                sec = currentSeconds,
                nanosec = currentNanoseconds
            },
            frame_id = "vp6242"
        };
    }

    private void PublishRGBImage()
    {
        RenderTexture.active = cameraRGB.targetTexture;

        textureRGB.ReadPixels(new Rect(0, 0, textureRGB.width, textureRGB.height), 0, 0);
        textureRGB.Apply();

        RenderTexture.active = null;

        ImageMsg imageMsg = textureRGB.ToImageMsg(InitializeHeaderMsg());

        rosConnection.Publish(topicCameraImageName, imageMsg);
    }

    private void PublishDepthImage()
    {
        RenderTexture.active = cameraDepth.targetTexture;

        textureDepth.ReadPixels(new Rect(0, 0, textureDepth.width, textureDepth.height), 0, 0);
        textureDepth.Apply();

        RenderTexture.active = null;

        ImageMsg imageMsg = textureDepth.ToImageMsg(InitializeHeaderMsg());

        rosConnection.Publish(topicCameraDepthName, imageMsg);
    }
}
