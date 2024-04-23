using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using System;

[Serializable, VolumeComponentMenu("Post-processing/Custom/PostProcessDepthGrayscale")]
public sealed class PostProcessDepthGrayscale : CustomPostProcessVolumeComponent, IPostProcessComponent
{
    [Tooltip("Enable Depth Grayscale")]
    public BoolParameter enabled = new BoolParameter(false);

    [Tooltip("Depth Distance")]
    public FloatParameter depthDistance = new FloatParameter(1f);
    
    Material m_Material;

    public override CustomPostProcessInjectionPoint injectionPoint => CustomPostProcessInjectionPoint.AfterPostProcess;

    bool IPostProcessComponent.IsActive() => enabled.value && m_Material != null;

    public override void Setup()
    {
        if (Shader.Find("Hidden/Shader/DepthGrayscale") != null)
        {
            m_Material = new Material(Shader.Find("Hidden/Shader/DepthGrayscale"));
        }
    }

    public override void Render(CommandBuffer cmd, HDCamera camera, RTHandle source, RTHandle destination)
    {
        if (m_Material == null)
        {
            return;
        }
        
        m_Material.SetFloat("_Distance", depthDistance.value);

        HDUtils.DrawFullScreen(cmd, m_Material, destination);
    }

    public override void Cleanup() {
        CoreUtils.Destroy(m_Material);
    }
}