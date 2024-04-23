Shader "Hidden/Shader/DepthGrayscale"
{
    SubShader
    {
        Pass
        {
            Name "DepthGrayscale"

            ZWrite Off
            ZTest Always
            Blend Off
            Cull Off

            HLSLPROGRAM
            #pragma fragment CustomPostProcess
            #pragma vertex Vert

            #pragma target 4.5
            #pragma only_renderers d3d11 playstation xboxone vulkan metal switch

            #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/ShaderLibrary/ShaderVariables.hlsl"
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/Material/NormalBuffer.hlsl"

            struct Attributes
            {
                uint vertexID : SV_VertexID;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float2 texcoord   : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            Varyings Vert(Attributes input)
            {
                Varyings output;

                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                output.positionCS = GetFullScreenTriangleVertexPosition(input.vertexID);
                output.texcoord = GetFullScreenTriangleTexCoord(input.vertexID);

                return output;
            }
            
            float _Distance;

            float4 CustomPostProcess(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

                uint2 positionSS = input.texcoord * _ScreenSize.xy;

                float depth = LoadCameraDepth(positionSS);
                float linearEyeDepth = LinearEyeDepth(depth, _ZBufferParams);

                float distanceFactor = saturate(linearEyeDepth / _Distance);
                float grayScaleValue = distanceFactor * 255.0;

                grayScaleValue = clamp(grayScaleValue, 0.0, 255.0);

                float normalizedGrayScale = grayScaleValue / 255.0;

                return float4(normalizedGrayScale, normalizedGrayScale, normalizedGrayScale, 1.0);
            }

            ENDHLSL
        }
    }

    Fallback Off
}