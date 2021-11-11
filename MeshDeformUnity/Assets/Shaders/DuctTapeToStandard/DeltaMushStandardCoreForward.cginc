// Unity built-in shader source. Copyright (c) 2016 Unity Technologies. MIT license (see license.txt)

#ifndef UNITY_STANDARD_CORE_FORWARD_INCLUDED
#define UNITY_STANDARD_CORE_FORWARD_INCLUDED

#if defined(UNITY_NO_FULL_STANDARD_SHADER)
#   define UNITY_STANDARD_SIMPLE 1
#endif

#include "UnityStandardConfig.cginc"

#if UNITY_STANDARD_SIMPLE
    #include "UnityStandardCoreForwardSimple.cginc"
#else
    #include "UnityStandardCore.cginc"
#endif

struct OutputVertex
{
    float3 pos;
    float3 normal;
};
StructuredBuffer<OutputVertex> Vertices;

VertexInput loadVertex(uint id, VertexInput v)
{
	VertexInput vi = v;
	vi.vertex = float4(Vertices[id].pos, 1);
	vi.normal = Vertices[id].normal;

	/*
	#ifdef _TANGENT_TO_WORLD
	    vi.tangent.xyz = Vertices[id].tangent;
	#endif*/

	return vi;
}

#if UNITY_STANDARD_SIMPLE
    VertexOutputBaseSimple vertBase_DeltaMush (uint id : SV_VertexID, VertexInput i) { return vertForwardBaseSimple(loadVertex(id, i)); }
    VertexOutputForwardAddSimple vertAdd_DeltaMush (uint id : SV_VertexID, VertexInput i) { return vertForwardAddSimple(loadVertex(id, i)); }
    half4 fragBase (VertexOutputBaseSimple i) : SV_Target { return fragForwardBaseSimpleInternal(i); }
    half4 fragAdd (VertexOutputForwardAddSimple i) : SV_Target { return fragForwardAddSimpleInternal(i); }
#else
    VertexOutputForwardBase vertBase_DeltaMush (uint id : SV_VertexID, VertexInput i) { return vertForwardBase(loadVertex(id, i)); }
    VertexOutputForwardAdd vertAdd_DeltaMush (uint id : SV_VertexID, VertexInput i) { return vertForwardAdd(loadVertex(id, i)); }
    half4 fragBase (VertexOutputForwardBase i) : SV_Target { return fragForwardBaseInternal(i); }
    half4 fragAdd (VertexOutputForwardAdd i) : SV_Target { return fragForwardAddInternal(i); }
#endif

#endif // UNITY_STANDARD_CORE_FORWARD_INCLUDED
