using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestPrecomputationDDM : MonoBehaviour
{
    public ComputeShader precomputeShader;

    public int iterations = 2;

    public float translationSmooth = 0.9f;
    public float rotationSmooth = 0.9f;

    public float adjacencyMatchingVertexTolerance = 1e-4f;

    internal Mesh mesh;
    internal SkinnedMeshRenderer skin;

    internal int[,] adjacencyMatrix;

    internal ComputeBuffer verticesCB; // float3
    internal ComputeBuffer normalsCB; // float3

    internal ComputeBuffer weightsCB; // float4 + int4
    internal ComputeBuffer bonesCB; // float4x4
    internal ComputeBuffer omegasCB; // float4x4 * 4
    internal ComputeBuffer outputCB; // float3 + float3

    internal DDMUtilsIterative.OmegaWithIndex[,] omegaWithIdxs;

    //////
    internal ComputeBuffer laplacianCB;
    //////laplacianCB

    internal Material ductTapedMaterial;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Assert(SystemInfo.supportsComputeShaders && precomputeShader != null);

        if (precomputeShader)
        {
            precomputeShader = Instantiate(precomputeShader);
        }
        skin = GetComponent<SkinnedMeshRenderer>();
        mesh = skin.sharedMesh;

        adjacencyMatrix = DDMSkinnedMeshGPU.GetCachedAdjacencyMatrix(mesh, adjacencyMatchingVertexTolerance);

        BoneWeight[] bws = mesh.boneWeights;

        int vCount = mesh.vertexCount;
        int bCount = skin.bones.Length;

        verticesCB = new ComputeBuffer(vCount, 3 * sizeof(float));
        normalsCB = new ComputeBuffer(vCount, 3 * sizeof(float));
        weightsCB = new ComputeBuffer(vCount, 4 * sizeof(float) + 4 * sizeof(int));
        bonesCB = new ComputeBuffer(bCount, 16 * sizeof(float));
        verticesCB.SetData(mesh.vertices);
        normalsCB.SetData(mesh.normals);
        weightsCB.SetData(bws);


        omegasCB = new ComputeBuffer(vCount * DDMSkinnedMeshGPU.maxOmegaCount, (10 * sizeof(float) + sizeof(int)));
        laplacianCB = new ComputeBuffer(vCount * DDMSkinnedMeshGPU.maxOmegaCount, (sizeof(int) + sizeof(float)));

        //omegaWithIdxs = new DDMUtilsIterative.OmegaWithIndex[vCount, DDMSkinnedMeshGPU.maxOmegaCount];
    }

    void CPU_Precomputation()
    {
        int bCount = skin.bones.Length;
        Vector3[] vertices = mesh.vertices;
        BoneWeight[] weights = mesh.boneWeights;

        UnityEngine.Profiling.Profiler.BeginSample("CPU_Precomputation");
        DDMUtilsGPU.IndexWeightPair[,] laplacianWithIndex = DDMUtilsGPU.ComputeLaplacianWithIndexFromAdjacency(adjacencyMatrix);
        omegaWithIdxs = DDMUtilsGPU.ComputeOmegasFromLaplacian(
            vertices,
            laplacianWithIndex,
            weights,
            bCount, iterations, translationSmooth);
        UnityEngine.Profiling.Profiler.EndSample();
    }

    void GPU_Precomputation()
    {
        int bCount = skin.bones.Length;
        UnityEngine.Profiling.Profiler.BeginSample("GPU_Precomputation");
        DDMUtilsGPU.computeLaplacianCBFromAdjacency(
            ref laplacianCB, precomputeShader, adjacencyMatrix);
        DDMUtilsGPU.computeOmegasCBFromLaplacianCB(
            ref omegasCB, precomputeShader,
            verticesCB, laplacianCB, weightsCB,
            bCount, iterations, translationSmooth);
        UnityEngine.Profiling.Profiler.EndSample();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey("t"))
        {
            Debug.Log("Test GPU precomputation.");
            GPU_Precomputation();
            Debug.Log("Test CPU precomputation");
            CPU_Precomputation();
        }

        //if (Input.GetKey("c"))
        //{
        //    Debug.Log("Test CPU precomputation");
        //    CPU_Precomputation();
        //}
        //if (Input.GetKey("g"))
        //{
        //    Debug.Log("Test GPU precomputation.");
        //    GPU_Precomputation();
        //}
    }

    private void OnDestroy()
    {
        verticesCB.Release();
        normalsCB.Release();
        weightsCB.Release();
        bonesCB.Release();

        omegasCB.Release();
        laplacianCB.Release();
    }
}
