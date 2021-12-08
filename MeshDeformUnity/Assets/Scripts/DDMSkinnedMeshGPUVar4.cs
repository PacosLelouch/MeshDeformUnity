//#define WITH_SCALE_MATRIX

using UnityEngine;
using UnityEditor;
using System;
using System.IO;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Solvers;

//[ExecuteInEditMode]
public class DDMSkinnedMeshGPUVar4 : DDMSkinnedMeshGPUBase
{
	// Compute
	internal ComputeBuffer lastomegasCB; // float
	internal ComputeBuffer psCB; // float3

	void Start()
	{
		InitBase();

		// Compute
		if (computeShader && ductTapedShader)
		{
			//Matrix4x4[,] compressedOmegas = DDMUtilsIterative.CompressOmegas2D(omegas, bws);
			//DDMUtilsIterative.OmegaWithIndex[,] convertedOmegas = DDMUtilsIterative.ConvertOmegas1D(omegas, maxOmegaCount);

			//verticesCB = new ComputeBuffer(vCount, 3 * sizeof(float));
			//normalsCB = new ComputeBuffer(vCount, 3 * sizeof(float));
			//weightsCB = new ComputeBuffer(vCount, 4 * sizeof(float) + 4 * sizeof(int));
			//bonesCB = new ComputeBuffer(bCount, 16 * sizeof(float));
			//verticesCB.SetData(mesh.vertices);
			//normalsCB.SetData(mesh.normals);
			//weightsCB.SetData(bws);

			////omegasCB = new ComputeBuffer(vCount, 16 * sizeof(float) * 4);
			////omegasCB.SetData(compressedOmegas);
			//omegasCB = new ComputeBuffer(vCount * maxOmegaCount, (10 * sizeof(float) + sizeof(int)));
			////omegasCB.SetData(convertedOmegas);

			lastomegasCB = new ComputeBuffer(vCount * maxOmegaCount, (sizeof(float) + sizeof(int)));
			psCB = new ComputeBuffer(vCount, 3 * sizeof(float));

			//outputCB = new ComputeBuffer(vCount, 6 * sizeof(float));

			//deformKernel = computeShader.FindKernel("DeformMesh");
			//computeShader.SetBuffer(deformKernel, "Vertices", verticesCB);
			//computeShader.SetBuffer(deformKernel, "Normals", normalsCB);
			//computeShader.SetBuffer(deformKernel, "Weights", weightsCB);
			//computeShader.SetBuffer(deformKernel, "Bones", bonesCB);
			computeShader.SetBuffer(deformKernel, "Lastomegas", lastomegasCB);
			computeShader.SetBuffer(deformKernel, "Ps", psCB);
			//computeShader.SetBuffer(deformKernel, "Output", outputCB);
			//computeShader.SetInt("VertexCount", vCount);

			//uint threadGroupSizeX, threadGroupSizeY, threadGroupSizeZ;
			//computeShader.GetKernelThreadGroupSizes(deformKernel, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);
			//computeThreadGroupSizeX = (int)threadGroupSizeX;

			//ductTapedMaterial = new Material(ductTapedShader);
			//ductTapedMaterial.CopyPropertiesFromMaterial(skin.sharedMaterial);
		}
		//else
		//{
		//	useCompute = false;
		//}

		//laplacianCB = new ComputeBuffer(vCount * maxOmegaCount, (sizeof(int) + sizeof(float)));
		//DDMUtilsGPU.ComputeLaplacianCBFromAdjacency(
		//	ref laplacianCB, precomputeShader, adjacencyMatrix);
		//DDMUtilsGPU.ComputeOmegasCBFromLaplacianCB(
		//	ref omegasCB, precomputeShader,
		//	verticesCB, laplacianCB, weightsCB, 
		//	bCount, iterations, translationSmooth);

		DDMUtilsGPU.computeLastomegasCBPsCBFromOmegasCB(ref lastomegasCB, ref psCB, precomputeShader, omegasCB, vCount, bCount);

		if (!useCompute)
        {
			//TODO
        }
	}

	void OnDestroy()
	{
		if (lastomegasCB != null)
		{
			lastomegasCB.Release();
			psCB.Release();
		}

		ReleaseBase();
	}

	//void LateUpdate()
	//{
	//	bool compareWithSkinning = debugMode == DebugMode.CompareWithSkinning;

	//	if (actuallyUseCompute)
	//		UpdateMeshOnGPU();
	//	//else
	//	//	UpdateMeshOnCPU();

	//	if (compareWithSkinning)
	//		DrawVerticesVsSkin();
	//	//else if (debugMode == DebugMode.Deltas)
	//	//	DrawDeltas();
	//	else
	//		DrawMesh();

	//	skin.enabled = compareWithSkinning;
	//}


	#region Direct Delta Mush implementation

	//protected override void UpdateMeshOnCPU()
	//{

	//}

	protected override void UpdateMeshOnGPU()
	{
		int threadGroupsX = (vCount + computeThreadGroupSizeX - 1) / computeThreadGroupSizeX;

		Matrix4x4[] boneMatrices = GenerateBoneMatrices();

		bonesCB.SetData(boneMatrices);
		computeShader.SetBuffer(deformKernel, "Bones", bonesCB);
		//computeShader.SetBuffer(deformKernel, "Vertices", verticesCB);
		//computeShader.SetBuffer(deformKernel, "Normals", normalsCB);
		//computeShader.SetBuffer(deformKernel, "Output", outputCB);
		computeShader.Dispatch(deformKernel, threadGroupsX, 1, 1);
		ductTapedMaterial.SetBuffer("Vertices", outputCB);
	}

#endregion

}