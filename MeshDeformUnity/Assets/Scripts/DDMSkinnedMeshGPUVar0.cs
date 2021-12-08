//#define WITH_SCALE_MATRIX

using UnityEngine;
using UnityEditor;
using System;
using System.IO;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Solvers;

//[ExecuteInEditMode]
public class DDMSkinnedMeshGPUVar0 : DDMSkinnedMeshGPUBase
{
	//internal DenseMatrix[,] omegas;
	//internal DDMUtilsIterative ddmUtils;

	//internal ComputeBuffer verticesCB; // float3
	//internal ComputeBuffer normalsCB; // float3

	//internal ComputeBuffer weightsCB; // float4 + int4
	//internal ComputeBuffer bonesCB; // float4x4
	//internal ComputeBuffer omegasCB; // float4x4 * 4
	//internal ComputeBuffer outputCB; // float3 + float3

	//////
	//internal ComputeBuffer laplacianCB;
	//////laplacianCB

	internal DDMUtilsIterative.OmegaWithIndex[,] omegaWithIdxs;


	void Start()
	{
		InitBase();

		//BoneWeight[] bws = mesh.boneWeights;

		// Compute
		if (computeShader && ductTapedShader)
		{
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

			//outputCB = new ComputeBuffer(vCount, 6 * sizeof(float));

			//deformKernel = computeShader.FindKernel("DeformMesh");
			//computeShader.SetBuffer(deformKernel, "Vertices", verticesCB);
			//computeShader.SetBuffer(deformKernel, "Normals", normalsCB);
			//computeShader.SetBuffer(deformKernel, "Weights", weightsCB);
			//computeShader.SetBuffer(deformKernel, "Bones", bonesCB);
			computeShader.SetBuffer(deformKernel, "Omegas", omegasCB);
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

		if(!useCompute)
        {
			omegaWithIdxs = new DDMUtilsIterative.OmegaWithIndex[vCount, maxOmegaCount];
			omegasCB.GetData(omegaWithIdxs);
		}
	}

	void OnDestroy()
	{
		//if (laplacianCB == null)
		//	return;

		//verticesCB.Release();
		//normalsCB.Release();
		//weightsCB.Release();
		//bonesCB.Release();
		//omegasCB.Release();
		//outputCB.Release();

		//laplacianCB.Release();
		ReleaseBase();
	}

#region Direct Delta Mush implementation
	protected override void UpdateMeshOnCPU()
	{
		Matrix4x4[] boneMatrices = GenerateBoneMatrices();

		//Debug.Log(boneMatrices[1]);

		BoneWeight[] bw = mesh.boneWeights;
		Vector3[] vs = mesh.vertices;
		Vector3[] ns = mesh.normals;

		DenseMatrix[] boneMatricesDense = new DenseMatrix[boneMatrices.Length];
		for(int i = 0; i < boneMatrices.Length; ++i)
		{
			boneMatricesDense[i] = new DenseMatrix(4);
			for(int row = 0; row < 4; ++row)
            {
				for(int col = 0; col < 4; ++col)
                {
					boneMatricesDense[i][row, col] = //mesh.bindposes[i][row, col];
						boneMatrices[i][row, col];
				}
            }
		}

		for (int vi = 0; vi < mesh.vertexCount; ++vi)
		{
#if WITH_SCALE_MATRIX
			Matrix4x4 scaleMatrix = (bw[vi].boneIndex0 >= 0 && bw[vi].weight0 > 0.0f) ? scaleMatrices[bw[vi].boneIndex0] : Matrix4x4.identity;
			if(bw[vi].boneIndex1 >= 0 && bw[vi].weight1 > 0.0f)
            {
				for(int idx = 0; idx < 16; ++idx)
				{
					scaleMatrix[idx] += scaleMatrices[bw[vi].boneIndex1][idx];
				}
			}
			if (bw[vi].boneIndex2 >= 0 && bw[vi].weight2 > 0.0f)
			{
				for (int idx = 0; idx < 16; ++idx)
				{
					scaleMatrix[idx] += scaleMatrices[bw[vi].boneIndex2][idx];
				}
			}
			if (bw[vi].boneIndex3 >= 0 && bw[vi].weight3 > 0.0f)
			{
				for (int idx = 0; idx < 16; ++idx)
				{
					scaleMatrix[idx] += scaleMatrices[bw[vi].boneIndex3][idx];
				}
			}
#endif // WITH_SCALE_MATRIX

			DenseMatrix mat4 = DenseMatrix.CreateIdentity(4);

			DDMUtilsIterative.OmegaWithIndex oswi0 = omegaWithIdxs[vi, 0];
			if (oswi0.boneIndex >= 0)
			{
				DenseMatrix omega0 = new DenseMatrix(4);
				omega0[0, 0] = oswi0.m00; omega0[0, 1] = oswi0.m01; omega0[0, 2] = oswi0.m02; omega0[0, 3] = oswi0.m03;
				omega0[1, 0] = oswi0.m01; omega0[1, 1] = oswi0.m11; omega0[1, 2] = oswi0.m12; omega0[1, 3] = oswi0.m13;
				omega0[2, 0] = oswi0.m02; omega0[2, 1] = oswi0.m12; omega0[2, 2] = oswi0.m22; omega0[2, 3] = oswi0.m23;
				omega0[3, 0] = oswi0.m03; omega0[3, 1] = oswi0.m13; omega0[3, 2] = oswi0.m23; omega0[3, 3] = oswi0.m33;
				mat4 = boneMatricesDense[oswi0.boneIndex] * omega0;
				for (int i = 1; i < maxOmegaCount; ++i)
				{
					DDMUtilsIterative.OmegaWithIndex oswi = omegaWithIdxs[vi, i];
					if (oswi.boneIndex < 0)
					{
						break;
					}
					DenseMatrix omega = new DenseMatrix(4);
					omega[0, 0] = oswi.m00; omega[0, 1] = oswi.m01; omega[0, 2] = oswi.m02; omega[0, 3] = oswi.m03;
					omega[1, 0] = oswi.m01; omega[1, 1] = oswi.m11; omega[1, 2] = oswi.m12; omega[1, 3] = oswi.m13;
					omega[2, 0] = oswi.m02; omega[2, 1] = oswi.m12; omega[2, 2] = oswi.m22; omega[2, 3] = oswi.m23;
					omega[3, 0] = oswi.m03; omega[3, 1] = oswi.m13; omega[3, 2] = oswi.m23; omega[3, 3] = oswi.m33; 
					mat4 += boneMatricesDense[oswi.boneIndex] * omega;
				}
			}

			DenseMatrix Qi = new DenseMatrix(3);
			for (int row = 0; row < 3; ++row)
			{
				for (int col = 0; col < 3; ++col)
				{
					Qi[row, col] = mat4[row, col];
				}
			}

			DenseVector qi = new DenseVector(3);
			qi[0] = mat4[0, 3];
			qi[1] = mat4[1, 3];
			qi[2] = mat4[2, 3];

			DenseVector pi = new DenseVector(3);
			pi[0] = mat4[3, 0];
			pi[1] = mat4[3, 1];
			pi[2] = mat4[3, 2];

			DenseMatrix qi_piT = new DenseMatrix(3);
			qi.OuterProduct(pi, qi_piT);
			DenseMatrix M = Qi - qi_piT;

			// SVD, still need to fix
			Matrix4x4 gamma = Matrix4x4.zero;

			//float detM = M.Determinant();
			//if(true)//(Math.Abs(detM) >= 1e-4f)
			//{
				var SVD = M.Svd(true);
				DenseMatrix U = (DenseMatrix)SVD.U;
				DenseMatrix VT = (DenseMatrix)SVD.VT;
				DenseMatrix R = U * VT;
				//for(int i = 0; i < 3; ++i)
				//{
				//	R.SetColumn(i, R.Column(i).Normalize(2d));
				//}

				DenseVector ti = qi - (R * pi);

				// Get gamma
				for (int row = 0; row < 3; ++row)
				{
					for (int col = 0; col < 3; ++col)
					{
						gamma[row, col] = R[row, col];
					}
				}
				gamma[0, 3] = ti[0];
				gamma[1, 3] = ti[1];
				gamma[2, 3] = ti[2];
				gamma[3, 3] = 1.0f;
			//}
			//         else
			//         {
			//	gamma = Matrix4x4.identity;
			//}
#if WITH_SCALE_MATRIX
			gamma *= scaleMatrix;
#endif // WITH_SCALE_MATRIX

			Vector3 vertex = gamma.MultiplyPoint3x4(vs[vi]);
			deformedMesh.vertices[vi] = vertex;

			//TODO: Bug
			Vector3 normal = gamma.MultiplyVector(ns[vi]);
			deformedMesh.normals[vi] = normal;// normal.normalized;

			//if (vi == 49)
			//{
			//    Debug.Log(gamma.ToString() + "\n" + vertex.ToString() + "\n" + normal.ToString());
			//}
		}

		Bounds bounds = new Bounds();
		for (int i = 0; i < deformedMesh.vertexCount; i++)
			bounds.Encapsulate(deformedMesh.vertices[i]);

		meshForCPUOutput.vertices = deformedMesh.vertices;
		meshForCPUOutput.normals = deformedMesh.normals;
		meshForCPUOutput.bounds = bounds;
	}

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