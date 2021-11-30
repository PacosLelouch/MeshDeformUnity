using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra.Single;

public class DDMUtilsGPU
{
	public struct IndexWeightPair
    {
		public int index;
		public float weight;
    }

	static public int omegaCount = 16;

	//internal ComputeBuffer verticesCB;
	//internal ComputeBuffer laplacianCB;
	//internal ComputeBuffer omegasCB;

	static public bool computeLaplacianCBFromAdjacency(ref ComputeBuffer laplacianCB, ComputeShader precomputeShader, int[,] adjacencyMatrix)
    {
		if(precomputeShader == null)
        {
			return false;
        }
		if(laplacianCB != null)
        {
			laplacianCB.Release();
			laplacianCB = null;
		}

		int vCount = adjacencyMatrix.GetLength(0);
		int aCount = adjacencyMatrix.GetLength(1);

		laplacianCB = new ComputeBuffer(vCount * omegaCount, (sizeof(int) + sizeof(float)));

		IndexWeightPair[,] indexWeightPairsCPU = new IndexWeightPair[vCount, omegaCount];
		
		for(int vi = 0; vi < vCount; ++vi)
		{
			float sum = 0.0f;
			for (int ai = 0; ai < aCount && ai < omegaCount; ++ai)
			{
				sum += 1.0f;
			}
			for (int ai = 0; ai < aCount && ai < omegaCount; ++ai)
            {
				int adjVi = adjacencyMatrix[vi, ai];
				if(adjVi < 0)
                {
					break;
                }
				indexWeightPairsCPU[vi, ai].index = adjVi;
				indexWeightPairsCPU[vi, ai].weight = 1.0f / sum;
            }
        }
		laplacianCB.SetData(indexWeightPairsCPU);

		return true;
    }

	static public bool computeOmegasCBFromLaplacianCB(
		ref ComputeBuffer omegasCB, 
		ComputeShader precomputeShader, 
		ComputeBuffer verticesCB, 
		ComputeBuffer laplacianCB, 
		ComputeBuffer weightsCB, 
		int boneCount, int iterations, float lambda)
	{
		if (precomputeShader == null)
		{
			return false;
		}
		if (omegasCB != null)
		{
			omegasCB.Release();
			omegasCB = null;
		}
		int vCount = verticesCB.count;
		int aCount = omegaCount;

		int stride = (10 * sizeof(float) + sizeof(int));
		omegasCB = new ComputeBuffer(vCount * aCount, stride);

		ComputeBuffer tmpOmegasCB0 = new ComputeBuffer(vCount * boneCount, stride);
		ComputeBuffer tmpOmegasCB1 = new ComputeBuffer(vCount * boneCount, stride);

		uint threadGroupSizeX, threadGroupSizeY, threadGroupSizeZ;

		int kernelInitOmegas = precomputeShader.FindKernel("InitOmegas");
		precomputeShader.GetKernelThreadGroupSizes(kernelInitOmegas, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);
		int threadGroupsX = (vCount * boneCount + (int)threadGroupSizeX - 1) / (int)threadGroupSizeX;
		precomputeShader.SetBuffer(kernelInitOmegas, "Omegas", tmpOmegasCB0);
		precomputeShader.Dispatch(kernelInitOmegas, threadGroupsX, 1, 1);
		precomputeShader.SetBuffer(kernelInitOmegas, "Omegas", tmpOmegasCB1);
		precomputeShader.Dispatch(kernelInitOmegas, threadGroupsX, 1, 1);

		int kernelPreStep = precomputeShader.FindKernel("ComputeOmegasPreStep");
		threadGroupsX = (vCount + (int)threadGroupSizeX - 1) / (int)threadGroupSizeX;

		precomputeShader.SetFloat("VertexCount", vCount);
		precomputeShader.SetFloat("BoneCount", boneCount);
		precomputeShader.SetFloat("Lambda", lambda);
		precomputeShader.SetBuffer(kernelPreStep, "Vertices", verticesCB);
		precomputeShader.SetBuffer(kernelPreStep, "Weights", weightsCB);

		precomputeShader.SetBuffer(kernelPreStep, "Omegas", tmpOmegasCB0);
		//precomputeShader.SetBuffer(kernel0, "Omegas", iterations == 0 ? omegasCB : tmpOmemgasCB0);

		precomputeShader.Dispatch(kernelPreStep, threadGroupsX, 1, 1);

		if (iterations > 0)
		{
			int kernelOneSweep = precomputeShader.FindKernel("ComputeOmegasOneSweep");
			//precomputeShader.GetKernelThreadGroupSizes(kernelOneSweep, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);

			precomputeShader.SetBuffer(kernelOneSweep, "Laplacian", laplacianCB);

			for(int it = 0; it < iterations; ++it)
			{
				precomputeShader.SetBuffer(kernelOneSweep, "PreOmegas", (it % 2 == 0) ? tmpOmegasCB0 : tmpOmegasCB1);
                //precomputeShader.SetBuffer(kernel1, "Omegas", (it + 1 == iterations) ? omegasCB : (
                //	(iterations % 2 == 0) ? tmpOmemgasCB1 : tmpOmemgasCB0)
                //	);
                precomputeShader.SetBuffer(kernelOneSweep, "Omegas", (it % 2 == 0) ? tmpOmegasCB1 : tmpOmegasCB0);
                precomputeShader.Dispatch(kernelOneSweep, threadGroupsX, 1, 1);

			}
		}

		int kernelCompressOmegas = precomputeShader.FindKernel("CompressOmegas");
		//precomputeShader.GetKernelThreadGroupSizes(kernelCompressOmegas, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);

		precomputeShader.SetBuffer(kernelCompressOmegas, "PreOmegas", (iterations % 2 == 0) ? tmpOmegasCB0 : tmpOmegasCB1);
		precomputeShader.SetBuffer(kernelCompressOmegas, "Omegas", omegasCB);

		precomputeShader.Dispatch(kernelCompressOmegas, threadGroupsX, 1, 1);

		// TEST
		DDMUtilsIterative.OmegaWithIndex[,] outOmegas = new DDMUtilsIterative.OmegaWithIndex[vCount, omegaCount];
		omegasCB.GetData(outOmegas);
		int testVCount = outOmegas.GetLength(0);
		int testOCount = outOmegas.GetLength(1);
		int length = outOmegas.Length;
		// TEST

		tmpOmegasCB0.Release();
		tmpOmegasCB1.Release();

		return true;
	}
}