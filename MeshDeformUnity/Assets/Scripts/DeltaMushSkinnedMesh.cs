using UnityEngine;
using UnityEditor;
using System;
using System.IO;

//[ExecuteInEditMode]
public class DeltaMushSkinnedMesh : MonoBehaviour
{
	public int iterations = 10;
	public bool deformNormals = true;
	public bool weightedSmooth = false;//true;
	public bool useCompute = true;

	public float adjacencyMatchingVertexTolerance = 1e-4f;

	public enum DebugMode { Off, CompareWithSkinning, SmoothOnly, Deltas }

	public DebugMode debugMode = DebugMode.Off;

	bool disableDeltaPass { get { return (debugMode == DebugMode.SmoothOnly || debugMode == DebugMode.Deltas ); } }
	bool actuallyUseCompute { get { return useCompute && debugMode != DebugMode.CompareWithSkinning && debugMode != DebugMode.Deltas && !usePrefilteredBoneWeights; } }

	internal Mesh mesh;
	internal Mesh meshForCPUOutput;
	internal SkinnedMeshRenderer skin;

	struct DeformedMesh
	{
		public DeformedMesh(int vertexCount_)
		{
			vertexCount = vertexCount_;
			vertices = new Vector3[vertexCount];
			normals = new Vector3[vertexCount];
			deltaV = new Vector3[vertexCount];
			deltaN = new Vector3[vertexCount];
		}
		public int vertexCount;
		public Vector3[] vertices;
		public Vector3[] normals;
		public Vector3[] deltaV;
		public Vector3[] deltaN;
	}
	DeformedMesh deformedMesh;

	internal int[,] adjacencyMatrix;
	internal Vector3[] deltaV;
	internal Vector3[] deltaN;
	internal int deltaIterations = -1;
	internal Func<Vector3[], int[,], Vector3[]> smoothFilter;

	// Compute
	//[HideInInspector]
	public Shader ductTapedShader;
	//[HideInInspector]
	public ComputeShader computeShader;
	private int deformKernel;
	private int laplacianKernel;
	private int computeThreadGroupSizeX;

	internal ComputeBuffer verticesCB;
	internal ComputeBuffer normalsCB;
	internal ComputeBuffer deltavCB;
	internal ComputeBuffer deltanCB;
	internal ComputeBuffer weightsCB;
	internal ComputeBuffer bonesCB;
	internal ComputeBuffer adjacencyCB;
	internal ComputeBuffer[] outputCB = new ComputeBuffer[3];
	internal Material ductTapedMaterial;

	// Experiment with blending bone weights
	internal float[,] prefilteredBoneWeights;
	public bool usePrefilteredBoneWeights = false;

	void Start()
	{
		if (computeShader)
		{
			computeShader = Instantiate(computeShader);
		}
		skin = GetComponent<SkinnedMeshRenderer>();
		mesh = skin.sharedMesh;
		meshForCPUOutput = Instantiate(mesh);

		deformedMesh = new DeformedMesh(mesh.vertexCount);

		adjacencyMatrix = DDMSkinnedMeshGPUBase.GetCachedAdjacencyMatrix(mesh, adjacencyMatchingVertexTolerance);

		// Compute
		if (SystemInfo.supportsComputeShaders && computeShader && ductTapedShader)
		{
			verticesCB = new ComputeBuffer(mesh.vertices.Length, 3*sizeof(float));
			normalsCB = new ComputeBuffer(mesh.vertices.Length, 3*sizeof(float));
			weightsCB = new ComputeBuffer(mesh.vertices.Length, 4*sizeof(float) + 4*sizeof(int));
			verticesCB.SetData(mesh.vertices);
			normalsCB.SetData(mesh.normals);
			weightsCB.SetData(mesh.boneWeights);

			adjacencyCB = new ComputeBuffer(adjacencyMatrix.Length, sizeof(int));
			var adjArray = new int[adjacencyMatrix.Length];
			Buffer.BlockCopy(adjacencyMatrix, 0, adjArray, 0, adjacencyMatrix.Length * sizeof(int));
			adjacencyCB.SetData(adjArray);

			bonesCB = new ComputeBuffer(skin.bones.Length, 16*sizeof(float));
			deltavCB = new ComputeBuffer(mesh.vertices.Length, 3*sizeof(float));
			deltanCB = new ComputeBuffer(mesh.vertices.Length, 3*sizeof(float));

			outputCB[0] = new ComputeBuffer(mesh.vertices.Length, 6*sizeof(float));
			outputCB[1] = new ComputeBuffer(mesh.vertices.Length, 6*sizeof(float));
			outputCB[2] = new ComputeBuffer(mesh.vertices.Length, 6*sizeof(float));

			deformKernel = computeShader.FindKernel("DeformMesh");
			computeShader.SetBuffer(deformKernel, "Vertices", verticesCB);
			computeShader.SetBuffer(deformKernel, "Normals", normalsCB);
			computeShader.SetBuffer(deformKernel, "Weights", weightsCB);
			computeShader.SetBuffer(deformKernel, "Bones", bonesCB);
			computeShader.SetInt("VertexCount", mesh.vertices.Length);

			laplacianKernel = GetSmoothKernel();
			computeShader.SetBuffer(laplacianKernel, "Adjacency", adjacencyCB);
			computeShader.SetInt("AdjacentNeighborCount", adjacencyMatrix.GetLength(1));

			uint threadGroupSizeX, threadGroupSizeY, threadGroupSizeZ;
			computeShader.GetKernelThreadGroupSizes(deformKernel, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);
			computeThreadGroupSizeX = (int)threadGroupSizeX;
			computeShader.GetKernelThreadGroupSizes(laplacianKernel, out threadGroupSizeX, out threadGroupSizeY, out threadGroupSizeZ);
			Debug.Assert(computeThreadGroupSizeX == (int)threadGroupSizeX);

			ductTapedMaterial = new Material(ductTapedShader);
			ductTapedMaterial.CopyPropertiesFromMaterial(skin.sharedMaterial);
		}
		else
			useCompute = false;

		UpdateDeltaVectors();

		// Experiment with blending bone weights
		BoneWeight[] bw = mesh.boneWeights;
		prefilteredBoneWeights = new float[mesh.vertexCount, skin.bones.Length];
		for (int i = 0; i < mesh.vertexCount; ++i)
		{
			prefilteredBoneWeights[i, bw[i].boneIndex0] = bw[i].weight0;
			prefilteredBoneWeights[i, bw[i].boneIndex1] = bw[i].weight1;
			prefilteredBoneWeights[i, bw[i].boneIndex2] = bw[i].weight2;
			prefilteredBoneWeights[i, bw[i].boneIndex3] = bw[i].weight3;
		}
		for (int i = 0; i < iterations; i++)
			prefilteredBoneWeights = SmoothFilter.distanceWeightedLaplacianFilter(mesh.vertices, prefilteredBoneWeights, adjacencyMatrix);

		var boneCount = skin.bones.Length;
		for (int i = 0; i < mesh.vertexCount; ++i)
		{
			float l = 0.0f;
			for (int b = 0; b < boneCount; ++b)
				l += prefilteredBoneWeights[i, b];
			for (int b = 0; b < boneCount; ++b)
				prefilteredBoneWeights[i, b] += prefilteredBoneWeights[i, b] * (1.0f-l);
		}
	}

	void OnDestroy()
	{
		if (verticesCB == null)
			return;

		verticesCB.Release();
		normalsCB.Release();
		deltavCB.Release();
		deltanCB.Release();
		weightsCB.Release();
		bonesCB.Release();
		adjacencyCB.Release();

		foreach (var cb in outputCB)
			cb.Release();
	}

	void LateUpdate()
	{
		UpdateDeltaVectors();

		bool compareWithSkinning = debugMode == DebugMode.CompareWithSkinning;

        if (actuallyUseCompute)
            UpdateMeshOnGPU();
        else
            UpdateMeshOnCPU();

		if (compareWithSkinning)
			DrawVerticesVsSkin();
		else if (debugMode == DebugMode.Deltas)
			DrawDeltas();
		else
			DrawMesh();

		skin.enabled = compareWithSkinning;
	}

	#region Adjacency matrix cache
	[System.Serializable] public struct AdjacencyMatrix
	{
		public int w, h;
		public int[] storage;

		public AdjacencyMatrix(int[,] src)
		{
			w = src.GetLength(0);
			h = src.GetLength(1);
			storage = new int[w * h];
			Buffer.BlockCopy(src, 0, storage, 0, storage.Length * sizeof(int));
		}

		public int[,] data
		{
			get
			{
				var retVal = new int[w, h];
				Buffer.BlockCopy(storage, 0, retVal, 0, storage.Length * sizeof(int));
				return retVal;
			}
		}
	}

	//static private int[,] GetCachedAdjacencyMatrix(Mesh mesh, float adjacencyMatchingVertexTolerance)
	//{
	//	int [,] adjacencyMatrix;
	//	//#if UNITY_EDITOR
	//	////var path = Path.Combine(Application.persistentDataPath, AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(mesh)) + ".adj");
	//	//var path = Path.Combine("", AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(mesh)) + "_" + adjacencyMatchingVertexTolerance.ToString() + ".adj");
	//	//Debug.Log(path);
	//	//if (File.Exists(path))
	//	//{
	//	//	string json = File.ReadAllText(path);
	//	//	adjacencyMatrix = JsonUtility.FromJson<AdjacencyMatrix>(json).data;
	//	//}
	//	//else
	//	//{
	//	//#endif
	//		adjacencyMatrix = MeshUtils.BuildAdjacencyMatrix(mesh.vertices, mesh.triangles, 16, adjacencyMatchingVertexTolerance*adjacencyMatchingVertexTolerance);
	//	//#if UNITY_EDITOR
	//	//	var json = JsonUtility.ToJson(new AdjacencyMatrix(adjacencyMatrix));
	//	//	Debug.Log(json);

	//	//	using (FileStream fs = new FileStream(path, FileMode.Create))
	//	//	{
	//	//		using (StreamWriter writer = new StreamWriter(fs))
	//	//		{
	//	//			writer.Write(json);
	//	//		}
	//	//	}
	//	//}
	//	//#endif
	//	return adjacencyMatrix;
	//}
	#endregion

	private int GetSmoothKernel()
	{
		if (weightedSmooth)
			return computeShader.FindKernel("WeightedLaplacianFilter");
		else
			return computeShader.FindKernel("LaplacianFilter");
	}

	#region Delta Mush implementation
	private Func<Vector3[], int[,], Vector3[]> GetSmoothFilter()
	{
		if (weightedSmooth)
			return SmoothFilter.distanceWeightedLaplacianFilter;
		else
			return SmoothFilter.laplacianFilter;
	}

	private static Vector3[] GetSmoothDeltas(Vector3[] vertices, int[,] adjacencyMatrix, Func<Vector3[], int[,], Vector3[]> filter, int iterations)
	{
		var smoothVertices = new Vector3[vertices.Length];
		for (int i = 0; i < vertices.Length; i++)
			smoothVertices[i] = vertices[i];

		for (int i = 0; i < iterations; i++)
			smoothVertices = filter(smoothVertices, adjacencyMatrix);

		var delta = new Vector3[vertices.Length];
		for (int i = 0; i < vertices.Length; i++)
			delta[i] = vertices[i] - smoothVertices[i];

		return delta;
	}

	void UpdateDeltaVectors()
	{
		var lastFilter = smoothFilter;
		smoothFilter = GetSmoothFilter();

		if (iterations == 0)
			return;

		if (smoothFilter == lastFilter && iterations == deltaIterations)
			return;

		deltaV = GetSmoothDeltas(mesh.vertices, adjacencyMatrix, smoothFilter, iterations);
		deltaN = GetSmoothDeltas(mesh.normals, adjacencyMatrix, smoothFilter, iterations);
		deltaIterations = iterations;

		// compute
		if (deltavCB != null && deltanCB != null)
		{
			deltavCB.SetData(deltaV);
			deltanCB.SetData(deltaN);
		}
	}

	void UpdateMeshOnCPU()
	{
		Matrix4x4[] boneMatrices = new Matrix4x4[skin.bones.Length];
		for (int i = 0; i < boneMatrices.Length; i++)
			boneMatrices[i] = skin.bones[i].localToWorldMatrix * mesh.bindposes[i];

		BoneWeight[] bw = mesh.boneWeights;
		Vector3[] vs = mesh.vertices;
		Vector3[] ns = mesh.normals;

		if (usePrefilteredBoneWeights)
		{
			var boneCount = boneMatrices.Length;
			for (int i = 0; i < mesh.vertexCount; i++)
			{
				var vertexMatrix = Matrix4x4.zero;

				for (int n = 0; n < 16; n++)
					for (int b = 0; b < boneCount; b++)
						vertexMatrix[n] += boneMatrices[b][n] * prefilteredBoneWeights[i, b];

				deformedMesh.vertices[i] = vertexMatrix.MultiplyPoint3x4(vs[i] - deltaV[i]);
				deformedMesh.normals[i] = vertexMatrix.MultiplyVector(ns[i] - deltaN[i]);
				deformedMesh.deltaV[i] = vertexMatrix.MultiplyVector(deltaV[i]);
				deformedMesh.deltaN[i] = vertexMatrix.MultiplyVector(deltaN[i]);
			}
				
			if (iterations > 0 && !disableDeltaPass)
				for (int i = 0; i < deformedMesh.vertexCount; i++)
					deformedMesh.vertices[i] = deformedMesh.vertices[i] + deformedMesh.deltaV[i];

			if (iterations > 0 && deformNormals)
				for (int i = 0; i < deformedMesh.vertexCount; i++)
				{
					deformedMesh.normals[i] = deformedMesh.normals[i] + deformedMesh.deltaN[i];
					//deformedMesh.normals[i].Normalize();
				}
		
			meshForCPUOutput.vertices = deformedMesh.vertices;
			meshForCPUOutput.normals = deformedMesh.normals;

			return;
		}

		for (int i = 0; i < mesh.vertexCount; i++)
		{
			BoneWeight boneWeight = bw[i];

			Matrix4x4 bm0 = boneMatrices[boneWeight.boneIndex0];
			Matrix4x4 bm1 = boneMatrices[boneWeight.boneIndex1];
			Matrix4x4 bm2 = boneMatrices[boneWeight.boneIndex2];
			Matrix4x4 bm3 = boneMatrices[boneWeight.boneIndex3];

			Matrix4x4 vertexMatrix = new Matrix4x4();

			if (skin.quality == SkinQuality.Bone1)
			{
				vertexMatrix = bm0;
			}
			else if (skin.quality == SkinQuality.Bone2)
			{
				for (int n = 0; n < 16; n++)
				{
					vertexMatrix[n] =
						bm0[n] * boneWeight.weight0 +
						bm1[n] * (1-boneWeight.weight0);
				}
			}
			else
			{
				for (int n = 0; n < 16; n++)
				{
					vertexMatrix[n] =
						bm0[n] * boneWeight.weight0 +
						bm1[n] * boneWeight.weight1 +
						bm2[n] * boneWeight.weight2 +
						bm3[n] * boneWeight.weight3;
				}
			}

			deformedMesh.vertices[i] = vertexMatrix.MultiplyPoint3x4(vs[i]);
			deformedMesh.normals[i] = vertexMatrix.MultiplyVector(ns[i]);
			deformedMesh.deltaV[i] = vertexMatrix.MultiplyVector(deltaV[i]);
			deformedMesh.deltaN[i] = vertexMatrix.MultiplyVector(deltaN[i]);
		}
			
		for (int i = 0; i < iterations; i++)
		{
			deformedMesh.vertices = smoothFilter(deformedMesh.vertices, adjacencyMatrix);
			if (deformNormals)
				deformedMesh.normals = smoothFilter(deformedMesh.normals, adjacencyMatrix);
		}

		if (iterations > 0 && !disableDeltaPass)
			for (int i = 0; i < deformedMesh.vertexCount; i++)
				deformedMesh.vertices[i] = deformedMesh.vertices[i] + deformedMesh.deltaV[i];

		if (iterations > 0 && deformNormals)
			for (int i = 0; i < deformedMesh.vertexCount; i++)
			{
				deformedMesh.normals[i] = deformedMesh.normals[i] + deformedMesh.deltaN[i];
				//deformedMesh.normals[i].Normalize();
			}

		Bounds bounds = new Bounds();
		for (int i = 0; i < deformedMesh.vertexCount; i++)
			bounds.Encapsulate(deformedMesh.vertices[i]);

		meshForCPUOutput.vertices = deformedMesh.vertices;
		meshForCPUOutput.normals = deformedMesh.normals;
		meshForCPUOutput.bounds = bounds;
	}

	void UpdateMeshOnGPU()
	{
		//TODO
		int threadGroupsX = (mesh.vertices.Length + computeThreadGroupSizeX - 1) / computeThreadGroupSizeX;

		Matrix4x4[] boneMatrices = new Matrix4x4[skin.bones.Length];
		for (int i = 0; i < boneMatrices.Length; i++)
			boneMatrices[i] = skin.bones[i].localToWorldMatrix * mesh.bindposes[i];
		bonesCB.SetData(boneMatrices);
		computeShader.SetBool("DeltaPass", false);
		computeShader.SetBuffer(deformKernel, "Bones", bonesCB);
		computeShader.SetBuffer(deformKernel, "Vertices", verticesCB);
		computeShader.SetBuffer(deformKernel, "Normals", normalsCB);
		computeShader.SetBuffer(deformKernel, "Output", outputCB[0]);
		computeShader.Dispatch(deformKernel, threadGroupsX, 1, 1);
		ductTapedMaterial.SetBuffer("Vertices", outputCB[0]);

		computeShader.SetBool("DeltaPass", true);
		computeShader.SetBuffer(deformKernel, "Vertices", deltavCB);
		computeShader.SetBuffer(deformKernel, "Normals", deltanCB);
		computeShader.SetBuffer(deformKernel, "Output", outputCB[2]);
		computeShader.Dispatch(deformKernel, threadGroupsX, 1, 1);

		for (int i = 0; i < iterations; i++)
		{	
			laplacianKernel = GetSmoothKernel();
			computeShader.SetBuffer(laplacianKernel, "Adjacency", adjacencyCB);
			computeShader.SetInt("AdjacentNeighborCount", adjacencyMatrix.GetLength(1));

			bool lastIteration = i == iterations - 1;
			if (lastIteration)
			{
				computeShader.SetBuffer(laplacianKernel, "Delta", outputCB[2]);
				ductTapedMaterial.SetBuffer("Vertices", outputCB[(i+1)%2]);
			}
			//computeShader.SetBuffer(laplacianKernel, "Delta", outputCB[2]);

			computeShader.SetBool("DeltaPass", lastIteration && !disableDeltaPass);
			computeShader.SetBuffer(laplacianKernel, "Input", outputCB[i%2]);
			computeShader.SetBuffer(laplacianKernel, "Output", outputCB[(i+1)%2]);
			computeShader.Dispatch(laplacianKernel, threadGroupsX, 1, 1);
		}
	}
	#endregion


	#region Helpers
	void DrawMesh()
	{
		if (actuallyUseCompute)
		{
			mesh.bounds = skin.bounds; // skin is actually disabled, so it only remembers the last animation frame
			Graphics.DrawMesh(mesh, Matrix4x4.identity, ductTapedMaterial, 0);
			//Graphics.DrawMesh(mesh, skin.transform.parent.worldToLocalMatrix * skin.transform.localToWorldMatrix, ductTapedMaterial, 0);
		}
		else
			Graphics.DrawMesh(meshForCPUOutput, Matrix4x4.identity, skin.sharedMaterial, 0);
			//Graphics.DrawMesh(meshForCPUOutput, skin.transform.parent.worldToLocalMatrix * skin.transform.localToWorldMatrix, skin.sharedMaterial, 0);
	}

	void DrawDeltas()
	{
		for (int i = 0; i < deformedMesh.vertexCount; i++)
		{
			Vector3 position = deformedMesh.vertices[i];
			Vector3 delta = deformedMesh.deltaV[i];

			Color color = Color.green;
			Debug.DrawRay(position, delta, color);
		}

		Graphics.DrawMesh(meshForCPUOutput, Matrix4x4.identity, skin.sharedMaterial, 0);
	}

	void DrawVerticesVsSkin()
	{
		//for (int i = 0; i < deformedMesh.vertexCount; i++)
		//{
		//	Vector3 position = deformedMesh.vertices[i];
		//	Vector3 normal = deformedMesh.normals[i];

		//	Color color = Color.green;
		//	Debug.DrawRay(position, normal*0.01f, color);
		//}
	}
	#endregion
}