using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestDDMRuntime : MonoBehaviour
{
    //public List<GameObject> meshObjects;

    
    public int iterations = -1;
    public float smoothLambda = -1.0f;
    public float adjacencyMatchingVertexTolerance = -1.0f;

    void UpdateValues(DDMSkinnedMeshGPUBase script)
    {
        if(script != null)
        {
            if(iterations >= 0)
            {
                script.iterations = iterations;
            }
            if (smoothLambda >= 0.0f)
            {
                script.smoothLambda = smoothLambda;
            }
            if (adjacencyMatchingVertexTolerance >= 0.0f)
            {
                script.adjacencyMatchingVertexTolerance = adjacencyMatchingVertexTolerance;
            }
        }
    }

    void UpdateValues(DeltaMushSkinnedMesh script)
    {
        if (script != null)
        {
            if (iterations >= 0)
            {
                script.iterations = iterations;
            }
            //if (smoothLambda >= 0.0f)
            //{
            //    script.smoothLambda = smoothLambda;
            //}
            if (adjacencyMatchingVertexTolerance >= 0.0f)
            {
                script.adjacencyMatchingVertexTolerance = adjacencyMatchingVertexTolerance;
            }
        }
    }

    void Awake()
    {
        Debug.Log("Test DDM runtime awake.");
        DDMSkinnedMeshGPUBase[] scriptsDDM = FindObjectsOfType<DDMSkinnedMeshGPUBase>(false);
        Debug.Log("Find " + scriptsDDM.Length.ToString() + " DDM scripts.");
        foreach (DDMSkinnedMeshGPUBase script in scriptsDDM)
        {
            UpdateValues(script);
        }
        DeltaMushSkinnedMesh[] scriptsDM = FindObjectsOfType<DeltaMushSkinnedMesh>(false);
        Debug.Log("Find " + scriptsDM.Length.ToString() + " DM scripts.");
        foreach (DeltaMushSkinnedMesh script in scriptsDM)
        {
            UpdateValues(script);
        }

        //foreach (GameObject meshObject in meshObjects)
        //{
        //    DDMSkinnedMeshGPUBase DDMBaseScript = meshObject.GetComponent<DDMSkinnedMeshGPUBase>();
        //    UpdateValues(DDMBaseScript);
        //}
    }


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
