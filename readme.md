# Real-Time Mesh Skinning with Direct Delta Mush (Unity)

Real-time mesh skinning with DDM implemented in Unity 2020.3.13.





## How To Build

### Requirement

- Unity 2020.3.13
- Visual Studio 2019



### Build

Add `MeshDeformUnity` to Unity Hub and select unity version. Then you can open this project. 



## Features

- Delta Mush with CPU or GPU.
- Direct Delta Mush with GPU (from variant 0 to variant 5).
- Precomputation of Direct Delta Mush with GPU.



These algorithms aim to do skinning with efficiency, quality and ease of authoring. 



![Delta Mush Principle 1](E:\Github\Unity\MeshDeformUnityProject\Readme\Principle_DeltaMush_1.png)



Direct Delta Mush is an algorithm that improves on the efficiency and control of Delta Mush. In brief, the original Delta Mush includes several processes. First in the precomputation step, we smooth the mesh to get the "mush", and compute the "delta" by subtracting the rest pose to the rest pose mush. During the animation, we can use some other method to do skinning, for example, linear blend skinning, to get the deformed mesh and "deformed delta". After that, we do the same smoothing to get the "deformed mush". Finally, we add the "deformed delta" and the "deformed mush" to get the "delta mush" result. 



![Delta Mush Principle 2](E:\Github\Unity\MeshDeformUnityProject\Readme\Principle_DeltaMush_2.png)

![Direct Delta Mush Principle 1](E:\Github\Unity\MeshDeformUnityProject\Readme\Principle_DirectDeltaMush_1.png)



DDM turns the iterative algorithm into a direct version, with the same ballpark computational form as some previous geometric weight blending algorithms. In this case, we move many computation into precomputation. Although the precomputation takes longer time to complete compared to the DM, it makes the quality independent of the iteration of smoothing. 



The [paper](https://www.ea.com/seed/news/siggraph2019-direct-delta-mush) also shows some variants which are equivalent to special cases of several previous skinning algorithms. 



See [technical notes](notes.md) for technical details. 



## Performance Analysis

### Precomputation





### Animation





## Credit & Reference

1. [Unity](https://unity.com/)
1. [Math.NET Numerics](https://github.com/mathnet/mathnet-numerics)
1. [Delta Mush: smoothing deformations while preserving detail](https://dl.acm.org/doi/10.1145/2633374.2633376)
1. [Direct Delta Mush Skinning and Variants](https://www.ea.com/seed/news/siggraph2019-direct-delta-mush)



## Presentations

1. [Pitch](https://docs.google.com/presentation/d/1vwb5RJlEHCoQyWLS116C5mvTnZ4lScZMC8LQFr1BcJU/)
2. [Milestone 1](https://docs.google.com/presentation/d/1DddtqMYNPFK_de73_3AZ3dXIFQ1iPYBxOBAKMeCrQ8A/)
3. [Milestone 2](https://docs.google.com/presentation/d/14nwoKlDBEHcIAdbmpu_0bEEPnFItTixbUZDCPtQ1mfM/)
3. [Milestone 3](https://docs.google.com/presentation/d/1FIu6bGBnXOtndSAxtpXztczM1mbGk7st8uuC3rlGfBQ/)
3. [Final]()
3. [Sample Video]()

