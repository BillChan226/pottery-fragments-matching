# Dataset Structure

The dataset is structured as follows:

```
unpaired: 
    ├── 0
    ├── 1
    ├── 2
    └── ...
```

```
reconstruction: 
    ├── q0
       ├── 0
          ├── f1
          ├── f2
          ├── ...
          ├── target
       ├── 1
       ├── ... 
    ├── q1
    └── ...
```

**unpaired** contains classified data without ground truth (the original object); and **reconstruction** contains classified data with ground truth, ready for reconstruction task.

For **reconstruction**, the folder is structured in a hierachical (density) way. Each q_ folder is the root folder of a new object. Inside the q_ folder, each sub-folder contains the multiple object density levels that can reconstruct the object. For example, f1 contains the largest pieces of highest semantic patterns, and further breaking them down, we have f2 (in other words, f2 can be matched and reconstructed to get f1, and f1 can be matched and reconstructed to get the target ground truth object). The amount of files is expected to increase exponentially as the density level increases (f3 >> f2 >> f1).
