## Pottery fragments matching

### [A computer tool to identify best matches for pottery fragments](https://www.sciencedirect.com/science/article/pii/S2352409X21001036?casa_token=nx0dgvIKgjEAAAAA:3IEBrs9BrPd0zi9GD2ShD8VTkyzt6GiOY4G7FBaMrdSHaXHcbLaeH50mOHVl9_ywh9C9A5uSnHQ)

1. Wilczek, J., Monna, F., Navarro, N., & Chateau-Smith, C. (2021). A computer tool to identify best matches for pottery fragments. *Journal of Archaeological Science: Reports*, *37*, 102891.

This paper focus on using supervised classification method to match pottery fragments to one of the predefined classes.

**Why is pottery restoration important?**

+ it provides information about **chronology**, and the evolution of technique and style;
+ unlike precious artefacts belonging only to the elite, ceramics have been used by **all social strata**. The socio-economic dynamics of ancient populations can be reconstructed;
+ the **fragility** of pottery reduces the information available for archaeological inquiries;

**Motivation for automatic method:** to reduce the uncertainties of the traditional archaeological approach and reduce costs for ceramic investigation

special features for pottery restoration: **rotationally symmetrical**: Usually, we reprent these 3D vessel objects by a 2D profile.

**Quantitative methods**: based on the calculation of **similarities between 2D profiles**, usually represented by polar or cartesian coordinates: a function of radius, tangent, curvature, b-spline coefficients(segments of the profile curve), polilines, or **dominant feature points**.

Matching: The best match for a given fragment (and hence its morphological class) is obtained by minimising differences with potential candidates in a referential database.



### [Pairwise Matching for 3D Fragment Reassembly Based on Boundary Curves and Concave-Convex Patches](https://ieeexplore.ieee.org/abstract/document/8938803/)

2. Li, Q., Geng, G., & Zhou, M. (2019). Pairwise matching for 3D fragment reassembly based on boundary curves and concave-convex patches. *IEEE Access*, *8*, 6153-6161.

**Main idea**: This paper leverages boundary curves and concave-convex patches to do pairwise matching method for 3D fragment reassembly.

3D reconstruction focuses mainly on the geometry of the fragments, since information such as color and texture is not reliable for assembly.

**Target object**: a pottery broken into several fragments, whose surfaces are often bumpy (bumps are noise).

#### Boundray curve extraction

1. smoothen the fragments (adaptive smoothing without shrinking) to control the degree of noise;

   ![image-20230515170953479](https://raw.githubusercontent.com/BillChan226/Notebook/main/image/image-20230515170953479.png)



2. identify edge and non-edge points which are distinguished using multi-scale curvedness;

   the **curvedness** at a point p on a surface can be estimated as:
   $$
   c_p=\sqrt{\frac{(k_1^2+k_2^2)}{2}}
   $$

3. segment the fragments into a set of surfaces that are bounded by sharp edges (patch-growing algorithm);

4. use the surface roughness to recognize coarse fracture surfaces created by the broken object;

5. apply the boundary tracking algorithm to obtain ordered and more accurate boundary points;

6. Smoothen the surface using a Gaussian filter;

   ![image-20230515203913158](https://raw.githubusercontent.com/BillChan226/Notebook/main/image/image-20230515203913158.png)

7. Compare the boundary curves $C^1$and $C^2$ which are described by strings $C^1={(k_1^1, \tau^1_1),\dots, (k_m^1, \tau^1_m)}$ and $C^2={(k_1^1, \tau^1_1),\dots, (k_n^1, \tau^1_n)}$($k$ is the discrete curature and $\tau$ is torsion), and find the similar curve segment(modeled as a circular substring matching problem);

8. find the convex patches(clustering peak points) and the concave patches(clustering pit points) of each surface;

9. define each patch $R$ as follows:
   $$
   R=\{\mu(R), \sigma(R), S(R), A(R)\}
   $$
   where $\mu=\frac{1}{m}\sum_{i=1}^mH_i$, $H_i$ is the curvature of a point $p_i$; and $\sigma(R)$ is the standard deviation of curvature, and $S(R)=(\lambda_0^R+\lambda_1^R+\lambda_2^R)^{1/2}$ and $A(R)=|\frac{\lambda^R_1}{\lambda^R_2}|^{1/2}$ are the size signature and the anisotropy signature, where $\lambda_i^R$ are the eigenvalues of the covariance matrix $M=\sum^m_{i=1}(p_i-\mu)(p_i-\mu)^T$ (principal component analysis);

10. calculate similarity $SS(R_1, R_2)$ and the anisotropy similarity $AS(R_1, R_2)$ between two patches and find the similar patch pair;

### Surface Segmentation

**[Region growing segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html)**

First of all it sorts the points by their curvature value. It needs to be done because the region begins its growth from the point that has the minimum curvature value. The reason for this is that the point with the minimum curvature is located in the flat area (growth from the flattest area allows to reduce the total number of segments).

So we have the sorted cloud. Until there are no unlabeled points in the cloud, the algorithm picks up the point with minimum curvature value and starts the growth of the region. This process occurs as follows:

- The picked point is added to the set called seeds;
- For every seed point, the algorithm finds its neighbouring points;
- Every neighbour is tested for the angle between its normal and normal of the current seed point. If the angle is less than the threshold value then current point is added to the current region;
- After that every neighbour is tested for the curvature value. If the curvature is less than the threshold value then this point is added to the seeds;
- Current seed is removed from the seeds;



### Unsupervised Learning of Visual Representations by Solving Jigsaw Puzzles

**Unsupervised Learning**: Most techniques build representations by exploiting **general-purpose priors** such as **smoothness**, **sharing of factors**, **factors organized hierarchically**, **belonging to a low-dimension manifold**, **temporal and spatial coherence**, and **sparsity**.

A general criterion to design a visual representation is not available! A natural choice is the goal of **disentangling factors of variation**.

**Categorization**:

+ probabilisitic: maximize the likelihood of the **latent variables** given the **observations**. Maximum-a-posteriori(MAP) estimation: $arg\max_x p(x|y)$; Restricted Boltzmann Machine (RBM)
+ direct mapping (autoencoder): **Prior**: minimize reconstruction error
+ manifold learning; **Prior**: local relationship



**Self-supervised Learning**: a recent variation on the unsupervised learning theme that exploits labeling that comes for “free” with the data (indirect labels: e.g. ego-motion, relevant audio, text, labels obtained from the structure of the data)

**Example**: [Paper](https://openaccess.thecvf.com/content_iccv_2015/html/Wang_Unsupervised_Learning_of_ICCV_2015_paper.html) learns feature representation with two images of the same instance, learned features focus on their similarities (such as color and texture) rather than their high-level structure.

**Example Procedures**:

+ Patch mining follows a two-step approach:

  + Extract SURF (general) feature points then use Improved Dense Trajectories (IDT) to obain motion of each SURF point; 
  + Find the best bounding box that contains most of the moving SURF points;

+ Given the initial bounding box, perform tracking using KCF tracker, and take the 30th frame in the video as "similar patch" w.r.t. the first patch ("query patch");

+ Siamese Triplet Network: the goal is to learn a feature space such that the query patch is closer to the tracked patch as compared to any other randomly sampled patch;

  ![image-20230709124831933](/Users/zhaorunchen/Library/Application Support/typora-user-images/image-20230709124831933.png)

Given an image $X$ as an input of the network, its mapping in the feature space is $f(X)$. Distance in the feature space: 
$$
D(X_1, X_2) = 1 - \frac{f(X_1)f(x_2)}{||f(X_1)||||f(X_2)||}
$$
Since we want to enforce the distance between the "query patch" and "random patch" is greater than that of the "tracked patch", i.e. $D(X_q, X_r) > D(X_q, X_t)$, the hinge loss is represented as:

$L(X_q, X_t, X_r) = max\{0, D(X_q, X_t)-D(X_q, X_r)+M\}$

Where $M$ is the minimum feature distance gap between these pairs of patches, thus the ranking loss function:
$$
\min_W \frac{\lambda}{2} ||W||^2_2 + \sum_{i=1}^N max\{0, D(X_q, X_t)-D(X_q, X_r)+M\}
$$














