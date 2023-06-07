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
