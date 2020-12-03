# Supplementary material for the study "A Benchmark of Popular Indoor 3D Reconstruction Technologies: Comparison of ARCore and RTAB-Map"

doi:10.3390/electronics01010005
# Authors
Ádám Wolf<sup>1,2,3</sup>, Péter Troll<sup>4</sup>, Stefan Romeder-Finger<sup>1</sup>, Andreas Archenti<sup>4</sup>,Károly Széll<sup>2,3</sup>and Péter Galambos<sup>2</sup>

<sup>1</sup>Baxalta Innovations GmbH, Vienna, AT

<sup>2</sup>Doctoral School of Applied Informatics and Applied Mathematics, Óbuda University, Budapest, HU

<sup>3</sup>Antal Bejczy Center for Intelligent Robotics, Óbuda University, Budapest, HU

<sup>4</sup>School of Industrial Engineering and Management, KTH Royal Institute of Technology, Stockholm, SE

Correspondence: adam.wolf@takeda.com

## Abstract

The fast evolution in computational and sensor technologies brings previously niche solutions to a wider userbase. As such, 3D reconstruction technologies are reaching new use-cases in scientific and everyday areas where they were not present before. Cost-effective and easy-to-use solutions include camera-based 3D scanning techniques, such as photogrammetry. This paper provides an overview of the available solutions and discusses in detail the depth-image based Real-time Appearance-based Mapping (RTAB-Map) technique as well as a smartphone-based solution that utilises ARCore, the Augmented Reality (AR) framework of Google. To qualitatively compare the two 3D reconstruction technologies, a simple length measurement-based method was applied with a purpose-designed reference object. The captured data were then analysed by a processing algorithm. In addition to the experimental results, specific case studies are briefly discussed, evaluating the applicability based on the capabilities of the technologies. As such, the paper presents the use-case of interior surveying in an automated laboratory as well as an example for using the discussed techniques for landmark surveying}. The major findings are that point clouds created with these technologies provide a direction- and shape-accurate model, but those contain mesh continuity errors, and the estimated scale factor has a large standard deviation.
## Contents
| Directory | Description | 
|---|---|
| MATLAB | Folder containing the matlab scripts |
| > Evaluator.m | Script for evaluating the scanned data |
| > readObj.m | Support function for reading obj files [\[Source\]](https://de.mathworks.com/matlabcentral/fileexchange/18957-readobj)  |
| Plots | Folder containing the generated plots |
| > Plots_ARCore.pdf | Evaluation plots of the ARCore scans |
| > Plots_RealSense_RTAB_Map.pdf | Evaluation plots of the RTAB-Map scans |
| > Plots_RealSense_Single.pdf | Evaluation plots of the single scans |
| Scans | Folder containing the raw scanned meshes and point clouds |
| > ARCore | Meshes created with ARCore |
| > RealSense | Meshes created with the RealSense camera |
| >> RTAB-Map | Meshes created with the RTAB-Map scanninc method |
| >> Single | Meshes created with single imaging |
| Results.xlsx | Table of the results |
