# Inverse Kinematics with Skinning

## Overview
Implemented skinning, forward kinematics (FK) and inverse kinematics (IK) to deform a character. The character is represented as an obj mesh.

![](https://github.com/Jiaqi-Zuo/csci520-IK-with-skinning/blob/50f591a3b891b90df8240f190bd6bf8a86603496/armadillo.gif)
![](https://github.com/Jiaqi-Zuo/csci520-IK-with-skinning/blob/50f591a3b891b90df8240f190bd6bf8a86603496/dragon.gif)
![](https://github.com/Jiaqi-Zuo/csci520-IK-with-skinning/blob/50f591a3b891b90df8240f190bd6bf8a86603496/hand.gif)

## Core Credit Features

- Linear blend skinning

- Forward kinematics	

- Perform IK with Tikhonov regularization

- Animation: when background color is white, Kikhonov IK is perfomed; when background color is grey, Pseudoinverse IK is perfomed. 

## Aditional Features

- Implement Pseudoinverse IK
 -- Compared to Kikhonov regularization, Pseudoinverse method can produce more accurate result. However, Pseudoinverse is instable when near singularities, while Kikhonov can still produce smoohter and stable animation.
