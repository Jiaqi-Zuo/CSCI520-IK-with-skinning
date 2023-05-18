Core Features:
- Linear blend skinning
- Forward kinematics	
- Perform IK with Tikhonov regularization
- Animation: when background color is white, Kikhonov IK is perfomed; when background color is grey, Pseudoinverse IK is perfomed. 

Extra Credits:
- Implement Pseudoinverse IK
  - Comparison between Kikhonov IK and Psedoinverse IK:
    - Compared to Kikhonov regularization, Pseudoinverse method can produce more accurate result. However, Pseudoinverse is instable when near singularities, while Kikhonov can still produce smoohter and stable animation.
		