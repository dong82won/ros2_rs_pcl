import numpy as np
from vision_pose_estimator.pose_transformer import xyz_rotvec_to_matrix, matrix_to_xyz_rotvec

def test_round_trip():
    pose=[.1,.2,.3,.0,.0,.5]
    assert np.allclose(matrix_to_xyz_rotvec(xyz_rotvec_to_matrix(pose)), pose)
