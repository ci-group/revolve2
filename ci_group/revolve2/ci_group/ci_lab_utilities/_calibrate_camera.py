import cv2
import numpy as np
from numpy.typing import NDArray


def calibrate_camera(
    calibration_images_paths: list[str], checkerboard_size: tuple[int, int] = (9, 9)
) -> tuple[tuple[int, ...], NDArray[np.float_], NDArray[np.float_]]:
    """
    Calibrate cameras for distortion and fisheye effects.

    In order to use this function effectively please use at least 5 valid calibration images, with differently places checkerboards.
    The checkerboard has to be fully visible with no occlusion, but it does mot have to lie flat on the ground.

    :param calibration_images_paths: The calibration images.
    :param checkerboard_size: The checkerboard size. Note if you have a 10 x 10 checkerboard the size should be (9, 9).
    :return: The dimension of the calibration images, the camera matrix and the distortion coefficient.
    """
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        + cv2.fisheye.CALIB_CHECK_COND
        + cv2.fisheye.CALIB_FIX_SKEW
    )

    objp = np.zeros((1, checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[
        0 : checkerboard_size[0], 0 : checkerboard_size[1]
    ].T.reshape(-1, 2)

    _img_shape = None

    object_points = []  # 3d point in real world space
    image_points = []  # 2d points in image plane

    for image_path in calibration_images_paths:
        image = cv2.imread(image_path)
        if _img_shape is None:
            _img_shape = image.shape[:2]
        else:
            assert _img_shape == image.shape[:2], "All images must share the same size."

        # Detect checkerboard
        ret, corners = cv2.findChessboardCorners(
            image,
            checkerboard_size,
            None,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ret:
            object_points.append(objp)
            cv2.cornerSubPix(image, corners, (3, 3), (-1, -1), subpix_criteria)
            image_points.append(corners)

    camera_matrix = np.zeros((3, 3))
    distortion_coefficients = np.zeros((4, 1))
    rotation_vectors = [
        np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(object_points))
    ]
    translation_vectors = [
        np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(object_points))
    ]
    rms, _, _, _, _ = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image.shape[::-1],
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
    )
    print("Found " + str(len(object_points)) + " valid images for calibration")
    print("dimensions =" + str(image.shape[:2][::-1]))
    print("Camera Matrix =np.array(" + str(camera_matrix.tolist()) + ")")
    print("Distortion Coefficients = " + str(distortion_coefficients.tolist()) + ")")
    return image.shape[:2][::-1], camera_matrix, distortion_coefficients
