import numpy as np
import scipy
from numpy import ndarray
from typing import Tuple

# Table for the 0.95 quantile of the chi-square distribution with N degrees of freedom (contains values for N=1, ..., 9)
# Taken from MATLAB/Octave's chi2inv function and used as Mahalanobis gating threshold.
chi2inv95 = {1: 3.8415, 2: 5.9915, 3: 7.8147, 4: 9.4877, 5: 11.070, 6: 12.592, 7: 14.067, 8: 15.507, 9: 16.919}


class KalmanFilterXYAH:
    """
    For bytetrack
    A simple Kalman filter for tracking bounding boxes in image space.

    The 8-dimensional state space

        x, y, a, h, vx, vy, va, vh

    contains the bounding box center position (x, y), aspect ratio a, height h,
    and their respective velocities.

    Object motion follows a constant velocity model. The bounding box location
    (x, y, a, h) is taken as direct observation of the state space (linear
    observation model).

    """

    def __init__(self) -> None:
        ndim = 4
        dt = 1.

        # Create Kalman filter model matrices.
        self._motion_mat = np.eye(2 * ndim, 2 * ndim, dtype=np.float32)
        self._motion_mat[:ndim, ndim:] = np.eye(ndim, ndim, dtype=np.float32) * dt
        self._update_mat = np.eye(ndim, 2 * ndim, dtype=np.float32)

        # Motion and observation uncertainty are chosen relative to the current
        # state estimate. These weights control the amount of uncertainty in
        # the model. This is a bit hacky.
        self._std_weight_position = 1. / 20
        self._std_weight_velocity = 1. / 160

    def initiate(self, xyah: ndarray) -> Tuple[ndarray, ndarray]:
        """Create track from unassociated xyah.

        Parameters
        ----------
        xyah : ndarray
            Bounding box coordinates (x, y, a, h) with center position (x, y),
            aspect ratio a, and height h.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector (8 dimensional) and covariance matrix (8x8
            dimensional) of the new track. Unobserved velocities are initialized
            to 0 mean.

        """

        mean_vel = np.zeros_like(xyah)
        mean = np.r_[xyah, mean_vel]
        aspect_ratio = xyah[3]

        std = np.array([
            2 * self._std_weight_position * aspect_ratio, 2 * self._std_weight_position * aspect_ratio, 1e-2,
            2 * self._std_weight_position * aspect_ratio, 10 * self._std_weight_velocity * aspect_ratio,
            10 * self._std_weight_velocity * aspect_ratio, 1e-5, 10 * self._std_weight_velocity * aspect_ratio],
            dtype=np.float32)

        covariance = np.diag(np.square(std))
        return mean, covariance

    def predict(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Run Kalman filter prediction step.

        Parameters
        ----------
        mean : ndarray
            The 8 dimensional mean vector of the object state at the previous
            time step.
        covariance : ndarray
            The 8x8 dimensional covariance matrix of the object state at the
            previous time step.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector and covariance matrix of the predicted
            state. Unobserved velocities are initialized to 0 mean.

        """
        aspect_ratio = mean[3]
        std_pos = np.array([
            self._std_weight_position * aspect_ratio, self._std_weight_position * aspect_ratio, 1e-2,
            self._std_weight_position * aspect_ratio],
            dtype=np.float32)
        std_vel = np.array([
            self._std_weight_velocity * aspect_ratio, self._std_weight_velocity * aspect_ratio, 1e-5,
            self._std_weight_velocity * aspect_ratio],
            dtype=np.float32)
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))

        # mean = np.dot(self._motion_mat, mean)
        mean = np.dot(mean, self._motion_mat.T)
        covariance = np.linalg.multi_dot((self._motion_mat, covariance, self._motion_mat.T)) + motion_cov

        return mean, covariance

    def project(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Project state distribution to xyah space.

        Parameters
        ----------
        mean : ndarray
            The state's mean vector (8 dimensional array).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).

        Returns
        -------
        (ndarray, ndarray)
            Returns the projected mean and covariance matrix of the given state
            estimate.

        """
        aspect_ratio = mean[3]
        std = np.array([
            self._std_weight_position * aspect_ratio, self._std_weight_position * aspect_ratio, 1e-1,
            self._std_weight_position * aspect_ratio],
            dtype=np.float32)
        innovation_cov = np.diag(np.square(std))

        mean = np.dot(self._update_mat, mean)
        covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T))
        return mean, covariance + innovation_cov

    def multi_predict(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Run Kalman filter prediction step (Vectorized version).
        Parameters
        ----------
        mean : ndarray
            The Nx8 dimensional mean matrix of the object states at the previous
            time step.
        covariance : ndarray
            The Nx8x8 dimensional covariance matrics of the object states at the
            previous time step.
        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector and covariance matrix of the predicted
            state. Unobserved velocities are initialized to 0 mean.
        """
        aspect_ratios = mean[:, 3]
        n = mean.shape[0]
        std_pos = np.array([
            self._std_weight_position * aspect_ratios, self._std_weight_position * aspect_ratios,
            1e-2 * np.ones_like(aspect_ratios), self._std_weight_position * aspect_ratios],
            dtype=np.float32)
        std_vel = np.array([
            self._std_weight_velocity * aspect_ratios, self._std_weight_velocity * aspect_ratios,
            1e-5 * np.ones_like(aspect_ratios), self._std_weight_velocity * aspect_ratios],
            dtype=np.float32)
        sqr = np.square(np.r_[std_pos, std_vel]).T

        motion_cov = [np.diag(sqr[i]) for i in range(n)]
        motion_cov = np.asarray(motion_cov)

        mean = np.dot(mean, self._motion_mat.T)
        left = np.dot(self._motion_mat, covariance).transpose((1, 0, 2))
        covariance = np.dot(left, self._motion_mat.T) + motion_cov

        return mean, covariance

    def update(self, mean: ndarray, covariance: ndarray, xyah: ndarray) -> Tuple[ndarray, ndarray]:
        """
        Run Kalman filter correction step.

        Parameters
        ----------
        mean : ndarray
            The predicted state's mean vector (8 dimensional).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).
        xyah : ndarray
            The 4 dimensional xyah vector (x, y, a, h), where (x, y)
            is the center position, a is the aspect ratio, and h is the height of the
            bounding box.

        Returns
        -------
        (ndarray, ndarray)
            Returns the xyah-corrected state distribution.

        """
        projected_mean, projected_cov = self.project(mean, covariance)

        chol_factor, lower = scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
        kalman_gain = scipy.linalg.cho_solve((chol_factor, lower),
                                             np.dot(covariance, self._update_mat.T).T,
                                             check_finite=False).T
        innovation = xyah - projected_mean

        new_mean = mean + np.dot(innovation, kalman_gain.T)
        new_covariance = covariance - np.linalg.multi_dot((kalman_gain, projected_cov, kalman_gain.T))
        return new_mean, new_covariance

    def gating_distance(self, mean: ndarray, covariance: ndarray, xyah: ndarray,
                        only_position: bool = False, metric: str = 'maha') -> ndarray:
        """Compute gating distance between state distribution and xywhs.
        A suitable distance threshold can be obtained from `chi2inv95`. If
        `only_position` is False, the chi-square distribution has 4 degrees of
        freedom, otherwise 2.
        Parameters
        ----------
        mean : ndarray
            Mean vector over the state distribution (8 dimensional).
        covariance : ndarray
            Covariance of the state distribution (8x8 dimensional).
        xyah : ndarray
            An Nx4 dimensional matrix of N xywhs, each in
            format (x, y, a, h) where (x, y) is the bounding box center
            position, a is the aspect ratio, and h is the height.
        only_position : Optional[bool]
            If True, distance computation is done with respect to the bounding
            box center position only.
        Returns
        -------
        ndarray
            Returns an array of length N, where the i-th element contains the
            squared Mahalanobis distance between (mean, covariance) and
            `xywhs[i]`.
        """
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            xyah = xyah[:, :2]

        d = xyah - mean
        if metric == 'gaussian':
            return np.sum(d * d, axis=1)
        elif metric == 'maha':
            cholesky_factor = np.linalg.cholesky(covariance)
            z = scipy.linalg.solve_triangular(cholesky_factor, d.T, lower=True, check_finite=False, overwrite_b=True)
            return np.sum(z * z, axis=0)  # square maha
        else:
            raise ValueError('invalid distance metric')


class KalmanFilterXYWH:
    """
    For BoT-SORT
    A simple Kalman filter for tracking bounding boxes in image space.

    The 8-dimensional state space

        x, y, w, h, vx, vy, vw, vh

    contains the bounding box center position (x, y), width w, height h,
    and their respective velocities.

    Object motion follows a constant velocity model. The bounding box location
    (x, y, w, h) is taken as direct observation of the state space (linear
    observation model).

    """

    def __init__(self):
        ndim = 4
        dt = 1.

        # Create Kalman filter model matrices.
        self._motion_mat = np.eye(2 * ndim, 2 * ndim, dtype=np.float32)
        self._motion_mat[:ndim, ndim:] = np.eye(ndim, ndim, dtype=np.float32) * dt
        self._update_mat = np.eye(ndim, 2 * ndim, dtype=np.float32)

        # Motion and observation uncertainty are chosen relative to the current
        # state estimate. These weights control the amount of uncertainty in
        # the model. This is a bit hacky.
        self._std_weight_position = 1. / 20
        self._std_weight_velocity = 1. / 160

    def initiate(self, xywh) -> Tuple[ndarray, ndarray]:
        """Create track from unassociated xywh.

        Parameters
        ----------
        xywh : ndarray
            Bounding box coordinates (x, y, w, h) with center position (x, y),
            width w, and height h.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector (8 dimensional) and covariance matrix (8x8
            dimensional) of the new track. Unobserved velocities are initialized
            to 0 mean.

        """
        w = xywh[2]
        h = xywh[3]
        mean_vel = np.zeros_like(xywh)
        mean = np.r_[xywh, mean_vel]

        std = np.array([
            2 * self._std_weight_position * w, 2 * self._std_weight_position * h,
            2 * self._std_weight_position * w, 2 * self._std_weight_position * h,
            10 * self._std_weight_velocity * w, 10 * self._std_weight_velocity * h,
            10 * self._std_weight_velocity * w, 10 * self._std_weight_velocity * h],
            dtype=np.float32)
        covariance = np.diag(np.square(std))
        return mean, covariance

    def predict(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Run Kalman filter prediction step.

        Parameters
        ----------
        mean : ndarray
            The 8 dimensional mean vector of the object state at the previous
            time step.
        covariance : ndarray
            The 8x8 dimensional covariance matrix of the object state at the
            previous time step.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector and covariance matrix of the predicted
            state. Unobserved velocities are initialized to 0 mean.

        """
        w = mean[2]
        h = mean[3]
        std_pos = np.array([
            self._std_weight_position * w, self._std_weight_position * h,
            self._std_weight_position * w, self._std_weight_position * h],
            dtype=np.float32)
        std_vel = np.array([
            self._std_weight_velocity * w, self._std_weight_velocity * h,
            self._std_weight_velocity * w, self._std_weight_velocity * h],
            dtype=np.float32)
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))

        mean = np.dot(mean, self._motion_mat.T)
        covariance = np.linalg.multi_dot((self._motion_mat, covariance, self._motion_mat.T)) + motion_cov

        return mean, covariance

    def project(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Project state distribution to xywh space.

        Parameters
        ----------
        mean : ndarray
            The state's mean vector (8 dimensional array).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).

        Returns
        -------
        (ndarray, ndarray)
            Returns the projected mean and covariance matrix of the given state
            estimate.

        """
        w = mean[2]
        h = mean[3]
        std = [
            self._std_weight_position * w, self._std_weight_position * h,
            self._std_weight_position * w, self._std_weight_position * h]
        innovation_cov = np.diag(np.square(std))

        mean = np.dot(self._update_mat, mean)
        covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T))
        return mean, covariance + innovation_cov

    def multi_predict(self, mean: ndarray, covariance: ndarray) -> Tuple[ndarray, ndarray]:
        """Run Kalman filter prediction step (Vectorized version).
        Parameters
        ----------
        mean : ndarray
            The Nx8 dimensional mean matrix of the object states at the previous
            time step.
        covariance : ndarray
            The Nx8x8 dimensional covariance matrics of the object states at the
            previous time step.
        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector and covariance matrix of the predicted
            state. Unobserved velocities are initialized to 0 mean.
        """
        w = mean[:, 2]
        h = mean[:, 3]
        n = mean.shape[0]
        std_pos = np.array([
            self._std_weight_position * w, self._std_weight_position * h,
            self._std_weight_position * w, self._std_weight_position * h],
            dtype=np.float32)
        std_vel = np.array([
            self._std_weight_velocity * w, self._std_weight_velocity * h,
            self._std_weight_velocity * w, self._std_weight_velocity * h],
            dtype=np.float32)
        sqr = np.square(np.r_[std_pos, std_vel]).T

        motion_cov = [np.diag(sqr[i]) for i in range(n)]
        motion_cov = np.asarray(motion_cov)

        mean = np.dot(mean, self._motion_mat.T)
        left = np.dot(self._motion_mat, covariance).transpose((1, 0, 2))
        covariance = np.dot(left, self._motion_mat.T) + motion_cov

        return mean, covariance

    def update(self, mean: ndarray, covariance: ndarray, xywh: ndarray):
        """Run Kalman filter correction step.

        Parameters
        ----------
        mean : ndarray
            The predicted state's mean vector (8 dimensional).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).
        xywh : ndarray
            The 4 dimensional xywh vector (x, y, w, h), where (x, y)
            is the center position, w the width, and h the height of the
            bounding box.

        Returns
        -------
        (ndarray, ndarray)
            Returns the xywh-corrected state distribution.

        """
        projected_mean, projected_cov = self.project(mean, covariance)

        chol_factor, lower = scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
        kalman_gain = scipy.linalg.cho_solve((chol_factor, lower),
                                             np.dot(covariance, self._update_mat.T).T,
                                             check_finite=False).T
        innovation = xywh - projected_mean

        new_mean = mean + np.dot(innovation, kalman_gain.T)
        new_covariance = covariance - np.linalg.multi_dot((kalman_gain, projected_cov, kalman_gain.T))
        return new_mean, new_covariance

    def gating_distance(self, mean: ndarray, covariance: ndarray, xywhs: ndarray,
                        only_position: bool = False, metric: str = 'maha') -> ndarray:
        """
        Compute gating distance between state distribution and xywhs.
        A suitable distance threshold can be obtained from `chi2inv95`. If
        `only_position` is False, the chi-square distribution has 4 degrees of
        freedom, otherwise 2.
        Parameters
        ----------
        mean : ndarray
            Mean vector over the state distribution (8 dimensional).
        covariance : ndarray
            Covariance of the state distribution (8x8 dimensional).
        xywhs : ndarray
            An Nx4 dimensional matrix of N xywhs, each in
            format (x, y, a, h) where (x, y) is the bounding box center
            position, a the aspect ratio, and h the height.
        only_position : Optional[bool]
            If True, distance computation is done with respect to the bounding
            box center position only.
        Returns
        -------
        ndarray
            Returns an array of length N, where the i-th element contains the
            squared Mahalanobis distance between (mean, covariance) and
            `xywhs[i]`.
        """
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            xywhs = xywhs[:, :2]

        d = xywhs - mean
        if metric == 'gaussian':
            return np.sum(d * d, axis=1)
        elif metric == 'maha':
            cholesky_factor = np.linalg.cholesky(covariance)
            z = scipy.linalg.solve_triangular(cholesky_factor, d.T, lower=True, check_finite=False, overwrite_b=True)
            return np.sum(z * z, axis=0)  # square maha
        else:
            raise ValueError('invalid distance metric')
