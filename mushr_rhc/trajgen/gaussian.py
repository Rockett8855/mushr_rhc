# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import torch


class Gaussian:
    # Size of control vector
    NCTRL = 2

    def __init__(self, params, logger, dtype):
        self.logger = logger
        self.params = params
        self.dtype = dtype

        self.reset()

    def reset(self):
        self.K = self.params.get_int("K", default=62)
        self.T = self.params.get_int("T", default=15)
        self.K_backward = self.params.get_int("K_backwards", default=0)
        self.K_forward = self.K - self.K_backward

        self.min_delta = self.params.get_float("trajgen/min_delta", default=-0.34)
        self.max_delta = self.params.get_float("trajgen/max_delta", default=0.34)

        self.desired_speed = self.params.get_float("trajgen/desired_speed", default=1.0)

        self.mean = self.dtype(self.K, self.T, self.NCTRL)
        self.std = self.dtype(self.K, self.T, self.NCTRL)
        self.mean.zero_()
        self.std[:, :, 0] = 0.1
        self.std[:, :, 1] = 0.4

        # The controls for TL are precomputed, and don't change
        self.ctrls = self.dtype(self.K, self.T, self.NCTRL)
        self.ctrls.zero_()
        self.ctrls[: self.K_forward, :, 0] = self.desired_speed
        if self.K_backward > 0:
            self.ctrls[self.K_forward :, :, 0] = -self.desired_speed

    def get_control_trajectories(self, velocity):
        """
        Returns:
        [(K, T, NCTRL) tensor] -- of controls
            ([:, :, 0] is the desired speed, [:, :, 1] is the control delta)
        """
        self.ctrls[: self.K_forward, :, 0] = velocity
        if self.K_backward > 0:
            self.ctrls[self.K_forward :, :, 0] = -velocity

        self.mean[:, :, 0] = velocity
        torch.normal(self.mean, self.std, out=self.ctrls)

        self.ctrls[:, :, 1].clamp_(self.min_delta, self.max_delta)
        # self.ctrls[:, :, 0] = torch.normal(self.mean[:, :, 0], self.std[:, :, 0], out=)
        # self.ctrls[:, :, 1] = torch.normal(self.mean[:, :, 1], self.std[:, :, 1])

        print self.mean[0, 0]
        # print self.ctrls[:, 0, 1]

        return self.ctrls

    def generate_control(self, controls, costs):
        """
        Args:
        controls [(K, T, NCTRL) tensor] -- Returned by get_control_trajectories
        costs [(K, 1) tensor] -- Cost to take a path

        Returns:
        [(T, NCTRL) tensor] -- The lowest cost trajectory to take
        """
        assert controls.size() == (self.K, self.T, 2)
        assert costs.size() == (self.K,)
        _, idx = torch.min(costs, 0)

        self.mean[:] = controls[idx]

        return controls[idx], idx
