import mushr_rhc
import mushr_rhc.utils
import torch
import pickle
import math
import numpy as np


class GPR:
    EPSILON = 1e-5
    NCTRL = 2

    def __init__(self, params, logger, dtype, **kwargs):
        self.logger = logger
        self.params = params
        self.dtype = dtype
        if kwargs is not None and "gprs" in kwargs:
            gprs = kwargs["gprs"]
            self.x_gpr = gprs[0]
            self.y_gpr = gprs[1]
            self.t_gpr = gprs[2]

        self.reset()

    def load_gpr(self, param):
        with open(self.params.get_str("%s_gpr_file" % param), "rb") as f:
            gpr = pickle.load(f)
        return gpr

    def reset(self):
        self.K = self.params.get_int("K", default=62)
        self.T = self.params.get_int("T", default=21)
        self.NPOS = self.params.get_int("npos", default=3)
        self.wheel_base = self.params.get_float("model/wheel_base", default=0.3)
        self.velocity_in_state = self.params.get_bool("velocity_in_state", default=False)

        self.beta = self.dtype(self.K)
        self.deltaTheta = self.dtype(self.K)
        self.deltaX = self.dtype(self.K)
        self.deltaY = self.dtype(self.K)
        self.sin_t = self.dtype(self.K)
        self.cos_t = self.dtype(self.K)
        self.sin_t_1 = self.dtype(self.K)
        self.cos_t_1 = self.dtype(self.K)

        self.gpr_input = self.dtype(self.K, 5)

        time_horizon = mushr_rhc.utils.get_time_horizon(self.params)
        self.dt = time_horizon / self.T

        self.x_gpr = self.load_gpr("x")
        self.y_gpr = self.load_gpr("y")
        self.t_gpr = self.load_gpr("t")

        cost_fn = self.params.get_str("cost_fn_name")
        self.with_cov = cost_fn == "block_ref_traj_cov"

        self.x_pusher_pos = self.params.get_float("/pusher_measures/x_pusher_pos_base_link")
        self.y_pusher_len = self.params.get_float("/pusher_measures/y_pusher_len")
        self.y_pusher_top = self.params.get_float("/pusher_measures/y_pusher_top")
        self.y_pusher_bot = self.params.get_float("/pusher_measures/y_pusher_bot")
        self.block_side_len = self.params.get_float("/pusher_measures/block_side_len")

        self.p1 = self.dtype(self.K, 2)
        self.p2 = self.dtype(self.K, 2)
        self.b1 = self.dtype(self.K, 2)
        self.b1_rel = self.dtype(self.K, 2)
        self.b2 = self.dtype(self.K, 2)
        self.b2_rel = self.dtype(self.K, 2)
        self.b3 = self.dtype(self.K, 2)
        self.b3_rel = self.dtype(self.K, 2)
        self.b_sin = self.dtype(self.K)
        self.b_cos = self.dtype(self.K)

        self.p1[:, 0] = self.x_pusher_pos
        self.p1[:, 1] = self.y_pusher_bot

        self.p2[:, 0] = self.x_pusher_pos
        self.p2[:, 1] = self.y_pusher_top

        self.b1_rel[:, 0] = -self.block_side_len / 2
        self.b1_rel[:, 1] = self.block_side_len / 2

        self.b2_rel[:, 0] = -self.block_side_len / 2
        self.b2_rel[:, 1] = -self.block_side_len / 2

        self.b3_rel[:, 0] = self.block_side_len / 2
        self.b3_rel[:, 1] = self.block_side_len / 2

    def rollout(self, state, trajs, rollouts):
        if self.params.get_bool("pause_on_rollout"):
            self.rollout_pause(True)
            print "good"

        rollouts.zero_()
        cov = self.dtype(self.K, self.T, 3)
        manip = self.dtype(self.K, self.T,)
        cont = self.dtype(self.K,)

        # For each K trial, the first position is at the current position
        rollouts[:, 0] = state.expand_as(rollouts[:, 0])

        # delta prior probability, so no covariance
        cov[:, 0, :] = 0.0
        manip[:, 0] = 0.0

        for t in range(1, self.T):
            cur_x = rollouts[:, t - 1]
            rollouts[:, t], cov[:, t], manip[:, t], cont[:] = self.apply(cur_x, trajs[:, t - 1])

        return cov, manip, cont

    def apply(self, pose, ctrl):
        """
        Args:
        pose [(K, NPOS) tensor] -- The current position in "world" coordinates
        ctrl [(K, NCTRL) tensor] -- Control to apply to the current position
        Return:
        [(K, NCTRL) tensor] The next position given the current control
        """
        assert pose.size() == (self.K, self.NPOS)
        assert ctrl.size() == (self.K, self.NCTRL)

        if self.velocity_in_state:
            cur_v = pose[:, 6]
        else:
            cur_v = ctrl[:, 0]

        self.beta.copy_(ctrl[:, 1]).tan_().div_(2.0).atan_().add_(self.EPSILON)
        sinbeta = self.beta.sin()

        self.deltaTheta.copy_(sinbeta).mul_(2).mul_(cur_v).div_(
            self.wheel_base).mul_(self.dt)

        cov = self.dtype(self.K, 3).zero_()
        manip = self.dtype(self.K,).zero_()

        nextpos = self.dtype(self.K, self.NPOS)
        nextpos.copy_(pose)
        nextpos[:, 2].add_(self.deltaTheta)

        if self.velocity_in_state:
            nextpos[:, 6] = torch.min(nextpos[:, 6] * 1.1, ctrl[:, 0])

        self.sin_t.copy_(pose[:, 2]).add_(self.beta).sin_()
        self.cos_t.copy_(pose[:, 2]).add_(self.beta).cos_()

        self.sin_t_1.copy_(nextpos[:, 2]).add_(self.beta).sin_()
        self.cos_t_1.copy_(nextpos[:, 2]).add_(self.beta).cos_()

        self.deltaX.copy_(self.sin_t_1).sub_(self.sin_t).mul_(
            self.wheel_base).div_(2).div_(sinbeta)

        self.deltaY.copy_(self.cos_t).sub_(self.cos_t_1).mul_(
            self.wheel_base).div_(2).div_(sinbeta)

        nextpos[:, 0].add_(self.deltaX)
        nextpos[:, 1].add_(self.deltaY)

        diffs = pose[:, 3:6] - pose[:, :3]
        rotated = self.dtype(self.K, 3)

        sin = pose[:, 2].sin()
        cos = pose[:, 2].cos()
        # do inverted rotation R(-theta) to center around car at (0, 0, 0)
        rotated[:, 0] = cos * diffs[:, 0] + sin * diffs[:, 1]
        rotated[:, 1] = -sin * diffs[:, 0] + cos * diffs[:, 1]
        rotated[:, 2] = diffs[:, 2]

        self.clamp_angles(rotated)

        i = self.contact(rotated)

        if torch.any(i):
            self.gpr_input.zero_()
            self.gpr_input[i, :3] = rotated[i]
            self.gpr_input[i, 3] = cur_v[i]
            self.gpr_input[i, 4] = ctrl[i, 1]

            bDeltaX, cov[i, 0] = self.predict(self.x_gpr, self.gpr_input[i], self.with_cov)
            bDeltaY, cov[i, 1] = self.predict(self.y_gpr, self.gpr_input[i], self.with_cov)
            bDeltaT, cov[i, 2] = self.predict(self.t_gpr, self.gpr_input[i], self.with_cov)

            manip[i] = self.manip(self.gpr_input[i])

            # rotate back into world frame only bDeltaX and bDeltaY
            dx = cos[i] * bDeltaX - sin[i] * bDeltaY
            dy = sin[i] * bDeltaX + cos[i] * bDeltaY

            nextpos[i, 3] = nextpos[i, 3].add(dx)
            nextpos[i, 4] = nextpos[i, 4].add(dy)
            nextpos[i, 5] = nextpos[i, 5].add(bDeltaT)

        return nextpos, cov, manip, i

    def predict(self, gpr, gpr_input, with_cov):
        if with_cov:
            out, cov = gpr.predict(gpr_input, return_std=True)
            return torch.from_numpy(out).type(self.dtype), torch.from_numpy(cov).type(self.dtype).pow(2)
        else:
            return torch.from_numpy(gpr.predict(gpr_input)).type(self.dtype), torch.zeros(len(gpr_input))

    def manip(self, gpr_input):
        m = self.dtype(len(gpr_input),)

        for k, inp in enumerate(gpr_input):
            J = np.empty((3, 5))
            J[0] = self.x_gpr.jac(inp)
            J[1] = self.y_gpr.jac(inp)
            J[2] = self.t_gpr.jac(inp)

            m[k] = np.sqrt(np.linalg.det(np.matmul(J, J.T)))

        return m

    def clamp_angles(self, diffs):
        # clamp angles to under pi/2
        i = (diffs[:, 2] >= math.pi / 2.0)
        while torch.any(i):
            diffs[i, 2] = diffs[i, 2] - math.pi / 2
            i = (diffs[:, 2] >= math.pi / 2.0)

        # clamp angles to greater than 0
        i = (diffs[:, 2] < 0)
        while torch.any(i):
            diffs[i, 2] = diffs[i, 2] + math.pi / 2
            i = (diffs[:, 2] < 0)

    def contact(self, diffs):
        # return torch.ones(len(diffs)).type(torch.bool)
        xy_atol = 0.01  # (1 cm) xy absolute tolerance

        con = torch.zeros(len(diffs)).type(torch.bool)

        # class 3: handling cases where the block face is parallel to the pusher
        # also handles class 2l and 2r when theta is almost zero and 2pi
        # fmt: off
        z = (diffs[:, 2] <= 0.05) | (diffs[:, 2] > math.pi / 2 - 0.05)
        con[z] = (
            torch.isclose(
                diffs[z, 0], self.dtype([self.x_pusher_pos + self.block_side_len / 2.0]), atol=xy_atol
            )
            & (diffs[z, 1] > self.y_pusher_bot - self.block_side_len / 2.0)
            & (diffs[z, 1] < self.y_pusher_top + self.block_side_len / 2.0)
        )
        # fmt: on

        # if all indices are accounted for, return early.
        if not torch.any(~z):
            return con

        # NOTE: make sure not to undo a "contact" from a previous computation.
        # once one branch says there's a hit, just keep it that way.

        # take all the other indices
        z = ~con

        self.b_sin = torch.sin(diffs[:, 2])
        self.b_cos = torch.cos(diffs[:, 2])

        # rotations and translation
        self.b1[z, 0] = self.b_cos[z] * self.b1_rel[z, 0] - self.b_sin[z] * self.b1_rel[z, 1] + diffs[z, 0]
        self.b1[z, 1] = self.b_sin[z] * self.b1_rel[z, 0] + self.b_cos[z] * self.b1_rel[z, 1] + diffs[z, 1]

        self.b2[z, 0] = self.b_cos[z] * self.b2_rel[z, 0] - self.b_sin[z] * self.b2_rel[z, 1] + diffs[z, 0]
        self.b2[z, 1] = self.b_sin[z] * self.b2_rel[z, 0] + self.b_cos[z] * self.b2_rel[z, 1] + diffs[z, 1]

        self.b3[z, 0] = self.b_cos[z] * self.b3_rel[z, 0] - self.b_sin[z] * self.b3_rel[z, 1] + diffs[z, 0]
        self.b3[z, 1] = self.b_sin[z] * self.b3_rel[z, 0] + self.b_cos[z] * self.b3_rel[z, 1] + diffs[z, 1]

        # class 1: the block is rotated, but the edge closest to the pusher is in front of the pusher.
        c1 = ~con & (self.b1[:, 1] > self.p1[:, 1])\
                  & (self.b1[:, 1] < self.p2[:, 1])

        if torch.any(c1):
            con[c1] = torch.isclose(self.b1[c1, 0], self.p1[c1, 0], atol=xy_atol)

        class2atol = 0.01

        # class 2r
        # see if the end of the pusher (a point) is on the line segment of the block
        ms = (self.b1[:, 1] - self.b3[:, 1]) / (self.b1[:, 0] - self.b3[:, 0])
        bs = self.b1[:, 1] - ms * self.b1[:, 0]

        c2r = ~con & (
            ((self.p1[:, 0] >= self.b1[:, 0]) & (self.p1[:, 0] <= self.b3[:, 0]))
            | ((self.p1[:, 1] >= self.b1[:, 1]) & (self.p1[:, 1] <= self.b3[:, 1])))

        if torch.any(c2r):
            alpha = self.p1[c2r, 0] * ms[c2r] + bs[c2r]
            beta = (self.p1[c2r, 1] - bs[c2r]) / ms[c2r]
            con[c2r] = torch.isclose(self.p1[c2r, 1], alpha, atol=xy_atol)\
                | torch.isclose(self.p1[c2r, 0], beta, atol=class2atol)

        # class 2l
        # see if the end of the pusher (a point) is on the line segment of the block
        ms = (self.b1[:, 1] - self.b2[:, 1]) / (self.b1[:, 0] - self.b2[:, 0])
        bs = self.b1[:, 1] - ms * self.b1[:, 0]

        c2l = ~con & (
            ((self.p2[:, 0] >= self.b1[:, 0]) & (self.p2[:, 0] <= self.b2[:, 0]))
            | ((self.p2[:, 1] >= self.b2[:, 1]) & (self.p2[:, 1] <= self.b1[:, 1])))

        if torch.any(c2l):
            alpha = self.p1[c2l, 0] * ms[c2l] + bs[c2l]
            beta = (self.p1[c2l, 1] - bs[c2l]) / ms[c2l]
            con[c2l] = torch.isclose(self.p2[c2l, 1], alpha, atol=xy_atol)\
                | torch.isclose(self.p2[c2l, 0], beta, atol=class2atol)

        return con
