# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import rospy


class RosParams:
    def get_str(self, path, default=None, global_=False):
        return str(self._get_raw(global_, path, default))

    def get_dict(self, path, default=None, global_=False):
        return self._get_raw(global_, path, default)

    def get_int(self, path, default=None, global_=False):
        return int(self._get_raw(global_, path, default))

    def get_float(self, path, default=None, global_=False):
        return float(self._get_raw(global_, path, default))

    def get_bool(self, path, default=None, global_=False):
        return bool(self._get_raw(global_, path, default))

    def _get_raw(self, global_, path, default=None):
        if global_:
            prefix = "/"
        else:
            prefix = "~"

        rospath = prefix + path
        if default is None:
            return rospy.get_param(rospath)
        else:
            return rospy.get_param(rospath, default)


class DictParams:
    def __init__(self, params, **kwargs):
        self.params = {}
        self._unroll(params)

        if kwargs is not None and "update" in kwargs:
            update_params = kwargs["update"]
            self._unroll(update_params)

    def _unroll(self, params):
        for k, v in params.items():
            if k.startswith("/"):
                k = k[1:]

            keys = k.split("/")
            if len(keys) == 1:
                self.params[k] = v
                continue

            cur = self.params
            for m in keys[:-1]:
                if m not in cur:
                    cur[m] = {}
                cur = cur[m]
            cur[keys[-1]] = v

    def get_str(self, path, default=None, global_=False):
        return str(self._get_raw(global_, path, default))

    def get_dict(self, path, default=None, global_=False):
        return self._get_raw(global_, path, default)

    def get_int(self, path, default=None, global_=False):
        return int(self._get_raw(global_, path, default))

    def get_float(self, path, default=None, global_=False):
        return float(self._get_raw(global_, path, default))

    def get_bool(self, path, default=None, global_=False):
        return bool(self._get_raw(global_, path, default))

    def _get_raw(self, global_, path, default=None):
        prefix = ""
        if global_:
            prefix = "/"

        rospath = prefix + path
        if rospath.startswith("/"):
            rospath = rospath[1:]

        keys = rospath.split("/")
        cur = self.params
        for k in keys:
            if k not in cur:
                return default
            cur = cur[k]
        return cur
