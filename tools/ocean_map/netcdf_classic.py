"""A reader for classic netCDF files (CDF-1, CDF-2 and CDF-5: the header, then each variable as raw big-endian data),
enough to memory-map GEBCO's grids with numpy - no netCDF4/HDF5 library needed for them.

    nc = NetCDF("GEBCO_2026_TID.nc")
    tid = nc.variable("tid")   # numpy.memmap, shape (lat, lon)
    lat = nc.variable("lat")[:]
"""
import struct

import numpy as np

_TYPES = {1: (">i1", 1), 2: ("S1", 1), 3: (">i2", 2), 4: (">i4", 4), 5: (">f4", 4), 6: (">f8", 8),
          7: (">u1", 1), 8: (">u2", 2), 9: (">u4", 4), 10: (">i8", 8), 11: (">u8", 8)}
_DIMENSION, _VARIABLE, _ATTRIBUTE = 10, 11, 12


class NetCDF:
    def __init__(self, path):
        self.path = path
        with open(path, "rb") as f:
            magic = f.read(4)
            if magic[:3] != b"CDF" or magic[3] not in (1, 2, 5):
                raise ValueError("%s: not a classic netCDF file" % path)
            self.version = magic[3]
            self._f = f
            self._nonneg = ">q" if self.version == 5 else ">i"
            self._offset = ">q" if self.version in (2, 5) else ">i"
            self.records = self._int(self._nonneg)
            self.dimensions = [(name, size) for name, size in self._list(_DIMENSION, self._dimension)]
            self.attributes = dict(self._list(_ATTRIBUTE, self._attribute))
            self.variables = {v["name"]: v for v in self._list(_VARIABLE, self._variable)}
        self._f = None

    def _int(self, fmt):
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, self._f.read(size))[0]

    def _name(self):
        n = self._int(self._nonneg)
        name = self._f.read(n).decode("utf-8")
        self._f.read((4 - n % 4) % 4)
        return name

    def _list(self, tag, item):
        kind = self._int(">i")
        n = self._int(self._nonneg)
        if kind == 0 and n == 0:
            return []
        if kind != tag:
            raise ValueError("%s: unexpected header tag %d (wanted %d)" % (self.path, kind, tag))
        return [item() for _ in range(n)]

    def _dimension(self):
        return self._name(), self._int(self._nonneg)

    def _attribute(self):
        name = self._name()
        kind = self._int(">i")
        n = self._int(self._nonneg)
        dtype, size = _TYPES[kind]
        raw = self._f.read(n * size)
        self._f.read((4 - (n * size) % 4) % 4)
        value = raw.decode("utf-8", "replace") if kind == 2 else np.frombuffer(raw, dtype=dtype)
        return name, value

    def _variable(self):
        name = self._name()
        ndims = self._int(self._nonneg)
        dims = [self._int(self._nonneg) for _ in range(ndims)]
        attributes = dict(self._list(_ATTRIBUTE, self._attribute))
        kind = self._int(">i")
        vsize = self._int(self._nonneg)
        begin = self._int(self._offset)
        return {"name": name, "dims": dims, "attributes": attributes, "dtype": _TYPES[kind][0], "begin": begin, "vsize": vsize}

    def variable(self, name):
        """The variable as a read-only numpy memmap (record variables are not supported)."""
        v = self.variables[name]
        shape = tuple(self.dimensions[d][1] for d in v["dims"])
        if any(self.dimensions[d][1] == 0 for d in v["dims"]):
            raise ValueError("%s: record variables are not supported" % name)
        return np.memmap(self.path, dtype=v["dtype"], mode="r", offset=v["begin"], shape=shape)
