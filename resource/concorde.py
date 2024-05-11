"""Wrapper for the Concorde TSP solver."""

from pathlib import Path
import platform
import subprocess
import tempfile

import time

from .solution import Solution
from .tsp import TSPSolver


class ConcordeError(Exception):
    """Base class for errors that happen during Concorde invocations."""


class Concorde:
    """Main entrypoint for the Concorde TSP solver."""

    def solve(self, problem, concorde_exe=None, extra_args=None):
        """Solve a given TSP problem.

        Parameters
        ----------
        problem : Problem
            The TSP problem to be solved.
        concorde_exe : str or None
            The location of the Concorde solver. If ``None``, use the
            Concorde binary provided with this package.
        extra_args : list or None
            Optional arguments to be passed to the Concorde solver.

        Returns
        -------
        solution : Solution
            The optimal tour that Concorde found.
        """

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            tsp_fname = tmp / "problem.tsp"
            problem.to_tsp(tsp_fname)

            solver = TSPSolver.from_tspfile(str(tsp_fname))
            solution = solver.solve(verbose=False)  # verbose = False diminish the out_put of concorde in stdout.
            return solution


_PLATFORM_MAP = {
    ("Linux", "x86_64"): "linux",
    ("Darwin", "x86_64"): "macos/x86_64",
    ("Darwin", "arm64"): "macos/arm64",
}


def find_concorde_binary():
    """Return location of concorde binary for the current platform."""
    project_dir = Path(__file__).parent.parent
    pyconcorde_binaries = project_dir / "external" / "pyconcorde-build" / "binaries"
    if pyconcorde_binaries.exists():
        # Git checkout, with pyconcorde-build as git subtree
        location = _PLATFORM_MAP[(platform.system(), platform.machine())]
        concorde_exe = pyconcorde_binaries / location / "concorde"
    else:
        # Not a Git checkout. Assume that we're working from a wheel, with a
        # platform-specific concorde located in the module.
        concorde_exe = project_dir / "concorde" / "concorde"
    return concorde_exe
