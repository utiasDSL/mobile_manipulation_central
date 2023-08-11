from pathlib import Path
import xacro


def _xacro_include(path):
    return f"""
    <xacro:include filename="{path}" />
    """


def _xacro_compile(s, mappings=None, max_runs=10):
    """Compile xacro string until a fixed point is reached.

    Parameters:
        s: the xacro string
        mappings: dict of arguments to pass into the xacro (optional)
        max_runs: maximum number of compilation runs (default: 10)

    Raises:
        ValueError if `max_runs` compilations are exceeded.

    Returns:
        The URDF XML document.
    """
    if mappings is None:
        mappings = {}
    doc = xacro.parse(s)
    s1 = doc.toxml()

    run = 1
    while run < max_runs:
        xacro.process_doc(doc, mappings=mappings)
        s2 = doc.toxml()
        if s1 == s2:
            break
        s1 = s2
        run += 1

    if run >= max_runs:
        raise ValueError("URDF file did not converge.")
    return doc


class XacroDoc:
    """Convenience class to build URDF strings and files out of xacro components."""
    def __init__(self, s, mappings=None, max_runs=10):
        self.doc = _xacro_compile(s, mappings, max_runs)

    @classmethod
    def from_file(cls, path, **kwargs):
        """Load URDF from a xacro file."""
        with open(path) as f:
            s = f.read()
        return cls(s, **kwargs)

    @classmethod
    def from_includes(cls, includes, **kwargs):
        """Build the document from xacro includes."""
        s = """
        <?xml version="1.0" ?>
        <robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        """.strip()
        for incl in includes:
            s += _xacro_include(incl)
        s += "</robot>"
        return cls(s, **kwargs)

    def to_urdf_file(self, path, compare_existing=True, verbose=False):
        """Write the URDF to file `path`."""
        s = self.to_urdf_string()

        # if the full path already exists, we can check if the contents are the
        # same to avoid writing it if it hasn't changed. This avoids some race
        # conditions if the file is being compiled by multiple processes
        # concurrently.
        if Path(path).exists() and compare_existing:
            with open(path) as f:
                s0 = f.read()
            if s0 == s:
                if verbose:
                    print("URDF files are the same - not writing.")
                return
            elif verbose:
                print("URDF files are not the same - writing.")

        with open(path, "w") as f:
            f.write(s)

    def to_urdf_string(self, pretty=True):
        """Get the URDF as a string."""
        if pretty:
            return self.doc.toprettyxml(indent="  ")
        return self.doc.toxml()
