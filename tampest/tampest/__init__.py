import subprocess
import re
from typing import Any, Tuple, Union, TYPE_CHECKING


if TYPE_CHECKING:
    # can't actually subclass Any at runtime
    AnyBaseClass = Any
else:
    AnyBaseClass = object


VERSION: Tuple[Union[int, str], ...] = (0, 5, 0)
__version__ = ".".join(str(x) for x in VERSION)


try:
    git_version = subprocess.check_output(
        ["git", "describe", "--tags", "--dirty=-wip"], stderr=subprocess.STDOUT
    )
    output = git_version.strip().decode("ascii")
    data = output.split("-")
    tag = data[0]
    match = re.match(r"^v(\d+)\.(\d)+\.(\d)$", tag)
    if match is not None:
        MAJOR, MINOR, REL = tuple(int(x) for x in match.groups())

    try:
        COMMITS = int(data[1])
    except ValueError:
        COMMITS = 0

    if data[-1] == "wip":
        if COMMITS == 0:
            VERSION = (MAJOR, MINOR, REL, "post", 1)
            __version__ = f"{MAJOR}.{MINOR}.{REL}.post1"
        else:
            VERSION = (MAJOR, MINOR, REL, COMMITS, "post", 1)
            __version__ = f"{MAJOR}.{MINOR}.{REL}.{COMMITS}.post1"
    else:
        VERSION = (MAJOR, MINOR, REL, COMMITS, "dev", 1)
        __version__ = f"{MAJOR}.{MINOR}.{REL}.{COMMITS}.dev1"
except Exception as ex:
    pass
