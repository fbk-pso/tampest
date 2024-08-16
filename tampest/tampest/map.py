from __future__ import annotations
import os
from PIL import Image, ImageChops
from typing import Optional, Tuple, Union
from shapely.affinity import *
import yaml
import trimesh

class Map:

    def __init__(self) -> None:
        pass

    def get_from_file(self, yaml_file: str) -> Union[Map2D, Map3D, None]:
        """Get a `Map` from a `yaml_file`."""
        map = None
        with open(yaml_file) as f:
            data = yaml.safe_load(f)
            if 'mesh' in data:
                map = Map3D()
                map.set_from_file(yaml_file)
            elif 'image' in data:
                map = Map2D()
                map.set_from_file(yaml_file)
            else:
                raise NotImplementedError
        return map

class Map3D:

    def __init__(self, mesh: Optional[trimesh.Trimesh] = None) -> None:
        self._mesh = mesh

    def __eq__(self, oth: object) -> bool:
        if isinstance(oth, trimesh.Trimesh):
            return (
                all(trimesh.comparison.identifier_simple(self._mesh) == trimesh.comparison.identifier_simple(oth.mesh))
            )
        else:
            return False

    @property
    def mesh(self) -> trimesh.Trimesh:
        """Returns the `Map` `mesh`."""
        return self._mesh
    
    @mesh.setter
    def mesh(self, mesh: trimesh.Trimesh):
        """Sets the `Map` `mesh`."""
        self._mesh = mesh

    def get_mesh(self, filename: str) -> trimesh.Trimesh:
        """Load a `mesh` from a `filename`."""
        if os.path.exists(filename):
            return trimesh.load(filename, force='mesh')
        else:
            raise FileNotFoundError(f"File {filename} not found.")

    def set_from_file(self, yaml_file: str):
        """Set a `Map` from a `yaml_file`."""
        with open(yaml_file) as f:
            data = yaml.safe_load(f)
            self._mesh = self.get_mesh(os.path.join(os.path.dirname(yaml_file), data['mesh']))


class Map2D:

    def __init__(
        self,
        image: Optional[Image] = None,
        resolution: Optional[float] = None,
        origin: Optional[Tuple[float, ...]] = None,
        negate: Optional[int] = None,
        occupied_thresh: Optional[float] = None,
        free_thresh: Optional[float] = None,
    ):
        self._image = image
        self._resolution = resolution
        self._origin = origin
        self._negate = negate
        self._occupied_thresh = occupied_thresh
        self._free_thresh = free_thresh

    def __eq__(self, oth: object) -> bool:
        if isinstance(oth, Map):
            return (
                not ImageChops.difference(self._image, oth.image).getbbox()
                and self._resolution == oth.resolution
                and self._origin == oth.origin
                and self._negate == oth.negate
                and self._occupied_thresh == oth.occupied_thresh
                and self._free_thresh == oth.free_thresh
            )
        else:
            return False

    @property
    def image(self) -> Image:
        """Returns the `Map` `image`."""
        return self._image
    
    @image.setter
    def image(self, image: Image):
        """Sets the `Map` `image`."""
        self._image = image

    @property
    def resolution(self) -> float:
        """Returns the `Map` `resolution`."""
        return self._resolution

    @resolution.setter
    def resolution(self, resolution):
        """Sets the `Map` `resolution`."""
        self._resolution = resolution

    @property
    def origin(self) -> Tuple[float, ...]:
        """Returns the `Map` `origin`."""
        return self._origin
    
    @origin.setter
    def origin(self, origin):
        """Sets the `Map` `origin`."""
        self._origin = origin

    @property
    def negate(self) -> int:
        """Returns the `Map` `negate`."""
        return self._negate
    
    @negate.setter
    def negate(self, negate):
        """Sets the `Map` `negate`."""
        self._negate = negate

    @property
    def occupied_thresh(self) -> float:
        """Returns the `Map` `occupied_thresh`."""
        return self._occupied_thresh
    
    @occupied_thresh.setter
    def occupied_thresh(self, occupied_thresh):
        """Sets the `Map` `occupied_thresh`."""
        self._occupied_thresh = occupied_thresh

    @property
    def free_thresh(self) -> float:
        """Returns the `Map` `free_thresh`."""
        return self._free_thresh
    
    @free_thresh.setter
    def free_thresh(self, free_thresh):
        """Sets the `Map` `free_thresh`."""
        self._free_thresh = free_thresh

    def get_image(self, filename: str) -> Image:
        """Load an `image` from a `filename`."""
        if os.path.exists(filename):
            return Image.open(filename)
        else:
            raise FileNotFoundError(f"File {filename} not found.")     
    
    def set_from_file(self, yaml_file: str):
        """Set a `Map` from a `yaml_file`."""
        with open(yaml_file) as f:
            data = yaml.safe_load(f)
            self._image = self.get_image(os.path.join(os.path.dirname(yaml_file), data['image']))
            self._resolution = data['resolution']
            self._origin = data['origin']
            self._negate = data['negate']
            self._occupied_thresh = data['occupied_thresh']
            self._free_thresh = data['free_thresh']
