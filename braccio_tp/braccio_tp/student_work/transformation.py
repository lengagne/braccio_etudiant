import numpy as np
from math import sin, cos, pi

class Transformation:
    def __init__(self, rotation=np.eye(3), position=np.zeros(3)):
        """
        Initialise une transformation avec une matrice de rotation (3x3) et un vecteur de position (3x1).
        Par défaut, la rotation est l'identité et la position est le vecteur nul.
        """
        self.rotation = rotation  # Matrice 3x3
        self.position = position  # Vecteur 3x1

    @classmethod
    def from_translation(cls, tx, ty, tz):
        """Crée une transformation à partir d'une translation."""
        return cls(position=np.array([tx, ty, tz]))

    @classmethod
    def from_rotation_x(cls, theta):
        """Crée une transformation à partir d'une rotation autour de l'axe X."""
        rotation = np.array([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)]
        ])
        return cls(rotation=rotation)

    @classmethod
    def from_rotation_y(cls, theta):
        """Crée une transformation à partir d'une rotation autour de l'axe Y."""
        rotation = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ])
        return cls(rotation=rotation)

    @classmethod
    def from_rotation_z(cls, theta):
        """Crée une transformation à partir d'une rotation autour de l'axe Z."""
        rotation = np.array([
            [cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]
        ])
        return cls(rotation=rotation)

    def to_homogeneous_matrix(self):
        """Retourne la matrice de transformation homogène (4x4) associée."""
        T = np.eye(4)
        T[0:3, 0:3] = self.rotation
        T[0:3, 3] = self.position
        return T

    def compose(self, other):
        """Compose cette transformation avec une autre (self * other)."""
        new_rotation = np.dot(self.rotation, other.rotation)
        new_position = self.position + np.dot(self.rotation, other.position)
        return Transformation(new_rotation, new_position)

    def inverse(self):
        """Retourne l'inverse de cette transformation."""
        inv_rotation = self.rotation.T
        inv_position = -np.dot(inv_rotation, self.position)
        return Transformation(inv_rotation, inv_position)

    def __mul__(self, other):
        """Définit l'opérateur * pour composer deux transformations (self * other)."""
        return self.compose(other)

    def __str__(self):
        """Affiche la transformation sous forme lisible."""
        return f"Rotation:\n{self.rotation}\nPosition: {self.position}"
