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


    def set_translation(self, tx, ty, tz):
        self.position[0] = tx
        self.position[1] = ty
        self.position[2] = tz

    def set_rotation(self, roll, pitch, yaw):
        self.rotation = RotationX(roll) # * ...

    def set_rotation_from_quaternion(self,x,y,z,w):
        self.rotation[0,0] = 1.0
        self.rotation[0,1] = 0.0
        self.rotation[0,2] = 0.0
        self.rotation[1,0] = 0.0
        self.rotation[1,1] = 1.0
        self.rotation[1,2] = 0.0
        self.rotation[2,0] = 0.0
        self.rotation[2,1] = 0.0
        self.rotation[2,2] = 1.0


    def RotationX(theta):
        """Crée la matrice de rotation  pour une rotation autour de l'axe X."""
        rotation = np.array([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)]
        ])
        return rotation

    def RotationY(theta):
        """Crée la matrice de rotation  pour une rotation autour de l'axe Y."""
        rotation = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ])
        return rotation

    def RotationZ(theta):
        """Crée la matrice de rotation  pour une rotation autour de l'axe Z."""
        rotation = np.array([
            [cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]
        ])
        return rotation

    def multiplication(self, other):
        """Compose cette transformation avec une autre (self * other)."""
        """Attention le signe * sur les np.array fait une multiplication terme à terme """
        """la multiplication matricielle se fait avec l'opérateur @ """

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
        return self.multiplication(other)

    def __str__(self):
        """Affiche la transformation sous forme lisible."""
        return f"Rotation:\n{self.rotation}\nPosition: {self.position}"
