import keras as ks
import numpy as np
import importlib as impl
import os as os


class ExcitationForces:

    model = ks.models.Model([], [])

    @staticmethod
    def load_model(structure_file_name, weights_file_name):

        # Set backend to Theano
        if ks.backend.backend() != "theano":
            os.environ['KERAS_BACKEND'] = "theano"
            impl.reload(ks.backend)
            assert ks.backend.backend() == "theano"

        temp = open(structure_file_name, 'r')
        ExcitationForces.model = ks.models.model_from_yaml(temp.read())
        ExcitationForces.model.load_weights(weights_file_name)

    def __init__(self):
        self.geometry = np.zeros((32, 32, 32, 1))
        self.frequency = np.linspace(0.2, 2.5, 64)
        self.surge = np.zeros(64)
        self.heave = np.zeros(64)
        self.pitch = np.zeros(64)

    def set_voxel_geometry(self, voxels):
        self.geometry = voxels

    def predict(self):
        temp = self.model.predict(self.geometry)
        self.surge = temp[0, :, 0]
        self.heave = temp[0, :, 1]
        self.pitch = temp[0, :, 2]
        return temp



