import numpy as np


class Spectrum:
    def __init__(self, type, **kwargs):
        if type is 'pierson-moskowitz':
            alpha_pm = 0.0081
            g = 9.81
            self.A = alpha_pm*g*g/np.power(2*np.pi, 4)
            self.B = 1.25*np.power(kwargs['fp'], 4)
        elif type is 'bretschneider':
            self.A = np.power(kwargs['Hm0'], 2)*np.power(1.057*kwargs['fp'], 4)/4
            self.B = np.power(1.057*kwargs['fp'], 4)
        elif type is 'IITC':
            self.A = 310*np.power(kwargs['Hm0'], 2)*np.power(kwargs['fp'], 4)/np.power(2*np.pi, 4)
            self.B = 1.25*np.power(kwargs['fp'], 4)

        self.f = self.get_values(0.05, 1, 20)

    def single_value(self, f):
        return self.A*np.power(f, -5)*np.exp(-self.B*np.power(f, -4))

    def get_values(self, minimum, maximum, number_of_steps):
        fs = np.linspace(minimum, maximum, number_of_steps)
        Sf = np.zeros(number_of_steps)

        for i in range(number_of_steps):
            Sf[i] = self.single_value(fs[i])

        return Sf


