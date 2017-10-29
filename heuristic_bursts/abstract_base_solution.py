from abc import ABCMeta, abstractmethod, abstractproperty


class AbstractBaseSolution:
    __metaclass__ = ABCMeta
    id_counter = 0
    number_of_metrics = 0
    number_of_highertier_operations = 3

    def stamp_solution(self):
        self.id = AbstractBaseSolution.id_counter
        AbstractBaseSolution.id_counter += 1

    # ########### Basic operations for solutions
    @abstractmethod
    def generate_initial(self):
        return

    @abstractmethod
    def evaluate(self):
        return

    @abstractmethod
    def __deepcopy__(self):
        return
    # ########### Basic operations for solutions


    # ########### Higher tier operations
    # Lucas: This is where function prototypes go for higher tier operations.
    # Since these are all defined as abstract methods, the actual function definitions don't go here.
    # ########### Higher tier operations


    # ########### Comparison operators
    @abstractmethod
    def __lt__(self, other):
        return

    @abstractmethod
    def __le__(self, other):
        return

    @abstractmethod
    def __gt__(self, other):
        return

    @abstractmethod
    def __ge__(self, other):
        return

    @abstractmethod
    def __eq__(self, other):
        return

    @abstractmethod
    def __ne__(self, other):
        return
    # ########### Comparison operators

