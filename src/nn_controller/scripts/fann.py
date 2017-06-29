from pyfann import libfann

class NeuralNetwork():
    
    def __init__(self, network=[2, 10, 1], learning_rate = 0.2, connection_rate = 1):
        
        self.ann = libfann.neural_net()
        self.ann.create_sparse_array(connection_rate, tuple(network))
        self.ann.set_learning_rate(learning_rate)
        self.ann.set_activation_function_hidden(libfann.SIGMOID_SYMMETRIC)
        self.ann.set_activation_function_output(libfann.LINEAR)
          
     
    def train(self, input, expected_output):
        
        self.ann.train(input, expected_output)   
        
     
    def run(self, input): 
                  
        return self.ann.run(input)
    
    def save(self):
        
        self.ann.save('testNetwork.net');

    def load(self, path=''):
        self.ann.create_from_file(path)

