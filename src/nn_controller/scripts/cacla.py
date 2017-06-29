import numpy as np
import math
from pyfann import libfann
import os
import util.scale as scale

# Constants
EXPLORE_GREEDY = 1     #: E-Greedy Exploration: probability of random action
EXPLORE_GAUSSIAN = 2   #: Gaussian exploration around current estimate


class Cacla(object):
    
    def __init__(self, base_path, network = [1, 2, 1], **kwargs):
        
        self.actor_file = os.path.join(base_path, "actor_float.net")
        self.critic_file = os.path.join(base_path,"critic_float.net")
        self.progress_file = os.path.join(base_path, "progress")
        self.critic_output_file = os.path.join(base_path, "critic_output")
            
        self.networkConfig = network
        
        self.last_output = None
        self.last_value = None
        self.last_state = None
        
        self.random_range = 0.1
        
        self.alpha = 0.01               # Learning rate for updating the Actor
        self.min_alpha = 0.0001         # Minimum learning rate for the Actor
        self.beta = 0.01                # Learning rate for the Critic
        self.min_beta = 0.0001          # Minimum learning rate for the Critic
        self.epsilon = 0.2              # Epsilon for e-greedy exploration
        self.min_epsilon = 0.1          # Minimum exploration
        self.sigma = 2.0                # Standard Deviation for Gaussian
                                        # exploration
        self.min_sigma = 0.2            # Minimum exploration
        self.discount_factor = 0.99     # Decay of the reward
        self.learning_decay = 1.0       # Decay of the learning rates
        self.td_var = 1.0               # Starting variance
        self.var_beta = 0.001           # Factor of running average of variance
        
        self.random_decay = 0.999       # Decay of the epsilon and sigma factors
        self.explore_strategy = EXPLORE_GAUSSIAN # Exploration strategy
        self.progress = 0               # Number of iterations already  
        
        try:
            self.progress = int(open(self.progress_file, 'r').read())
            print "[%s] Continuing from step %d" % (self.progress_file, self.progress)
        except:
            self.progress = 0
        
        
        if kwargs:
            self.set_parameters(**kwargs)
     
    def set_parameters(self, **kwargs):
        """
        Set the parameters for the algorithm. Each parameter may be specified as
        a keyword argument. Available parameters are:

        alpha               The learning rate for the Critic in [0, 1]
        beta                The learning rate for the Actor in [0, 1]
        epsilon             The exploration probability for 
                            e-greedy exploration in [0, 1]
        sigma               The standard deviation for Gaussian exploration > 0
        discount_factor     The value attributed to future rewards in (0, 1)
        random_decay        The decay of the epsilon and sigma paramters after
                            each save of the algorithm. This is the factor
                            the value is multiplied with. Should be in [0, 1]
        explore_Strategy    EXPLORE_GAUSSIAN or EXPLORE_GREEDY
        num_outputs         The number of outputs required from the actor. This
                            should be an integer greater than 0.
        ensemble_size       The number of Neural Networks to use in ensemble to
                            optimize the output of the actor and critic. The
                            output of these networks is averaged to obtain the
                            next action.
        td_var              The initial variance of the TD-error. Default 1.0
        var_beta            The factor of the update of the running average of the
                            varianceof the TD-error. Default: 0.001
        """
        for key, value in kwargs.iteritems():
            if key == "explore_strategy":
                if value == EXPLORE_GAUSSIAN or \
                   value == EXPLORE_GREEDY:
                   self.explore_strategy = value
                else:
                    raise Exception("Invalid exploration strategy %d" % value)                
            elif key == "sigma" and type(value) is tuple:
                self.min_sigma, self.sigma = value
            elif key == "epsilon" and type(value) is tuple:
                self.min_epsilon, self.epsilon = value
            elif key == "alpha" and type(value) is tuple:
                self.min_alpha, self.alpha = value
            elif key == "beta" and type(value) is tuple:
                self.min_beta, self.beta = value
            elif key in self.__dict__:
                self.__dict__[key] = float(value)
            elif not key in self.__dict__:
                raise Exception("Unknown setting: %s = %s" % (key, repr(value)))   
        
    def reset(self):        
        self.last_output = None
        self.last_value = None
        self.last_state = None
        self.first = True
      
    def setup_ann(self, nn_input):
        
        hidden_activation = libfann.SIGMOID_SYMMETRIC
        
        try:
            num_inputs = int(nn_input)
        except:
            num_inputs = len(nn_input)
        network = [num_inputs]
        
        for n in self.networkConfig:
            network.append(n)
        
        
        self.actor = libfann.neural_net()
        if (self.actor.create_from_file(self.actor_file)):
            print 'Loaded Actor from file'
        else:
            self.actor.create_standard_array(network)
            self.actor.randomize_weights(-self.random_range, self.random_range)
            self.actor.set_activation_function_hidden(hidden_activation)
            self.actor.set_activation_function_output(libfann.LINEAR)
        
        
        self.critic = libfann.neural_net()
        if (self.critic.create_from_file(self.critic_file)):
            print 'Loaded Critic from file' 
        else:
            network[len(network)-1] = 1
            self.critic.create_standard_array(network)
            self.critic.randomize_weights(-self.random_range, self.random_range)
            self.critic.set_activation_function_hidden(hidden_activation)
            self.critic.set_activation_function_output(libfann.LINEAR)
            
        self.sigma = max(self.min_sigma, self.sigma * (self.random_decay ** self.progress))
        self.epsilon = max(self.min_epsilon, self.epsilon * (self.random_decay ** self.progress))
        self.alpha = max(self.min_alpha, self.alpha * (self.learning_decay ** self.progress))
        self.beta = max(self.min_beta, self.beta * (self.learning_decay ** self.progress))
        
        self.actor.set_learning_rate(self.alpha)
        self.critic.set_learning_rate(self.beta)
        
    def save(self):
        
        self.actor.save(self.actor_file)
        self.critic.save(self.critic_file)
        
       # if self.progress % 3 == 0: # reduce the exploration reduction
        self.sigma = max(self.min_sigma, self.sigma * self.random_decay)
        self.epsilon = max(self.min_epsilon, self.epsilon * self.random_decay)

        self.alpha = self.alpha * self.learning_decay
        self.beta = self.beta * self.learning_decay

        self.critic.set_learning_rate(self.beta)
        self.actor.set_learning_rate(self.alpha)
        
        self.progress += 1

        f = open(self.progress_file, "w")
        f.write("%d" % self.progress)
        f.close()
        
        self.reset()
        
    def exploration(self, estimate):
        
        if self.explore_strategy == EXPLORE_GREEDY:

            explore = np.random.random(1)[0] <= self.epsilon
            if explore:
                ran_values = np.random.random(len(estimate))
                result = [scale.scale(value, 0.0, 1.0, -36.0, 36.0) for value in ran_values]
            else:
                result = estimate
        elif self.explore_strategy == EXPLORE_GAUSSIAN:
            
            variances = [(self.sigma ** 2)] * len(estimate)
            cov = np.diag(variances)

            # Draw random sample from Gaussian distribution
            result = np.random.multivariate_normal(estimate, cov)

            # Make sure all values are in the interval [-1, 1]
            result = [max(min(36.0, x), -36.0) for x in result]

        return result
    
    def update_networks(self, current_value, reward, terminal):
        if not self.last_output or not self.last_value:
            #print "[CACLA] Skipping first step - no previous step"
            return

        # Train Critic
        # Determine TD-error (formula 2)
        if terminal:
            #print "Using terminal TD rule"
            td_error = reward - self.last_value
        else:
            td_error = reward + self.discount_factor * current_value \
                       - self.last_value

        if False:
            # This is the update rule, specified by the algorithm. 
            new_value = self.last_value + self.alpha * td_error
        else:
            # ANN implementation: just calculate the new value and train the
            # ANN on that. Alpha will be set as learning rate so it is still
            # used.
            new_value = self.last_value + td_error

        self.critic.train(self.last_state, [new_value])
        
      #  f = open(self.critic_output_file, 'a')
      #  f.write(str(new_value) + ", " + str(current_value) + ", " + str(td_error) + ", " + str(self.last_output) +  '\n')
      #  f.close()

        # Update running variance of the TD-error when using CACLA+Var
        if self.var_beta > 0:
            self.td_var = (1 - self.var_beta) * self.td_var + \
                          self.var_beta * (td_error ** 2)

        # Train Actor if state value improved as a result of performing the action
        # performed in the last state.
        if td_error > 0:
            updates = 1
            if self.var_beta > 0:
                updates = int(math.ceil(td_error / math.sqrt(self.td_var)))
                
            for i in range(updates):
                self.actor.train(self.last_state, self.last_output)

     
    def run(self, state, reward, terminal=False):
        
        # Check if the ANNs have been set up, and if not, do so.
        if not hasattr(self, 'actor'):
            self.setup_ann(state)

        # Run the ANNs
        nn_output = self.actor.run(state)
        value = self.critic.run(state)[0]

        # Update actor and critic
        self.update_networks(value, reward, terminal)
 
        # Apply exploration policy
        result = self.exploration(nn_output)

        # Store output for evaluation in the next iteration
        self.last_output = result[:]
        self.last_value = value
        self.last_state = state[:]


        # We've got the next action, return it
        return result   
        