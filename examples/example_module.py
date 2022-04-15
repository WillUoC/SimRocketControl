# Necessary imports go here

class ExampleClass:
    """
    A concise explanation on what this class is and what it does
    Notice how all docstrings are DIRECTLY below the class/function declaration with an empty line
    separating it from the code after it.

    ...
    Attributes
    -----------
    attribute_name : variable_type
        Explanation of attribute
    example_attribute : string
        Attributes are variables passed into a class when it is defined! 
        Useful for setting up important features or constants of a class
        I.E. Setting the mass or other initial conditions of a class (or object).
    
    Methods
    -------
    example_external_function
    """

    # CLASS VARIABLES are initialized here:
    #       These variables are shared by EVERY instance of ExampleClass. If you change their value,
    #       it changes for EVERY ExampleClass!
    
    # Initializing class variables with default values. Most will be overriden in the init method.
    example_class_variable = 0                  # Regular variable with all lower-case underscored names to improve readability
    _example_internal_class_variable = None     # Internal variables (Anything NOT shared outside the class) start with a leading underscore
    EXAMPLE_CLASS_CONSTANT = 0                  # Constant variables whose value never changes (besides the initial setting of the value) are capitalized
    

    # The __init__ function is a requirement for EVERY class. It holds the code that is ran when the class is first instantiated.
    # This function is where you initialize all the variables you will need for running its functions as well as any other setup necessary.
    def __init__(self, class_attribute_1, class_attribute_2, instance_attribute):
        """
        Parameters
        ----------
        class_attribute_1 : float
            A simple float to demonstrate attributes
        class_attribute_2 : float, nxn matrix
            A more complex float matrix to show fun attributes
        instance_attribute : int
            An int to show what an instance attribute is
        """

        # __init__ docstring above simply lists out the attributes expected for the class, their expected type, and a short explanation of what they are
        self.example_class_variable = class_attribute_1                 # Setting the class attribute to a parameter. This will likely be accessed multiple times
        self._example_internal_class_variable = class_attribute_2       # Setting the internal class variable to the second attribute. This will only ever be changed or accessed by internal functions
        self.new_instance_variable = instance_attribute                 # Creating a new variable and setting it to the third attribute passed in

        # If either of the first two variables change, they will change for every instance of the class. However, changing the third variable will
        # only change that variable for its specific instance.

    # An internal function. Usually something useful for this class to work, but not something you need code outside this class to use.
    # Like internal variables, internal function names are preceeded with an underscore.
    # Also, note how all functions start with the parameter 'self'. This is a requirement for all functions within a class.
    def _example_internal_function(self, input_1, input_2):
        """A concise explanation of what this internal function does (what does it update/return/change?)

        Parameters
        ----------
        input_1 : int
            A frivolous variable for explanation purposes
        input_2 : float
            Another useless input for demonstration
        
        Returns
        -------
        output : float
            What you can expect the function to return
        """

        function_var = input_1                                  # Create a new variable for this function only. When this function is done running, the variable is discarded.
        function_var -= 2

        self.new_instance_variable += input_2                   # When calling a class or instance variable, you must use 'self.{variable_name}'. 
                                                                # Otherwise it will assume it's a function variable (and probably break)
        output = function_var + self.new_instance_variable      # Setting output to a variable before returning, useful for debugging
        return(output)

    # This is where I put external functions. This is the most important part of Object Oriented Programming, as this is how classes interact with outside code!
    def example_external_function(self, ext_input_1, ext_input_2):
        """A concise explanation of what this external function does (what does it update/return/change?)

        Parameters
        ----------
        ext_input_1 : int
            A frivolous variable for explanation purposes
        ext_input_2 : float
            Another useless input for demonstration
        
        Returns
        -------
        output : float
            What you can expect the function to return
        """
        
        output = self._example_internal_function(ext_input_1, ext_input_2) / self.EXAMPLE_CLASS_CONSTANT
        return(output)

