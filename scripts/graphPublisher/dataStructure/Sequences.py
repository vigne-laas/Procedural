class Sequences:
    def __init__(self, args, sequence):
        self.variables = {}
        self.actions = []
        self.parse_args(args)
        self.parse_sequence(sequence)

    def parse_args(self, args):
        for arg in args:
            for key, value in arg.items():
                self.variables[key] = value

    def parse_sequence(self, sequence):
        for action in sequence:
            for action_name, params in action.items():
                action_params = {}
                for param in params:
                    for literal, value in param.items():
                        action_params[literal] = self.variables.get(value, value)
                self.actions.append((action_name, action_params))
