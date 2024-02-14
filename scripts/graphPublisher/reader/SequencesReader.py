import yaml


class SequencesLoader:

    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.data = self.load_yaml()

    def load_yaml(self):
        with open(self.yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def get_args(self):
        return self.data.get('Args', [])

    def get_sequence(self):
        return self.data.get('Sequence', [])
